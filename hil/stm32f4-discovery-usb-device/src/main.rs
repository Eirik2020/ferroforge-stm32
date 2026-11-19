//! The USB device port against a real host: `ferroforge_stm32::usb_device`
//! on the STM32F4 Discovery (MB997, STM32F407VGT6), whose CN5 micro-AB
//! connector is the F407's own OTG_FS port, nothing to fit.
//!
//! Routing (UM1472 section 6.10; DS8626 table 9): OTG_FS DM on PA11 and DP
//! on PA12, alternate function 10, both at very high speed. The clock is the
//! ST-LINK's 8 MHz MCO in bypass (UM1472 section 6.11.1): 8 MHz / 8 * 336 =
//! 336 MHz VCO, / 2 = 168 MHz core, / 7 = 48 MHz for OTG_FS (RM0090 section
//! 6.3.2). PD4 is the CS43L22's reset, held low so the audio DAC stays
//! powered down (UM1472 section 6.13).
//!
//! The firmware's stack holds the vector service, the device and one
//! `usbd-serial` CDC class behind one lock (PORT-502). The echo task takes
//! the lock once per round, one poll and one read or write, so the longest
//! hold is one round's, and FerroHAL's `hil/usb_cdc_host.py` checks every
//! byte that comes back.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `OTG_FS` | `usb`'s member | 3 |
//! | `SPI2`, `SPI3` | RTIC: `echo` (2), `checks` (1) | |
//!
//! Each check prints `PASS` or `FAIL` with the requirements it verifies in
//! brackets; the run passes when every check does.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use ferroforge_stm32::usb_device::{UsbBus, UsbStack, UsbVector};
use ferrohal_stm32::{
    chip::Peripherals,
    gpio::Speed,
    rcc::{
        AhbPrescaler, ApbPrescaler, Config, Hse, HseMode, Pll, PllP, PllSource, Rcc, Supply, Sysclk,
    },
    time::Hertz,
    usb::{Wiring, usb_device::device::UsbDevice},
};
use usbd_serial::SerialPort;

/// The host's end marker (`usb_cdc_host.py`).
const END: &[u8] = b"BENCHEND";
/// The serial number `run-discovery-usb.sh` asks the host script for.
const SERIAL: &str = "f407-usb-device";
const SYSCLK_HZ: u32 = 168_000_000;

const CLOCK_CONFIG: Config = Config {
    hse: Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Bypass,
    }),
    pll: Some(Pll {
        source: PllSource::Hse,
        prediv: 8,
        mul: 336,
        divp: PllP::Div2,
        divq: Some(7),
    }),
    sys: Sysclk::PllP,
    ahb_pre: AhbPrescaler::Div1,
    apb1_pre: ApbPrescaler::Div4,
    apb2_pre: ApbPrescaler::Div2,
    supply: Supply::V2_7To3_6,
};

/// The clocks, and CN5's wiring.
fn usb_wiring(p: Peripherals) -> (Rcc, Wiring) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
    let mut d = p.GPIOD.split(&mut rcc);
    let mut codec_reset = d.p4.into_push_pull_output(&mut d.ctrl);
    codec_reset.set_low();
    let mut a = p.GPIOA.split(&mut rcc);
    let mut dm = a.p11.into_alternate::<10>(&mut a.ctrl);
    dm.set_speed(&mut a.ctrl, Speed::VeryHigh);
    let mut dp = a.p12.into_alternate::<10>(&mut a.ctrl);
    dp.set_speed(&mut a.ctrl, Speed::VeryHigh);
    let wiring = Wiring {
        usb: p.USB_OTG_FS,
        dm,
        dp,
    };
    (rcc, wiring)
}

/// The firmware's stack: the vector service, the device and its one class.
pub struct Stack {
    vector: UsbVector,
    device: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
}

impl UsbStack for Stack {
    fn vector(&mut self) -> &mut UsbVector {
        &mut self.vector
    }
}

/// What the echo task has seen.
#[derive(Clone, Copy, Default)]
pub struct Stats {
    configured: bool,
    wakes: u32,
    received: u32,
    echoed: u32,
    ended: bool,
    /// The longest one round held the stack's lock, in CPU cycles.
    hold_max: u32,
}

/// The echo: what came in, waiting to go back out.
struct Echo {
    pending: [u8; 64],
    head: usize,
    len: usize,
    tail: [u8; 8],
}

impl Echo {
    const fn new() -> Self {
        Self {
            pending: [0; 64],
            head: 0,
            len: 0,
            tail: [0; 8],
        }
    }

    /// One round on the locked stack: a poll, then a read if nothing waits
    /// or a write of what waits. Whether anything moved.
    fn round(&mut self, stack: &mut Stack, stats: &mut Stats) -> bool {
        let Stack { device, serial, .. } = stack;
        let mut moved = device.poll(&mut [serial]);
        if self.len == 0 {
            self.head = 0;
            if let Ok(count) = serial.read(&mut self.pending) {
                let count = count.min(self.pending.len());
                moved |= count > 0;
                self.len = count;
                stats.received += count as u32;
                for &byte in &self.pending[..count] {
                    self.tail.rotate_left(1);
                    self.tail[7] = byte;
                    if self.tail == *END {
                        stats.ended = true;
                    }
                }
            }
        } else if let Ok(count) = serial.write(&self.pending[self.head..self.head + self.len]) {
            let count = count.min(self.len);
            moved |= count > 0;
            self.head += count;
            self.len -= count;
            stats.echoed += count as u32;
        }
        stats.configured |=
            device.state() == ferrohal_stm32::usb::usb_device::device::UsbDeviceState::Configured;
        moved
    }
}

/// Prints the check's verdict; 1 for a failure, to count them.
fn verdict(name: &str, pass: bool) -> u32 {
    if pass {
        defmt::info!("PASS {}", name);
        0
    } else {
        defmt::error!("FAIL {}", name);
        1
    }
}

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);
    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::usb_device::{self, Memory, UsbBus, UsbBusAllocator, UsbWaiter, UsbWake};
    use ferrohal_stm32::usb::usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
    use usbd_serial::SerialPort;
    use crate::{Echo, SERIAL, SYSCLK_HZ, Stack, Stats, usb_wiring, verdict};

    #[shared]
    struct Shared {
        stats: Stats,
        usb: Stack,
    }

    #[local]
    struct Local {
        wake: UsbWake,
        waiter: UsbWaiter,
    }

    #[init(local = [ep_memory: [u32; 1024] = [0; 1024], memory: Memory = Memory::new()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        Mono::start(core.SYST, SYSCLK_HZ);
        let (mut rcc, wiring) = usb_wiring(cx.device);
        let port = match usb_device::new(&mut rcc, wiring, cx.local.ep_memory, cx.local.memory) {
            Ok(port) => port,
            Err(refused) => defmt::panic!("USB: {}", refused.error()),
        };
        let bus: &'static UsbBusAllocator<UsbBus> =
            cortex_m::singleton!(: UsbBusAllocator<UsbBus> = port.bus)
                .unwrap_or_else(|| defmt::panic!("bus"));
        let serial = SerialPort::new(bus);
        let Ok(builder) = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd)).strings(&[
            StringDescriptors::default()
                .manufacturer("FerroForge")
                .product("FerroForge USB port bench")
                .serial_number(SERIAL),
        ]) else {
            defmt::panic!("USB strings")
        };
        let device = builder.device_class(usbd_serial::USB_CLASS_CDC).build();
        defmt::info!("usb_device on the STM32F4 Discovery's CN5");
        checks::spawn().ok();
        echo::spawn().ok();
        (
            Shared {
                stats: Stats::default(),
                usb: Stack { vector: port.vector, device, serial },
            },
            Local {
                wake: port.wake,
                waiter: port.waiter,
            },
        )
    }

    #[group(from = ferroforge_stm32::usb_device, shared = [stack = usb], local = [wake = wake])]
    mod usb {
        #[task(binds = OTG_FS, priority = 3)]
        fn on_usb;
    }

    /// The firmware's task: per wake, rounds under the stack's lock until
    /// one moves nothing.
    #[task(priority = 2, shared = [stats, usb], local = [waiter, echo: Echo = Echo::new()])]
    async fn echo(mut cx: echo::Context) {
        let echo = cx.local.echo;
        let mut stats = Stats::default();
        loop {
            cx.local.waiter.wait().await;
            stats.wakes = stats.wakes.wrapping_add(1);
            loop {
                let started = DWT::cycle_count();
                let moved = cx.shared.usb.lock(|usb| echo.round(usb, &mut stats));
                stats.hold_max = stats.hold_max.max(DWT::cycle_count().wrapping_sub(started));
                if !moved {
                    break;
                }
            }
            cx.shared.stats.lock(|shared| *shared = stats);
        }
    }

    /// Waits for the host's session, then a quiet second, and judges both.
    #[task(priority = 1, shared = [stats])]
    async fn checks(mut cx: checks::Context) {
        let start = Mono::now();
        let mut first = None;
        let mut seen = Stats::default();
        while (Mono::now() - start).to_millis() < 60_000 {
            Mono::delay(1.millis()).await;
            seen = cx.shared.stats.lock(|stats| *stats);
            if first.is_none() && seen.received > 0 {
                first = Some(Mono::now());
            }
            if seen.ended {
                break;
            }
        }
        let session_ms = first.map(|first| (Mono::now() - first).to_millis()).unwrap_or(0);
        let mut failed = 0;
        failed += verdict("the host configured the device [PORT-501]", seen.configured);
        failed += verdict(
            "the host finished, every byte echoed but the marker [PORT-501, PORT-502]",
            seen.ended && seen.echoed + 8 >= seen.received,
        );
        // The host closes its port after the session; let its last frames end.
        Mono::delay(200.millis()).await;
        let before = cx.shared.stats.lock(|stats| stats.wakes);
        Mono::delay(1000.millis()).await;
        let quiet = cx.shared.stats.lock(|stats| stats.wakes).wrapping_sub(before);
        failed += verdict("a configured, quiet second woke the task not at all [PORT-501]", quiet == 0);
        defmt::info!(
            "MEASURE session: {} ms, {} bytes received, {} echoed, {} task wakes, longest lock hold {} cycles; quiet second: {} wakes",
            session_ms,
            seen.received,
            seen.echoed,
            seen.wakes,
            seen.hold_max,
            quiet
        );
        Mono::delay(200.millis()).await;
        if failed == 0 {
            defmt::info!("all checks passed");
            debug::exit(EXIT_SUCCESS);
        } else {
            defmt::error!("{} checks failed", failed);
            debug::exit(EXIT_FAILURE);
        }
    }
}
