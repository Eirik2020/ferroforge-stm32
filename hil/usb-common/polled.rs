// PROTOTYPE bench app, polled variant, shared by every board's USB bench:
// each board crate's src/bin/usb_polled.rs includes it, and its paths are
// relative to that file. The board module supplies the clocks, the OTG_FS
// wiring on PA11/PA12, the serial-number string and the checks.

use defmt_rtt as _;
use panic_probe as _;

use ferroforge_stm32::usb_polled::{UsbBus, UsbStack};
use ferrohal_stm32::usb::usb_device::device::UsbDevice;
use usbd_serial::SerialPort;

/// The firmware's stack: the device and its one class, polled together.
pub struct Cdc {
    device: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
}

impl UsbStack for Cdc {
    fn poll(&mut self) -> bool {
        self.device.poll(&mut [&mut self.serial])
    }
}

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::usb_polled::{self, UsbBus};
    use ferrohal_stm32::usb::usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };
    use usbd_serial::SerialPort;
    use crate::{Cdc, board, common::{self, Echo}};

    #[shared]
    struct Shared {
        stats: board::Stats,
        usb: Cdc,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        Mono::start(core.SYST, board::SYSCLK_HZ);
        let (mut rcc, wiring) = board::usb_wiring(cx.device);
        let memory = cortex_m::singleton!(: [u32; 1024] = [0; 1024])
            .unwrap_or_else(|| defmt::panic!("endpoint memory"));
        let bus = match usb_polled::new(&mut rcc, wiring, memory) {
            Ok(bus) => bus,
            Err(refused) => defmt::panic!("USB: {}", refused.error()),
        };
        let bus: &'static UsbBusAllocator<UsbBus> =
            cortex_m::singleton!(: UsbBusAllocator<UsbBus> = bus)
                .unwrap_or_else(|| defmt::panic!("bus"));
        let serial = SerialPort::new(bus);
        let Ok(builder) = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd)).strings(&[
            StringDescriptors::default()
                .manufacturer("FerroForge")
                .product("FerroForge USB port bench")
                .serial_number(board::SERIAL_POLLED),
        ]) else {
            defmt::panic!("USB strings")
        };
        let device = builder.device_class(usbd_serial::USB_CLASS_CDC).build();
        defmt::info!("variant P (usb_polled, polled in the vector) on {}", board::NAME);
        checks::spawn().ok();
        (
            Shared { stats: board::Stats::new(), usb: Cdc { device, serial } },
            Local {},
        )
    }

    #[group(from = ferroforge_stm32::usb_polled, shared = [stack = usb], spawn = [events = usb_events])]
    mod usb_port {
        #[task(binds = OTG_FS, priority = 3)]
        fn on_usb_polled;
    }

    /// The firmware's task: locks the stack and echoes through its class.
    #[task(priority = 2, shared = [stats, usb], local = [echo: Echo = Echo::new()])]
    async fn usb_events(mut cx: usb_events::Context) {
        let echo = cx.local.echo;
        let configured = cx.shared.usb.lock(|usb| {
            let serial = &mut usb.serial;
            echo.step(serial, |s, buf| s.read(buf).ok(), |s, buf| s.write(buf).ok());
            usb.device.state() == UsbDeviceState::Configured
        });
        cx.shared.stats.lock(|stats| stats.update(configured, echo));
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        common::idle_meter()
    }

    /// Times the session and judges it.
    #[task(priority = 1, shared = [stats])]
    async fn checks(mut cx: checks::Context) {
        let failed = board::run_checks(&mut cx.shared.stats, || async {
            Mono::delay(1.millis()).await;
        }, || Mono::now().duration_since_epoch().to_millis() as u32,
        || 0)
        .await;
        Mono::delay(200.millis()).await;
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }
}
