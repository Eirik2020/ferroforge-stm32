// PROTOTYPE bench app, locked variant (L), shared by every board's USB bench:
// each board crate's src/bin/usb_locked.rs includes it, and its paths are
// relative to that file. The board module supplies the clocks, the OTG_FS
// wiring on PA11/PA12, the serial-number string and the checks.

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::usb_locked::{self, UsbBus, UsbVector, UsbWaiter, UsbWake};
    use ferrohal_stm32::usb::usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };
    use usbd_serial::SerialPort;
    use crate::{board, common::{self, Echo}};

    #[shared]
    struct Shared {
        stats: board::Stats,
        vector: UsbVector,
    }

    #[local]
    struct Local {
        wake: UsbWake,
        waiter: UsbWaiter,
        device: UsbDevice<'static, UsbBus>,
        serial: SerialPort<'static, UsbBus>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        Mono::start(core.SYST, board::SYSCLK_HZ);
        let (mut rcc, wiring) = board::usb_wiring(cx.device);
        let memory = cortex_m::singleton!(: [u32; 1024] = [0; 1024])
            .unwrap_or_else(|| defmt::panic!("endpoint memory"));
        let resources = match usb_locked::new(&mut rcc, wiring, memory) {
            Ok(resources) => resources,
            Err(refused) => defmt::panic!("USB: {}", refused.error()),
        };
        let bus: &'static UsbBusAllocator<UsbBus> =
            cortex_m::singleton!(: UsbBusAllocator<UsbBus> = resources.bus)
                .unwrap_or_else(|| defmt::panic!("bus"));
        let serial = SerialPort::new(bus);
        let Ok(builder) = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd)).strings(&[
            StringDescriptors::default()
                .manufacturer("FerroForge")
                .product("FerroForge USB port bench")
                .serial_number(board::SERIAL_LOCKED),
        ]) else {
            defmt::panic!("USB strings")
        };
        let device = builder.device_class(usbd_serial::USB_CLASS_CDC).build();
        defmt::info!("variant L (usb_locked, locked) on {}", board::NAME);
        checks::spawn().ok();
        usb_task::spawn().ok();
        (
            Shared {
                stats: board::Stats::new(),
                vector: resources.vector,
            },
            Local {
                wake: resources.wake,
                waiter: resources.waiter,
                device,
                serial,
            },
        )
    }

    #[group(from = ferroforge_stm32::usb_locked, shared = [vector = vector], local = [wake = wake])]
    mod usb {
        #[task(binds = OTG_FS, priority = 3)]
        fn on_usb_locked;
    }

    /// The firmware's task: owns the device and its class, and runs each
    /// poll and echo pass under the lock it shares with the member.
    #[task(priority = 2, shared = [stats, vector], local = [waiter, device, serial, echo: Echo = Echo::new()])]
    async fn usb_task(mut cx: usb_task::Context) {
        let device = cx.local.device;
        let serial = cx.local.serial;
        let echo = cx.local.echo;
        loop {
            cx.local.waiter.wait().await;
            // The lock keeps the member, and so every other access to the
            // core, out of this pass's IN packet writes.
            cx.shared.vector.lock(|_| {
            for _ in 0..4 {
                let busy = device.poll(&mut [serial]);
                let before = echo.progress();
                echo.step(serial, |s, buf| s.read(buf).ok(), |s, buf| s.write(buf).ok());
                if !busy && echo.progress() == before {
                    break;
                }
            }
            });
            let configured = device.state() == UsbDeviceState::Configured;
            cx.shared.stats.lock(|stats| stats.update(configured, echo));
        }
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
        || usb_locked::VECTOR_ENTRIES.load(core::sync::atomic::Ordering::Relaxed))
        .await;
        Mono::delay(200.millis()).await;
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }
}
