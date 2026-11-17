//! PROTOTYPE, variant B: `uart_queue_port`'s members own FerroHAL's `Uart`
//! and the port's queue ends; each consumer owns its two handles as locals
//! and names no UART, wiring or lock. Links and routing as
//! `nucleo-f401re-uart-loopback` (J1, J2; RM0368 tables 9 and 29).

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI1, SPI2],

    use ferrohal_stm32::{
        chip::{USART1, USART6},
        dma::Stream,
        gpio::{Alternate, Pin, Pull, Speed},
        rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
        time::Hertz,
        uart::{self, Wiring},
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::uart_queue::{self, Port, Queues, RxHandle, TxHandle};
    use ferrohal_stm32::device::Interrupt;
    use nucleo_f401re_uart_ab::Loop;

    /// 84 MHz from HSI, the F401's ceiling (DS10086 section 3.13): 16 MHz /
    /// 16 * 336 = 336 MHz VCO, / 4 = 84 MHz. APB1 at 42 MHz, its limit.
    const CLOCK_CONFIG: Config = Config {
        hse: None,
        pll: Some(Pll {
            source: PllSource::Hsi,
            prediv: 16,
            mul: 336,
            divp: PllP::Div4,
            divq: Some(7),
        }),
        sys: Sysclk::PllP,
        ahb_pre: AhbPrescaler::Div1,
        apb1_pre: ApbPrescaler::Div2,
        apb2_pre: ApbPrescaler::Div1,
        supply: Supply::V2_7To3_6,
    };

    /// Checked while compiling, as FerroHAL's benches check theirs.
    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the F401's limits"),
    };

    const FRAME: uart::Config = uart::Config::new(Hertz(115_200));
    const _: () = assert!(uart::check::<USART1>(&CLOCKS, &FRAME).is_ok());
    const _: () = assert!(uart::check::<USART6>(&CLOCKS, &FRAME).is_ok());

    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    type Af8<const P: char, const N: u8> = Pin<P, N, Alternate<8>>;


    const RX_RING: usize = 256;
    const TX_BUFFER: usize = 256;

    /// Bytes each port sends in the main run (about 1.4 s at 115200).
    const TOTAL: u32 = 16_384;
    /// Extra bytes sent while the consumer holds off reading.
    const FLOOD: u32 = 2_048;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// Waits until `done` or about `ms` milliseconds at 84 MHz.
    fn wait_for(ms: u32, mut done: impl FnMut() -> bool) {
        for _ in 0..ms {
            if done() {
                return;
            }
            cortex_m::asm::delay(84_000);
        }
    }

    fn verdict(name: &str, pass: bool) -> u32 {
        if pass {
            defmt::info!("PASS {}", name);
            0
        } else {
            defmt::error!("FAIL {}", name);
            1
        }
    }

    fn report(variant: &str, name: &str, l: &Loop) -> u32 {
        defmt::info!(
            "{} {}: sent {} read {} wrong {} gaps {} wakes {} longest step {} cycles",
            variant, name, l.sent, l.read, l.wrong, l.gaps, l.wakes, l.longest_step
        );
        verdict("main run: every byte back, in order", l.read == TOTAL && l.wrong == 0 && l.gaps == 0)
    }

    fn finish(failed: u32) -> ! {
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        cortex_m::asm::delay(16_800_000);
        if failed == 0 { debug::exit(EXIT_SUCCESS) } else { debug::exit(EXIT_FAILURE) }
        loop {}
    }

    const QUEUE: usize = 512;

    type Port1 = Port<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    type Port6 = Port<USART6, Af8<'A', 11>, Af8<'A', 12>, Stream<2, 6>, Stream<2, 1>>;

    /// What a consumer owns: its two handles. The same type for every UART.
    pub struct Handles {
        pub rx: RxHandle,
        pub tx: TxHandle,
    }

    impl embedded_io::ErrorType for Handles {
        type Error = uart_queue::PortError;
    }
    impl embedded_io::Read for Handles {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> { self.rx.read(buf) }
    }
    impl embedded_io::ReadReady for Handles {
        fn read_ready(&mut self) -> Result<bool, Self::Error> { self.rx.read_ready() }
    }
    impl embedded_io::Write for Handles {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> { self.tx.write(buf) }
        fn flush(&mut self) -> Result<(), Self::Error> { self.tx.flush() }
    }
    impl embedded_io::WriteReady for Handles {
        fn write_ready(&mut self) -> Result<bool, Self::Error> { self.tx.write_ready() }
    }

    #[shared]
    struct Shared {
        port1: Port1,
        port6: Port6,
        loop1: Loop,
        loop6: Loop,
    }

    #[local]
    struct Local {
        h1: Handles,
        h6: Handles,
    }

    #[init(local = [
        tx1: [u8; TX_BUFFER] = [0; TX_BUFFER],
        rx1: [u8; RX_RING] = [0; RX_RING],
        tx6: [u8; TX_BUFFER] = [0; TX_BUFFER],
        rx6: [u8; RX_RING] = [0; RX_RING],
        q1: Queues<QUEUE, QUEUE> = Queues::new(),
        q6: Queues<QUEUE, QUEUE> = Queues::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let p = cx.device;
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpioa = p.GPIOA.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let ctrl = &mut gpioa.ctrl;
        let mut tx1 = gpioa.p9.into_alternate::<7>(ctrl);
        tx1.set_speed(ctrl, Speed::Medium);
        let mut rx1 = gpioa.p10.into_alternate::<7>(ctrl);
        rx1.set_pull(ctrl, Pull::Up);
        let mut tx6 = gpioa.p11.into_alternate::<8>(ctrl);
        tx6.set_speed(ctrl, Speed::Medium);
        let mut rx6 = gpioa.p12.into_alternate::<8>(ctrl);
        rx6.set_pull(ctrl, Pull::Up);
        let w1 = Wiring { uart: p.USART1, tx: tx1, rx: rx1, tx_dma: dma2.s7, rx_dma: dma2.s2 };
        let w6 = Wiring { uart: p.USART6, tx: tx6, rx: rx6, tx_dma: dma2.s6, rx_dma: dma2.s1 };
        let local = cx.local;
        // The firmware, not the consumer, names the vector the handles pend.
        let r1 = uart_queue::new(&mut rcc, w1, &FRAME, local.tx1, local.rx1, local.q1,
            || rtic::pend(Interrupt::DMA2_STREAM7))
            .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let r6 = uart_queue::new(&mut rcc, w6, &FRAME, local.tx6, local.rx6, local.q6,
            || rtic::pend(Interrupt::DMA2_STREAM6))
            .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));
        defmt::info!("variant B on the Nucleo-F401RE");
        checks::spawn().ok();
        let start = Loop { limit: TOTAL, ..Loop::default() };
        (
            Shared { port1: r1.port, port6: r6.port, loop1: start, loop6: start },
            Local { h1: Handles { rx: r1.rx, tx: r1.tx }, h6: Handles { rx: r6.rx, tx: r6.tx } },
        )
    }

    #[group(from = ferroforge_stm32::uart_queue_port, shared = [port = port1], spawn = [ready = usart1_ready])]
    mod usart1 {
        #[task(binds = USART1, priority = 3)]
        fn q_on_uart;
        #[task(binds = DMA2_STREAM2, priority = 3)]
        fn q_on_rx_dma;
        #[task(binds = DMA2_STREAM7, priority = 3)]
        fn q_on_tx_dma;
    }

    #[group(from = ferroforge_stm32::uart_queue_port, shared = [port = port6], spawn = [ready = usart6_ready])]
    mod usart6 {
        #[task(binds = USART6, priority = 3)]
        fn q_on_uart;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn q_on_rx_dma;
        #[task(binds = DMA2_STREAM6, priority = 3)]
        fn q_on_tx_dma;
    }

    /// USART1's consumer: owns its handles, takes no port lock.
    #[task(priority = 2, local = [h1], shared = [loop1])]
    async fn usart1_ready(mut cx: usart1_ready::Context) {
        let h = cx.local.h1;
        cx.shared.loop1.lock(|l| {
            let t = cycles();
            l.step(h);
            l.longest_step = l.longest_step.max(cycles().wrapping_sub(t));
        });
    }

    #[task(priority = 2, local = [h6], shared = [loop6])]
    async fn usart6_ready(mut cx: usart6_ready::Context) {
        let h = cx.local.h6;
        cx.shared.loop6.lock(|l| {
            let t = cycles();
            l.step(h);
            l.longest_step = l.longest_step.max(cycles().wrapping_sub(t));
        });
    }

    #[task(priority = 1, shared = [port1, loop1, loop6])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut port1, mut loop1, mut loop6, .. } = cx.shared;
        usart1_ready::spawn().ok();
        usart6_ready::spawn().ok();
        wait_for(3_000, || loop1.lock(|l| l.read) >= TOTAL && loop6.lock(|l| l.read) >= TOTAL);
        cortex_m::asm::delay(8_400_000);
        let mut failed = report("B", "USART1", &loop1.lock(|l| *l));
        failed += report("B", "USART6", &loop6.lock(|l| *l));

        loop1.lock(|l| {
            l.hold = true;
            l.limit = TOTAL + FLOOD;
        });
        for _ in 0..200 {
            usart1_ready::spawn().ok();
            cortex_m::asm::delay(84_000);
        }
        let before = loop1.lock(|l| (l.read, l.gaps));
        loop1.lock(|l| l.hold = false);
        for _ in 0..20 {
            usart1_ready::spawn().ok();
            cortex_m::asm::delay(840_000);
        }
        let after = loop1.lock(|l| (l.sent, l.read, l.gaps));
        let health = port1.lock(|p| p.uart().health());
        defmt::info!(
            "B slow: sent {} read {} of {} flooded, gaps seen by consumer {}, FerroHAL buffer_overruns {}",
            after.0, after.1 - before.0, FLOOD, after.2 - before.1, health.buffer_overruns
        );
        failed += verdict("slow: loss reported to the consumer", after.2 > before.1);
        finish(failed);
    }
}
