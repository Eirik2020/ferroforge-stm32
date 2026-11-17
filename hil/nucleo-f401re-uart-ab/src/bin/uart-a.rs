//! PROTOTYPE, variant A: the consumer shares FerroHAL's `Uart` with
//! `uart_port`'s members under RTIC's lock. Links and routing as
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
        uart::{self, Uart, Wiring},
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::uart_port::{self, Events};
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

    /// USART1 over J1: PA9 into PA10, DMA2 stream 7 transmitting, stream 2
    /// receiving.
    type Usart1Wiring = Wiring<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    /// USART6 over J2: PA11 into PA12, DMA2 stream 6 transmitting, stream 1
    /// receiving.
    type Usart6Wiring = Wiring<USART6, Af8<'A', 11>, Af8<'A', 12>, Stream<2, 6>, Stream<2, 1>>;

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

    #[shared]
    struct Shared {
        usart1: Uart<Usart1Wiring>,
        usart6: Uart<Usart6Wiring>,
        loop1: Loop,
        loop6: Loop,
    }

    #[local]
    struct Local {}

    #[init(local = [
        tx1: [u8; TX_BUFFER] = [0; TX_BUFFER],
        rx1: [u8; RX_RING] = [0; RX_RING],
        tx6: [u8; TX_BUFFER] = [0; TX_BUFFER],
        rx6: [u8; RX_RING] = [0; RX_RING],
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
        let usart1 = uart_port::new(&mut rcc, w1, &FRAME, local.tx1, local.rx1)
            .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let usart6 = uart_port::new(&mut rcc, w6, &FRAME, local.tx6, local.rx6)
            .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));
        defmt::info!("variant A on the Nucleo-F401RE");
        checks::spawn().ok();
        let start = Loop { limit: TOTAL, ..Loop::default() };
        (Shared { usart1: usart1.uart, usart6: usart6.uart, loop1: start, loop6: start }, Local {})
    }

    #[group(from = ferroforge_stm32::uart_port, shared = [uart = usart1], spawn = [events = usart1_events])]
    mod usart1 {
        #[task(binds = USART1, priority = 3)]
        fn on_uart;
        #[task(binds = DMA2_STREAM2, priority = 3)]
        fn on_rx_dma;
        #[task(binds = DMA2_STREAM7, priority = 3)]
        fn on_tx_dma;
    }

    #[group(from = ferroforge_stm32::uart_port, shared = [uart = usart6], spawn = [events = usart6_events])]
    mod usart6 {
        #[task(binds = USART6, priority = 3)]
        fn on_uart;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn on_rx_dma;
        #[task(binds = DMA2_STREAM6, priority = 3)]
        fn on_tx_dma;
    }

    /// USART1's consumer: names the wiring through the shared `Uart`'s type,
    /// and holds the port's lock for its whole step.
    #[task(priority = 2, shared = [usart1, loop1])]
    async fn usart1_events(cx: usart1_events::Context, _events: Events) {
        let usart1_events::SharedResources { mut usart1, mut loop1, .. } = cx.shared;
        loop1.lock(|l| {
            let t = cycles();
            usart1.lock(|uart| l.step(uart));
            l.longest_step = l.longest_step.max(cycles().wrapping_sub(t));
        });
    }

    #[task(priority = 2, shared = [usart6, loop6])]
    async fn usart6_events(cx: usart6_events::Context, _events: Events) {
        let usart6_events::SharedResources { mut usart6, mut loop6, .. } = cx.shared;
        loop6.lock(|l| {
            let t = cycles();
            usart6.lock(|uart| l.step(uart));
            l.longest_step = l.longest_step.max(cycles().wrapping_sub(t));
        });
    }

    #[task(priority = 1, shared = [usart1, usart6, loop1, loop6])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut usart1, mut usart6, mut loop1, mut loop6, .. } = cx.shared;
        // Kick: the first step sends; arrivals drive the rest.
        loop1.lock(|l| usart1.lock(|u| l.step(u)));
        loop6.lock(|l| usart6.lock(|u| l.step(u)));
        wait_for(3_000, || loop1.lock(|l| l.read) >= TOTAL && loop6.lock(|l| l.read) >= TOTAL);
        cortex_m::asm::delay(8_400_000);
        let mut failed = report("A", "USART1", &loop1.lock(|l| *l));
        failed += report("A", "USART6", &loop6.lock(|l| *l));

        // Slow consumer on USART1: send FLOOD bytes without reading.
        loop1.lock(|l| {
            l.hold = true;
            l.limit = TOTAL + FLOOD;
        });
        for _ in 0..200 {
            loop1.lock(|l| usart1.lock(|u| l.step(u)));
            cortex_m::asm::delay(84_000);
        }
        let before = loop1.lock(|l| (l.read, l.gaps));
        loop1.lock(|l| {
            l.hold = false;
            usart1.lock(|u| l.step(u));
        });
        cortex_m::asm::delay(8_400_000);
        let after = loop1.lock(|l| (l.sent, l.read, l.gaps));
        let health = usart1.lock(|u| u.health());
        defmt::info!(
            "A slow: sent {} read {} of {} flooded, gaps seen by consumer {}, FerroHAL buffer_overruns {}",
            after.0, after.1 - before.0, FLOOD, after.2 - before.1, health.buffer_overruns
        );
        failed += verdict("slow: loss reported to the consumer", after.2 > before.1);
        finish(failed);
    }
}
