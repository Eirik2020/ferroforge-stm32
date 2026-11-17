//! PROTOTYPE, variant A: the consumer shares FerroHAL's `Uart` with
//! `uart_port`'s members under RTIC's lock. Links and routing as
//! `nucleo-h753zi-uart-loopback` (Z3, Z2; DS12117 table 9, RM0433 section 17.3).

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
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
        },
        time::Hertz,
        uart::{self, Uart, Wiring},
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::uart_port::{self, Events};
    use nucleo_h753zi_uart_ab::Loop;

    /// 400 MHz from HSI, as FerroHAL's H753 bench runs: 64 MHz / 4 = 16 MHz
    /// reference, * 50 = 800 MHz VCO, / 2 = 400 MHz core; AXI and AHB at
    /// 200 MHz, every APB at 100 MHz (DS12117 table 23, VOS1).
    const CLOCK_CONFIG: Config = Config {
        hse: None,
        pll1: Some(Pll {
            source: PllSource::Hsi,
            prediv: 4,
            mul: 50,
            divp: Some(2),
            divq: Some(8),
            divr: None,
        }),
        sys: Sysclk::Pll1P,
        cpu_pre: AhbPrescaler::Div1,
        ahb_pre: AhbPrescaler::Div2,
        apb1_pre: ApbPrescaler::Div2,
        apb2_pre: ApbPrescaler::Div2,
        apb3_pre: ApbPrescaler::Div2,
        apb4_pre: ApbPrescaler::Div2,
        supply: Supply::Ldo,
        voltage_scale: VoltageScale::Scale1,
    };

    /// Checked while compiling, as FerroHAL's benches check theirs.
    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the H753's limits"),
    };

    /// CPU cycles in a millisecond, for the busy waits.
    const MS: u32 = 400_000;

    const FRAME: uart::Config = uart::Config::new(Hertz(115_200));
    const _: () = assert!(uart::check::<USART1>(&CLOCKS, &FRAME).is_ok());
    const _: () = assert!(uart::check::<USART6>(&CLOCKS, &FRAME).is_ok());

    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;

    /// USART1 over Z3: PB6 into PB7, DMA2 stream 5 transmitting, stream 4
    /// receiving.
    type Usart1Wiring = Wiring<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    /// USART6 over Z2: PC6 into PC7, DMA1 stream 6 transmitting, stream 0
    /// receiving.
    type Usart6Wiring = Wiring<USART6, Af7<'C', 6>, Af7<'C', 7>, Stream<1, 6>, Stream<1, 0>>;

    const RX_RING: usize = 256;
    const TX_BUFFER: usize = 256;

    /// Bytes each port sends in the main run (about 1.4 s at 115200).
    const TOTAL: u32 = 16_384;
    /// Extra bytes sent while the consumer holds off reading.
    const FLOOD: u32 = 2_048;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// Waits until `done` or about `ms` milliseconds at the core clock.
    fn wait_for(ms: u32, mut done: impl FnMut() -> bool) {
        for _ in 0..ms {
            if done() {
                return;
            }
            cortex_m::asm::delay(MS);
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
            "{} {}: sent {} read {} wrong {} chunks {} idle-ended {} gaps {} wakes {} all-sent {} longest step {} cycles oldest chunk {} cycles",
            variant, name, l.sent, l.read, l.wrong, l.chunks, l.idles, l.gaps, l.wakes, l.all_sent, l.longest_step, l.oldest
        );
        let mut failed = 0;
        if l.flood == 0 {
            failed += verdict("main run: every byte back, in order", l.read == TOTAL && l.wrong == 0 && l.gaps == 0);
        } else {
            let (read0, gaps0) = l.before;
            defmt::info!(
                "{} {} slow: {} of {} flooded bytes read, {} gaps reported to the consumer",
                variant, name, l.read - read0, l.flood, l.gaps - gaps0
            );
            failed += verdict("main run: every byte back before the flood", read0 >= TOTAL);
            failed += verdict("slow: loss reported to the consumer as a gap", l.gaps > gaps0);
        }
        failed += verdict("frame ends reach the consumer (idle chunks)", l.idles > 0);
        failed += verdict("the consumer learned everything was sent", l.all_sent > 0);
        failed
    }

    fn finish(failed: u32) -> ! {
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        cortex_m::asm::delay(200 * MS);
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
        let mut gpiob = p.GPIOB.split(&mut rcc);
        let mut gpioc = p.GPIOC.split(&mut rcc);
        let dma1 = p.DMA1.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let mut tx1 = gpiob.p6.into_alternate::<7>(&mut gpiob.ctrl);
        tx1.set_speed(&mut gpiob.ctrl, Speed::Medium);
        let mut rx1 = gpiob.p7.into_alternate::<7>(&mut gpiob.ctrl);
        rx1.set_pull(&mut gpiob.ctrl, Pull::Up);
        let mut tx6 = gpioc.p6.into_alternate::<7>(&mut gpioc.ctrl);
        tx6.set_speed(&mut gpioc.ctrl, Speed::Medium);
        let mut rx6 = gpioc.p7.into_alternate::<7>(&mut gpioc.ctrl);
        rx6.set_pull(&mut gpioc.ctrl, Pull::Up);

        let w1 = Wiring { uart: p.USART1, tx: tx1, rx: rx1, tx_dma: dma2.s5, rx_dma: dma2.s4 };
        let w6 = Wiring { uart: p.USART6, tx: tx6, rx: rx6, tx_dma: dma1.s6, rx_dma: dma1.s0 };
        let local = cx.local;
        let usart1 = uart_port::new(&mut rcc, w1, &FRAME, local.tx1, local.rx1)
            .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let usart6 = uart_port::new(&mut rcc, w6, &FRAME, local.tx6, local.rx6)
            .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));
        defmt::info!("variant A on the Nucleo-H753ZI");
        checks::spawn().ok();
        let start = Loop::new(TOTAL);
        let flood = Loop { flood: FLOOD, ..start };
        (Shared { usart1: usart1.uart, usart6: usart6.uart, loop1: flood, loop6: start }, Local {})
    }

    #[group(from = ferroforge_stm32::uart_port, shared = [uart = usart1], spawn = [events = usart1_events])]
    mod usart1 {
        #[task(binds = USART1, priority = 3)]
        fn on_uart;
        #[task(binds = DMA2_STREAM4, priority = 3)]
        fn on_rx_dma;
        #[task(binds = DMA2_STREAM5, priority = 3)]
        fn on_tx_dma;
    }

    #[group(from = ferroforge_stm32::uart_port, shared = [uart = usart6], spawn = [events = usart6_events])]
    mod usart6 {
        #[task(binds = USART6, priority = 3)]
        fn on_uart;
        #[task(binds = DMA1_STREAM0, priority = 3)]
        fn on_rx_dma;
        #[task(binds = DMA1_STREAM6, priority = 3)]
        fn on_tx_dma;
    }

    /// USART1's consumer: names the wiring through the shared `Uart`'s type,
    /// and holds the port's lock for its whole step.
    #[task(priority = 2, shared = [usart1, loop1])]
    async fn usart1_events(cx: usart1_events::Context, _events: Events) {
        let usart1_events::SharedResources { mut usart1, mut loop1, .. } = cx.shared;
        consume(&mut usart1, &mut loop1);
    }

    #[task(priority = 2, shared = [usart6, loop6])]
    async fn usart6_events(cx: usart6_events::Context, _events: Events) {
        let usart6_events::SharedResources { mut usart6, mut loop6, .. } = cx.shared;
        consume(&mut usart6, &mut loop6);
    }

    fn consume<P: nucleo_h753zi_uart_ab::Port>(
        port: &mut impl rtic::Mutex<T = P>,
        tally: &mut impl rtic::Mutex<T = Loop>,
    ) {
        let pause = tally.lock(|l| {
            let t = cycles();
            port.lock(|p| l.step(p, cycles));
            l.longest_step = l.longest_step.max(cycles().wrapping_sub(t));
            l.wants_pause()
        });
        if pause {
            // The slow consumer: busy elsewhere, holding no lock.
            cortex_m::asm::delay(300 * MS);
            tally.lock(|l| {
                l.release();
                port.lock(|p| l.step(p, cycles));
            });
        }
    }

    #[task(priority = 1, shared = [usart1, usart6, loop1, loop6])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut usart1, mut usart6, mut loop1, mut loop6, .. } = cx.shared;
        // Kick: the first step sends; the port's wakes drive the rest.
        consume(&mut usart1, &mut loop1);
        consume(&mut usart6, &mut loop6);
        wait_for(8_000, || {
            loop1.lock(|l| l.flooding && !l.hold && l.read >= l.before.0 + 256)
                && loop6.lock(|l| l.read) >= TOTAL
        });
        cortex_m::asm::delay(100 * MS);
        let mut failed = report("A", "USART1", &loop1.lock(|l| *l));
        failed += report("A", "USART6", &loop6.lock(|l| *l));
        let health = usart1.lock(|u| u.health());
        defmt::info!("A USART1 FerroHAL buffer_overruns {}", health.buffer_overruns);
        finish(failed);
    }
}
