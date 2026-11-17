//! PROTOTYPE, variant B at mixed priorities: each port's transmit stream
//! member at 4, the other two at 3, so the lock among them is real.
//! Otherwise as `uart-b`.
//!
//! Variant B: `uart_queue_port`'s members own FerroHAL's `Uart`
//! and the port's queue ends; each consumer owns its two handles as locals
//! and names no UART, wiring or lock. Links and routing as
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
        uart::{self, Wiring},
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::uart_queue::{self, Handle, Hooks, Port, Queues};
    use ferrohal_stm32::device::Interrupt;
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

    /// Slots per queue (heapless keeps one free): 7 chunks or frames of up
    /// to 64 bytes each way.
    const SLOTS: usize = 8;

    type Port1 = Port<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    type Port6 = Port<USART6, Af7<'C', 6>, Af7<'C', 7>, Stream<1, 6>, Stream<1, 0>>;

    /// Which UART the first consumer takes, decided at boot (D12). Read
    /// through `black_box` so the compiler cannot fold it: the choice is a
    /// run-time value, as a stored setting would be.
    static SWAP: bool = true;

    #[shared]
    struct Shared {
        port1: Port1,
        port6: Port6,
        /// Tallies by UART: 0 is USART1, 1 is USART6.
        loops: [Loop; 2],
    }

    #[local]
    struct Local {
        first: (Handle, usize),
        second: (Handle, usize),
    }

    #[init(local = [
        tx1: [u8; TX_BUFFER] = [0; TX_BUFFER],
        rx1: [u8; RX_RING] = [0; RX_RING],
        tx6: [u8; TX_BUFFER] = [0; TX_BUFFER],
        rx6: [u8; RX_RING] = [0; RX_RING],
        q1: Queues<SLOTS, SLOTS> = Queues::new(),
        q6: Queues<SLOTS, SLOTS> = Queues::new(),
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
        // The firmware, not the consumer, names the vector the handle pends.
        let r1 = uart_queue::new(
            &mut rcc, w1, &FRAME, uart::Buffers { tx: local.tx1, rx: local.rx1 }, local.q1,
            Hooks { kick: || rtic::pend(Interrupt::DMA2_STREAM5), now: cycles },
        )
        .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let r6 = uart_queue::new(
            &mut rcc, w6, &FRAME, uart::Buffers { tx: local.tx6, rx: local.rx6 }, local.q6,
            Hooks { kick: || rtic::pend(Interrupt::DMA1_STREAM6), now: cycles },
        )
        .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));
        let start = Loop { limit: TOTAL, ..Loop::default() };
        let flood = Loop { flood: FLOOD, ..start };
        // Boot-time binding: both handles have one type, so a run-time
        // value picks which task gets which UART.
        let (first, second) = if core::hint::black_box(SWAP) {
            ((r6.handle, 1), (r1.handle, 0))
        } else {
            ((r1.handle, 0), (r6.handle, 1))
        };
        defmt::info!("variant B, mixed priorities, on the Nucleo-H753ZI; first consumer bound to {} at boot",
            if first.1 == 0 { "USART1" } else { "USART6" });
        consumer_first::spawn().ok();
        consumer_second::spawn().ok();
        checks::spawn().ok();
        (Shared { port1: r1.port, port6: r6.port, loops: [flood, start] }, Local { first, second })
    }

    #[group(from = ferroforge_stm32::uart_queue_port, shared = [port = port1])]
    mod usart1 {
        #[task(binds = USART1, priority = 3)]
        fn q_on_uart;
        #[task(binds = DMA2_STREAM4, priority = 3)]
        fn q_on_rx_dma;
        #[task(binds = DMA2_STREAM5, priority = 4)]
        fn q_on_tx_dma;
    }

    #[group(from = ferroforge_stm32::uart_queue_port, shared = [port = port6])]
    mod usart6 {
        #[task(binds = USART6, priority = 3)]
        fn q_on_uart;
        #[task(binds = DMA1_STREAM0, priority = 3)]
        fn q_on_rx_dma;
        #[task(binds = DMA1_STREAM6, priority = 4)]
        fn q_on_tx_dma;
    }

    /// The consumers: each owns a handle, takes no port lock, and does not
    /// know which UART it was given.
    #[task(priority = 2, local = [first], shared = [loops])]
    async fn consumer_first(mut cx: consumer_first::Context) {
        let (handle, index) = cx.local.first;
        consume(handle, *index, &mut cx.shared.loops).await;
    }

    #[task(priority = 2, local = [second], shared = [loops])]
    async fn consumer_second(mut cx: consumer_second::Context) {
        let (handle, index) = cx.local.second;
        consume(handle, *index, &mut cx.shared.loops).await;
    }

    async fn consume(handle: &mut Handle, index: usize, loops: &mut impl rtic::Mutex<T = [Loop; 2]>) -> ! {
        loop {
            let pause = loops.lock(|loops| {
                let l = &mut loops[index];
                let t = cycles();
                l.step(handle, cycles);
                l.longest_step = l.longest_step.max(cycles().wrapping_sub(t));
                l.wants_pause()
            });
            if pause {
                cortex_m::asm::delay(300 * MS);
                loops.lock(|loops| loops[index].release());
                continue;
            }
            handle.wait().await;
        }
    }

    #[task(priority = 1, shared = [port1, port6, loops])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut port1, mut port6, mut loops, .. } = cx.shared;
        wait_for(8_000, || {
            loops.lock(|l| l[0].flooding && !l[0].hold && l[0].read >= l[0].before.0 + 256 && l[1].read >= TOTAL)
        });
        cortex_m::asm::delay(100 * MS);
        let l = loops.lock(|l| *l);
        let mut failed = report("B-mixed", "USART1", &l[0]);
        failed += report("B-mixed", "USART6", &l[1]);
        let (h1, t1) = port1.lock(|p| (p.uart().health(), p.longest()));
        let t6 = port6.lock(|p| p.longest());
        defmt::info!(
            "B-mixed USART1 FerroHAL buffer_overruns {}; longest port handler: USART1 {} cycles, USART6 {} cycles",
            h1.buffer_overruns, t1, t6
        );
        finish(failed);
    }
}
