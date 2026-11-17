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
    use ferroforge_stm32::uart_queue::{self, Handle, Port, Queues};
    use nucleo_f401re_uart_ab::{Loop, TAIL, report, report_split};

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

    /// Slots per queue (heapless keeps one free): 7 chunks or frames of up
    /// to 64 bytes each way.
    const SLOTS: usize = 8;

    type Port1 = Port<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    type Port6 = Port<USART6, Af8<'A', 11>, Af8<'A', 12>, Stream<2, 6>, Stream<2, 1>>;

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
        // `usartN::kick` is generated from the group's `pends`: the vector is
        // written once, on the member below.
        let r1 = uart_queue::new(
            &mut rcc, w1, &FRAME, local.tx1, local.rx1, local.q1,
            usart1::kick, cycles,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let r6 = uart_queue::new(
            &mut rcc, w6, &FRAME, local.tx6, local.rx6, local.q6,
            usart6::kick, cycles,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));
        // USART6 runs the tail (short, 64-byte and long frames); USART1
        // floods.
        let start = Loop { tail: true, ..Loop::new(TOTAL) };
        let flood = Loop { flood: FLOOD, ..Loop::new(TOTAL) };
        // Boot-time binding: both handles have one type, so a run-time
        // value picks which task gets which UART.
        let (first, second) = if core::hint::black_box(SWAP) {
            ((r6.handle, 1), (r1.handle, 0))
        } else {
            ((r1.handle, 0), (r6.handle, 1))
        };
        defmt::info!("variant B on the Nucleo-F401RE; first consumer bound to {} at boot",
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
        #[task(binds = DMA2_STREAM2, priority = 3)]
        fn q_on_rx_dma;
        #[task(binds = DMA2_STREAM7, priority = 3)]
        fn q_on_tx_dma;
    }

    #[group(from = ferroforge_stm32::uart_queue_port, shared = [port = port6])]
    mod usart6 {
        #[task(binds = USART6, priority = 3)]
        fn q_on_uart;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn q_on_rx_dma;
        #[task(binds = DMA2_STREAM6, priority = 3)]
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
                cortex_m::asm::delay(25_200_000);
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
            loops.lock(|l| l[0].flooding && !l[0].hold && l[0].read >= l[0].before.0 + 256
                && usize::from(l[1].phase) == TAIL.len() && l[1].read >= l[1].limit)
        });
        cortex_m::asm::delay(8_400_000);
        let l = loops.lock(|l| *l);
        let mut failed = report("B", "USART1", &l[0], false);
        failed += report("B", "USART6", &l[1], false);
        let (h1, s1) = port1.lock(|p| (p.uart().health(), p.split()));
        let s6 = port6.lock(|p| p.split());
        defmt::info!("B USART1 FerroHAL buffer_overruns {}", h1.buffer_overruns);
        report_split("B", "USART1", &s1);
        report_split("B", "USART6", &s6);
        finish(failed);
    }
}
