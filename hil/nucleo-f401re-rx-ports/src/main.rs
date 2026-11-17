//! The receive-only ports and the tick on the Nucleo-F401RE (MB1136): a
//! hand-written `ferroforge::app!` selecting `ferroforge_stm32::uart_rx_port`
//! twice, on USART1 and USART6, and `tick` on TIM4 and TIM3.
//!
//! A receive-only UART has nothing of its own to send with, so each line is
//! driven from its loop's other end as a GPIO pin, bit by bit against the
//! core's cycle counter, with interrupts masked for one frame at a time. The
//! UART receives as it would from a real sender: its DMA runs while the core
//! is masked, and the idle line that ends the frame is served as soon as the
//! mask lifts. The links are FerroHAL's F401 bench's J1 and J2 (UM1724 tables
//! 16 and 29, no solder bridge on either), the same as the uart_port loopback:
//!
//! | Link | Driven from | Into | Port |
//! | --- | --- | --- | --- |
//! | J1 | CN5-1 (D8, PA9), GPIO | CN9-3 (D2, PA10), USART1 RX | `uart_rx_port`, 100 000 baud 8E2 |
//! | J2 | CN10-14 (PA11), GPIO | CN10-12 (PA12), USART6 RX | `uart_rx_port`, 115 200 baud 8N1 |
//!
//! The two ports differ in framing as well as instance, so the port is shown
//! to carry a configuration with parity and two stop bits as well as the
//! default 8N1; neither port decodes anything. Routing (RM0368 table 9, alternate functions in DS10086 table 9; DMA2
//! request mapping in RM0368 table 28): USART1 RX on AF7 and DMA2 stream 2
//! channel 4; USART6 RX on AF8 and DMA2 stream 1 channel 5.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `USART1`, `DMA2_STREAM2` | `framed`'s members | 3 |
//! | `USART6`, `DMA2_STREAM1` | `plain`'s members | 3 |
//! | `TIM4`, `TIM3` | `fast`'s and `slow`'s `on_tick` | 3 |
//! | `SPI2` | RTIC, dispatching the firmware's readers | 2 |
//! | `SPI1` | RTIC, dispatching `checks` | 1 |
//!
//! Each check prints `PASS` or `FAIL` with the requirements it verifies in
//! brackets; the run passes when every check does.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI1, SPI2],

    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::uart_rx_port::Events;
    use ferrohal_stm32::{
        chip::{TIM3, TIM4, USART1, USART6},
        dma::Stream,
        gpio::{Alternate, Output, Pin, Pull},
        rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
        time::Hertz,
        timer::Tick,
        uart::{self, Parity, RxWiring, StopBits, Uart},
    };

    /// 84 MHz from HSI, the F401's ceiling (DS10086 section 3.13): 16 MHz /
    /// 16 * 336 = 336 MHz VCO, / 4 = 84 MHz. APB1 at 42 MHz, its limit, so
    /// TIM3 and TIM4 count at 84 MHz.
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

    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the F401's limits"),
    };
    /// Core cycles per millisecond.
    const MS: u32 = 84_000;

    const FRAMED: uart::Config = uart::Config::new(Hertz(100_000))
        .with_parity(Parity::Even)
        .with_stop_bits(StopBits::Two);
    const PLAIN: uart::Config = uart::Config::new(Hertz(115_200));
    const _: () = assert!(uart::check::<USART1>(&CLOCKS, &FRAMED).is_ok());
    const _: () = assert!(uart::check::<USART6>(&CLOCKS, &PLAIN).is_ok());

    const FAST: Hertz = Hertz(1_000);
    const SLOW: Hertz = Hertz(250);
    const _: () = assert!(Tick::<TIM4>::check(&CLOCKS, FAST).is_ok());
    const _: () = assert!(Tick::<TIM3>::check(&CLOCKS, SLOW).is_ok());

    /// Core cycles per bit on each line: 840 exactly, and 729 (0.02 % fast).
    const FRAMED_BIT: u32 = 84_000_000 / 100_000;
    const PLAIN_BIT: u32 = 84_000_000 / 115_200;

    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    type Af8<const P: char, const N: u8> = Pin<P, N, Alternate<8>>;
    type FramedUart = Uart<RxWiring<USART1, Af7<'A', 10>, Stream<2, 2>>>;
    type PlainUart = Uart<RxWiring<USART6, Af8<'A', 12>, Stream<2, 1>>>;

    const RX_RING: usize = 256;
    /// Bytes sent into each port, in frames of 1 to 32 bytes.
    const TOTAL: u32 = 1024;

    /// What a tick's consumer has seen.
    #[derive(Clone, Copy, Default)]
    pub struct Ticks {
        pub count: u32,
        pub gaps: u32,
        pub last: u32,
    }

    /// What a port's reader has seen; the bytes sent are a counter wrapping
    /// at 256.
    #[derive(Clone, Copy, Default)]
    pub struct Tally {
        pub bytes: u32,
        pub mismatches: u32,
        pub gaps: u32,
        pub wakes: u32,
        pub idles: u32,
        pub expected: u8,
    }

    #[shared]
    struct Shared {
        framed: FramedUart,
        plain: PlainUart,
        fast_timer: Tick<TIM4>,
        slow_timer: Tick<TIM3>,
        fast_ticks: Ticks,
        slow_ticks: Ticks,
        framed_tally: Tally,
        plain_tally: Tally,
    }

    #[local]
    struct Local {
        framed_line: Pin<'A', 9, Output>,
        plain_line: Pin<'A', 11, Output>,
    }

    #[init(local = [
        framed_ring: [u8; RX_RING] = [0; RX_RING],
        plain_ring: [u8; RX_RING] = [0; RX_RING],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpioa = p.GPIOA.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let ctrl = &mut gpioa.ctrl;
        // The senders idle at mark, high, before either receiver opens.
        let mut framed_line = gpioa.p9.into_push_pull_output(ctrl);
        framed_line.set_high();
        let mut plain_line = gpioa.p11.into_push_pull_output(ctrl);
        plain_line.set_high();
        let mut framed_rx = gpioa.p10.into_alternate::<7>(ctrl);
        framed_rx.set_pull(ctrl, Pull::Up);
        let mut plain_rx = gpioa.p12.into_alternate::<8>(ctrl);
        plain_rx.set_pull(ctrl, Pull::Up);

        let local = cx.local;
        let framed = ferroforge_stm32::uart_rx_port::new(
            &mut rcc,
            RxWiring { uart: p.USART1, rx: framed_rx, rx_dma: dma2.s2 },
            &FRAMED,
            local.framed_ring,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let plain = ferroforge_stm32::uart_rx_port::new(
            &mut rcc,
            RxWiring { uart: p.USART6, rx: plain_rx, rx_dma: dma2.s1 },
            &PLAIN,
            local.plain_ring,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));
        let fast = ferroforge_stm32::tick::new(&mut rcc, p.TIM4, FAST)
            .unwrap_or_else(|refused| defmt::panic!("TIM4: {}", refused.error()));
        let slow = ferroforge_stm32::tick::new(&mut rcc, p.TIM3, SLOW)
            .unwrap_or_else(|refused| defmt::panic!("TIM3: {}", refused.error()));

        defmt::info!("receive ports and ticks on the Nucleo-F401RE, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared {
                framed: framed.uart,
                plain: plain.uart,
                fast_timer: fast.timer,
                slow_timer: slow.timer,
                fast_ticks: Ticks::default(),
                slow_ticks: Ticks::default(),
                framed_tally: Tally::default(),
                plain_tally: Tally::default(),
            },
            Local { framed_line, plain_line },
        )
    }

    #[group(from = ferroforge_stm32::uart_rx_port, shared = [uart = framed], spawn = [events = framed_events])]
    mod framed {
        #[task(binds = USART1, priority = 3)]
        fn on_uart;
        #[task(binds = DMA2_STREAM2, priority = 3)]
        fn on_rx_dma;
    }

    #[group(from = ferroforge_stm32::uart_rx_port, shared = [uart = plain], spawn = [events = plain_events])]
    mod plain {
        #[task(binds = USART6, priority = 3)]
        fn on_uart;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn on_rx_dma;
    }

    #[group(from = ferroforge_stm32::tick, shared = [timer = fast_timer], spawn = [tick = fast_tick])]
    mod fast {
        #[task(binds = TIM4, priority = 3)]
        fn on_tick;
    }

    #[group(from = ferroforge_stm32::tick, shared = [timer = slow_timer], spawn = [tick = slow_tick])]
    mod slow {
        #[task(binds = TIM3, priority = 3)]
        fn on_tick;
    }

    /// The 8E2 port's reader: drains the ring, checking every byte.
    #[task(priority = 2, shared = [framed, framed_tally])]
    async fn framed_events(cx: framed_events::Context, events: Events) {
        let framed_events::SharedResources { mut framed, mut framed_tally, .. } = cx.shared;
        framed_tally.lock(|tally| framed.lock(|uart| drain(uart, tally, events)));
    }

    /// The 8N1 port's reader: drains the ring, checking every byte.
    #[task(priority = 2, shared = [plain, plain_tally])]
    async fn plain_events(cx: plain_events::Context, events: Events) {
        let plain_events::SharedResources { mut plain, mut plain_tally, .. } =
            cx.shared;
        plain_tally.lock(|tally| plain.lock(|uart| drain(uart, tally, events)));
    }

    #[task(priority = 2, shared = [fast_ticks])]
    async fn fast_tick(mut cx: fast_tick::Context, sequence: u32) {
        cx.shared.fast_ticks.lock(|ticks| count(ticks, sequence));
    }

    #[task(priority = 2, shared = [slow_ticks])]
    async fn slow_tick(mut cx: slow_tick::Context, sequence: u32) {
        cx.shared.slow_ticks.lock(|ticks| count(ticks, sequence));
    }

    /// Ticks for a measured while, then drives both lines and judges what
    /// each port made of them.
    #[task(
        priority = 1,
        shared = [framed, plain, fast_timer, slow_timer, fast_ticks, slow_ticks, framed_tally, plain_tally],
        local = [framed_line, plain_line],
    )]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources {
            mut framed,
            mut plain,
            mut fast_timer,
            mut slow_timer,
            mut fast_ticks,
            mut slow_ticks,
            mut framed_tally,
            mut plain_tally,
            ..
        } = cx.shared;
        let checks::LocalResources { framed_line, plain_line, .. } = cx.local;
        let mut failed = 0u32;

        let bound = framed.lock(|uart| uart.vectors().verify()).is_ok()
            && plain.lock(|uart| uart.vectors().verify()).is_ok()
            && fast_timer.lock(|timer| timer.vectors().verify()).is_ok()
            && slow_timer.lock(|timer| timer.vectors().verify()).is_ok();
        failed += verdict("every member's vector is enabled [PORT-101, PORT-111]", bound);

        // Ticks: 200 ms by the cycle counter, from a clean count.
        let (fast_start, slow_start) =
            (fast_ticks.lock(|t| t.count), slow_ticks.lock(|t| t.count));
        wait(200 * MS);
        let fast = fast_ticks.lock(|t| *t);
        let slow = slow_ticks.lock(|t| *t);
        let fast_count = fast.count - fast_start;
        let slow_count = slow.count - slow_start;
        defmt::info!(
            "ticks in 200 ms: TIM4 {} ({} gaps, last {}), TIM3 {} ({} gaps, last {})",
            fast_count, fast.gaps, fast.last, slow_count, slow.gaps, slow.last
        );
        failed += verdict(
            "TIM4 at 1 kHz ticked 200 times, numbered without a gap [PORT-111]",
            (199..=201).contains(&fast_count) && fast.gaps == 0,
        );
        failed += verdict(
            "TIM3 at 250 Hz ticked 50 times, numbered without a gap [PORT-111]",
            (49..=51).contains(&slow_count) && slow.gaps == 0,
        );
        // The lines below are driven with interrupts masked a frame at a time;
        // the ticks would only queue behind them.
        fast_timer.lock(|timer| timer.pause());
        slow_timer.lock(|timer| timer.pause());

        // Each port: a counter in frames of 1 to 32 bytes, each followed by
        // three characters of idle line.
        stream(plain_line, PLAIN_BIT, false, 1);
        stream(framed_line, FRAMED_BIT, true, 2);
        wait(10 * MS);
        for (name, tally, health) in [
            ("USART6, 115 200 8N1", plain_tally.lock(|t| *t), plain.lock(|u| u.health())),
            ("USART1, 100 000 8E2", framed_tally.lock(|t| *t), framed.lock(|u| u.health())),
        ] {
            defmt::info!(
                "{}: {} bytes read, {} wrong, {} gaps, {} wakes ({} idle); line {}",
                name, tally.bytes, tally.mismatches, tally.gaps, tally.wakes, tally.idles, health
            );
        }
        let plain = plain_tally.lock(|t| *t);
        let framed = framed_tally.lock(|t| *t);
        failed += verdict(
            "every 8N1 byte came back, in order, once [PORT-101]",
            intact(&plain),
        );
        failed += verdict(
            "every 8E2 byte came back, in order, once [PORT-101]",
            intact(&framed),
        );
        failed += verdict(
            "each port woke the firmware's reader on idle lines [PORT-101]",
            plain.wakes > 0 && plain.idles > 0 && framed.wakes > 0 && framed.idles > 0,
        );

        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit ends
        // that, so give it the last lines first (about 200 ms).
        wait(200 * MS);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }

    /// Sends `TOTAL` bytes of a counter on `line`, in frames of 1 to 32
    /// bytes, each followed by three characters of idle line.
    fn stream<const N: u8>(line: &mut Pin<'A', N, Output>, bit: u32, parity: bool, stop: u32) {
        let mut next = 0u8;
        let mut sent = 0u32;
        let mut length = 1u32;
        while sent < TOTAL {
            let len = length.min(TOTAL - sent);
            let mut frame = [0u8; 32];
            for byte in &mut frame[..len as usize] {
                *byte = next;
                next = next.wrapping_add(1);
            }
            send(line, &frame[..len as usize], bit, parity, stop);
            sent += len;
            length = length % 32 + 1;
            wait(36 * bit);
        }
    }

    /// Every byte sent came back once, in order.
    fn intact(tally: &Tally) -> bool {
        tally.bytes == TOTAL && tally.mismatches == 0 && tally.gaps == 0
    }

    /// Spins until `cycles` core cycles have passed. Interrupts still run.
    fn wait(cycles: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < cycles {}
    }

    /// Sends `bytes` on `line` as a UART would, `bit` core cycles a bit:
    /// a start bit, eight data bits least significant first, an even parity
    /// bit when `parity`, and `stop` stop bits. Every edge is timed from the
    /// first, so no error builds up, and interrupts are masked for the frame.
    fn send<const N: u8>(line: &mut Pin<'A', N, Output>, bytes: &[u8], bit: u32, parity: bool, stop: u32) {
        cortex_m::interrupt::free(|_| {
            let mut edge = DWT::cycle_count();
            let mut level = |line: &mut Pin<'A', N, Output>, high: bool| {
                if high {
                    line.set_high();
                } else {
                    line.set_low();
                }
                edge = edge.wrapping_add(bit);
                while (DWT::cycle_count().wrapping_sub(edge) as i32) < 0 {}
            };
            for &byte in bytes {
                level(line, false);
                for index in 0..8 {
                    level(line, byte >> index & 1 == 1);
                }
                if parity {
                    level(line, byte.count_ones() % 2 == 1);
                }
                for _ in 0..stop {
                    level(line, true);
                }
            }
        });
    }

    /// Counts a tick and any gap in its numbering.
    fn count(ticks: &mut Ticks, sequence: u32) {
        if ticks.count > 0 && sequence != ticks.last.wrapping_add(1) {
            ticks.gaps += 1;
        }
        ticks.count += 1;
        ticks.last = sequence;
    }

    /// Reads everything there is, checking each byte against the counter.
    fn drain<W: uart::Parts>(uart: &mut Uart<W>, tally: &mut Tally, events: Events) {
        tally.wakes += 1;
        tally.idles += u32::from(events.idle);
        let mut buf = [0u8; 64];
        // Bounded: a ring holds at most RX_RING bytes.
        for _ in 0..RX_RING {
            match uart.try_read(&mut buf) {
                Ok(0) => break,
                Ok(len) => {
                    for &byte in &buf[..len] {
                        if byte != tally.expected {
                            tally.mismatches += 1;
                        }
                        tally.expected = byte.wrapping_add(1);
                    }
                    tally.bytes += len as u32;
                }
                Err(error) => {
                    defmt::warn!("read: {}", error);
                    tally.gaps += 1;
                }
            }
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
}
