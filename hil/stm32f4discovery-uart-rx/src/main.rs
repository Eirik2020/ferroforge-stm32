//! The receive-only UART port on the STM32F4 Discovery (MB997, STM32F407VGT6),
//! die 0x413: the board that proves `ferroforge_stm32::uart_rx_port`. A
//! hand-written `ferroforge::app!` selecting it twice, each handle owned by a
//! consumer task, which task gets which port chosen at boot.
//!
//! A receive-only UART has nothing of its own to send with, so each line is
//! driven from its link's other end as a GPIO pin, bit by bit against the
//! core's cycle counter, with interrupts masked for one frame at a time. The
//! UART's DMA runs while the core is masked, and the idle line that ends the
//! frame is served once the mask lifts. The links are FerroHAL's Discovery
//! bench's D1 and D2 (UM1472 table 7):
//!
//! | Link | Driven from | Into | Port |
//! | --- | --- | --- | --- |
//! | D1 | P2-37 (PC10), GPIO | P2-38 (PC11), USART3 RX | 100 000 baud 8E2 |
//! | D2 | P2-35 (PC12), GPIO | P2-34 (PD2), UART5 RX | 115 200 baud 8N1 |
//!
//! Routing: USART3 RX on PC11 is AF7, UART5 RX on PD2 AF8 (DS8626 table 9);
//! DMA1 channel 4, USART3 RX on stream 1 and UART5 RX on stream 0 (RM0090
//! table 43). UART5 RX on PD2 and DMA1 stream 0 is the receive-only route
//! flight controller targets on this die assign to ESC telemetry.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `USART3`, `DMA1_STREAM1` | `usart3`'s members | 3 |
//! | `UART5`, `DMA1_STREAM0` | `uart5`'s members | 3 |
//! | `SysTick` | the monotonic, at 1 kHz | RTIC's |
//! | `SPI2` | RTIC, dispatching the two consumers | 2 |
//! | `SPI1` | RTIC, dispatching `checks` | 1 |
//!
//! The run: 1024 bytes of a counter into each port, in frames of 1 to 32
//! bytes. Then, with UART5's consumer holding every buffer, 1024 more into
//! UART5, more than its pool and ring hold; then the hold ends and 64 more
//! follow. Each check prints `PASS` or `FAIL` with the requirements it
//! verifies in brackets; the run passes when every check does.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI1, SPI2],

    use cortex_m::peripheral::DWT;
    use rtic_monotonics::systick::prelude::*;

    // The monotonic the port's members stamp each service with, at 1 kHz.
    // Interrupts masked while a frame is driven hold its tick back, so the
    // count runs slow but never backwards.
    systick_monotonic!(Mono, 1000);

    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::{
        uart_rx::{RxHandle, RxMemory, RxPort},
        uart_rx_port,
    };
    use ferrohal_stm32::{
        chip::{UART5, USART3},
        dma::Stream,
        gpio::{Alternate, Output, Pin, Pull},
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
            Sysclk,
        },
        time::Hertz,
        uart::{self, ChunkEnd, Parity, RxWiring, StopBits},
    };

    /// 168 MHz, the die's ceiling (DS8626 section 3.13), from the ST-LINK's
    /// 8 MHz MCO on PH0 in bypass (UM1472 section 6.11.1): 8 MHz / 8 * 336 =
    /// 336 MHz VCO, / 2 = 168 MHz. APB1 at 42 MHz, its limit, for both UARTs.
    const CLOCK_CONFIG: Config = Config {
        hse: Some(Hse { freq: Hertz::mhz(8), mode: HseMode::Bypass }),
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

    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the F407's limits"),
    };
    const CPU_HZ: u32 = 168_000_000;
    /// Core cycles per millisecond.
    const MS: u32 = CPU_HZ / 1000;

    const FRAMED: uart::Config = uart::Config::new(Hertz(100_000))
        .with_parity(Parity::Even)
        .with_stop_bits(StopBits::Two);
    const PLAIN: uart::Config = uart::Config::new(Hertz(115_200));
    const _: () = assert!(uart::check::<USART3>(&CLOCKS, &FRAMED).is_ok());
    const _: () = assert!(uart::check::<UART5>(&CLOCKS, &PLAIN).is_ok());

    /// Core cycles per bit on each line: 1680 exactly, and 1458 (0.02 % fast).
    const FRAMED_BIT: u32 = CPU_HZ / 100_000;
    const PLAIN_BIT: u32 = CPU_HZ / 115_200;

    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    type Af8<const P: char, const N: u8> = Pin<P, N, Alternate<8>>;
    type FramedPort = RxPort<USART3, Af7<'C', 11>, Stream<1, 1>>;
    type PlainPort = RxPort<UART5, Af8<'D', 2>, Stream<1, 0>>;

    /// The ring, and a pool of four 32-byte buffers each, at most half the
    /// ring. Five queue slots hold the four (one stays free).
    const RX_RING: usize = 128;
    const POOL: usize = 4;
    const SLOTS: usize = 5;
    const BUF: usize = 32;
    /// Bytes sent into each port, in frames of 1 to 32 bytes.
    const TOTAL: u32 = 1024;
    /// Bytes sent into UART5 while its consumer holds every buffer, and after.
    const FLOOD: u32 = 1024;
    const AFTER: u32 = 64;

    /// Which port the first consumer takes, decided at boot. Read through
    /// `black_box`, so it is a run-time value, as a stored setting would be.
    static SWAP: bool = true;

    /// What one consumer has seen; the bytes sent are a counter wrapping at
    /// 256.
    #[derive(Clone, Copy)]
    pub struct Tally {
        pub read: u32,
        pub wrong: u32,
        pub chunks: u32,
        pub idles: u32,
        pub gaps: u32,
        pub wakes: u32,
        pub expected: u8,
        /// The next byte follows a gap: take it as the counter's value.
        pub resync: bool,
        pub generation: Option<u32>,
        /// Chunks of another generation before the first gap, and whether one
        /// after it advanced.
        pub moved: u32,
        pub advanced: bool,
        pub first_us: u64,
        pub last_us: u64,
        pub backwards: u32,
        pub future: u32,
        /// Hold every buffer: read nothing.
        pub hold: bool,
        /// `read` when the first gap was taken, `chunks` when the hold began,
        /// and chunks taken between the two: what the pool kept.
        pub read_at_gap: Option<u32>,
        pub chunks_at_hold: u32,
        pub kept: Option<u32>,
        pub lost: u32,
    }

    impl Tally {
        const fn new() -> Self {
            Self {
                read: 0, wrong: 0, chunks: 0, idles: 0, gaps: 0, wakes: 0, expected: 0,
                resync: false, generation: None, moved: 0, advanced: false,
                first_us: u64::MAX, last_us: 0, backwards: 0, future: 0, hold: false,
                read_at_gap: None, chunks_at_hold: 0, kept: None, lost: 0,
            }
        }

        /// Reads every chunk there is, in place, unless holding.
        fn step(&mut self, handle: &mut RxHandle) {
            self.wakes += 1;
            self.lost = handle.lost();
            if self.hold {
                return;
            }
            for _ in 0..64 {
                let Some(got) = handle.next_chunk() else { break };
                let now = now_us();
                self.future += u32::from(got.at_us > now);
                self.backwards += u32::from(got.at_us < self.last_us);
                self.first_us = self.first_us.min(got.at_us);
                self.last_us = self.last_us.max(got.at_us);
                let gap = matches!(got.chunk.end, ChunkEnd::Gap(_));
                match self.generation {
                    None => self.generation = Some(got.chunk.generation),
                    Some(g) if self.gaps == 0 && !gap && got.chunk.generation != g => self.moved += 1,
                    Some(g) if self.gaps > 0 && got.chunk.generation > g => self.advanced = true,
                    Some(_) => {}
                }
                for &byte in got.bytes() {
                    if self.resync {
                        self.resync = false;
                    } else if byte != self.expected {
                        self.wrong += 1;
                    }
                    self.expected = byte.wrapping_add(1);
                }
                self.read += got.bytes().len() as u32;
                self.chunks += 1;
                match got.chunk.end {
                    ChunkEnd::Idle => self.idles += 1,
                    ChunkEnd::Full => {}
                    ChunkEnd::Gap(_) => {
                        self.gaps += 1;
                        self.resync = true;
                        self.read_at_gap.get_or_insert(self.read);
                        let kept = self.chunks - self.chunks_at_hold;
                        self.kept.get_or_insert(kept);
                    }
                }
                // Dropping `got` returns the buffer to the port.
            }
        }
    }

    #[shared]
    struct Shared {
        framed: FramedPort,
        plain: PlainPort,
        /// Tallies by port: 0 is USART3 (8E2), 1 is UART5 (8N1).
        tallies: [Tally; 2],
    }

    #[local]
    struct Local {
        first: (RxHandle, usize),
        second: (RxHandle, usize),
        framed_line: Pin<'C', 10, Output>,
        plain_line: Pin<'C', 12, Output>,
    }

    #[init(local = [
        framed_ring: [u8; RX_RING] = [0; RX_RING],
        plain_ring: [u8; RX_RING] = [0; RX_RING],
        framed_memory: RxMemory<SLOTS> = RxMemory::new(),
        plain_memory: RxMemory<SLOTS> = RxMemory::new(),
        framed_pool: [[u8; BUF]; POOL] = [[0; BUF]; POOL],
        plain_pool: [[u8; BUF]; POOL] = [[0; BUF]; POOL],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        Mono::start(core.SYST, CPU_HZ);
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpioc = p.GPIOC.split(&mut rcc);
        let mut gpiod = p.GPIOD.split(&mut rcc);
        let dma1 = p.DMA1.split(&mut rcc);
        // The CS43L22 on PC10 and PC12 held in reset, powered down.
        let mut dac_reset = gpiod.p4.into_push_pull_output(&mut gpiod.ctrl);
        dac_reset.set_low();
        // The senders idle at mark, high, before either receiver opens.
        let mut framed_line = gpioc.p10.into_push_pull_output(&mut gpioc.ctrl);
        framed_line.set_high();
        let mut plain_line = gpioc.p12.into_push_pull_output(&mut gpioc.ctrl);
        plain_line.set_high();
        let mut framed_rx = gpioc.p11.into_alternate::<7>(&mut gpioc.ctrl);
        framed_rx.set_pull(&mut gpioc.ctrl, Pull::Up);
        let mut plain_rx = gpiod.p2.into_alternate::<8>(&mut gpiod.ctrl);
        plain_rx.set_pull(&mut gpiod.ctrl, Pull::Up);

        let local = cx.local;
        let mut framed_pool = local.framed_pool.iter_mut().map(|b| -> &'static mut [u8] { b });
        let Ok(framed) = uart_rx_port::new(
            &mut rcc,
            RxWiring { uart: p.USART3, rx: framed_rx, rx_dma: dma1.s1 },
            &FRAMED, local.framed_ring, local.framed_memory, &mut framed_pool, usart3::kick,
        ) else {
            defmt::panic!("USART3 refused");
        };
        let mut plain_pool = local.plain_pool.iter_mut().map(|b| -> &'static mut [u8] { b });
        let Ok(plain) = uart_rx_port::new(
            &mut rcc,
            RxWiring { uart: p.UART5, rx: plain_rx, rx_dma: dma1.s0 },
            &PLAIN, local.plain_ring, local.plain_memory, &mut plain_pool, uart5::kick,
        ) else {
            defmt::panic!("UART5 refused");
        };
        let (first, second) = if core::hint::black_box(SWAP) {
            ((plain.handle, 1), (framed.handle, 0))
        } else {
            ((framed.handle, 0), (plain.handle, 1))
        };
        defmt::info!("uart_rx_port on the STM32F4 Discovery, clocks {}; first consumer bound to {} at boot",
            CLOCKS, if first.1 == 0 { "USART3" } else { "UART5" });
        consumer_first::spawn().ok();
        consumer_second::spawn().ok();
        checks::spawn().ok();
        (
            Shared { framed: framed.port, plain: plain.port, tallies: [Tally::new(), Tally::new()] },
            Local { first, second, framed_line, plain_line },
        )
    }

    #[group(from = ferroforge_stm32::uart_rx_port, shared = [port = framed])]
    mod usart3 {
        #[task(binds = USART3, priority = 3)]
        fn on_rx_uart;
        #[task(binds = DMA1_STREAM1, priority = 3)]
        fn on_rx_stream;
    }

    #[group(from = ferroforge_stm32::uart_rx_port, shared = [port = plain])]
    mod uart5 {
        #[task(binds = UART5, priority = 3)]
        fn on_rx_uart;
        #[task(binds = DMA1_STREAM0, priority = 3)]
        fn on_rx_stream;
    }

    /// The consumers: each owns a handle, takes no port lock, and does not
    /// know which UART it was given.
    #[task(priority = 2, local = [first], shared = [tallies])]
    async fn consumer_first(mut cx: consumer_first::Context) {
        let (handle, index) = cx.local.first;
        consume(handle, *index, &mut cx.shared.tallies).await;
    }

    #[task(priority = 2, local = [second], shared = [tallies])]
    async fn consumer_second(mut cx: consumer_second::Context) {
        let (handle, index) = cx.local.second;
        consume(handle, *index, &mut cx.shared.tallies).await;
    }

    async fn consume(handle: &mut RxHandle, index: usize, tallies: &mut impl rtic::Mutex<T = [Tally; 2]>) -> ! {
        loop {
            let hold = tallies.lock(|t| {
                t[index].step(handle);
                t[index].hold
            });
            if hold {
                // Holding reads nothing, so no chunk wakes it again: look
                // each millisecond for the hold's end.
                Mono::delay(1.millis()).await;
            } else {
                handle.wait().await;
            }
        }
    }

    /// Drives both lines and judges what each port's consumer made of them.
    #[task(priority = 1, shared = [framed, plain, tallies], local = [framed_line, plain_line])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut framed, mut plain, mut tallies, .. } = cx.shared;
        let checks::LocalResources { framed_line, plain_line, .. } = cx.local;
        let bound = framed.lock(|p| p.uart().vectors().verify()).is_ok()
            && plain.lock(|p| p.uart().vectors().verify()).is_ok();
        let mut failed = verdict("every member's vector is enabled [PORT-101]", bound);

        let started_us = now_us();
        let next = stream(plain_line, PLAIN_BIT, false, 1, 0, TOTAL);
        stream(framed_line, FRAMED_BIT, true, 2, 0, TOTAL);
        wait(10 * MS);
        let main = tallies.lock(|t| *t);

        // UART5's consumer holds every buffer while more than its pool and
        // ring hold arrives; then the hold ends and a little more follows.
        tallies.lock(|t| {
            t[1].hold = true;
            t[1].chunks_at_hold = t[1].chunks;
        });
        let next = stream(plain_line, PLAIN_BIT, false, 1, next, FLOOD);
        tallies.lock(|t| t[1].hold = false);
        let last = stream(plain_line, PLAIN_BIT, false, 1, next, AFTER);
        wait(10 * MS);
        let ended_us = now_us();
        let t = tallies.lock(|t| *t);
        let (h3, h5) = (framed.lock(|p| p.uart().health()), plain.lock(|p| p.uart().health()));
        for (name, tally, health) in [("USART3, 100 000 8E2", &t[0], h3), ("UART5, 115 200 8N1", &t[1], h5)] {
            defmt::info!(
                "{}: read {} wrong {} chunks {} idle-ended {} gaps {} wakes {}; stamps {}..{} us, {} backwards, {} ahead; buffers lost {}; line {}",
                name, tally.read, tally.wrong, tally.chunks, tally.idles, tally.gaps, tally.wakes,
                tally.first_us, tally.last_us, tally.backwards, tally.future, tally.lost, health
            );
        }
        failed += verdict(
            "every 8E2 byte came back once, in order, no gap [PORT-101]",
            t[0].read == TOTAL && t[0].wrong == 0 && t[0].gaps == 0,
        );
        failed += verdict(
            "every 8N1 byte came back once, in order, before the hold [PORT-101]",
            main[1].read == TOTAL && main[1].wrong == 0 && main[1].gaps == 0,
        );
        failed += verdict(
            "frame ends reached each consumer as idle chunks [PORT-005]",
            t[0].idles > 0 && t[1].idles > 0,
        );
        failed += verdict(
            "no generation change before the first gap [PORT-005]",
            t[0].moved == 0 && t[1].moved == 0 && t[0].generation.is_some(),
        );
        let stamped = |tally: &Tally| {
            tally.backwards == 0 && tally.future == 0
                && tally.first_us >= started_us && tally.last_us <= ended_us
        };
        failed += verdict(
            "every chunk was stamped from the monotonic, in order, within the run, no later than its read [PORT-005]",
            stamped(&t[0]) && stamped(&t[1]),
        );
        failed += verdict(
            "the 8N1 port's main run was stamped before the 8E2 port's, as sent [PORT-005]",
            main[1].last_us <= main[0].first_us,
        );
        let at_gap = t[1].read_at_gap.unwrap_or(0);
        // The gap chunk itself is counted among the chunks before it.
        let kept = t[1].kept.unwrap_or(0).saturating_sub(1);
        defmt::info!("UART5 held: {} chunks ({} bytes) kept by the pool and read before the gap, {} bytes after",
            kept, at_gap.saturating_sub(TOTAL), t[1].read - at_gap);
        failed += verdict(
            "the loss while held reached the consumer as a gap, the stream in order on both sides [PORT-006]",
            t[1].gaps > 0 && t[1].wrong == 0 && t[1].advanced,
        );
        failed += verdict(
            "while held, the bytes the pool took were kept and read first, the rest after the gap [PORT-007]",
            kept == POOL as u32 && at_gap > TOTAL && t[1].read > at_gap && t[1].expected == last,
        );
        failed += verdict("no buffer was lost [PORT-101]", t[0].lost == 0 && t[1].lost == 0);
        failed += verdict(
            "each consumer drained the port init bound it to at run time [PORT-102]",
            core::hint::black_box(SWAP) && t[0].chunks > 0 && t[1].chunks > 0,
        );
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit ends
        // that, so give it the last lines first (about 200 ms).
        wait(200 * MS);
        if failed == 0 { debug::exit(EXIT_SUCCESS) } else { debug::exit(EXIT_FAILURE) }
    }

    /// Sends `total` bytes of a counter from `first` on `line`, in frames of
    /// 1 to 32 bytes, each followed by three characters of idle line.
    /// Returns the next counter value.
    fn stream<const N: u8>(line: &mut Pin<'C', N, Output>, bit: u32, parity: bool, stop: u32, first: u8, total: u32) -> u8 {
        let mut next = first;
        let mut sent = 0u32;
        let mut length = 1u32;
        while sent < total {
            let len = length.min(total - sent);
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
        next
    }

    /// Spins until `cycles` core cycles have passed. Interrupts still run.
    fn wait(cycles: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < cycles {}
    }

    /// Sends `bytes` on `line` as a UART would, `bit` core cycles a bit: a
    /// start bit, eight data bits least significant first, an even parity bit
    /// when `parity`, and `stop` stop bits. Every edge is timed from the
    /// first, so no error builds up, and interrupts are masked for the frame.
    fn send<const N: u8>(line: &mut Pin<'C', N, Output>, bytes: &[u8], bit: u32, parity: bool, stop: u32) {
        cortex_m::interrupt::free(|_| {
            let mut edge = DWT::cycle_count();
            let mut level = |line: &mut Pin<'C', N, Output>, high: bool| {
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

    /// The monotonic's time, as the members stamp it.
    fn now_us() -> u64 {
        Mono::now().duration_since_epoch().to_micros()
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
