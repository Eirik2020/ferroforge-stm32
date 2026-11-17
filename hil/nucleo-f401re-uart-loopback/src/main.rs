//! The UART port group's golden reference: a hand-written `ferroforge::app!`
//! on the Nucleo-F401RE (MB1136) that selects `ferroforge_stm32::uart_port`
//! twice and sends a byte stream round each loop.
//!
//! This is what a generated firmware must match: the same resources, the same
//! selections, the same constructor calls, and the same bench result
//! (FerroSmith design record v3, section 10).
//!
//! The links, as FerroHAL's F401 bench fits them (its J1 and J2; UM1724
//! tables 16 and 29, no solder bridge on either):
//!
//! | Link | From | To | Port |
//! | --- | --- | --- | --- |
//! | J1 | CN5-1 (D8, PA9) | CN9-3 (D2, PA10) | USART1 TX into its own RX |
//! | J2 | CN10-14 (PA11) | CN10-12 (PA12) | USART6 TX into its own RX |
//!
//! Routing (RM0368 table 9, alternate functions in DS10086 table 9; DMA2
//! request mapping in RM0368 table 28): USART1 on AF7, TX on DMA2 stream 7
//! channel 4, RX on DMA2 stream 2 channel 4; USART6 on AF8, TX on DMA2
//! stream 6 channel 5, RX on DMA2 stream 1 channel 5. FerroHAL's routing
//! traits refuse any other pairing at compile time.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `USART1`, `DMA2_STREAM2`, `DMA2_STREAM7` | `usart1`'s members | 3 |
//! | `USART6`, `DMA2_STREAM1`, `DMA2_STREAM6` | `usart6`'s members | 3 |
//! | `SPI2` | RTIC, dispatching the `events` tasks | 2 |
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
    /// Bytes sent round each loop, in frames of 1 to 64 bytes.
    const TOTAL: u32 = 4096;

    /// What one port's reader has seen. The bytes sent are a counter
    /// wrapping at 256, so each byte read is checked against the next value.
    #[derive(Clone, Copy, Default)]
    pub struct Tally {
        pub bytes: u32,
        pub mismatches: u32,
        pub gaps: u32,
        pub wakes: u32,
        pub idles: u32,
        pub sent: u32,
        pub expected: u8,
    }

    #[shared]
    struct Shared {
        usart1: Uart<Usart1Wiring>,
        usart6: Uart<Usart6Wiring>,
        tally1: Tally,
        tally6: Tally,
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

        let local = cx.local;
        let usart1 = uart_port::new(
            &mut rcc,
            Wiring { uart: p.USART1, tx: tx1, rx: rx1, tx_dma: dma2.s7, rx_dma: dma2.s2 },
            &FRAME,
            local.tx1,
            local.rx1,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let usart6 = uart_port::new(
            &mut rcc,
            Wiring { uart: p.USART6, tx: tx6, rx: rx6, tx_dma: dma2.s6, rx_dma: dma2.s1 },
            &FRAME,
            local.tx6,
            local.rx6,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));

        defmt::info!("uart_port loopback on the Nucleo-F401RE, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared {
                usart1: usart1.uart,
                usart6: usart6.uart,
                tally1: Tally::default(),
                tally6: Tally::default(),
            },
            Local {},
        )
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

    /// USART1's reader: drains the ring and checks every byte.
    #[task(priority = 2, shared = [usart1, tally1])]
    async fn usart1_events(cx: usart1_events::Context, events: Events) {
        let usart1_events::SharedResources { mut usart1, mut tally1, .. } = cx.shared;
        tally1.lock(|tally| usart1.lock(|uart| drain(uart, tally, events)));
    }

    /// USART6's reader, as USART1's.
    #[task(priority = 2, shared = [usart6, tally6])]
    async fn usart6_events(cx: usart6_events::Context, events: Events) {
        let usart6_events::SharedResources { mut usart6, mut tally6, .. } = cx.shared;
        tally6.lock(|tally| usart6.lock(|uart| drain(uart, tally, events)));
    }

    /// Sends the stream round both loops and judges what came back.
    #[task(priority = 1, shared = [usart1, usart6, tally1, tally6])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut usart1, mut usart6, mut tally1, mut tally6, .. } =
            cx.shared;

        let bound = usart1.lock(|uart| uart.vectors().verify()).is_ok()
            && usart6.lock(|uart| uart.vectors().verify()).is_ok();
        let mut failed = 0u32;
        failed += verdict("every member's vector is enabled [PORT-003]", bound);

        // Both ports at once, frame by frame: a frame that does not fit
        // waits for the transmit stream's vector to free room.
        let mut next = [0u8; 2];
        let mut sent = [0u32; 2];
        let mut length = 1usize;
        let mut spins = 0u32;
        while sent[0] < TOTAL || sent[1] < TOTAL {
            let mut frame = [0u8; 64];
            for port in 0..2 {
                if sent[port] >= TOTAL {
                    continue;
                }
                let len = length.min((TOTAL - sent[port]) as usize);
                for byte in &mut frame[..len] {
                    *byte = next[port];
                    next[port] = next[port].wrapping_add(1);
                }
                let result = if port == 0 {
                    usart1.lock(|uart| uart.try_send(&frame[..len]))
                } else {
                    usart6.lock(|uart| uart.try_send(&frame[..len]))
                };
                match result {
                    Ok(()) => sent[port] += len as u32,
                    Err(_) => next[port] = next[port].wrapping_sub(len as u8),
                }
            }
            length = length % 64 + 1;
            spins += 1;
            if spins > 200_000 {
                break;
            }
        }
        failed += verdict("both ports queued every byte [PORT-003]", sent == [TOTAL, TOTAL]);

        // At 115200 baud 4096 bytes take about 360 ms; wait up to a second.
        for _ in 0..1000 {
            let done = tally1.lock(|t| t.bytes) >= TOTAL && tally6.lock(|t| t.bytes) >= TOTAL;
            if done {
                break;
            }
            cortex_m::asm::delay(84_000);
        }
        // Let a stray extra byte show, if there is one.
        cortex_m::asm::delay(840_000);

        for (name, tally) in [("USART1", tally1.lock(|t| *t)), ("USART6", tally6.lock(|t| *t))] {
            defmt::info!(
                "{}: {} bytes read, {} wrong, {} gaps, {} wakes ({} idle, {} sent)",
                name, tally.bytes, tally.mismatches, tally.gaps, tally.wakes, tally.idles, tally.sent
            );
            failed += verdict("every byte came back, in order, once [PORT-003]",
                tally.bytes == TOTAL && tally.mismatches == 0 && tally.gaps == 0);
            failed += verdict("the group woke the firmware's reader [PORT-003]", tally.wakes > 0 && tally.idles > 0);
            failed += verdict("the transmit vector reported frames sent [PORT-003]", tally.sent > 0);
        }
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit ends
        // that, so give it the last lines first (about 200 ms).
        cortex_m::asm::delay(16_800_000);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }

    /// Reads every whole chunk there is, checking each byte against the
    /// counter.
    fn drain<W: uart::Parts>(uart: &mut Uart<W>, tally: &mut Tally, events: Events) {
        tally.wakes += 1;
        tally.idles += u32::from(events.idle);
        tally.sent += u32::from(events.sent);
        let mut buf = [0u8; 64];
        // Bounded: a ring holds at most RX_RING bytes, so this many reads
        // empty it however the faults fall.
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
