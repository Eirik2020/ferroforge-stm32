//! The UART port group on `usart_v4`: the F401 loopback's checks on the
//! Nucleo-H753ZI (MB1364), selecting `ferroforge_stm32::uart_port` twice.
//!
//! The links, as FerroHAL's H753 bench fits them (its Z2 and Z3, on the Zio
//! headers; UM2407 tables 18 to 21):
//!
//! | Link | From | To | Port |
//! | --- | --- | --- | --- |
//! | Z3 | CN10-14 (PB6) | CN10-16 (PB7) | USART1 TX into its own RX |
//! | Z2 | CN7-1 (PC6) | CN7-11 (PC7) | USART6 TX into its own RX |
//!
//! Routing: USART1 on PB6/PB7 and USART6 on PC6/PC7 are alternate function 7
//! (DS12117 table 9). Through DMAMUX1 any DMA1 or DMA2 stream serves
//! any USART request (RM0433 section 17.3); these are the streams FerroHAL's
//! run A gives the same ports: USART1 on DMA2 streams 5 (TX) and 4 (RX),
//! USART6 on DMA1 streams 6 (TX) and 0 (RX).
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `USART1`, `DMA2_STREAM4`, `DMA2_STREAM5` | `usart1`'s members | 3 |
//! | `USART6`, `DMA1_STREAM0`, `DMA1_STREAM6` | `usart6`'s members | 3 |
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
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
        },
        time::Hertz,
        uart::{self, Uart, Wiring},
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::uart_port::{self, Events};

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

        let local = cx.local;
        let usart1 = uart_port::new(
            &mut rcc,
            Wiring { uart: p.USART1, tx: tx1, rx: rx1, tx_dma: dma2.s5, rx_dma: dma2.s4 },
            &FRAME,
            local.tx1,
            local.rx1,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART1: {}", refused.error()));
        let usart6 = uart_port::new(
            &mut rcc,
            Wiring { uart: p.USART6, tx: tx6, rx: rx6, tx_dma: dma1.s6, rx_dma: dma1.s0 },
            &FRAME,
            local.tx6,
            local.rx6,
        )
        .unwrap_or_else(|refused| defmt::panic!("USART6: {}", refused.error()));

        defmt::info!("uart_port loopback on the Nucleo-H753ZI, clocks: {}", CLOCKS);
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
            cortex_m::asm::delay(MS);
        }
        // Let a stray extra byte show, if there is one.
        cortex_m::asm::delay(10 * MS);

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
        cortex_m::asm::delay(200 * MS);
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
