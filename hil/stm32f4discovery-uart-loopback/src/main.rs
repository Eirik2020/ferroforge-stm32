//! The UART port on the STM32F4 Discovery (MB997, STM32F407VGT6), die
//! 0x413: the board that proves `ferroforge_stm32::uart_port`. A hand-written
//! `ferroforge::app!` selecting it twice, with the consumer in `consumer.rs`.
//!
//! This is the group's golden reference: what a generated firmware must
//! match, in resources, selections, constructor calls and bench result
//! (FerroSmith design record v3, section 10).
//!
//! The links, as FerroHAL's Discovery bench fits them (its D1 and D2, on the
//! P2 header; UM1472 table 7):
//!
//! | Link | From | To | Port |
//! | --- | --- | --- | --- |
//! | D1 | P2-37 (PC10) | P2-38 (PC11) | USART3 TX into its own RX |
//! | D2 | P2-35 (PC12) | P2-34 (PD2) | UART5 TX into its own RX |
//!
//! Routing: USART3 on PC10/PC11 is AF7, UART5 on PC12 and PD2 AF8 (DS8626
//! table 9). DMA1 channel 4 (RM0090 table 43): USART3 TX on stream 3, RX on
//! stream 1; UART5 TX on stream 7, RX on stream 0. These are pins and streams
//! flight controller targets on this die assign to the same UARTs.
//!
//! PC10 and PC12 also reach the CS43L22 audio DAC; its reset, PD4, is held
//! low, which powers it down, as FerroHAL's Discovery bench does.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `USART3`, `DMA1_STREAM1`, `DMA1_STREAM3` | `usart3`'s members | 3 |
//! | `UART5`, `DMA1_STREAM0`, `DMA1_STREAM7` | `uart5`'s members | 3 |
//! | `SysTick` | the monotonic, at 1 kHz | RTIC's |
//! | `SPI2` | RTIC, dispatching the two consumers | 2 |
//! | `SPI1` | RTIC, dispatching `checks` | 1 |
//!
//! USART3 floods: after its main run it sends 2048 bytes while its consumer
//! holds every buffer for 300 ms. UART5 runs the tail, with a 512-byte ring
//! and 256-byte pool buffers so the 280-byte frame comes back in two chunks.
//! Which consumer task gets which handle is decided at boot from a run-time
//! value. Each check prints `PASS` or `FAIL` with the requirements it
//! verifies in brackets; the run passes when every check does.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod consumer;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI1, SPI2],

    use rtic_monotonics::systick::prelude::*;

    // The monotonic the port's members stamp each service with, at 1 kHz.
    systick_monotonic!(Mono, 1000);

    use ferrohal_stm32::{
        chip::{UART5, USART3},
        dma::Stream,
        gpio::{Alternate, Pin, Pull, Speed},
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
            Sysclk,
        },
        time::Hertz,
        uart::{self, Wiring},
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::{uart::{Handle, Memory, Port}, uart_port};
    use crate::consumer::{Loop, report, verdict};

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

    /// Checked while compiling, as FerroHAL's benches check theirs.
    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the F407's limits"),
    };
    const CPU_HZ: u32 = 168_000_000;

    const BAUD: u32 = 115_200;
    const FRAME: uart::Config = uart::Config::new(Hertz(BAUD));
    const _: () = assert!(uart::check::<USART3>(&CLOCKS, &FRAME).is_ok());
    const _: () = assert!(uart::check::<UART5>(&CLOCKS, &FRAME).is_ok());
    /// One 8N1 character on the wire, in CPU cycles.
    const CHAR_CYCLES: u32 = CPU_HZ / BAUD * 10;

    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    type Af8<const P: char, const N: u8> = Pin<P, N, Alternate<8>>;

    const RX_RING: usize = 256;
    const RX_RING6: usize = 512;
    /// Room for the long frame, whole.
    const TX_BUFFER: usize = 320;
    /// Bytes each port sends in its main run (about 1.4 s at 115200).
    const TOTAL: u32 = 16_384;
    /// Bytes USART3 sends while its consumer holds every buffer.
    const FLOOD: u32 = 2_048;
    /// Queue slots (one stays free): 7 buffers each way. USART3's pool
    /// buffers are 64 bytes, UART5's 256, at most half each ring. Each
    /// handle lends six 64-byte buffers and one of 288 for the long frame.
    const SLOTS: usize = 8;
    const POOL: usize = 7;
    const SHORT: usize = 6;
    const LONG_BUFFER: usize = 288;

    type Port1 = Port<USART3, Af7<'C', 10>, Af7<'C', 11>, Stream<1, 3>, Stream<1, 1>>;
    type Port6 = Port<UART5, Af8<'C', 12>, Af8<'D', 2>, Stream<1, 7>, Stream<1, 0>>;

    /// Which UART the first consumer takes, decided at boot. Read through
    /// `black_box`, so it is a run-time value, as a stored setting would be.
    static SWAP: bool = true;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    fn now_us() -> u64 {
        Mono::now().duration_since_epoch().to_micros()
    }

    #[shared]
    struct Shared {
        port1: Port1,
        port6: Port6,
        /// Tallies by UART: 0 is USART3, 1 is UART5.
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
        rx6: [u8; RX_RING6] = [0; RX_RING6],
        q1: Memory<SLOTS, SLOTS> = Memory::new(),
        q6: Memory<SLOTS, SLOTS> = Memory::new(),
        pool1: [[u8; 64]; POOL] = [[0; 64]; POOL],
        pool6: [[u8; 256]; POOL] = [[0; 256]; POOL],
        lent1: [[u8; 64]; SHORT] = [[0; 64]; SHORT],
        lent6: [[u8; 64]; SHORT] = [[0; 64]; SHORT],
        long1: [u8; LONG_BUFFER] = [0; LONG_BUFFER],
        long6: [u8; LONG_BUFFER] = [0; LONG_BUFFER],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        Mono::start(core.SYST, CPU_HZ);
        let p = cx.device;
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpioc = p.GPIOC.split(&mut rcc);
        let mut gpiod = p.GPIOD.split(&mut rcc);
        let dma1 = p.DMA1.split(&mut rcc);
        // The CS43L22 held in reset, powered down.
        let mut dac_reset = gpiod.p4.into_push_pull_output(&mut gpiod.ctrl);
        dac_reset.set_low();
        let ctrl = &mut gpioc.ctrl;
        let mut tx1 = gpioc.p10.into_alternate::<7>(ctrl);
        tx1.set_speed(ctrl, Speed::Medium);
        let mut rx1 = gpioc.p11.into_alternate::<7>(ctrl);
        rx1.set_pull(ctrl, Pull::Up);
        let mut tx6 = gpioc.p12.into_alternate::<8>(ctrl);
        tx6.set_speed(ctrl, Speed::Medium);
        let mut rx6 = gpiod.p2.into_alternate::<8>(&mut gpiod.ctrl);
        rx6.set_pull(&mut gpiod.ctrl, Pull::Up);
        let w1 = Wiring { uart: p.USART3, tx: tx1, rx: rx1, tx_dma: dma1.s3, rx_dma: dma1.s1 };
        let w6 = Wiring { uart: p.UART5, tx: tx6, rx: rx6, tx_dma: dma1.s7, rx_dma: dma1.s0 };
        let local = cx.local;
        // `usartN::kick` is generated from the group's `pends`.
        let mut pool1 = local.pool1.iter_mut().map(|b| -> &'static mut [u8] { b });
        let mut lent1 = local.lent1.iter_mut().map(|b| -> &'static mut [u8] { b })
            .chain(core::iter::once::<&'static mut [u8]>(local.long1));
        let Ok(r1) = uart_port::new(
            &mut rcc, w1, &FRAME, local.tx1, local.rx1, local.q1,
            &mut pool1, &mut lent1, usart3::kick,
        ) else {
            defmt::panic!("USART3 refused");
        };
        let mut pool6 = local.pool6.iter_mut().map(|b| -> &'static mut [u8] { b });
        let mut lent6 = local.lent6.iter_mut().map(|b| -> &'static mut [u8] { b })
            .chain(core::iter::once::<&'static mut [u8]>(local.long6));
        let Ok(r6) = uart_port::new(
            &mut rcc, w6, &FRAME, local.tx6, local.rx6, local.q6,
            &mut pool6, &mut lent6, uart5::kick,
        ) else {
            defmt::panic!("UART5 refused");
        };
        let tail = Loop { tail: true, ..Loop::new(TOTAL) };
        let flood = Loop { flood: FLOOD, ..Loop::new(TOTAL) };
        // Boot-time binding: both handles have one type, so a run-time value
        // picks which task gets which UART.
        let (first, second) = if core::hint::black_box(SWAP) {
            ((r6.handle, 1), (r1.handle, 0))
        } else {
            ((r1.handle, 0), (r6.handle, 1))
        };
        defmt::info!("uart_port on the STM32F4 Discovery, clocks {}; first consumer bound to {} at boot",
            CLOCKS, if first.1 == 0 { "USART3" } else { "UART5" });
        consumer_first::spawn().ok();
        consumer_second::spawn().ok();
        checks::spawn().ok();
        (Shared { port1: r1.port, port6: r6.port, loops: [flood, tail] }, Local { first, second })
    }

    #[group(from = ferroforge_stm32::uart_port, shared = [port = port1])]
    mod usart3 {
        #[task(binds = USART3, priority = 3)]
        fn on_uart;
        #[task(binds = DMA1_STREAM1, priority = 3)]
        fn on_rx_dma;
        #[task(binds = DMA1_STREAM3, priority = 3)]
        fn on_tx_dma;
    }

    #[group(from = ferroforge_stm32::uart_port, shared = [port = port6])]
    mod uart5 {
        #[task(binds = UART5, priority = 3)]
        fn on_uart;
        #[task(binds = DMA1_STREAM0, priority = 3)]
        fn on_rx_dma;
        #[task(binds = DMA1_STREAM7, priority = 3)]
        fn on_tx_dma;
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
                l.step(handle, now_us, cycles);
                l.wants_pause()
            });
            if pause {
                // Hold every buffer for 300 ms: the ring laps.
                cortex_m::asm::delay(CPU_HZ / 10 * 3);
                loops.lock(|loops| loops[index].release());
                continue;
            }
            handle.wait().await;
        }
    }

    #[task(priority = 1, shared = [port1, port6, loops])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut port1, mut port6, mut loops, .. } = cx.shared;
        let bound = port1.lock(|p| p.uart().vectors().verify()).is_ok()
            && port6.lock(|p| p.uart().vectors().verify()).is_ok();
        let mut failed = verdict("every member's vector is enabled [PORT-003]", bound);
        // About 1.5 s for the main runs, the flood's hold and the tail.
        for _ in 0..8_000 {
            let done = loops.lock(|l| {
                l[0].flood_done() && l[1].tail_done()
            });
            if done {
                break;
            }
            cortex_m::asm::delay(CPU_HZ / 1000);
        }
        cortex_m::asm::delay(CPU_HZ / 10);
        let l = loops.lock(|l| *l);
        let (unsure1, overruns) = port1.lock(|p| (p.unsure(), p.uart().health().buffer_overruns));
        let unsure6 = port6.lock(|p| p.unsure());
        defmt::info!("USART3 FerroHAL buffer overruns {}", overruns);
        failed += report("USART3", &l[0], unsure1, (POOL * 64) as u32, CHAR_CYCLES);
        failed += report("UART5", &l[1], unsure6, (POOL * 256) as u32, CHAR_CYCLES);
        failed += verdict(
            "each consumer drove the UART init bound it to at run time [PORT-004]",
            core::hint::black_box(SWAP) && l[0].back.frames > 0 && l[1].back.frames > 0,
        );
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit ends
        // that, so give it the last lines first (about 200 ms).
        cortex_m::asm::delay(CPU_HZ / 5);
        if failed == 0 { debug::exit(EXIT_SUCCESS) } else { debug::exit(EXIT_FAILURE) }
        loop {}
    }
}
