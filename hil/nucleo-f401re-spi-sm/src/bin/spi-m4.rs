//! PROTOTYPE, variant M with four devices: `spi_multi_port` on SPI1 of the
//! Nucleo-F401RE, over FerroHAL's link S1 (PA7 into PA6; UM1724 table 29).
//! Device 0's chip select is PA4, as `nucleo-f401re-spi-loopback`'s;
//! devices 1 to 3 are PC8 (CN10-2), PC5 (CN10-6) and PC4 (CN10-34), spare
//! outputs with nothing on them (UM1724 table 29; none is in FerroHAL's F401
//! link list). The checks read back the level the port drives.
//!
//! Routing as the loopback bench: SPI1 on AF5, SCK PA5, MISO PA6, MOSI PA7
//! (DS10086 table 9); TX on DMA2 stream 3, RX on DMA2 stream 0, channel 3
//! (RM0368 table 29).
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | the port's member, also pended by every handle | 4 |
//! | `SPI2`, `SPI3`, `SPI4` | RTIC: `stress_high` (3), `stress_low` (2), `checks` (1) | |

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3, SPI4],

    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin, Speed},
        rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
        spi::{self, Wiring, MODE_1, MODE_2, MODE_3},
        time::Hertz,
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::spi_queue::{self, MultiMemory, Port, SpiHandle};
    use nucleo_f401re_spi_sm as bench;

    /// 84 MHz from HSI (DS10086 section 3.13), as the loopback bench.
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
    const CPU_HZ: u32 = 84_000_000;

    /// One setting per device. SPI1's kernel clock is 84 MHz (RM0368
    /// section 20.5.1): 400 kHz gives 328 kHz, 2 MHz gives 1.3125 MHz,
    /// 1 MHz gives 656 kHz.
    const CONFIGS: [spi::Config; 4] = [
        spi::Config::new(Hertz::khz(400)),
        spi::Config::new(Hertz::mhz(2)).with_mode(MODE_3),
        spi::Config::new(Hertz::mhz(1)).with_mode(MODE_1),
        spi::Config::new(Hertz::khz(400)).with_mode(MODE_2),
    ];
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[0]).is_ok());
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[1]).is_ok());
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[2]).is_ok());
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[3]).is_ok());

    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    type Spi1Wiring = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'A', 7>, Stream<2, 3>, Stream<2, 0>>;
    type Cs = (Pin<'A', 4, Output>, Pin<'C', 8, Output>, Pin<'C', 5, Output>, Pin<'C', 4, Output>);

    /// Queue entries for the bus, and result slots per device (16 hold 15).
    const QUEUE: usize = 8;
    const RESULTS: usize = 16;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// Which device the checker's first handle is, decided at boot (D12):
    /// a run-time value, as a stored setting would be.
    static FIRST: usize = 0;

    #[shared]
    struct Shared {
        port: Port<Spi1Wiring, Cs>,
        stressed: [(u32, u32, u32); 2],
    }

    #[local]
    struct Local {
        main: [SpiHandle; 2],
        low: SpiHandle,
        high: SpiHandle,
    }

    #[init(local = [
        tx_buf: [u8; 64] = [0; 64],
        rx_buf: [u8; 64] = [0; 64],
        memory: MultiMemory<QUEUE, RESULTS, 4> = MultiMemory::new(),
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
        let mut gpioc = p.GPIOC.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let ctrl = &mut gpioa.ctrl;
        let mut sck = gpioa.p5.into_alternate::<5>(ctrl);
        sck.set_speed(ctrl, Speed::Medium);
        let miso = gpioa.p6.into_alternate::<5>(ctrl);
        let mut mosi = gpioa.p7.into_alternate::<5>(ctrl);
        mosi.set_speed(ctrl, Speed::Medium);
        let cs = (
            gpioa.p4.into_push_pull_output(ctrl),
            gpioc.p8.into_push_pull_output(&mut gpioc.ctrl),
            gpioc.p5.into_push_pull_output(&mut gpioc.ctrl),
            gpioc.p4.into_push_pull_output(&mut gpioc.ctrl),
        );
        let local = cx.local;
        let built = spi_queue::new_multi(
            &mut rcc,
            Wiring { spi: p.SPI1, sck, miso, mosi, tx_dma: dma2.s3, rx_dma: dma2.s0 },
            cs,
            &CONFIGS,
            local.tx_buf,
            local.rx_buf,
            local.memory,
            spi1::kick,
            cycles,
        );
        let Ok(built) = built else {
            defmt::panic!("SPI1 refused");
        };
        let mut handles = built.handles;
        // Boot-time binding: one handle type, picked by a run-time value.
        let first = core::hint::black_box(FIRST);
        let (Some(a), Some(b), Some(c), Some(d)) = (
            handles.take(first),
            handles.take((first + 1) % 4),
            handles.take((first + 2) % 4),
            handles.take((first + 3) % 4),
        ) else {
            defmt::panic!("a handle was missing");
        };
        defmt::info!("M with four devices on the Nucleo-F401RE; checker bound to device {} at boot", a.device());
        checks::spawn().ok();
        (
            Shared { port: built.port, stressed: [(0, 0, 0); 2] },
            Local { main: [a, b], low: c, high: d },
        )
    }

    #[group(from = ferroforge_stm32::spi_multi_port, shared = [port = port])]
    mod spi1 {
        #[task(binds = DMA2_STREAM0, priority = 4)]
        fn m_on_spi_rx_dma;
    }

    #[task(priority = 1, local = [main], shared = [port, stressed])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut port, mut stressed, .. } = cx.shared;
        let main = cx.local.main;
        let [a, b] = main;
        let mut failed = 0u32;
        failed += bench::verdict("boot binding: the handles came from a run-time choice [D12]", a.device() == 0);
        failed += bench::length("M4", a, &mut port, cycles, &[4, 64]).await;
        failed += bench::latency("M4", a, cycles, 1000).await;
        failed += bench::multipart(a).await;
        failed += bench::chip_select(a, &mut port).await;
        failed += bench::chip_select(b, &mut port).await;
        failed += bench::full(a, cycles, QUEUE as u32, ferroforge_stm32::spi_queue::SubmitError::Full).await;
        {
            // Low and high on device 1, the mediums on device 0: the high
            // overtakes another device's queued work.
            let mut hs = [&mut *a, &mut *b];
            failed += bench::priority(&mut hs, [1, 0, 0, 1], cycles, true);
        }
        failed += bench::switch(a, b, &mut port, CPU_HZ).await;
        failed += bench::cancel(a, Some(b), &mut port, cycles, 3).await;
        // Device 0 fills the bus's one queue; device 1 is then refused.
        failed += bench::isolation(a, b, cycles, false).await;

        // Two tasks at different priorities push into the one queue while
        // this one does too.
        stress_low::spawn().ok();
        stress_high::spawn().ok();
        let mine = bench::stress(a, 500, 1).await;
        let start = cycles();
        while stressed.lock(|s| s[0].0 < 500 || s[1].0 < 500) && cycles().wrapping_sub(start) < CPU_HZ * 4 {
            cortex_m::asm::delay(CPU_HZ / 1000);
        }
        let s = stressed.lock(|s| *s);
        defmt::info!(
            "concurrent: checker {} done {} wrong {} retried; priority 1 task {} done {} wrong {} retried; priority 2 task {} done {} wrong {} retried",
            mine.0, mine.1, mine.2, s[0].0, s[0].1, s[0].2, s[1].0, s[1].1, s[1].2
        );
        failed += bench::verdict(
            "concurrent: three tasks at two priorities, every transaction back with its bytes",
            mine.0 == 500 && mine.1 == 0 && s[0].0 == 500 && s[0].1 == 0 && s[1].0 == 500 && s[1].1 == 0,
        );
        failed += bench::closing("M4", &mut port);
        finish(failed);
    }

    #[task(priority = 2, local = [low], shared = [stressed])]
    async fn stress_low(mut cx: stress_low::Context) {
        let r = bench::stress(cx.local.low, 500, 2).await;
        cx.shared.stressed.lock(|s| s[0] = r);
    }

    #[task(priority = 3, local = [high], shared = [stressed])]
    async fn stress_high(mut cx: stress_high::Context) {
        let r = bench::stress(cx.local.high, 500, 3).await;
        cx.shared.stressed.lock(|s| s[1] = r);
    }

    fn finish(failed: u32) -> ! {
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        cortex_m::asm::delay(CPU_HZ / 5);
        if failed == 0 { debug::exit(EXIT_SUCCESS) } else { debug::exit(EXIT_FAILURE) }
        loop {}
    }
}
