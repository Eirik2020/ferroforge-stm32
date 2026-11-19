//! The SPI port group's reference: a hand-written `ferroforge::app!` on the
//! Nucleo-F401RE (MB1136) that selects `ferroforge_stm32::spi_port` once,
//! with four devices on SPI1, and runs the checks of
//! `hil/common/spi_port.rs` round a MOSI-to-MISO loop.
//!
//! The link, as FerroHAL's F401 bench fits it (its S1; UM1724 table 29):
//!
//! | Link | From | To | Port |
//! | --- | --- | --- | --- |
//! | S1 | CN10-15 (PA7) | CN10-13 (PA6) | SPI1 MOSI into its own MISO |
//!
//! Routing (DS10086 table 9, alternate functions; RM0368 table 29, DMA2
//! requests): SPI1 on AF5, SCK PA5, MISO PA6, MOSI PA7; TX on DMA2 stream 3
//! channel 3, RX on DMA2 stream 0 channel 3. SCK is also the board's LD2.
//! The chip selects are plain push-pull outputs with nothing on them, and
//! the checks read back the level the port drives: device 0 on PA4
//! (CN7-32), devices 1 to 3 on PC8 (CN10-2), PC5 (CN10-6) and PC4 (CN10-34)
//! (UM1724 table 29; none is in FerroHAL's F401 link list).
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | `spi1`'s member, also pended by every handle | 4 |
//! | `SPI2`, `SPI3`, `SPI4` | RTIC: `stress_high` (3), `stress_low` (2), `checks` (1) | |
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
    use ferroforge_stm32::spi_port::{self, Memory, SpiHandle, SpiPort};
    use nucleo_f401re_spi_loopback as bench;

    /// 84 MHz from HSI, the F401's ceiling (DS10086 section 3.13): 16 MHz /
    /// 16 * 336 = 336 MHz VCO, / 4 = 84 MHz. APB2 at 84 MHz, SPI1's kernel.
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
    const CPU_HZ: u32 = 84_000_000;

    /// One setting per device. SPI1's kernel clock is 84 MHz (RM0368
    /// section 20.5.1): 400 kHz gives its slowest SCK, 328 kHz, so a long
    /// transfer lasts long enough to see chip select held; 2 MHz gives
    /// 1.3125 MHz, 1 MHz gives 656 kHz.
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
    /// SPI1 over S1: PA7 into PA6, DMA2 stream 3 transmitting, stream 0
    /// receiving.
    type Spi1Wiring = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'A', 7>, Stream<2, 3>, Stream<2, 0>>;
    type Cs = (Pin<'A', 4, Output>, Pin<'C', 8, Output>, Pin<'C', 5, Output>, Pin<'C', 4, Output>);

    /// FerroHAL's DMA buffers, and so the longest message.
    const BUS: usize = 256;
    /// Each device's lent buffers, (device, length): device 0 seven of 64
    /// bytes and one of 256; device 1 two; devices 2 and 3, the stress
    /// tasks', three each. A queue of `SLOTS` slots holds `SLOTS - 1`.
    static LENT: [(usize, usize); 16] = [
        (0, 64), (0, 64), (0, 64), (0, 64), (0, 64), (0, 64), (0, 64), (0, 256),
        (1, 64), (1, 64),
        (2, 64), (2, 64), (2, 64),
        (3, 64), (3, 64), (3, 64),
    ];
    const POOL: usize = 7 * 64 + 256 + 8 * 64;
    const SLOTS: usize = 9;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// Which device the checker's first handle is, decided at boot (D12):
    /// a run-time value, as a stored setting would be.
    static FIRST: usize = 0;

    #[shared]
    struct Shared {
        port: SpiPort<Spi1Wiring, Cs>,
        stressed: [(u32, u32, u32); 2],
    }

    #[local]
    struct Local {
        main: [SpiHandle; 2],
        low: SpiHandle,
        high: SpiHandle,
    }

    #[init(local = [
        tx_buf: [u8; BUS] = [0; BUS],
        rx_buf: [u8; BUS] = [0; BUS],
        memory: Memory<SLOTS, 4> = Memory::new(),
        pool: [u8; POOL] = [0; POOL],
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
        let built = spi_port::new(
            &mut rcc,
            Wiring { spi: p.SPI1, sck, miso, mosi, tx_dma: dma2.s3, rx_dma: dma2.s0 },
            cs,
            &CONFIGS,
            local.tx_buf,
            local.rx_buf,
            local.memory,
            &mut bench::carve(local.pool, &LENT),
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
        defmt::info!("spi_port with four devices on the Nucleo-F401RE, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared { port: built.port, stressed: [(0, 0, 0); 2] },
            Local { main: [a, b], low: c, high: d },
        )
    }

    #[group(from = ferroforge_stm32::spi_port, shared = [port = port])]
    mod spi1 {
        #[task(binds = DMA2_STREAM0, priority = 4)]
        fn on_spi_rx_dma;
    }

    #[task(priority = 1, local = [main], shared = [port, stressed])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut port, mut stressed, .. } = cx.shared;
        let [a, b] = cx.local.main;
        let mut failed = 0u32;
        failed += bench::verdict("the member's vector is enabled [PORT-201]", bench::View::vectors_enabled(&mut port));
        failed += bench::verdict(
            "boot binding: every handle one type, bound to its task by a run-time choice [PORT-207]",
            a.device() == 0 && b.device() == 1,
        );
        let idle = (0..4).all(|d| bench::View::selected(&mut port, d) == Some(false));
        failed += bench::verdict("every device starts deselected [PORT-202]", idle);
        failed += bench::length(a, &mut port, cycles, &[4, 64, 256]).await;
        failed += bench::latency(a, cycles, 1000).await;
        failed += bench::multipart(a).await;
        failed += bench::chip_select(a, &mut port).await;
        failed += bench::chip_select(b, &mut port).await;
        failed += bench::full(a, bench::Bus::Loop, cycles).await;
        failed += bench::priority(&mut [&mut *a], &[bench::Bus::Loop], [0, 0, 0, 0], cycles, false);
        // Low and high on device 1, the mediums on device 0: the high
        // overtakes another device's queued messages.
        failed += bench::priority(&mut [&mut *a, &mut *b], &[bench::Bus::Loop; 2], [1, 0, 0, 1], cycles, true);
        failed += bench::switch(a, bench::Bus::Loop, b, bench::Bus::Loop, &mut port, CPU_HZ).await;
        failed += bench::cancel(a, bench::Bus::Loop, b, bench::Bus::Loop, &mut port, cycles, 3).await;
        failed += bench::isolation(a, bench::Bus::Loop, b, bench::Bus::Loop, cycles).await;

        // Two tasks at different priorities submit to their own devices
        // while this one does too.
        stress_low::spawn().ok();
        stress_high::spawn().ok();
        let mine = bench::stress(a, bench::Bus::Loop, 500, 1).await;
        let start = cycles();
        while stressed.lock(|s| s[0].0 < 500 || s[1].0 < 500) && cycles().wrapping_sub(start) < CPU_HZ * 4 {
            cortex_m::asm::delay(CPU_HZ / 1000);
        }
        let s = stressed.lock(|s| *s);
        defmt::info!(
            "concurrent: checker {} done {} wrong {} retried; priority 2 task {} done {} wrong {} retried; priority 3 task {} done {} wrong {} retried",
            mine.0, mine.1, mine.2, s[0].0, s[0].1, s[0].2, s[1].0, s[1].1, s[1].2
        );
        failed += bench::verdict(
            "concurrent: three tasks at three priorities, every message back done with its bytes [PORT-201]",
            mine.0 == 500 && mine.1 == 0 && s[0].0 == 500 && s[0].1 == 0 && s[1].0 == 500 && s[1].1 == 0,
        );
        failed += bench::closing(&mut port);
        finish(failed);
    }

    #[task(priority = 2, local = [low], shared = [stressed])]
    async fn stress_low(mut cx: stress_low::Context) {
        let r = bench::stress(cx.local.low, bench::Bus::Loop, 500, 2).await;
        cx.shared.stressed.lock(|s| s[0] = r);
    }

    #[task(priority = 3, local = [high], shared = [stressed])]
    async fn stress_high(mut cx: stress_high::Context) {
        let r = bench::stress(cx.local.high, bench::Bus::Loop, 500, 3).await;
        cx.shared.stressed.lock(|s| s[1] = r);
    }

    fn finish(failed: u32) -> ! {
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit
        // ends that, so give it the last lines first (about 200 ms).
        cortex_m::asm::delay(CPU_HZ / 5);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS)
        } else {
            debug::exit(EXIT_FAILURE)
        }
        loop {}
    }
}
