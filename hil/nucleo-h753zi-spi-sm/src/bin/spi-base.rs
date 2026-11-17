//! PROTOTYPE, baseline: today's `spi_port`, unchanged, timed the way S and
//! M are, on the Nucleo-H753ZI. The consumer shares the `Device` under
//! RTIC's lock, starts each transfer itself, and is spawned with `done`.
//!
//! Routing as `nucleo-h753zi-spi-loopback`: SPI1 on AF5, SCK PA5, MISO PA6,
//! MOSI PD7 over FerroHAL's link S1, CN9-2 into CN7-12 (DS12117 table 9;
//! UM2407 tables 18 to 21). Through DMAMUX1 (RM0433 section 17.3): TX on
//! DMA1 stream 5, RX on DMA1 stream 4. Chip select PD14 (CN7-16), nothing
//! on it: the checks read back the level the port drives. The AXI SRAM
//! holds every buffer, the data cache off (`memory.x`).
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA1_STREAM4` | `spi1`'s member | 3 |
//! | `SPI2` | RTIC: `spi1_done` | 2 |
//! | `SPI3` | RTIC: `checks` | 1 |

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin, Speed},
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
        },
        spi::{self, Wiring},
        time::Hertz,
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::spi_port::{self, Device, Error};
    use nucleo_h753zi_spi_sm::{Spread, verdict};

    /// 400 MHz from HSI, as FerroHAL's H753 bench runs: 64 MHz / 4 = 16 MHz
    /// reference, * 50 = 800 MHz VCO, / 2 = 400 MHz core; PLL1Q at 100 MHz
    /// is SPI1's kernel clock (DS12117 table 23, VOS1).
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
    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the H753's limits"),
    };
    const CPU_HZ: u32 = 400_000_000;

    /// SPI1's slowest SCK, its 100 MHz kernel clock / 256 = 391 kHz
    /// (RM0433 section 50.11.3).
    const BUS: spi::Config = spi::Config::new(Hertz::khz(400));
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &BUS).is_ok());

    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    type Spi1Wiring = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'D', 7>, Stream<1, 5>, Stream<1, 4>>;
    type Cs = Pin<'D', 14, Output>;

    const TRANSFERS: u32 = 1000;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// What `spi1_done` saw of the last transfer: when it ran, and whether
    /// the bytes were the ones sent.
    #[derive(Clone, Copy, Default)]
    pub struct Seen {
        pub ended: u32,
        pub at: u32,
        pub good: u32,
    }

    #[shared]
    struct Shared {
        spi1: Device<Spi1Wiring, Cs>,
        seen: Seen,
        expected: [u8; 4],
    }

    #[local]
    struct Local {}

    #[init(local = [
        tx_buf: [u8; 64] = [0; 64],
        rx_buf: [u8; 64] = [0; 64],
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
        let mut gpiod = p.GPIOD.split(&mut rcc);
        let dma1 = p.DMA1.split(&mut rcc);
        let ctrl = &mut gpioa.ctrl;
        let mut sck = gpioa.p5.into_alternate::<5>(ctrl);
        sck.set_speed(ctrl, Speed::Medium);
        let miso = gpioa.p6.into_alternate::<5>(ctrl);
        let mut mosi = gpiod.p7.into_alternate::<5>(&mut gpiod.ctrl);
        mosi.set_speed(&mut gpiod.ctrl, Speed::Medium);
        let cs = gpiod.p14.into_push_pull_output(&mut gpiod.ctrl);
        let local = cx.local;
        let spi1 = spi_port::new(
            &mut rcc,
            Wiring { spi: p.SPI1, sck, miso, mosi, tx_dma: dma1.s5, rx_dma: dma1.s4 },
            cs,
            &BUS,
            local.tx_buf,
            local.rx_buf,
        )
        .unwrap_or_else(|refused| defmt::panic!("SPI1: {}", refused.error()));
        defmt::info!("baseline spi_port on the Nucleo-H753ZI");
        checks::spawn().ok();
        (Shared { spi1: spi1.spi, seen: Seen::default(), expected: [0; 4] }, Local {})
    }

    #[group(from = ferroforge_stm32::spi_port, shared = [spi = spi1], spawn = [done = spi1_done])]
    mod spi1 {
        #[task(binds = DMA1_STREAM4, priority = 3)]
        fn on_spi_rx_dma;
    }

    /// The consumer's end: spawned with the outcome, reads the bytes under
    /// the lock.
    #[task(priority = 2, shared = [spi1, seen, expected])]
    async fn spi1_done(cx: spi1_done::Context, result: Result<usize, Error>) {
        let at = cycles();
        let spi1_done::SharedResources { mut spi1, mut seen, mut expected, .. } = cx.shared;
        let expected = expected.lock(|e| *e);
        let good = result.is_ok() && spi1.lock(|spi| spi.received() == expected);
        seen.lock(|s| {
            s.ended += 1;
            s.at = at;
            s.good += u32::from(good);
        });
    }

    #[task(priority = 1, shared = [spi1, seen, expected])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut spi1, mut seen, mut expected, .. } = cx.shared;
        let mut to_start = Spread::default();
        let mut to_result = Spread::default();
        let mut refused = 0u32;
        for i in 0..TRANSFERS {
            let b = i.to_le_bytes();
            let bytes = [b[0], b[1] ^ 0x5A, b[2], b[3] ^ 0xA5];
            expected.lock(|e| *e = bytes);
            let t0 = cycles();
            let started = spi1.lock(|spi| spi.start(&bytes));
            let t1 = cycles();
            if started.is_err() {
                refused += 1;
                continue;
            }
            let start = cycles();
            while seen.lock(|s| s.ended) < i + 1 - refused && cycles().wrapping_sub(start) < CPU_HZ / 10 {}
            let at = seen.lock(|s| s.at);
            to_start.add(t1.wrapping_sub(t0));
            to_result.add(at.wrapping_sub(t0));
        }
        let s = seen.lock(|s| *s);
        defmt::info!(
            "BASE latency over {} transfers of 4 bytes, cycles: submit to start min {} mean {} max {}; submit to result taken min {} mean {} max {}",
            TRANSFERS, to_start.min, to_start.mean(), to_start.max,
            to_result.min, to_result.mean(), to_result.max
        );
        let failed = verdict(
            "latency: every transfer came back with its bytes",
            refused == 0 && s.ended == TRANSFERS && s.good == TRANSFERS,
        );
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        cortex_m::asm::delay(CPU_HZ / 5);
        if failed == 0 { debug::exit(EXIT_SUCCESS) } else { debug::exit(EXIT_FAILURE) }
    }
}
