//! PROTOTYPE, variant S: `spi_single_port` on SPI1 of the Nucleo-F401RE.
//! The consumer owns one `SpiHandle` and names no SPI, wiring or lock.
//!
//! Routing as `nucleo-f401re-spi-loopback`: SPI1 on AF5, SCK PA5, MISO PA6,
//! MOSI PA7 over FerroHAL's link S1 (DS10086 table 9; UM1724 table 29); TX
//! on DMA2 stream 3, RX on DMA2 stream 0, channel 3 (RM0368 table 29). Chip
//! select PA4, nothing on it: the checks read back the level the port drives.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | the port's member, also pended by the handle | 3 |
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
        rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
        spi::{self, Wiring},
        time::Hertz,
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::spi_queue::{self, SingleMemory, Port, SpiHandle};
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

    /// SPI1's slowest SCK, 84 MHz / 256 = 328 kHz (RM0368 section 20.5.1).
    const BUS: spi::Config = spi::Config::new(Hertz::khz(400));
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &BUS).is_ok());

    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    type Spi1Wiring = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'A', 7>, Stream<2, 3>, Stream<2, 0>>;
    type Cs = Pin<'A', 4, Output>;

    /// Result slots (4 hold 3): the slot holds one, one runs, one spare.
    const RESULTS: usize = 4;

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    #[shared]
    struct Shared {
        port: Port<Spi1Wiring, (Cs,)>,
    }

    #[local]
    struct Local {
        handle: SpiHandle,
    }

    #[init(local = [
        tx_buf: [u8; 64] = [0; 64],
        rx_buf: [u8; 64] = [0; 64],
        memory: SingleMemory<RESULTS> = SingleMemory::new(),
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
        let mut sck = gpioa.p5.into_alternate::<5>(ctrl);
        sck.set_speed(ctrl, Speed::Medium);
        let miso = gpioa.p6.into_alternate::<5>(ctrl);
        let mut mosi = gpioa.p7.into_alternate::<5>(ctrl);
        mosi.set_speed(ctrl, Speed::Medium);
        let cs = gpioa.p4.into_push_pull_output(ctrl);
        let local = cx.local;
        let wiring = Wiring { spi: p.SPI1, sck, miso, mosi, tx_dma: dma2.s3, rx_dma: dma2.s0 };
        let Ok(built) = spi_queue::new_single(
            &mut rcc, wiring, cs, &BUS, local.tx_buf, local.rx_buf, local.memory,
            spi1::kick, cycles,
        ) else {
            defmt::panic!("SPI1 refused");
        };
        let (port, handle) = (built.port, built.handle);
        defmt::info!("S on the Nucleo-F401RE, SCK {} Hz", port.stats().sck[0]);
        checks::spawn().ok();
        (Shared { port }, Local { handle })
    }

    #[group(from = ferroforge_stm32::spi_single_port, shared = [port = port])]
    mod spi1 {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn s_on_spi_rx_dma;
    }

    #[task(priority = 1, local = [handle], shared = [port])]
    async fn checks(cx: checks::Context) {
        let mut port = cx.shared.port;
        let h = cx.local.handle;
        let mut failed = 0u32;
        failed += bench::latency("S", h, cycles, 1000).await;
        failed += bench::multipart(h).await;
        failed += bench::chip_select(h, &mut port).await;
        failed += bench::full(h, cycles, 1).await;
        failed += bench::cancel(h, None, &mut port, cycles, 1).await;
        failed += bench::closing("S", &mut port);
        finish(failed);
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
