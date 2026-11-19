//! The SPI port against a real device: `ferroforge_stm32::spi_port` on the
//! STM32F4 Discovery (MB997), whose LIS3DSH accelerometer sits on SPI1 with
//! nothing to fit (UM1472 section 6.13 and table 7; the MB997 schematic).
//!
//! | Device | Chip select | Settings | What is on it |
//! | --- | --- | --- | --- |
//! | 0 | PE3 (`CS_I2C/SPI`) | mode 3, 6 MHz asked, 5.25 MHz | the LIS3DSH |
//! | 1 | PE7, a free I/O | mode 0, 1 MHz asked, 656 kHz | nothing: the checks read back the line |
//! | 2 | PE8, a free I/O | mode 0, 2 MHz asked, 1.3125 MHz | nothing; the stress task's |
//!
//! Routing (DS8626 table 9, alternate functions; RM0090 table 43, DMA2
//! requests): SPI1 on AF5, SCK PA5 (the LIS3DSH's SPC), MISO PA6 (its SDO),
//! MOSI PA7 (its SDI); TX on DMA2 stream 3 channel 3, RX on DMA2 stream 0
//! channel 3. The clock is the ST-LINK's 8 MHz MCO in bypass, as FerroHAL's
//! Discovery bench runs: 168 MHz core, APB2 at 84 MHz, SPI1's kernel.
//!
//! The LIS3DSH (its datasheet, DocID022405, section 5.2): SPC idles high and
//! data is captured on its rising edge, so mode 3, at most 10 MHz; the first
//! byte is the read bit (MSB) and a 7-bit address; with ADD_INC set, its
//! default (CTRL_REG6, table 66), a multi-byte access steps the address.
//! WHO_AM_I (0Fh) reads 3Fh (table 20); OFF_X, OFF_Y and OFF_Z (10h to 12h)
//! are read and write, default 0.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | `spi1`'s member, also pended by every handle | 4 |
//! | `SPI2`, `SPI3` | RTIC: `stress` (2), `checks` (1) | |
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
    dispatchers = [SPI2, SPI3],

    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin, Speed},
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
            Sysclk,
        },
        spi::{self, Wiring, MODE_0, MODE_3},
        time::Hertz,
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::spi_port::{self, Memory, Outcome, SpiHandle, SpiPort};
    use stm32f4_discovery_spi_device::{self as bench, Bus, Seg, View};

    /// 168 MHz from the ST-LINK's 8 MHz MCO in bypass (UM1472 section
    /// 6.3.1): 8 / 8 * 336 = 336 MHz VCO, / 2 = 168 MHz; APB2 / 2 = 84 MHz
    /// (DS8626 section 3.13).
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

    /// SPI1's kernel clock is 84 MHz (RM0090 section 28.5.1): 6 MHz gives
    /// 5.25 MHz, under the LIS3DSH's 10 MHz; 1 MHz gives 656 kHz; 2 MHz
    /// gives 1.3125 MHz.
    const CONFIGS: [spi::Config; 3] = [
        spi::Config::new(Hertz::mhz(6)).with_mode(MODE_3),
        spi::Config::new(Hertz::mhz(1)).with_mode(MODE_0),
        spi::Config::new(Hertz::mhz(2)).with_mode(MODE_0),
    ];
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[0]).is_ok());
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[1]).is_ok());
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &CONFIGS[2]).is_ok());

    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    type Spi1Wiring = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'A', 7>, Stream<2, 3>, Stream<2, 0>>;
    type Cs = (Pin<'E', 3, Output>, Pin<'E', 7, Output>, Pin<'E', 8, Output>);

    /// The LIS3DSH's registers used here, and its read bit.
    const WHO_AM_I: u8 = 0x0F;
    const ID: u8 = 0x3F;
    const OFF_X: u8 = 0x10;
    const READ: u8 = 0x80;

    const BUS: usize = 64;
    /// Device 0 four buffers of 16 bytes; device 1 four of 64; device 2,
    /// the stress task's, three of 16.
    static LENT: [(usize, usize); 11] = [
        (0, 16), (0, 16), (0, 16), (0, 16),
        (1, 64), (1, 64), (1, 64), (1, 64),
        (2, 16), (2, 16), (2, 16),
    ];
    const POOL: usize = 4 * 16 + 4 * 64 + 3 * 16;
    /// What is on each device's bus.
    const LIS3DSH: Bus = Bus::Register { register: WHO_AM_I, answer: ID };

    fn cycles() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    static FIRST: usize = 0;

    #[shared]
    struct Shared {
        port: SpiPort<Spi1Wiring, Cs>,
        side: (u32, u32, u32),
    }

    #[local]
    struct Local {
        handles: [SpiHandle; 2],
        side: SpiHandle,
    }

    #[init(local = [
        tx_buf: [u8; BUS] = [0; BUS],
        rx_buf: [u8; BUS] = [0; BUS],
        memory: Memory<5, 3> = Memory::new(),
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
        let mut gpioe = p.GPIOE.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let ctrl = &mut gpioa.ctrl;
        let mut sck = gpioa.p5.into_alternate::<5>(ctrl);
        sck.set_speed(ctrl, Speed::Medium);
        let miso = gpioa.p6.into_alternate::<5>(ctrl);
        let mut mosi = gpioa.p7.into_alternate::<5>(ctrl);
        mosi.set_speed(ctrl, Speed::Medium);
        let cs = (
            gpioe.p3.into_push_pull_output(&mut gpioe.ctrl),
            gpioe.p7.into_push_pull_output(&mut gpioe.ctrl),
            gpioe.p8.into_push_pull_output(&mut gpioe.ctrl),
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
        let first = core::hint::black_box(FIRST);
        let (Some(a), Some(b), Some(c)) =
            (handles.take(first), handles.take((first + 1) % 3), handles.take((first + 2) % 3))
        else {
            defmt::panic!("a handle was missing");
        };
        defmt::info!("spi_port on the STM32F4 Discovery's LIS3DSH, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (Shared { port: built.port, side: (0, 0, 0) }, Local { handles: [a, b], side: c })
    }

    #[group(from = ferroforge_stm32::spi_port, shared = [port = port])]
    mod spi1 {
        #[task(binds = DMA2_STREAM0, priority = 4)]
        fn on_spi_rx_dma;
    }

    /// One register read from the LIS3DSH: the address with the read bit,
    /// then `N` bytes kept.
    async fn read<const N: usize>(h: &mut SpiHandle, register: u8) -> Option<[u8; N]> {
        bench::send(h, 0, &[Seg::W(&[READ | register]), Seg::R(N)]).ok()?;
        loop {
            if let Some(t) = h.take() {
                let mut out = [0u8; N];
                let got = t.read(0);
                if t.outcome != Outcome::Done || got.len() != N {
                    return None;
                }
                out.copy_from_slice(got);
                return Some(out);
            }
            h.wait().await;
        }
    }

    /// One register write: the address, then the bytes.
    async fn write(h: &mut SpiHandle, register: u8, bytes: &[u8]) -> bool {
        let mut frame = [0u8; 4];
        let n = bytes.len().min(3);
        frame[0] = register;
        frame[1..=n].copy_from_slice(&bytes[..n]);
        bench::send(h, 0, &[Seg::W(&frame[..=n])]).is_ok()
            && bench::next(h, &[]).await.outcome == Outcome::Done
    }

    #[task(priority = 2, local = [side], shared = [side])]
    async fn stress(mut cx: stress::Context) {
        let r = bench::stress(cx.local.side, Bus::Blind, 500, 2).await;
        cx.shared.side.lock(|s| *s = r);
    }

    #[task(priority = 1, local = [handles], shared = [port, side])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut port, mut side, .. } = cx.shared;
        let [a, b] = cx.local.handles;
        let mut failed = 0u32;
        failed += bench::verdict("the member's vector is enabled [PORT-201]", port.vectors_enabled());
        failed += bench::verdict(
            "boot binding: every handle one type, bound to its task by a run-time choice [PORT-207]",
            a.device() == 0 && b.device() == 1,
        );
        let idle = (0..3).all(|d| port.selected(d) == Some(false));
        failed += bench::verdict("every device starts deselected [PORT-202]", idle);

        let mut ids = 0u32;
        for _ in 0..100 {
            if read::<1>(a, WHO_AM_I).await == Some([ID]) {
                ids += 1;
            }
        }
        defmt::info!("WHO_AM_I: {} of 100 reads gave 0x3F", ids);
        failed += bench::verdict(
            "a real device: WHO_AM_I reads 0x3F, the address part written, the read part kept [PORT-201, PORT-209]",
            ids == 100,
        );

        // OFF_X to OFF_Z written in one message and read back in another.
        let mut back = 0u32;
        for i in 0..100u32 {
            let pattern = bench::tagged(i);
            let wrote = write(a, OFF_X, &pattern[..3]).await;
            if wrote && read::<3>(a, OFF_X).await.is_some_and(|r| r == pattern[..3]) {
                back += 1;
            }
        }
        defmt::info!("OFF_X to OFF_Z: {} of 100 three-byte patterns read back", back);
        failed += bench::verdict(
            "a real device: three registers written in one message read back in another [PORT-209]",
            back == 100,
        );

        // Four messages queued at once, priorities against their order:
        // write 0x33, read, write 0x44, read. The device sees them as
        // submitted, so each read gives the write before it.
        let mut sent = true;
        for (p, value) in [(1u8, 0x33u8), (5, 0x44)] {
            sent &= bench::send(a, p, &[Seg::W(&[OFF_X, value])]).is_ok();
            sent &= bench::send(a, 9, &[Seg::W(&[READ | OFF_X]), Seg::R(1)]).is_ok();
        }
        let mut reads = [0u8; 2];
        let mut k = 0;
        let mut got_all = true;
        for _ in 0..4 {
            let start = cycles();
            loop {
                if let Some(t) = a.take() {
                    got_all &= t.outcome == Outcome::Done;
                    if t.parts() == 1 && k < 2 {
                        reads[k] = t.read(0).first().copied().unwrap_or(0);
                        k += 1;
                    }
                    break;
                }
                if cycles().wrapping_sub(start) > CPU_HZ / 10 {
                    got_all = false;
                    break;
                }
            }
        }
        defmt::info!("order on a real device: the reads gave {:#04x} and {:#04x}", reads[0], reads[1]);
        failed += bench::verdict(
            "a real device: one device's messages reach it as submitted, priority not applied [PORT-203]",
            sent && got_all && reads == [0x33, 0x44],
        );

        // Alternate the LIS3DSH (mode 3, 5.25 MHz) with device 1 (mode 0,
        // 656 kHz): a wrong mode or rate left in force garbles the reads.
        let mut ok = 0u32;
        for i in 0..100u32 {
            let tag = bench::tagged(i);
            let other = bench::send(b, 0, &[Seg::W(&tag)]).is_ok()
                && bench::next(b, &[]).await.outcome == Outcome::Done;
            if other && read::<1>(a, WHO_AM_I).await == Some([ID]) {
                ok += 1;
            }
        }
        let health = port.health();
        defmt::info!(
            "switch: {} of 100 alternations read 0x3F; {} switches, longest {} cycles; SCK device 0 {} Hz, device 1 {} Hz",
            ok,
            health.switches,
            health.longest_switch,
            port.frequency(0),
            port.frequency(1)
        );
        failed += bench::verdict(
            "a real device: its own settings in force after every switch from another device [PORT-208]",
            ok == 100 && health.switches >= 199,
        );

        failed += bench::latency_probe(a, LIS3DSH, cycles, 1000).await;
        failed += bench::chip_select(b, &mut port).await;
        failed += bench::full(b, Bus::Blind, cycles).await;
        // Low and high on device 1, the mediums WHO_AM_I reads on the
        // LIS3DSH: the high overtakes another device's queued messages.
        failed += bench::priority(&mut [&mut *a, &mut *b], &[LIS3DSH, Bus::Blind], [1, 0, 0, 1], cycles, true);
        failed += bench::cancel(b, Bus::Blind, a, LIS3DSH, &mut port, cycles, 2).await;
        failed += bench::isolation(b, Bus::Blind, a, LIS3DSH, cycles).await;

        // The stress task submits to device 2 from priority 2 while this
        // task reads the LIS3DSH from priority 1.
        stress::spawn().ok();
        let mine = bench::stress(a, LIS3DSH, 500, 1).await;
        let start = cycles();
        while side.lock(|s| s.0 < 500) && cycles().wrapping_sub(start) < CPU_HZ * 4 {
            cortex_m::asm::delay(CPU_HZ / 1000);
        }
        let s = side.lock(|s| *s);
        defmt::info!(
            "concurrent: checker {} done {} wrong {} retried; priority 2 task {} done {} wrong {} retried",
            mine.0, mine.1, mine.2, s.0, s.1, s.2
        );
        failed += bench::verdict(
            "concurrent: two tasks at two priorities, every message back done with its data [PORT-201]",
            mine.0 == 500 && mine.1 == 0 && s.0 == 500 && s.1 == 0,
        );
        failed += bench::verdict(
            "the LIS3DSH back to its offsets at reset",
            write(a, OFF_X, &[0, 0, 0]).await,
        );
        failed += bench::closing(&mut port);
        finish(failed);
    }

    fn finish(failed: u32) -> ! {
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        cortex_m::asm::delay(CPU_HZ / 5);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS)
        } else {
            debug::exit(EXIT_FAILURE)
        }
        loop {}
    }
}
