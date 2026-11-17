//! The SPI port group on `spi_v4`: the F401 loopback's checks on the
//! Nucleo-H753ZI (MB1364), selecting `ferroforge_stm32::spi_port` once.
//!
//! The link, as FerroHAL's H753 bench fits it (its S1, on the Zio headers;
//! UM2407 tables 18 to 21):
//!
//! | Link | From | To | Port |
//! | --- | --- | --- | --- |
//! | S1 | CN9-2 (PD7) | CN7-12 (PA6) | SPI1 MOSI into its own MISO |
//!
//! Routing: SPI1 on AF5, SCK PA5, MISO PA6, MOSI PD7 (DS12117 table 9).
//! Through DMAMUX1 any DMA1 or DMA2 stream serves any SPI request
//! (RM0433 section 17.3): TX on DMA1 stream 5, RX on DMA1 stream 4. Chip
//! select is PD14, a plain push-pull output with nothing on it: the checks
//! read back the level the port drives.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA1_STREAM4` | `spi1`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `spi1_done` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |
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
            AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
        },
        spi::{self, Wiring},
        time::Hertz,
    };
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::spi_port::{self, Device, Error};

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

    /// SPI1's slowest SCK, its 100 MHz kernel clock (PLL1Q) / 256 = 391 kHz
    /// (RM0433 section 50.11.3), so a full frame lasts long enough to see chip
    /// select held.
    const BUS: spi::Config = spi::Config::new(Hertz::khz(400));
    const _: () = assert!(spi::check::<SPI1>(&CLOCKS, &BUS).is_ok());

    /// CPU cycles in a millisecond, for the busy waits.
    
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;

    /// SPI1 over S1: PD7 into PA6, DMA1 stream 5 transmitting, stream 4
    /// receiving.
    type Spi1Wiring = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'D', 7>, Stream<1, 5>, Stream<1, 4>>;
    type Cs = Pin<'D', 14, Output>;

    /// The bus's buffers, and so its longest transfer.
    const FRAME: usize = 64;
    /// Transfers run round the loop, of 1 to `FRAME` bytes.
    const TRANSFERS: u32 = 256;

    /// What the firmware's task has seen of the transfers that ended.
    #[derive(Clone, Copy, Default)]
    pub struct Tally {
        /// Outcomes passed on by the group.
        pub ended: u32,
        pub completed: u32,
        pub faults: u32,
        /// Completed transfers whose length was not the one started.
        pub wrong_length: u32,
        /// Completed transfers whose bytes were not the ones sent.
        pub wrong_bytes: u32,
        /// Outcomes that arrived with the device still selected.
        pub selected_at_end: u32,
    }

    /// The frame on the wire, for the firmware's task to compare against.
    #[derive(Clone, Copy)]
    pub struct Sent {
        pub bytes: [u8; FRAME],
        pub len: usize,
    }

    #[shared]
    struct Shared {
        spi1: Device<Spi1Wiring, Cs>,
        tally: Tally,
        sent: Sent,
    }

    #[local]
    struct Local {}

    #[init(local = [
        tx_buf: [u8; FRAME] = [0; FRAME],
        rx_buf: [u8; FRAME] = [0; FRAME],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpioa = p.GPIOA.split(&mut rcc);
        let mut gpiod = p.GPIOD.split(&mut rcc);
        let dma1 = p.DMA1.split(&mut rcc);
        let mut sck = gpioa.p5.into_alternate::<5>(&mut gpioa.ctrl);
        sck.set_speed(&mut gpioa.ctrl, Speed::Medium);
        let miso = gpioa.p6.into_alternate::<5>(&mut gpioa.ctrl);
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

        defmt::info!("spi_port loopback on the Nucleo-H753ZI, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared {
                spi1: spi1.spi,
                tally: Tally::default(),
                sent: Sent { bytes: [0; FRAME], len: 0 },
            },
            Local {},
        )
    }

    #[group(from = ferroforge_stm32::spi_port, shared = [spi = spi1], spawn = [done = spi1_done])]
    mod spi1 {
        #[task(binds = DMA1_STREAM4, priority = 3)]
        fn on_spi_rx_dma;
    }

    /// The firmware's task behind the port: first sees whether the device
    /// was released before it ran, then checks what came back.
    #[task(priority = 2, shared = [spi1, tally, sent])]
    async fn spi1_done(cx: spi1_done::Context, result: Result<usize, Error>) {
        let spi1_done::SharedResources { mut spi1, mut tally, mut sent, .. } = cx.shared;
        let sent = sent.lock(|sent| *sent);
        tally.lock(|tally| {
            spi1.lock(|spi| {
                tally.ended += 1;
                if !spi.chip_select().is_set_high() {
                    tally.selected_at_end += 1;
                }
                match result {
                    Ok(len) => {
                        tally.completed += 1;
                        if len != sent.len {
                            tally.wrong_length += 1;
                        } else if spi.received() != &sent.bytes[..sent.len] {
                            tally.wrong_bytes += 1;
                        }
                    }
                    Err(error) => {
                        defmt::warn!("transfer: {}", error);
                        tally.faults += 1;
                    }
                }
            })
        });
    }

    /// Runs the transfers and judges them.
    #[task(priority = 1, shared = [spi1, tally, sent])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut spi1, mut tally, mut sent, .. } = cx.shared;
        let mut failed = 0u32;

        let bound = spi1.lock(|spi| spi.vectors().verify()).is_ok();
        failed += verdict("the member's vector is enabled [PORT-201]", bound);
        let idle = spi1.lock(|spi| spi.chip_select().is_set_high());
        failed += verdict("the device starts deselected [PORT-202]", idle);

        // One transfer at a time, each waited for: lengths 1 to FRAME, the
        // bytes a counter that carries on from one to the next.
        let mut next = 0u8;
        let mut selected_while_running = 0u32;
        let mut refused = 0u32;
        let mut late = 0u32;
        for n in 0..TRANSFERS {
            let len = (n as usize % FRAME) + 1;
            let mut frame = Sent { bytes: [0; FRAME], len };
            for byte in &mut frame.bytes[..len] {
                *byte = next;
                next = next.wrapping_add(1);
            }
            sent.lock(|sent| *sent = frame);
            let started = spi1.lock(|spi| {
                let started = spi.start(&frame.bytes[..len]);
                // Read before the end can come: a full frame takes 1.3 ms.
                if started.is_ok() && len == FRAME && !spi.chip_select().is_set_high() {
                    selected_while_running += 1;
                }
                started
            });
            if started.is_err() {
                refused += 1;
                continue;
            }
            // A frame takes at most 1.3 ms; wait up to 10.
            let mut waited = 0;
            while tally.lock(|t| t.ended) < n + 1 - refused && waited < 100 {
                cortex_m::asm::delay(MS / 10);
                waited += 1;
            }
            if waited == 100 {
                late += 1;
            }
        }
        let t = tally.lock(|t| *t);
        defmt::info!(
            "{} transfers: {} ended, {} completed, {} faults, {} wrong length, {} wrong bytes, {} refused, {} late",
            TRANSFERS, t.ended, t.completed, t.faults, t.wrong_length, t.wrong_bytes, refused, late
        );
        failed += verdict("every transfer's end reached the firmware's task [PORT-201]",
            refused == 0 && late == 0 && t.ended == TRANSFERS);
        failed += verdict("every transfer completed with the bytes sent [PORT-201]",
            t.completed == TRANSFERS && t.faults == 0 && t.wrong_length == 0 && t.wrong_bytes == 0);
        failed += verdict("the device was selected while a transfer ran [PORT-202]",
            selected_while_running == TRANSFERS / FRAME as u32);
        failed += verdict("the device was released before the firmware's task ran [PORT-202]",
            t.selected_at_end == 0);

        // An abort mid-frame releases the device, passes no outcome on, and
        // leaves the bus ready.
        let mut frame = Sent { bytes: [0; FRAME], len: FRAME };
        for (i, byte) in frame.bytes.iter_mut().enumerate() {
            *byte = i as u8;
        }
        let (aborted_selected, released) = spi1.lock(|spi| {
            let _ = spi.start(&frame.bytes);
            let selected = !spi.chip_select().is_set_high();
            spi.abort();
            (selected, spi.chip_select().is_set_high() && !spi.is_busy())
        });
        cortex_m::asm::delay(5 * MS);
        let ended_after_abort = tally.lock(|t| t.ended);
        failed += verdict("an abort releases the device and ends the transfer [PORT-202]",
            aborted_selected && released);
        failed += verdict("an aborted transfer passes no outcome on [PORT-201]",
            ended_after_abort == TRANSFERS);
        sent.lock(|sent| *sent = frame);
        let restarted = spi1.lock(|spi| spi.start(&frame.bytes)).is_ok();
        cortex_m::asm::delay(5 * MS);
        let t = tally.lock(|t| *t);
        failed += verdict("the bus runs again after an abort [PORT-201]",
            restarted && t.ended == TRANSFERS + 1 && t.completed == TRANSFERS + 1
                && t.wrong_bytes == 0 && t.selected_at_end == 0);

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
