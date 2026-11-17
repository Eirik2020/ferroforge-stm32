//! The ADC sequence group's reference: a hand-written `ferroforge::app!` on the
//! Nucleo-F401RE (MB1136) that selects `ferroforge_stm32::adc_scan` once and
//! converts a pin it drives, and the internal pair, over DMA.
//!
//! The link, as FerroHAL's F401 bench fits it (its S1; UM1724 table 29):
//!
//! | Link | From | To | Use |
//! | --- | --- | --- | --- |
//! | S1 | CN10-15 (PA7) | CN10-13 (PA6) | a GPIO output driving ADC1 channel 6 |
//!
//! PA6 is analog, ADC1's channel 6 (DS10086 table 9); the sequence is PA6,
//! VREFINT, then the temperature sensor (RM0368 section 11.9), on DMA2
//! stream 0, channel 0 (RM0368 table 29). PA7 is a push-pull output.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | `adc1`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `adc1_done` | 2 |
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

    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::adc_scan::{self, AdcScan, Channel, Error};
    use ferrohal_stm32::{
        adc,
        chip::ADC1,
        dma::Stream,
        gpio::{Output, Pin},
        rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
    };

    /// 84 MHz from HSI, the F401's ceiling (DS10086 section 3.13): 16 MHz /
    /// 16 * 336 = 336 MHz VCO, / 4 = 84 MHz.
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
    /// CPU cycles in a millisecond, for the busy waits.
    const MS: u32 = 84_000;

    type Scan = AdcScan<ADC1, Stream<2, 0>>;
    /// Channels in the sequence: the driven pin, VREFINT, temperature.
    const CHANNELS: usize = 3;
    /// Sequences run one at a time, the drive alternating low and high.
    const SEQUENCES: u32 = 200;

    /// What the firmware's task has seen.
    #[derive(Clone, Copy, Default)]
    pub struct Log {
        /// Sequences whose end reached the task, and those that completed.
        pub ended: u32,
        pub completed: u32,
        /// The last completed sequence's samples, in channel order.
        pub samples: [u16; CHANNELS],
    }

    #[shared]
    struct Shared {
        adc1: Scan,
        log: Log,
    }

    #[local]
    struct Local {
        drive: Pin<'A', 7, Output>,
    }

    #[init(local = [samples: [u16; CHANNELS] = [0; CHANNELS]])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpioa = p.GPIOA.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let mut drive = gpioa.p7.into_push_pull_output(&mut gpioa.ctrl);
        drive.set_low();
        let analog = gpioa.p6.into_analog(&mut gpioa.ctrl);
        let channels: [Channel<ADC1>; CHANNELS] =
            [Channel::of(&analog), Channel::vrefint(), Channel::temperature()];
        let adc1 = adc_scan::new(
            &mut rcc,
            p.ADC1,
            dma2.s0,
            &adc_scan::Config::slow_source(),
            &channels,
            cx.local.samples,
        )
        .unwrap_or_else(|refused| defmt::panic!("ADC1: {}", refused.error));

        defmt::info!("adc_scan on the Nucleo-F401RE, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (Shared { adc1: adc1.scan, log: Log::default() }, Local { drive })
    }

    #[group(from = ferroforge_stm32::adc_scan, shared = [scan = adc1], spawn = [done = adc1_done])]
    mod adc1 {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn on_adc_dma;
    }

    /// The firmware's task: takes each sequence's outcome and its samples.
    #[task(priority = 2, shared = [adc1, log])]
    async fn adc1_done(cx: adc1_done::Context, result: Result<(), Error>) {
        let adc1_done::SharedResources { mut adc1, mut log, .. } = cx.shared;
        let mut samples = [0u16; CHANNELS];
        adc1.lock(|scan| {
            for (to, from) in samples.iter_mut().zip(scan.samples()) {
                *to = *from;
            }
        });
        log.lock(|log| {
            log.ended += 1;
            if result.is_ok() {
                log.completed += 1;
                log.samples = samples;
            }
        });
    }

    /// Drives the pin, runs the sequences and judges what the group made of
    /// them.
    #[task(priority = 1, shared = [adc1, log], local = [drive])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut adc1, mut log, .. } = cx.shared;
        let drive = cx.local.drive;
        let mut failed = 0u32;

        failed += verdict(
            "the member's vector is enabled [PORT-401]",
            adc1.lock(|scan| scan.vectors().verify()).is_ok(),
        );

        let full = u32::from(adc1.lock(|scan| scan.max_sample()));
        let (mut refused, mut wrong_rail, mut wrong_internal) = (0u32, 0u32, 0u32);
        let (mut lowest_mv, mut highest_mv) = (u32::MAX, 0u32);
        let (mut coolest, mut warmest) = (i32::MAX, i32::MIN);
        for index in 0..SEQUENCES {
            let high = index % 2 == 1;
            if high {
                drive.set_high();
            } else {
                drive.set_low();
            }
            wait(MS);
            let before = log.lock(|log| log.ended);
            if adc1.lock(|scan| scan.start()).is_err() {
                refused += 1;
                continue;
            }
            await_ended(&mut log, before + 1);
            let [pin, vrefint, temperature] = log.lock(|log| log.samples);
            // A rail reads within 3 parts in 100 of its end of the scale.
            let pin = u32::from(pin);
            let on_rail = if high {
                pin * 100 >= full * 97
            } else {
                pin * 100 <= full * 3
            };
            wrong_rail += u32::from(!on_rail);
            // VREFINT against its factory calibration gives VDDA; the sensor
            // gives the die's temperature in tenths of a degree.
            let vdda = adc::vdda_from_vrefint(vrefint).unwrap_or(0);
            let die = adc::temperature_from_samples(temperature, vdda).unwrap_or(i32::MIN);
            lowest_mv = lowest_mv.min(vdda);
            highest_mv = highest_mv.max(vdda);
            coolest = coolest.min(die);
            warmest = warmest.max(die);
            wrong_internal += u32::from(!(3_000..=3_600).contains(&vdda) || !(0..=850).contains(&die));
        }
        let seen = log.lock(|log| *log);
        defmt::info!(
            "{} sequences: {} ended, {} completed, {} refused, {} off their rail, {} internal out of range; VDDA {} to {} mV, die {} to {} dC",
            SEQUENCES, seen.ended, seen.completed, refused, wrong_rail, wrong_internal,
            lowest_mv, highest_mv, coolest, warmest
        );
        failed += verdict(
            "every sequence's end reached the firmware's task, completed [PORT-401]",
            refused == 0 && seen.ended == SEQUENCES && seen.completed == SEQUENCES,
        );
        failed += verdict(
            "the driven pin's sample followed its drive in every sequence [PORT-401]",
            wrong_rail == 0,
        );
        failed += verdict(
            "the internal pair read in range, in their places in the sequence [PORT-401]",
            wrong_internal == 0,
        );

        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit ends
        // that, so give it the last lines first (about 200 ms).
        wait(200 * MS);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }

    /// Spins until the task has taken `ended` outcomes, for at most 10 ms.
    fn await_ended(log: &mut impl rtic::Mutex<T = Log>, ended: u32) {
        for _ in 0..100 {
            if log.lock(|log| log.ended) >= ended {
                return;
            }
            wait(MS / 10);
        }
    }

    /// Spins until `cycles` core cycles have passed. Interrupts still run.
    fn wait(cycles: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < cycles {}
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
