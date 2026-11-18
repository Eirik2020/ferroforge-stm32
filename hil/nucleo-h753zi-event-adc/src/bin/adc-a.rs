//! PROTOTYPE, A: today's `adc_scan` on the Nucleo-H753ZI with the member's
//! marks, beside P (`adc-p`). The pin comes over link Z10 (`src/lib.rs`).
//! The scan is shared under RTIC's lock: the firmware's task locks it to
//! start, the member spawns `done(result)`, and the task locks it again to
//! read the samples.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | `adc1`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `adc1_done` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::{adc_scan::{self, AdcScan, Channel, Error}, probe};
    use ferrohal_stm32::{adc, chip::ADC3, dma::Stream, gpio::{Output, Pin}};
    use nucleo_h753zi_event_adc::{self as bench, BOARD, CLOCK_CONFIG, CLOCKS, MS, Stat, cycles, verdict, wait};

    type Scan = AdcScan<ADC3, Stream<2, 0>>;
    type Drive = Pin<'F', 7, Output>;
    const CHANNELS: usize = 3;
    const SEQUENCES: u32 = 200;

    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub ended: u32,
        pub completed: u32,
        pub samples: [u16; CHANNELS],
        /// From the member's entry at the end to this task's first line.
        pub to_task: Stat,
        /// How long this task holds the scan's lock to read the samples.
        pub lock_hold: Stat,
    }

    #[shared]
    struct Shared {
        adc1: Scan,
        log: Log,
    }

    #[local]
    struct Local {
        drive: Drive,
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
        let mut gpiof = p.GPIOF.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let mut drive = gpiof.p7.into_push_pull_output(&mut gpiof.ctrl);
        drive.set_low();
        let mut sense = gpiof.p6.into_input(&mut gpiof.ctrl);
        bench::check_link("Z10 PF7 to PF6", &mut drive, &mut sense);
        let analog = sense.into_analog(&mut gpiof.ctrl);
        let channels: [Channel<ADC3>; CHANNELS] =
            [Channel::of(&analog), Channel::vrefint(), Channel::temperature()];
        let adc1 = adc_scan::new(
            &mut rcc,
            p.ADC3,
            dma2.s0,
            &adc_scan::Config::slow_source(),
            &channels,
            cx.local.samples,
        )
        .unwrap_or_else(|refused| defmt::panic!("ADC3: {}", refused.error));
        defmt::info!("A: adc_scan on the {}, clocks: {}", BOARD, CLOCKS);
        checks::spawn().ok();
        (Shared { adc1: adc1.scan, log: Log::default() }, Local { drive })
    }

    #[group(from = ferroforge_stm32::adc_scan, shared = [scan = adc1], spawn = [done = adc1_done])]
    mod adc1 {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn on_adc_dma;
    }

    /// The firmware's task, as today's consumer writes it.
    #[task(priority = 2, shared = [adc1, log])]
    async fn adc1_done(cx: adc1_done::Context, result: Result<(), Error>) {
        let ran = cycles();
        let entered = probe::SCAN.last_served();
        let adc1_done::SharedResources { mut adc1, mut log, .. } = cx.shared;
        let mut samples = [0u16; CHANNELS];
        let locked = cycles();
        adc1.lock(|scan| {
            for (to, from) in samples.iter_mut().zip(scan.samples()) {
                *to = *from;
            }
        });
        let held = cycles().wrapping_sub(locked);
        log.lock(|log| {
            log.ended += 1;
            log.to_task.add(ran.wrapping_sub(entered));
            log.lock_hold.add(held);
            if result.is_ok() {
                log.completed += 1;
                log.samples = samples;
            }
        });
    }

    #[task(priority = 1, shared = [adc1, log], local = [drive])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut adc1, mut log, .. } = cx.shared;
        let drive = cx.local.drive;
        let mut failed = bench::link_verdict();
        failed += verdict("A: the member's vector is enabled", adc1.lock(|scan| scan.vectors().verify()).is_ok());

        // The existing bench's checks, with the start's cost.
        probe::SCAN.clear();
        let full = u32::from(adc1.lock(|scan| scan.max_sample()));
        let (mut refused, mut wrong_rail, mut wrong_internal) = (0u32, 0u32, 0u32);
        let mut start_cost = Stat::default();
        let mut conversion = Stat::default();
        for index in 0..SEQUENCES {
            let high = index % 2 == 1;
            if high { drive.set_high() } else { drive.set_low() }
            wait(MS);
            let before = log.lock(|log| log.ended);
            let kicked = cycles();
            let started = adc1.lock(|scan| scan.start());
            let after_start = cycles();
            if started.is_err() {
                refused += 1;
                continue;
            }
            start_cost.add(after_start.wrapping_sub(kicked));
            await_ended(&mut log, before + 1);
            conversion.add(probe::SCAN.last_served().wrapping_sub(after_start));
            let [pin, vrefint, temperature] = log.lock(|log| log.samples);
            let pin = u32::from(pin);
            wrong_rail += u32::from(!if high { pin * 100 >= full * 97 } else { pin * 100 <= full * 3 });
            let vdda = adc::vdda_from_vrefint(vrefint).unwrap_or(0);
            let die = adc::temperature_from_samples(temperature, vdda).unwrap_or(i32::MIN);
            wrong_internal += u32::from(!bench::VDDA_MV.contains(&vdda) || !(0..=850).contains(&die));
            if index < 2 {
                defmt::info!(
                    "A: raw, drive high {}: pin {} of {}, VREFINT {}, temperature {}; VDDA {} mV, die {} dC",
                    high, pin, full, vrefint, temperature, vdda, die
                );
            }
        }
        let seen = log.lock(|log| *log);
        defmt::info!(
            "A: {} sequences: {} ended, {} completed, {} refused, {} off their rail, {} internal out of range",
            SEQUENCES, seen.ended, seen.completed, refused, wrong_rail, wrong_internal
        );
        start_cost.show("A: kick to start (lock, start)");
        conversion.show("A: start returned to the end's member entry");
        bench::show("A: member run at the end (entry to return)", bench::stable(&probe::SCAN.run));
        seen.to_task.show("A: end to task (member entry to task's first line)");
        seen.lock_hold.show("A: the task's lock hold to read the samples");
        failed += verdict("A: every sequence's end reached the task, completed", refused == 0 && seen.ended == SEQUENCES && seen.completed == SEQUENCES);
        failed += verdict("A: the driven pin's sample followed its drive", wrong_rail == 0);
        failed += verdict("A: the internal pair read in range", wrong_internal == 0);

        // The lost-result window: a second start before the task has read
        // the first. The task is held off by locking its log; the first
        // sequence converts the pin low, the second high.
        let before = log.lock(|log| *log);
        let second_started = log.lock(|_| {
            drive.set_low();
            wait(MS);
            if adc1.lock(|scan| scan.start()).is_err() {
                return false;
            }
            wait(2 * MS);
            drive.set_high();
            wait(MS);
            let ok = adc1.lock(|scan| scan.start()).is_ok();
            wait(2 * MS);
            ok
        });
        wait(2 * MS);
        let after = log.lock(|log| *log);
        let read_high = u32::from(after.samples[0]) * 100 >= full * 97;
        defmt::info!(
            "A: lost result: two sequences ended (second start accepted: {}); the task ran {} time(s), and read the {} pin's samples as the first's result; nothing counted the other",
            second_started, after.ended - before.ended, if read_high { "second (high)" } else { "first (low)" }
        );
        failed += verdict("A: lost result shown: one task run for two ends", second_started && after.ended - before.ended == 1);
        drive.set_low();
        finish(failed);
    }

    /// Spins until the task has taken `ended` outcomes, for at most 10 ms.
    fn await_ended(log: &mut impl rtic::Mutex<T = Log>, ended: u32) {
        for _ in 0..1_000 {
            if log.lock(|log| log.ended) >= ended {
                return;
            }
            wait(MS / 100);
        }
    }

    fn finish(failed: u32) -> ! {
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        wait(200 * MS);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
        loop {}
    }
}
