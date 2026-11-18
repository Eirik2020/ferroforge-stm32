//! PROTOTYPE, P: `adc_event_port` on the Nucleo-H753ZI, beside A
//! (`adc-a`). The member owns the scan; the checks task holds only the kick,
//! and the firmware's task receives each sequence by value. The pin comes
//! over link Z10 (`src/lib.rs`). With `ch16` the sequence is sixteen
//! channels.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | `adc1`'s member: an end, or a kick | 3 |
//! | `SPI2` | RTIC, dispatching `sampled` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |
//! | `SysTick` | the monotonic, at 1 kHz | as rtic-monotonics sets it |

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use rtic_monotonics::systick::prelude::*;

    systick_monotonic!(Mono, 1000);
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::{
        adc_event_port::{self, AdcPort, AdcScan, Channel, Kick, Sequence},
        probe,
    };
    use ferrohal_stm32::{adc, chip::ADC3, dma::Stream, gpio::{Output, Pin}};
    use nucleo_h753zi_event_adc::{self as bench, BOARD, CHANNELS, CLOCK_CONFIG, CLOCKS, MS, Stat, SYSCLK_HZ, cycles, verdict, wait};

    type Port = AdcPort<AdcScan<ADC3, Stream<2, 0>>>;
    type Drive = Pin<'F', 7, Output>;
    const SEQUENCES: u32 = 200;

    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub ended: u32,
        pub completed: u32,
        pub last: Option<Sequence>,
        pub missed: u32,
        pub late_kicks: u32,
        /// From the member's entry at the end to this task's first line.
        pub to_task: Stat,
    }

    #[shared]
    struct Shared {
        log: Log,
    }

    #[local]
    struct Local {
        drive: Drive,
        adc1: Port,
        kick: Kick,
        full: u16,
    }

    #[init(local = [
        samples: [u16; CHANNELS] = [0; CHANNELS],
        memory: adc_event_port::KickMemory = adc_event_port::KickMemory::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        Mono::start(core.SYST, SYSCLK_HZ);
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut gpiof = p.GPIOF.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let mut drive = gpiof.p7.into_push_pull_output(&mut gpiof.ctrl);
        drive.set_low();
        let analog = gpiof.p6.into_analog(&mut gpiof.ctrl);
        // The driven pin, VREFINT and temperature, then the pin again.
        let mut channels = [Channel::of(&analog); CHANNELS];
        channels[1] = Channel::vrefint();
        channels[2] = Channel::temperature();
        let built = adc_event_port::new(
            &mut rcc,
            p.ADC3,
            dma2.s0,
            &adc_event_port::Config::slow_source(),
            &channels,
            cx.local.samples,
            cx.local.memory,
            adc1::pend,
        )
        .unwrap_or_else(|refused| defmt::panic!("ADC3: {}", refused.error));
        defmt::info!("P: adc_event_port on the {}, {} channels, clocks: {}", BOARD, CHANNELS, CLOCKS);
        checks::spawn().ok();
        (Shared { log: Log::default() }, Local { drive, full: built.port.max_sample(), adc1: built.port, kick: built.kick })
    }

    #[group(from = ferroforge_stm32::adc_event_port, local = [port = adc1], spawn = [sequence = sampled])]
    mod adc1 {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn p_on_adc_dma;
    }

    /// The firmware's task: names no scan and takes no lock but its own.
    #[task(priority = 2, shared = [log])]
    async fn sampled(mut cx: sampled::Context, result: Sequence) {
        let ran = cycles();
        let entered = probe::ADC_PORT.last_served();
        cx.shared.log.lock(|log| {
            log.ended += 1;
            log.completed += u32::from(result.outcome.is_ok());
            log.missed += result.missed;
            log.late_kicks += result.late_kicks;
            log.to_task.add(ran.wrapping_sub(entered));
            log.last = Some(result);
        });
    }

    #[task(priority = 1, shared = [log], local = [drive, kick, full])]
    async fn checks(cx: checks::Context) {
        let mut log = cx.shared.log;
        let drive = cx.local.drive;
        let kick = *cx.local.kick;
        let mut failed = 0u32;
        let full = u32::from(*cx.local.full);
        defmt::info!("P: a sequence moves {} bytes by value", core::mem::size_of::<Sequence>());

        probe::ADC_PORT.clear();
        let (mut wrong_rail, mut wrong_internal, mut short) = (0u32, 0u32, 0u32);
        for index in 0..SEQUENCES {
            let high = index % 2 == 1;
            if high { drive.set_high() } else { drive.set_low() }
            wait(MS);
            let before = log.lock(|log| log.ended);
            kick.kick();
            await_ended(&mut log, before + 1);
            let Some(sequence) = log.lock(|log| log.last) else { short += 1; continue };
            let samples = sequence.samples();
            if samples.len() != CHANNELS {
                short += 1;
                continue;
            }
            let pins = samples.iter().enumerate().filter(|(i, _)| *i == 0 || *i >= 3);
            for (_, &pin) in pins {
                let pin = u32::from(pin);
                wrong_rail += u32::from(!if high { pin * 100 >= full * 97 } else { pin * 100 <= full * 3 });
            }
            let vdda = adc::vdda_from_vrefint(samples[1]).unwrap_or(0);
            let die = adc::temperature_from_samples(samples[2], vdda).unwrap_or(i32::MIN);
            wrong_internal += u32::from(!(3_000..=3_600).contains(&vdda) || !(0..=850).contains(&die));
        }
        let seen = log.lock(|log| *log);
        defmt::info!(
            "P: {} sequences: {} ended, {} completed, {} short, {} pin samples off their rail, {} internal out of range, {} missed, {} late kicks",
            SEQUENCES, seen.ended, seen.completed, short, wrong_rail, wrong_internal, seen.missed, seen.late_kicks
        );
        bench::show("P: kick to start (kick, pend, member, start)", bench::stable(&probe::ADC_PORT.kick_to_start));
        bench::show("P: start to the end's member entry", bench::stable(&probe::ADC_PORT.conversion));
        bench::show("P: member run at the end (copy, spawn)", bench::stable(&probe::ADC_PORT.run));
        seen.to_task.show("P: end to task (member entry to task's first line)");
        failed += verdict("P: every sequence's end reached the task, completed", seen.ended == SEQUENCES && seen.completed == SEQUENCES && short == 0);
        failed += verdict("P: the driven pin's samples followed its drive", wrong_rail == 0);
        failed += verdict("P: the internal pair read in range", wrong_internal == 0);
        failed += verdict("P: nothing missed and no late kick, one kick at a time", seen.missed == 0 && seen.late_kicks == 0);

        // The lost-result window, as A's: a second sequence ends before the
        // task has taken the first.
        let before = log.lock(|log| *log);
        log.lock(|_| {
            drive.set_low();
            wait(MS);
            kick.kick();
            wait(2 * MS);
            drive.set_high();
            wait(MS);
            kick.kick();
            wait(2 * MS);
        });
        wait(2 * MS);
        let first = log.lock(|log| *log);
        let first_low = first.last.is_some_and(|s| s.samples().first().is_some_and(|&pin| u32::from(pin) * 100 <= full * 3));
        drive.set_low();
        wait(MS);
        kick.kick();
        await_ended(&mut log, first.ended + 1);
        let next = log.lock(|log| *log);
        let carried = next.last.map_or(0, |s| s.missed);
        defmt::info!(
            "P: lost result: the task took {} of two, with its own (low) samples: {}; the next carried {} missed",
            first.ended - before.ended, first_low, carried
        );
        failed += verdict(
            "P: a second end before the task took the first: the first keeps its samples, the second is counted",
            first.ended - before.ended == 1 && first_low && carried == 1,
        );

        // A late kick: a second kick while the sequence runs starts nothing.
        let before = log.lock(|log| *log);
        let starts = bench::stable(&probe::ADC_PORT.kick_to_start).count;
        kick.kick();
        wait(MS / 50);
        kick.kick();
        await_ended(&mut log, before.ended + 1);
        wait(2 * MS);
        let after = log.lock(|log| *log);
        let started = bench::stable(&probe::ADC_PORT.kick_to_start).count - starts;
        let late = after.last.map_or(0, |s| s.late_kicks);
        defmt::info!("P: late kick: two kicks, {} started, {} ended, the result carrying {} late", started, after.ended - before.ended, late);
        failed += verdict("P: a kick while a sequence runs starts nothing and is counted", started == 1 && after.ended - before.ended == 1 && late == 1);
        finish(failed);
    }

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
