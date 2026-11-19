//! The ADC port's checks on the STM32F4 Discovery (MB997, STM32F407VGT6): a
//! hand-written `ferroforge::app!` that selects
//! `ferroforge_stm32::adc_event_port` once and converts a pin it drives, and
//! the internal pair, over DMA.
//!
//! One jumper on the P1 header, between pins UM1472 table 7 lists as free
//! I/O, side by side so a cap fits:
//!
//! | Link | From | To | Use |
//! | --- | --- | --- | --- |
//! | A1 | P1-14 (PA2) | P1-13 (PA3) | a GPIO output driving ADC1 channel 3 |
//!
//! PA3 is analog, ADC123_IN3 (DS8626 table 7); the sequence is PA3, VREFINT,
//! then the temperature sensor, on DMA2 stream 0, channel 0 (RM0090 table
//! 44). PA2 is a push-pull output.
//!
//! The clock is FerroHAL's Discovery bench's: the ST-LINK's 8 MHz MCO on PH0
//! in bypass (UM1472 section 7.11.1), 168 MHz core. The board feeds VDD from
//! its 3 V rail (UM1472 section 7.4), so VDDA is checked against 3 V less
//! and more 10 %.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0` | `adc1`'s member: an end, or a kick | 3 |
//! | `SPI2` | RTIC, dispatching `sampled` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |
//! | `SysTick` | the monotonic, at 1 kHz | 0, above every vector here |
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

    use rtic_monotonics::systick::prelude::*;

    // The monotonic the member stamps each start with: 1 kHz, so a stamp is
    // good to a millisecond.
    systick_monotonic!(Mono, 1000);
    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::adc_event_port::{self, AdcPort, AdcScan, Channel, Kick, KickMemory, Sequence};
    use ferrohal_stm32::{
        adc,
        chip::ADC1,
        dma::Stream,
        gpio::{Output, Pin},
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
            Sysclk,
        },
        time::Hertz,
        resource::Vectors,
    };

    /// 168 MHz, the die's ceiling (DS8626, Features), as FerroHAL's
    /// Discovery bench: 8 MHz / 8 * 336 = 336 MHz VCO, / 2 = 168 MHz; APB1 at
    /// 42 MHz and APB2 at 84 MHz (RM0090 section 6.2).
    const CLOCK_CONFIG: Config = Config {
        hse: Some(Hse {
            freq: Hertz::mhz(8),
            mode: HseMode::Bypass,
        }),
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
    const SYSCLK_HZ: u32 = 168_000_000;
    /// CPU cycles in a millisecond, for the busy waits.
    const MS: u32 = 168_000;
    /// VDDA as the internal-pair check accepts it, in millivolts.
    const VDDA_MV: core::ops::RangeInclusive<u32> = 2_700..=3_300;

    type Port = AdcPort<AdcScan<ADC1, Stream<2, 0>>>;
    type Drive = Pin<'A', 2, Output>;
    /// Channels in the sequence: the driven pin, VREFINT, temperature.
    const CHANNELS: usize = 3;
    /// Sequences run one kick at a time, the drive alternating low and high.
    const SEQUENCES: u32 = 200;
    /// The monotonic's tick, the stamps' resolution, in microseconds.
    const RESOLUTION_US: u64 = 1_000;

    /// What the firmware's task has seen.
    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub ended: u32,
        pub completed: u32,
        pub last: Option<Sequence>,
        pub missed: u32,
        pub late_kicks: u32,
        /// Sequences stamped later than the task that took them ran.
        pub stamped_late: u32,
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
        vectors: Vectors,
    }

    #[init(local = [
        samples: [u16; CHANNELS] = [0; CHANNELS],
        memory: KickMemory = KickMemory::new(),
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
        let mut gpioa = p.GPIOA.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let mut drive = gpioa.p2.into_push_pull_output(&mut gpioa.ctrl);
        drive.set_low();
        let analog = gpioa.p3.into_analog(&mut gpioa.ctrl);
        let channels: [Channel<ADC1>; CHANNELS] =
            [Channel::of(&analog), Channel::vrefint(), Channel::temperature()];
        let built = adc_event_port::new(
            &mut rcc,
            p.ADC1,
            dma2.s0,
            &adc_event_port::Config::slow_source(),
            &channels,
            cx.local.samples,
            cx.local.memory,
            adc1::pend,
        )
        .unwrap_or_else(|refused| defmt::panic!("ADC1: {}", refused.error));

        defmt::info!("adc_event_port on the STM32F4 Discovery, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared { log: Log::default() },
            Local {
                drive,
                full: built.port.max_sample(),
                vectors: built.port.vectors(),
                adc1: built.port,
                kick: built.kick,
            },
        )
    }

    #[group(from = ferroforge_stm32::adc_event_port, local = [port = adc1], spawn = [sequence = sampled])]
    mod adc1 {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn on_adc_dma;
    }

    /// The firmware's task: names no scan and takes no lock but its own.
    #[task(priority = 2, shared = [log])]
    async fn sampled(mut cx: sampled::Context, result: Sequence) {
        let ran_us = now_us();
        cx.shared.log.lock(|log| {
            log.ended += 1;
            log.completed += u32::from(result.outcome.is_ok());
            log.missed += result.missed;
            log.late_kicks += result.late_kicks;
            log.stamped_late += u32::from(result.at_us > ran_us);
            log.last = Some(result);
        });
    }

    /// Drives the pin, kicks the sequences and judges what the port made of
    /// them.
    #[task(priority = 1, shared = [log], local = [drive, kick, full, vectors])]
    async fn checks(cx: checks::Context) {
        let mut log = cx.shared.log;
        let drive = cx.local.drive;
        let kick = *cx.local.kick;
        let full = u32::from(*cx.local.full);
        let mut failed = 0u32;

        failed += verdict(
            "the member's vector is enabled [PORT-402, PORT-403]",
            cx.local.vectors.verify().is_ok(),
        );

        // One kick at a time, each awaited: every sequence carries its own
        // samples, stamped at its start.
        let (mut short, mut wrong_rail, mut wrong_internal, mut unstamped) = (0u32, 0u32, 0u32, 0u32);
        for index in 0..SEQUENCES {
            let high = index % 2 == 1;
            if high {
                drive.set_high();
            } else {
                drive.set_low();
            }
            wait(MS);
            let before = log.lock(|log| log.ended);
            let kicked_us = now_us();
            kick.kick();
            await_ended(&mut log, before + 1);
            let Some(sequence) = log.lock(|log| log.last) else {
                short += 1;
                continue;
            };
            if sequence.at_us < kicked_us || sequence.at_us > kicked_us + RESOLUTION_US {
                unstamped += 1;
            }
            let [pin, vrefint, temperature] = sequence.samples() else {
                short += 1;
                continue;
            };
            wrong_rail += u32::from(!on_rail(*pin, full, high));
            // VREFINT against its factory calibration gives VDDA; the sensor
            // gives the die's temperature in tenths of a degree.
            let vdda = adc::vdda_from_vrefint(*vrefint).unwrap_or(0);
            let die = adc::temperature_from_samples(*temperature, vdda).unwrap_or(i32::MIN);
            wrong_internal += u32::from(!VDDA_MV.contains(&vdda) || !(0..=850).contains(&die));
        }
        let seen = log.lock(|log| *log);
        defmt::info!(
            "{} kicks: {} ended, {} completed, {} short, {} off their rail, {} internal out of range, {} stamped outside their kick's tick, {} stamped after their task ran; {} missed, {} late kicks",
            SEQUENCES, seen.ended, seen.completed, short, wrong_rail, wrong_internal,
            unstamped, seen.stamped_late, seen.missed, seen.late_kicks
        );
        failed += verdict(
            "every kick started one sequence, and its end reached the firmware's task, completed, nothing missed or late [PORT-402, PORT-403, PORT-404]",
            seen.ended == SEQUENCES && seen.completed == SEQUENCES && short == 0
                && seen.missed == 0 && seen.late_kicks == 0,
        );
        failed += verdict(
            "each sequence carried its own samples: the pin followed its drive and the internal pair read in range, in channel order [PORT-403]",
            wrong_rail == 0 && wrong_internal == 0,
        );
        failed += verdict(
            "each sequence was stamped within a monotonic tick of its kick, no later than its task ran [PORT-403]",
            unstamped == 0 && seen.stamped_late == 0,
        );

        // A second sequence ends before the task has taken the first: the
        // first keeps its own (low) samples, and the second is counted on
        // the next sequence the task takes. The lock raises this task to the
        // task's priority, which the member's vector is above.
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
        let first_low = first
            .last
            .is_some_and(|s| s.samples().first().is_some_and(|&pin| on_rail(pin, full, false)));
        drive.set_low();
        wait(MS);
        kick.kick();
        await_ended(&mut log, first.ended + 1);
        let next = log.lock(|log| *log);
        let carried = next.last.map_or(0, |s| s.missed);
        defmt::info!(
            "two ends before the task ran: it took {}, with its own low samples: {}; the next carried {} missed",
            first.ended - before.ended, first_low, carried
        );
        failed += verdict(
            "a sequence keeps its own samples when a later one ends before the task took it [PORT-403]",
            first.ended - before.ended == 1 && first_low,
        );
        failed += verdict(
            "a sequence that found the task queued is counted on the next it takes [PORT-404]",
            carried == 1,
        );

        // A late kick: a second kick while the sequence runs starts nothing,
        // and is counted on that sequence.
        let before = log.lock(|log| *log);
        kick.kick();
        wait(MS / 50);
        kick.kick();
        await_ended(&mut log, before.ended + 1);
        wait(5 * MS);
        let after = log.lock(|log| *log);
        let late = after.last.map_or(0, |s| s.late_kicks);
        defmt::info!(
            "two kicks 20 us apart: {} ended, the result carrying {} late",
            after.ended - before.ended, late
        );
        failed += verdict(
            "a kick while a sequence runs starts nothing and is counted [PORT-402]",
            after.ended - before.ended == 1 && late == 1,
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

    /// Whether a sample reads within 3 parts in 100 of the rail driven.
    fn on_rail(sample: u16, full: u32, high: bool) -> bool {
        let sample = u32::from(sample);
        if high {
            sample * 100 >= full * 97
        } else {
            sample * 100 <= full * 3
        }
    }

    /// Spins until the task has taken `ended` sequences, for at most 10 ms.
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

    /// The monotonic's time, as the member stamps it.
    fn now_us() -> u64 {
        Mono::now().duration_since_epoch().to_micros()
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
