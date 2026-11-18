//! PROTOTYPE, TP: periodic sampling on the Nucleo-H753ZI. The tick (K,
//! TIM4 at the build's `TICK_HZ`) kicks the ADC port (P) from its task; the
//! pin comes over link Z10 (`src/lib.rs`). Measures how evenly sequences
//! start, under the three loads, and the highest rate before a late kick.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `TIM5` | the higher-priority load (50 us at 1.25 kHz), or the guard | 4 |
//! | `TIM4` | `control`'s member | 3 |
//! | `DMA2_STREAM0` | `adc1`'s member: an end, or a kick | 3 |
//! | `SPI2` | RTIC, dispatching `sample_tick` and `sampled` | 2 |
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
    use core::sync::atomic::Ordering::Relaxed;
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::{
        adc_event_port::{self, AdcPort, AdcScan, Channel, Kick, Sequence},
        probe::{self, Read},
        timer_tick,
    };
    use ferrohal_stm32::{chip::{ADC3, TIM4, TIM5}, device::Interrupt, dma::Stream, timer::Tick};
    use nucleo_h753zi_event_adc::{
        self as bench, BOARD, CHANNELS, CLOCK_CONFIG, CLOCKS, GUARD_ARMED, GUARD_FIRED, GUARD_PASSES, HIGH,
        HIGH_LOAD, Load, MS, PERIOD, SWEEP, SYSCLK_HZ, TICK, TICK_HZ, stable, verdict, wait,
    };

    type Port = AdcPort<AdcScan<ADC3, Stream<2, 0>>>;
    /// Sequences in each measured window.
    const WINDOW: u32 = 10_000;

    #[derive(Clone, Copy, Default)]
    pub struct Snap {
        pub start_period: Read,
        pub kick_to_start: Read,
        pub ended: u32,
        pub completed: u32,
        pub missed: u32,
        pub late_kicks: u32,
        pub tick_missed: u32,
    }

    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub ended: u32,
        pub completed: u32,
        pub missed: u32,
        pub late_kicks: u32,
        pub tick_missed: u32,
        pub target: u32,
        pub snap: Option<Snap>,
    }

    #[shared]
    struct Shared {
        log: Log,
        high: Tick<TIM5>,
    }

    #[local]
    struct Local {
        tim4: Tick<TIM4>,
        adc1: Port,
        kick: Kick,
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
        let Ok(mut high) = Tick::new(&mut rcc, p.TIM5, HIGH) else {
            defmt::panic!("TIM5 at the load's rate")
        };
        if !SWEEP {
            high.pause();
        }
        let Ok(tim4) = Tick::new(&mut rcc, p.TIM4, TICK) else {
            defmt::panic!("TIM4 at TICK_HZ")
        };
        let control = timer_tick::new(tim4);
        defmt::info!("TP: timer_tick at {} Hz kicking adc_event_port on the {}, clocks: {}", TICK_HZ, BOARD, CLOCKS);
        checks::spawn().ok();
        (
            Shared { log: Log::default(), high },
            Local { tim4: control.tick, adc1: built.port, kick: built.kick },
        )
    }

    #[group(from = ferroforge_stm32::timer_tick, local = [tick = tim4], spawn = [tick = sample_tick])]
    mod control {
        #[task(binds = TIM4, priority = 3)]
        fn on_tick;
    }

    #[group(from = ferroforge_stm32::adc_event_port, local = [port = adc1], spawn = [sequence = sampled])]
    mod adc1 {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn p_on_adc_dma;
    }

    #[task(binds = TIM5, priority = 4, shared = [high])]
    fn high_load(mut cx: high_load::Context) {
        cx.shared.high.lock(|t| t.on_interrupt());
        if bench::high_pass() {
            cortex_m::peripheral::NVIC::mask(Interrupt::TIM4);
        }
    }

    /// The tick's task: one kick a tick.
    #[task(priority = 2, shared = [log], local = [kick])]
    async fn sample_tick(mut cx: sample_tick::Context, _at_us: u64, missed: u32) {
        cx.local.kick.kick();
        if missed != 0 {
            cx.shared.log.lock(|log| log.tick_missed += missed);
        }
    }

    /// The samples' task.
    #[task(priority = 2, shared = [log])]
    async fn sampled(mut cx: sampled::Context, result: Sequence) {
        cx.shared.log.lock(|log| {
            log.ended += 1;
            log.completed += u32::from(result.outcome.is_ok());
            log.missed += result.missed;
            log.late_kicks += result.late_kicks;
            if log.ended == log.target && log.snap.is_none() {
                log.snap = Some(Snap {
                    start_period: stable(&probe::ADC_PORT.start_period),
                    kick_to_start: stable(&probe::ADC_PORT.kick_to_start),
                    ended: log.ended,
                    completed: log.completed,
                    missed: log.missed,
                    late_kicks: log.late_kicks,
                    tick_missed: log.tick_missed,
                });
            }
        });
    }

    #[task(priority = 1, shared = [log, high])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut log, mut high, .. } = cx.shared;
        let mut failed = 0u32;
        let loads: &[Load] = if SWEEP { &[Load::Quiet] } else { &[Load::Quiet, Load::BusyLow, Load::High] };
        for &load in loads {
            log.lock(|log| {
                *log = Log { target: WINDOW, ..Log::default() };
                probe::ADC_PORT.clear();
            });
            if SWEEP {
                GUARD_PASSES.store(0, Relaxed);
                GUARD_ARMED.store(true, Relaxed);
            }
            if load == Load::High {
                HIGH_LOAD.store(true, Relaxed);
                high.lock(|t| t.resume());
            }
            loop {
                if log.lock(|log| log.snap.is_some()) || GUARD_FIRED.load(Relaxed) {
                    break;
                }
                match load {
                    Load::BusyLow => bench::busy(MS / 10),
                    _ => Mono::delay(1.millis()).await,
                }
            }
            if load == Load::High {
                high.lock(|t| t.pause());
                HIGH_LOAD.store(false, Relaxed);
            }
            GUARD_ARMED.store(false, Relaxed);
            let Some(snap) = log.lock(|log| log.snap) else {
                defmt::warn!("TP {} Hz {}: the guard masked TIM4 before the window filled", TICK_HZ, load);
                failed += verdict("TP: the window filled", false);
                break;
            };
            defmt::info!(
                "TP {} Hz {}: {} sequences ended, {} completed, {} missed, {} late kicks, {} ticks missed",
                TICK_HZ, load, snap.ended, snap.completed, snap.missed, snap.late_kicks, snap.tick_missed
            );
            bench::show("TP: start to start", snap.start_period);
            defmt::info!(
                "TP {} Hz {}: start spread {} ns (nominal {} ns)",
                TICK_HZ, load, bench::ns(snap.start_period.most - snap.start_period.least), bench::ns(PERIOD)
            );
            bench::show("TP: kick to start", snap.kick_to_start);
            failed += verdict("TP: every sequence completed", snap.completed == snap.ended);
            if load == Load::Quiet {
                failed += verdict("TP: quiet, no late kick, nothing missed", snap.late_kicks == 0 && snap.missed == 0 && snap.tick_missed == 0);
            }
        }
        finish(failed);
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
