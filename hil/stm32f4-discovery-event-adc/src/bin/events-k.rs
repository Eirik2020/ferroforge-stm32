//! PROTOTYPE, K: `timer_tick` on the STM32F4 Discovery, TIM4 at the build's
//! `TICK_HZ`, beside E (`events-e`). No link: the timer is the source.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `TIM5` | the higher-priority load (50 us at 1.25 kHz), or the guard | 4 |
//! | `TIM4` | `control`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `control_step` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |
//! | `SysTick` | the monotonic, at 1 kHz | as rtic-monotonics sets it |
//!
//! At 10 kHz and below the image measures each load; above it, the highest
//! rate only (`SWEEP`), with the guard masking TIM4 if the core cannot keep
//! up, so the run still ends.

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
    use ferroforge_stm32::{probe::{self, Read}, timer_tick};
    use ferrohal_stm32::{chip::{TIM4, TIM5}, device::Interrupt, timer::Tick};
    use stm32f4_discovery_event_adc::{
        self as bench, BOARD, CLOCK_CONFIG, CLOCKS, GUARD_ARMED, GUARD_FIRED, GUARD_PASSES, HIGH, HIGH_LOAD,
        Load, MS, PER_US, PERIOD, Stat, SWEEP, SYSCLK_HZ, TICK, TICK_HZ, cycles, stable, verdict, wait,
    };

    /// Ticks in each measured window.
    const WINDOW: u32 = 10_000;

    /// The member's marks and the task's counts when a window filled.
    #[derive(Clone, Copy, Default)]
    pub struct Snap {
        pub period: Read,
        pub run: Read,
        pub to_task: Stat,
        pub ticks: u32,
        pub missed: u32,
        pub worst_stamp_us: u64,
    }

    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub ticks: u32,
        pub missed: u32,
        /// The last count a tick carried that was not zero.
        pub carried: u32,
        pub to_task: Stat,
        pub last: Option<(u64, u32)>,
        pub worst_stamp_us: u64,
        /// Cycles the task spins once, at its next tick.
        pub stall: u32,
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
    }

    #[init]
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
        defmt::info!("K: timer_tick on the {} at {} Hz, clocks: {}", BOARD, TICK_HZ, CLOCKS);
        checks::spawn().ok();
        (Shared { log: Log::default(), high }, Local { tim4: control.tick })
    }

    #[group(from = ferroforge_stm32::timer_tick, local = [tick = tim4], spawn = [tick = control_step])]
    mod control {
        #[task(binds = TIM4, priority = 3)]
        fn on_tick;
    }

    /// The higher-priority task: the 50 us load when asked, and the guard.
    #[task(binds = TIM5, priority = 4, shared = [high])]
    fn high_load(mut cx: high_load::Context) {
        cx.shared.high.lock(|t| t.on_interrupt());
        if bench::high_pass() {
            cortex_m::peripheral::NVIC::mask(Interrupt::TIM4);
        }
    }

    /// The firmware's task: names no timer and no vector.
    #[task(priority = 2, shared = [log])]
    async fn control_step(mut cx: control_step::Context, at_us: u64, missed: u32) {
        let ran = cycles();
        let entered = probe::TICK.last_served();
        let stall = cx.shared.log.lock(|log| {
            log.ticks += 1;
            log.missed += missed;
            if missed != 0 {
                log.carried = missed;
            }
            log.to_task.add(ran.wrapping_sub(entered));
            if let Some((at0, c0)) = log.last {
                let by_cycles = u64::from(entered.wrapping_sub(c0) / PER_US);
                log.worst_stamp_us = log.worst_stamp_us.max((at_us - at0).abs_diff(by_cycles));
            }
            log.last = Some((at_us, entered));
            if log.ticks == log.target && log.snap.is_none() {
                log.snap = Some(Snap {
                    period: stable(&probe::TICK.period),
                    run: stable(&probe::TICK.run),
                    to_task: log.to_task,
                    ticks: log.ticks,
                    missed: log.missed,
                    worst_stamp_us: log.worst_stamp_us,
                });
            }
            core::mem::take(&mut log.stall)
        });
        if stall > 0 {
            wait(stall);
        }
    }

    #[task(priority = 1, shared = [log, high])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut log, mut high, .. } = cx.shared;
        let mut failed = 0u32;
        stamp_quality();

        let loads: &[Load] = if SWEEP { &[Load::Quiet] } else { &[Load::Quiet, Load::BusyLow, Load::High] };
        for &load in loads {
            start_window(&mut log, WINDOW);
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
                let ticks = log.lock(|log| log.ticks);
                defmt::warn!("K {} Hz {}: the guard masked TIM4 after {} ticks reached the task: the core cannot keep up", TICK_HZ, load, ticks);
                failed += verdict("K: the window filled", false);
                break;
            };
            failed += report(load, &snap);
        }

        if !SWEEP {
            // A slow consumer: the task stalls five and a half periods.
            start_window(&mut log, u32::MAX);
            Mono::delay(20.millis()).await;
            log.lock(|log| log.stall = PERIOD * 11 / 2);
            Mono::delay(20.millis()).await;
            let seen = log.lock(|log| *log);
            let served = stable(&probe::TICK.run).count;
            defmt::info!(
                "K: slow consumer, stalled 5.5 periods: the member took {}, {} delivered, {} missed, the tick after the stall carrying {}",
                served, seen.ticks, seen.missed, seen.carried
            );
            failed += verdict(
                "K: slow consumer: delivered plus missed is every tick the member took, and the next carries the count",
                served.abs_diff(seen.ticks + seen.missed) <= 1 && seen.carried == 5,
            );
        }
        finish(failed);
    }

    /// Starts a window of `target` ticks.
    fn start_window(log: &mut impl rtic::Mutex<T = Log>, target: u32) {
        log.lock(|log| {
            *log = Log { target, ..Log::default() };
            probe::TICK.clear();
        });
    }

    /// Prints one window and judges it; 1 for each failure.
    fn report(load: Load, snap: &Snap) -> u32 {
        let served = snap.run.count;
        let expected = (snap.period.sum + PERIOD / 2) / PERIOD + 1;
        defmt::info!(
            "K {} Hz {}: the member took {} of {} ticks the timer made; {} delivered, {} missed",
            TICK_HZ, load, served, expected, snap.ticks, snap.missed
        );
        bench::show("K: tick period", snap.period);
        defmt::info!(
            "K {} Hz {}: period spread {} ns (nominal {} ns)",
            TICK_HZ, load, bench::ns(snap.period.most - snap.period.least), bench::ns(PERIOD)
        );
        bench::show("K: member run (entry to return)", snap.run);
        snap.to_task.show("K: event to task (member entry to task's first line)");
        defmt::info!("K: stamp against the cycle counter, tick to tick: worst {} us apart", snap.worst_stamp_us);
        let mut failed = 0;
        failed += verdict(
            "K: delivered plus missed is every tick the member took",
            served.abs_diff(snap.ticks + snap.missed) <= 1,
        );
        failed += verdict("K: the member took every tick the timer made", served.abs_diff(expected) <= 1);
        failed += verdict("K: every stamp is within one monotonic tick of the cycle counter", snap.worst_stamp_us <= 1_000);
        if load == Load::Quiet {
            failed += verdict("K: an empty task, quiet, misses nothing", snap.missed == 0);
        }
        failed
    }

    fn stamp_quality() {
        let start = Mono::now();
        while Mono::now() == start {}
        let a = Mono::now();
        while Mono::now() == a {}
        let b = Mono::now();
        let step = (b - a).to_micros();
        let reads = 1_000u32;
        let before = cycles();
        for _ in 0..reads {
            core::hint::black_box(Mono::now().duration_since_epoch().to_micros());
        }
        let cost = cycles().wrapping_sub(before) / reads;
        defmt::info!("stamp: resolution {} us, a read costs {} ns", step, bench::ns(cost));
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
