//! The timer tick group's reference: a hand-written `ferroforge::app!` on the
//! Nucleo-F401RE (MB1136) that selects `ferroforge_stm32::timer_tick` once,
//! on TIM4 at 1 kHz, and judges every tick its task takes. No link: the
//! timer is the source.
//!
//! TIM4 has its own update vector, `TIM4` (RM0368 table 38). It is on APB1,
//! at 42 MHz under this clock tree, so it counts at 84 MHz (RM0368 section
//! 6.2, figure 12).
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `TIM4` | `control`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `control_step` | 2 |
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

    // The monotonic the member stamps each tick with: 1 kHz, so a stamp is
    // good to a millisecond.
    systick_monotonic!(Mono, 1000);
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use cortex_m::peripheral::DWT;
    use ferroforge_stm32::timer_tick;
    use ferrohal_stm32::{
        chip::TIM4,
        rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
        resource::Vectors,
        time::Hertz,
        timer::Tick,
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
    const SYSCLK_HZ: u32 = 84_000_000;
    /// CPU cycles in a millisecond, for the busy waits.
    const MS: u32 = 84_000;

    /// The tick's rate, and its period in microseconds.
    const TICK: Hertz = Hertz(1_000);
    const PERIOD_US: u64 = 1_000;
    /// The monotonic's tick, the stamps' resolution, in microseconds.
    const RESOLUTION_US: u64 = 1_000;
    /// Ticks in the quiet window.
    const WINDOW: u32 = 1_000;
    /// How long the task stalls once, in tenths of a period: five ticks
    /// arrive while it runs.
    const STALL_TENTHS: u32 = 55;
    /// How long a check holds the task off, in milliseconds.
    const HOLD_MS: u32 = 5;

    const _: () = assert!(Tick::<TIM4>::check(&CLOCKS, TICK).is_ok(), "TIM4 cannot tick at 1 kHz exactly");

    /// What the firmware's task has seen since the log was last cleared.
    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub ticks: u32,
        pub missed_total: u32,
        /// The last count a tick carried that was not zero.
        pub carried: u32,
        /// The first and the last tick's stamps, and when the task took the
        /// last, in microseconds of the monotonic.
        pub first_at_us: Option<u64>,
        pub at_us: u64,
        pub ran_us: u64,
        /// The furthest two ticks taken in a row were stamped from the
        /// periods between them, the missed ones included.
        pub worst_gap_off_us: u64,
        /// Ticks stamped later than the task that took them ran.
        pub stamped_late: u32,
        /// Cycles the task spins once, at its next tick.
        pub stall: u32,
    }

    impl Log {
        /// Ticks the timer made from the first taken to the last, by the
        /// count the member kept: those taken and those it counted.
        fn made(&self) -> u32 {
            self.ticks + self.missed_total
        }

        /// How far the stamps' span is from the periods the member counted.
        fn span_off_us(&self) -> u64 {
            let span = self.at_us - self.first_at_us.unwrap_or(self.at_us);
            let counted = u64::from(self.made().saturating_sub(1)) * PERIOD_US;
            span.abs_diff(counted)
        }
    }

    #[shared]
    struct Shared {
        log: Log,
    }

    #[local]
    struct Local {
        tim4: Tick<TIM4>,
        vectors: Vectors,
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
        let Ok(tim4) = Tick::new(&mut rcc, p.TIM4, TICK) else {
            defmt::panic!("TIM4 at 1 kHz")
        };
        let vectors = tim4.vectors();
        let control = timer_tick::new(tim4);

        defmt::info!("timer_tick on the Nucleo-F401RE, TIM4 at {} Hz, clocks: {}", TICK.0, CLOCKS);
        checks::spawn().ok();
        (Shared { log: Log::default() }, Local { tim4: control.tick, vectors })
    }

    #[group(from = ferroforge_stm32::timer_tick, local = [tick = tim4], spawn = [tick = control_step])]
    mod control {
        #[task(binds = TIM4, priority = 3)]
        fn on_tick;
    }

    /// The firmware's task: names no timer and no vector.
    #[task(priority = 2, shared = [log])]
    async fn control_step(mut cx: control_step::Context, at_us: u64, missed: u32) {
        let ran_us = now_us();
        let stall = cx.shared.log.lock(|log| {
            if log.first_at_us.is_none() {
                log.first_at_us = Some(at_us);
            } else {
                let gap = at_us.saturating_sub(log.at_us);
                let periods = u64::from(missed) + 1;
                log.worst_gap_off_us = log.worst_gap_off_us.max(gap.abs_diff(periods * PERIOD_US));
            }
            log.ticks += 1;
            log.missed_total += missed;
            if missed != 0 {
                log.carried = missed;
            }
            log.at_us = at_us;
            log.ran_us = ran_us;
            log.stamped_late += u32::from(at_us > ran_us);
            core::mem::take(&mut log.stall)
        });
        if stall > 0 {
            wait(stall);
        }
    }

    /// Lets the tick run and judges what the group made of it.
    #[task(priority = 1, shared = [log], local = [vectors])]
    async fn checks(cx: checks::Context) {
        let mut log = cx.shared.log;
        let mut failed = 0u32;

        failed += verdict(
            "the member's vector is enabled [PORT-501]",
            cx.local.vectors.verify().is_ok(),
        );

        // A quiet window: an empty task takes every tick, each stamped one
        // period after the last.
        let started_us = now_us();
        clear(&mut log);
        Mono::delay((u64::from(WINDOW) * PERIOD_US).micros()).await;
        let seen = log.lock(|log| *log);
        let elapsed_us = now_us() - started_us;
        defmt::info!(
            "quiet: {} ticks in {} us: {} reached the task, {} missed, {} stamped after their task ran; stamps' span {} us off the counted periods, worst gap {} us off",
            seen.made(), elapsed_us, seen.ticks, seen.missed_total, seen.stamped_late,
            seen.span_off_us(), seen.worst_gap_off_us
        );
        failed += verdict(
            "every tick of a quiet window reached the firmware's task, none missed [PORT-501]",
            seen.missed_total == 0 && u64::from(seen.ticks).abs_diff(elapsed_us / PERIOD_US) <= 2,
        );
        failed += verdict(
            "every tick was stamped a period after the one before, within a monotonic tick, and no later than its task ran [PORT-502]",
            seen.stamped_late == 0
                && seen.worst_gap_off_us <= RESOLUTION_US
                && seen.span_off_us() <= RESOLUTION_US,
        );

        // A slow consumer: the task stalls five and a half periods once, so
        // five ticks find it running; the tick after carries five, and the
        // stamps still account for every period.
        clear(&mut log);
        Mono::delay(20.millis()).await;
        log.lock(|log| log.stall = (SYSCLK_HZ / TICK.0) * STALL_TENTHS / 10);
        Mono::delay(20.millis()).await;
        let slow = log.lock(|log| *log);
        defmt::info!(
            "slow: the task stalled {}.{} periods: {} reached the task, {} missed, the tick after carrying {}; stamps' span {} us off the counted periods",
            STALL_TENTHS / 10, STALL_TENTHS % 10, slow.ticks, slow.missed_total, slow.carried, slow.span_off_us()
        );
        failed += verdict(
            "ticks that found the task running are counted on the next [PORT-501]",
            slow.carried == 5 && slow.missed_total == 5,
        );
        failed += verdict(
            "taken and counted ticks account for every period [PORT-501, PORT-502]",
            slow.span_off_us() <= RESOLUTION_US && slow.worst_gap_off_us <= RESOLUTION_US,
        );

        // A task held off: the lock raises this task to the tick task's
        // priority, which the member's vector and SysTick are above. Starting
        // just after a tick, the next one is queued and the four after it
        // find it queued; the queued one keeps its own stamp.
        let before = log.lock(|log| log.ticks);
        while log.lock(|log| log.ticks) == before {}
        let held_us = log.lock(|_| {
            let held_us = now_us();
            wait(HOLD_MS * MS);
            held_us
        });
        // The task ran the queued tick as the lock dropped.
        let held = log.lock(|log| *log);
        Mono::delay(5.millis()).await;
        let after = log.lock(|log| *log);
        defmt::info!(
            "held {} ms from {} us: the queued tick stamped {} us, taken {} us; the tick after carried {}",
            HOLD_MS, held_us, held.at_us, held.ran_us, after.carried
        );
        failed += verdict(
            "a tick's stamp is its own, not its task's [PORT-502]",
            held.at_us <= held_us + PERIOD_US + RESOLUTION_US
                && held.ran_us >= held.at_us + u64::from(HOLD_MS - 2) * 1_000,
        );
        failed += verdict(
            "ticks that found the task queued are counted on the next [PORT-501]",
            after.carried == HOLD_MS - 1,
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

    /// Starts a fresh log.
    fn clear(log: &mut impl rtic::Mutex<T = Log>) {
        log.lock(|log| *log = Log::default());
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
