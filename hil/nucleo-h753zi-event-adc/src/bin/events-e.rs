//! PROTOTYPE, E: `exti_line` on the Nucleo-H753ZI with the member's marks,
//! beside K (`events-k`). The edge comes over link Z1 (`src/lib.rs`).
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `EXTI9_5` | `line6`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `line6_edge` | 2 |
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

    // The firmware's monotonic, as the existing bench's: 1 kHz, so a stamp
    // is good to a millisecond.
    systick_monotonic!(Mono, 1000);
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::{exti_line::{self, ExtiHandle}, probe};
    use ferrohal_stm32::{
        exti::Edge,
        gpio::{Output, Pin, Pull},
    };
    use nucleo_h753zi_event_adc::{self as bench, BOARD, CLOCK_CONFIG, CLOCKS, MS, PER_US, Stat, SYSCLK_HZ, cycles, verdict, wait};

    type Drive = Pin<'D', 5, Output>;

    /// Edges driven one at a time, each awaited.
    const EDGES: u32 = 1_000;

    /// What the firmware's edge task has seen.
    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub edges: u32,
        pub at_us: u64,
        pub missed: u32,
        pub missed_total: u32,
        /// From the member's entry to this task's first line.
        pub to_task: Stat,
        /// The member's entry for the last edge.
        pub entered: u32,
    }

    #[shared]
    struct Shared {
        log: Log,
    }

    #[local]
    struct Local {
        drive: Drive,
        line6: ExtiHandle,
        probe: ExtiHandle,
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
        let mut gpiod = p.GPIOD.split(&mut rcc);
        let exti = p.EXTI.split(p.SYSCFG.split(&mut rcc).exticr);
        let ctrl = &mut gpiod.ctrl;
        // The driver idles low before the line is routed, so routing sees
        // no edge.
        let mut drive = gpiod.p5.into_push_pull_output(ctrl);
        drive.set_low();
        let mut sense = gpiod.p6.into_input(ctrl);
        sense.set_pull(ctrl, Pull::Down);
        bench::check_link("Z1 PD5 to PD6", &mut drive, &mut sense);
        let mut exti_ctrl = exti.ctrl;
        let routed = exti.l6.route::<'D'>(&mut exti_ctrl, Edge::Rising);
        let line6 = exti_line::new(routed);
        defmt::info!("E: exti_line on the {}, clocks: {}", BOARD, CLOCKS);
        checks::spawn().ok();
        (
            Shared { log: Log::default() },
            Local { drive, line6: line6.line, probe: line6.line },
        )
    }

    #[group(from = ferroforge_stm32::exti_line, local = [line = line6], spawn = [edge = line6_edge])]
    mod line6 {
        #[task(binds = EXTI9_5, priority = 3)]
        fn on_exti_line;
    }

    /// The firmware's task: takes each edge the member passes on.
    #[task(priority = 2, shared = [log])]
    async fn line6_edge(mut cx: line6_edge::Context, at_us: u64, missed: u32) {
        let ran = cycles();
        let entered = probe::EXTI.last_served();
        cx.shared.log.lock(|log| {
            log.edges += 1;
            log.at_us = at_us;
            log.missed = missed;
            log.missed_total += missed;
            log.to_task.add(ran.wrapping_sub(entered));
            log.entered = entered;
        });
    }

    #[task(priority = 1, shared = [log], local = [drive, probe])]
    async fn checks(cx: checks::Context) {
        let mut log = cx.shared.log;
        let drive = cx.local.drive;
        let probe_line = *cx.local.probe;
        let mut failed = bench::link_verdict();

        failed += verdict("E: the member's vector is enabled", probe_line.vectors().verify().is_ok());
        stamp_quality();

        // One edge at a time, each awaited.
        probe::EXTI.clear();
        log.lock(|log| *log = Log::default());
        let mut pin_to_entry = Stat::default();
        let mut worst_stamp_us = 0u64;
        let mut first: Option<(u64, u32)> = None;
        let mut late = 0u32;
        for _ in 0..EDGES {
            let before = log.lock(|log| log.edges);
            let written = cycles();
            drive.set_high();
            await_edges(&mut log, before + 1);
            drive.set_low();
            let seen = log.lock(|log| *log);
            if seen.edges != before + 1 {
                late += 1;
                continue;
            }
            pin_to_entry.add(seen.entered.wrapping_sub(written));
            // The stamp against the cycle counter, from the first edge.
            match first {
                None => first = Some((seen.at_us, seen.entered)),
                Some((at0, c0)) => {
                    let by_cycles = u64::from(seen.entered.wrapping_sub(c0) / PER_US);
                    let by_stamp = seen.at_us - at0;
                    worst_stamp_us = worst_stamp_us.max(by_stamp.abs_diff(by_cycles));
                }
            }
            wait(MS / 5);
        }
        let seen = log.lock(|log| *log);
        defmt::info!("E: {} edges driven one at a time: {} reached the task, {} missed, {} not seen in 10 ms", EDGES, seen.edges, seen.missed_total, late);
        pin_to_entry.show("E: pin write to member entry");
        seen.to_task.show("E: event to task (member entry to task's first line)");
        bench::show("E: member run (entry to return)", bench::stable(&probe::EXTI.run));
        defmt::info!("E: stamp against the cycle counter over the run: worst {} us apart", worst_stamp_us);
        failed += verdict("E: every edge reached the task, none missed", seen.edges == EDGES && seen.missed_total == 0);
        failed += verdict("E: every stamp is within one monotonic tick of the cycle counter", worst_stamp_us <= 1_000);

        // A slow consumer: ten edges a millisecond apart while the task is
        // held, then one more.
        let held = 10u32;
        let before = log.lock(|log| *log);
        probe::EXTI.clear();
        log.lock(|_| {
            for _ in 0..held {
                drive.set_high();
                wait(MS / 50);
                drive.set_low();
                wait(MS);
            }
        });
        await_edges(&mut log, before.edges + 1);
        drive.set_high();
        await_edges(&mut log, before.edges + 2);
        drive.set_low();
        wait(MS);
        let after = log.lock(|log| *log);
        let delivered = after.edges - before.edges;
        let missed = after.missed_total - before.missed_total;
        let served = bench::stable(&probe::EXTI.run).count;
        defmt::info!("E: slow consumer: {} driven, the member took {}, {} delivered, {} missed, the last carrying {}", held + 1, served, delivered, missed, after.missed);
        failed += verdict(
            "E: slow consumer: delivered plus missed is every edge, and the next carries the count",
            delivered + missed == held + 1 && served == held + 1 && after.missed == held - 1,
        );

        // The highest rate: an empty task, edges ever closer.
        let (mut highest_clean, mut broken) = (0u32, false);
        for half_us in [50u32, 25, 10, 5, 3, 2, 1] {
            let before = log.lock(|log| *log);
            probe::EXTI.clear();
            wait(MS);
            let burst = 1_000u32;
            for _ in 0..burst {
                drive.set_high();
                wait(half_us * PER_US);
                drive.set_low();
                wait(half_us * PER_US);
            }
            // One more after a pause, to carry what the burst left counted.
            wait(2 * MS);
            let flushed = log.lock(|log| log.edges);
            drive.set_high();
            await_edges(&mut log, flushed + 1);
            drive.set_low();
            wait(MS);
            let after = log.lock(|log| *log);
            let delivered = after.edges - before.edges;
            let missed = after.missed_total - before.missed_total;
            let served = bench::stable(&probe::EXTI.run).count;
            let hz = 1_000_000 / (2 * half_us);
            defmt::info!(
                "E: rate {} Hz: {} driven, the member took {}, {} delivered, {} missed, {} lost at the line",
                hz, burst + 1, served, delivered, missed, (burst + 1).saturating_sub(served)
            );
            failed += verdict("E: at this rate, delivered plus missed is what the member took", delivered + missed == served);
            if missed == 0 && served == burst + 1 && !broken {
                highest_clean = hz;
            } else {
                broken = true;
            }
        }
        defmt::info!("E: highest rate with nothing missed or lost: {} Hz", highest_clean);
        failed += verdict("E: the line is left clear", !probe_line.is_pending());

        finish(failed);
    }

    /// The monotonic as the member reads it: its resolution and the cost of
    /// a read.
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

    /// Spins until the task has taken `edges` edges, for at most 10 ms.
    fn await_edges(log: &mut impl rtic::Mutex<T = Log>, edges: u32) {
        for _ in 0..1_000 {
            if log.lock(|log| log.edges) >= edges {
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
