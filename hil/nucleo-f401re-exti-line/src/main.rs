//! The EXTI line group's reference: a hand-written `ferroforge::app!` on the
//! Nucleo-F401RE (MB1136) that selects `ferroforge_stm32::exti_line` once and
//! drives edges into it over a wire.
//!
//! The link, as FerroHAL's F401 bench fits it (its J2; UM1724 table 29):
//!
//! | Link | From | To | Use |
//! | --- | --- | --- | --- |
//! | J2 | CN10-14 (PA11) | CN10-12 (PA12) | a GPIO output driving line 12 |
//!
//! PA12 is an input with a pull-down, routed to EXTI line 12 on its rising
//! edge; lines 10 to 15 share the `EXTI15_10` vector (RM0368 table 38). PA11
//! is a push-pull output.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `EXTI15_10` | `line12`'s member | 3 |
//! | `SPI2` | RTIC, dispatching `line12_edge` | 2 |
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

    // The monotonic the member stamps each edge with: 1 kHz, so a stamp is
    // good to a millisecond.
    systick_monotonic!(Mono, 1000);
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use cortex_m::peripheral::DWT;
    use ferroforge_stm32::exti_line::{self, ExtiHandle};
    use ferrohal_stm32::{
        exti::Edge,
        gpio::{Output, Pin, Pull},
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
    const SYSCLK_HZ: u32 = 84_000_000;
    /// CPU cycles in a millisecond, for the busy waits.
    const MS: u32 = 84_000;

    /// Edges driven one at a time, each awaited.
    const EDGES: u32 = 100;
    /// How long a check holds the edge task off, in milliseconds.
    const HOLD_MS: u32 = 5;

    /// What the firmware's edge task has seen.
    #[derive(Clone, Copy, Default)]
    pub struct Log {
        pub edges: u32,
        /// The last edge's stamp, and when the task took it, in
        /// microseconds of the monotonic.
        pub at_us: u64,
        pub ran_us: u64,
        /// The last edge's count of edges dropped before it, and the sum.
        pub missed: u32,
        pub missed_total: u32,
        /// Edges stamped later than the task that took them ran.
        pub stamped_late: u32,
    }

    #[shared]
    struct Shared {
        log: Log,
    }

    #[local]
    struct Local {
        drive: Pin<'A', 11, Output>,
        line12: ExtiHandle,
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
        let mut gpioa = p.GPIOA.split(&mut rcc);
        let exti = p.EXTI.split(p.SYSCFG.split(&mut rcc).exticr);
        let ctrl = &mut gpioa.ctrl;
        // The driver idles low before the line is routed, so routing sees
        // no edge.
        let mut drive = gpioa.p11.into_push_pull_output(ctrl);
        drive.set_low();
        let mut sense = gpioa.p12.into_input(ctrl);
        sense.set_pull(ctrl, Pull::Down);
        let mut exti_ctrl = exti.ctrl;
        let routed = exti.l12.route::<'A'>(&mut exti_ctrl, Edge::Rising);
        let line12 = exti_line::new(routed);

        defmt::info!("exti_line on the Nucleo-F401RE, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared { log: Log::default() },
            Local { drive, line12: line12.line, probe: line12.line },
        )
    }

    #[group(from = ferroforge_stm32::exti_line, local = [line = line12], spawn = [edge = line12_edge])]
    mod line12 {
        #[task(binds = EXTI15_10, priority = 3)]
        fn on_exti_line;
    }

    /// The firmware's task: takes each edge the member passes on.
    #[task(priority = 2, shared = [log])]
    async fn line12_edge(mut cx: line12_edge::Context, at_us: u64, missed: u32) {
        let ran_us = now_us();
        cx.shared.log.lock(|log| {
            log.edges += 1;
            log.at_us = at_us;
            log.ran_us = ran_us;
            log.missed = missed;
            log.missed_total += missed;
            log.stamped_late += u32::from(at_us > ran_us);
        });
    }

    /// Drives the line and judges what the group made of it.
    #[task(priority = 1, shared = [log], local = [drive, probe])]
    async fn checks(cx: checks::Context) {
        let mut log = cx.shared.log;
        let drive = cx.local.drive;
        let probe = *cx.local.probe;
        let mut failed = 0u32;

        failed += verdict(
            "the member's vector is enabled [PORT-301]",
            probe.vectors().verify().is_ok(),
        );

        // One edge at a time, each awaited: every one reaches the task,
        // stamped no earlier than it was driven and within a tick of it.
        let mut unstamped = 0u32;
        for _ in 0..EDGES {
            let before = log.lock(|log| log.edges);
            let driven_us = now_us();
            pulse(drive);
            await_edges(&mut log, before + 1);
            let seen = log.lock(|log| *log);
            if seen.at_us < driven_us || seen.at_us > driven_us + 1_000 {
                unstamped += 1;
            }
        }
        let seen = log.lock(|log| *log);
        defmt::info!(
            "{} edges driven: {} reached the task, {} missed, {} stamped outside their tick, {} stamped after their task ran",
            EDGES, seen.edges, seen.missed_total, unstamped, seen.stamped_late
        );
        failed += verdict(
            "every edge reached the firmware's task, none missed [PORT-301]",
            seen.edges == EDGES && seen.missed_total == 0,
        );
        failed += verdict(
            "every edge was stamped within a tick of being driven [PORT-302]",
            unstamped == 0 && seen.stamped_late == 0,
        );

        // An edge whose task is held off: the stamp is still the edge's.
        // The lock raises this task to the edge task's priority, which the
        // member's vector and SysTick are above.
        let before = seen.edges;
        let driven_us = log.lock(|_| {
            let driven_us = now_us();
            pulse(drive);
            wait(HOLD_MS * MS);
            driven_us
        });
        await_edges(&mut log, before + 1);
        let held = log.lock(|log| *log);
        defmt::info!(
            "held {} ms: driven at {} us, stamped {} us, taken {} us",
            HOLD_MS, driven_us, held.at_us, held.ran_us
        );
        failed += verdict(
            "an edge's stamp is its own, not its task's [PORT-302]",
            held.at_us >= driven_us
                && held.at_us <= driven_us + 1_000
                && held.ran_us >= held.at_us + u64::from(HOLD_MS - 1) * 1_000,
        );

        // Three edges while the task is held: the first is queued, the next
        // two find it queued and are counted, and the edge after carries
        // that count.
        let before = held.edges;
        log.lock(|_| {
            for _ in 0..3 {
                pulse(drive);
                wait(MS);
            }
        });
        await_edges(&mut log, before + 1);
        pulse(drive);
        await_edges(&mut log, before + 2);
        wait(MS);
        let dropped = log.lock(|log| *log);
        defmt::info!(
            "3 edges held, then 1: {} reached the task, the last carrying {} missed",
            dropped.edges - before, dropped.missed
        );
        failed += verdict(
            "edges dropped while the task was queued are counted on the next [PORT-301]",
            dropped.edges == before + 2 && dropped.missed == 2,
        );
        failed += verdict("the line is left clear [PORT-301]", !probe.is_pending());

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

    /// A rising edge and back: high for about 20 us.
    fn pulse(drive: &mut Pin<'A', 11, Output>) {
        drive.set_high();
        wait(MS / 50);
        drive.set_low();
        wait(MS / 50);
    }

    /// Spins until the task has taken `edges` edges, for at most 10 ms.
    fn await_edges(log: &mut impl rtic::Mutex<T = Log>, edges: u32) {
        for _ in 0..100 {
            if log.lock(|log| log.edges) >= edges {
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
