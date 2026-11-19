//! The checks both USB prototype variants run, on both boards: the session's
//! timeline and load, read from what the firmware's task records.

use core::future::Future;

use crate::common::{Echo, Window, verdict};

/// What the firmware's task has seen, and when.
#[derive(Clone, Copy)]
pub struct Stats {
    pub configured: bool,
    pub wakes: u32,
    pub received: u32,
    pub echoed: u32,
    pub ended: bool,
    pub write_max_cycles: u32,
    pub hold_max_cycles: u32,
}

impl Stats {
    pub const fn new() -> Self {
        Self {
            configured: false,
            wakes: 0,
            received: 0,
            echoed: 0,
            ended: false,
            write_max_cycles: 0,
            hold_max_cycles: 0,
        }
    }

    /// One run of the firmware's task.
    pub fn update(&mut self, configured: bool, echo: &Echo) {
        self.configured = configured;
        self.wakes = self.wakes.wrapping_add(1);
        self.received = echo.received;
        self.echoed = echo.echoed;
        self.ended = echo.ended;
        self.write_max_cycles = echo.write_max_cycles;
        self.hold_max_cycles = echo.hold_max_cycles;
    }
}

/// Waits for the host: enumeration, the echo session, then a second with
/// the bus configured and quiet. Prints the measurements and returns the
/// failures.
/// `entries` reads how often the port's vector has run (0 if not counted).
pub async fn run_checks<M, D, F>(
    stats: &mut M,
    mut tick: D,
    now_ms: impl Fn() -> u32,
    entries: impl Fn() -> u32,
) -> u32
where
    M: rtic::Mutex<T = Stats>,
    D: FnMut() -> F,
    F: Future<Output = ()>,
{
    let start = now_ms();
    let mut failed = 0;
    let mut configured_ms = None;
    let mut first_ms = None;
    let mut session: Option<Window> = None;
    let mut session_wakes = 0;
    let mut session_entries = 0;
    let mut ended_ms = None;
    let mut session_load = (0, 0);
    // The whole session: 60 s at most.
    while now_ms().wrapping_sub(start) < 60_000 {
        tick().await;
        let seen = stats.lock(|stats| *stats);
        if configured_ms.is_none() && seen.configured {
            configured_ms = Some(now_ms().wrapping_sub(start));
        }
        if first_ms.is_none() && seen.received > 0 {
            first_ms = Some(now_ms());
            session = Some(Window::start());
            session_wakes = seen.wakes;
            session_entries = entries();
        }
        if seen.ended {
            ended_ms = Some(now_ms());
            if let Some(window) = session {
                session_load = window.busy_permille();
            }
            session_wakes = seen.wakes.wrapping_sub(session_wakes);
            session_entries = entries().wrapping_sub(session_entries);
            break;
        }
    }
    let seen = stats.lock(|stats| *stats);
    defmt::info!(
        "configured after {} ms; {} bytes received, {} echoed",
        configured_ms.unwrap_or(0),
        seen.received,
        seen.echoed
    );
    failed += verdict("the host configured the device", configured_ms.is_some());
    failed += verdict(
        "the host finished, every byte echoed but the marker",
        seen.ended && seen.echoed + 8 >= seen.received,
    );
    let session_ms = match (first_ms, ended_ms) {
        (Some(first), Some(end)) => end.wrapping_sub(first),
        _ => 0,
    };
    defmt::info!(
        "MEASURE session: {} ms, busy {} permille, {} task wakes, {} vector entries, longest write {} cycles, longest IN critical section {} cycles, longest lock hold {} cycles",
        session_ms,
        session_load.0,
        session_wakes,
        session_entries,
        seen.write_max_cycles,
        synopsys_usb_otg::CRITICAL_SECTION_MAX_CYCLES.load(core::sync::atomic::Ordering::Relaxed),
        seen.hold_max_cycles
    );
    // A second configured and quiet: the host has closed its port.
    let quiet = Window::start();
    let wakes = stats.lock(|stats| stats.wakes);
    let quiet_entries = entries();
    for _ in 0..1000 {
        tick().await;
    }
    let (busy, _) = quiet.busy_permille();
    let quiet_wakes = stats.lock(|stats| stats.wakes).wrapping_sub(wakes);
    let quiet_entries = entries().wrapping_sub(quiet_entries);
    defmt::info!(
        "MEASURE quiet second: busy {} permille, {} task wakes, {} vector entries",
        busy,
        quiet_wakes,
        quiet_entries
    );
    if failed == 0 {
        defmt::info!("all checks passed");
    } else {
        defmt::error!("{} checks failed", failed);
    }
    failed
}
