//! PROTOTYPE (event and ADC ports): what the event and ADC port benches
//! share on both boards. The board's `lib.rs` includes it and supplies
//! `PER_US`, the core cycles in a microsecond.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use ferroforge_stm32::probe::{Read, Span};

/// The cycle counter, which `init` enables.
pub fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

/// Spins until `cycles` core cycles have passed. Interrupts still run.
pub fn wait(cycles: u32) {
    let start = self::cycles();
    while self::cycles().wrapping_sub(start) < cycles {}
}

/// Spins for `cycles` moving memory, as a busy lower-priority task does.
pub fn busy(cycles: u32) {
    let start = self::cycles();
    let mut block = [0u32; 64];
    let mut round = 0u32;
    while self::cycles().wrapping_sub(start) < cycles {
        for word in block.iter_mut() {
            *word = core::hint::black_box(word.wrapping_add(round));
        }
        round = round.wrapping_add(1);
    }
    core::hint::black_box(&block);
}

/// Nanoseconds in `cycles`.
pub fn ns(cycles: u32) -> u32 {
    (u64::from(cycles) * 1_000 / u64::from(crate::PER_US)) as u32
}

/// Prints the check's verdict; 1 for a failure, to count them.
pub fn verdict(name: &str, pass: bool) -> u32 {
    if pass {
        defmt::info!("PASS {}", name);
        0
    } else {
        defmt::error!("FAIL {}", name);
        1
    }
}

/// A span read twice until both reads agree, so a member writing between
/// its fields cannot tear it.
pub fn stable(span: &Span) -> Read {
    loop {
        let first = span.read();
        if span.read() == first {
            return first;
        }
    }
}

/// Prints a span in nanoseconds: least, mean and most, of how many.
pub fn show(label: &str, read: Read) {
    defmt::info!(
        "{}: least {} ns, mean {} ns, most {} ns, of {}",
        label,
        ns(read.least),
        ns(read.mean()),
        ns(read.most),
        read.count
    );
}

/// A span a bench task keeps for itself: one writer, no atomics needed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stat {
    pub least: u32,
    pub most: u32,
    pub sum: u64,
    pub count: u32,
}

impl Default for Stat {
    fn default() -> Self {
        Self {
            least: u32::MAX,
            most: 0,
            sum: 0,
            count: 0,
        }
    }
}

impl Stat {
    pub fn add(&mut self, cycles: u32) {
        self.least = self.least.min(cycles);
        self.most = self.most.max(cycles);
        self.sum += u64::from(cycles);
        self.count += 1;
    }

    /// Prints it as [`show`] does.
    pub fn show(&self, label: &str) {
        let mean = self.sum.checked_div(u64::from(self.count)).unwrap_or(0) as u32;
        defmt::info!(
            "{}: least {} ns, mean {} ns, most {} ns, of {}",
            label,
            ns(if self.count == 0 { 0 } else { self.least }),
            ns(mean),
            ns(self.most),
            self.count
        );
    }
}

/// The load the higher-priority task puts on: when set, each of its passes
/// spins this long.
pub static HIGH_LOAD: AtomicBool = AtomicBool::new(false);
/// Its spin.
pub const HIGH_LOAD_US: u32 = 50;
/// The guard: once armed, the higher-priority task counts its passes, and
/// at this many masks the tick's vector so a rate the core cannot keep up
/// with still ends the run.
pub static GUARD_ARMED: AtomicBool = AtomicBool::new(false);
pub static GUARD_PASSES: AtomicU32 = AtomicU32::new(0);
pub static GUARD_FIRED: AtomicBool = AtomicBool::new(false);
pub const GUARD_AT: u32 = 200;

/// One pass of the higher-priority task: the load if asked, and the guard.
/// Says whether the guard fired on this pass.
pub fn high_pass() -> bool {
    if HIGH_LOAD.load(Relaxed) {
        wait(HIGH_LOAD_US * crate::PER_US);
    }
    if GUARD_ARMED.load(Relaxed) {
        let passes = GUARD_PASSES.load(Relaxed) + 1;
        GUARD_PASSES.store(passes, Relaxed);
        if passes == GUARD_AT {
            GUARD_FIRED.store(true, Relaxed);
            return true;
        }
    }
    false
}

/// The three loads a tick's evenness is measured under.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Load {
    /// The checks task sleeps.
    Quiet,
    /// The checks task, below the member, moves memory.
    BusyLow,
    /// A task above the member runs 50 us at its own rate.
    High,
}

/// `TICK_HZ` from the build's environment, 1 kHz without it: the tick's
/// rate is fixed when `init` builds it, so each rate is an image.
pub const fn tick_hz(value: Option<&str>) -> u32 {
    let Some(text) = value else {
        return 1_000;
    };
    let bytes = text.as_bytes();
    let mut index = 0;
    let mut hz = 0u32;
    while index < bytes.len() {
        let digit = bytes[index];
        assert!(digit.is_ascii_digit(), "TICK_HZ must be a number of hertz");
        hz = hz * 10 + (digit - b'0') as u32;
        index += 1;
    }
    hz
}
