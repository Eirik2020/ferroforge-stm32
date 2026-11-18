//! PROTOTYPE (event and ADC ports): cycle-counter marks that the event and ADC
//! port benches read, compiled in only with the `prototype-timing` feature.
//! Without it every mark is an empty inline function, so the members a normal
//! firmware builds are as they were.
//!
//! Each member writes its own [`Marks`]; the bench reads them. One writer per
//! field: the member writes everything but [`Marks::kicked`], which the ADC
//! port's kick writes. A bench asks for a reset with [`Marks::clear`], and the
//! writer applies it at its next mark, so a reset never races a write. Two
//! selections of one group in one firmware would share a static: the benches
//! select each group once.

// Without the feature the marks are empty, and what they would write unused.
#![cfg_attr(not(feature = "prototype-timing"), allow(dead_code))]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

/// The cycle counter, as the benches enable it; 0 without the feature.
#[inline(always)]
pub fn cycles() -> u32 {
    #[cfg(feature = "prototype-timing")]
    {
        cortex_m::peripheral::DWT::cycle_count()
    }
    #[cfg(not(feature = "prototype-timing"))]
    {
        0
    }
}

/// Least, most and sum of a run of cycle counts, written by one context.
pub struct Span {
    least: AtomicU32,
    most: AtomicU32,
    sum: AtomicU32,
    count: AtomicU32,
}

/// A [`Span`] as read: cycles.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "prototype-timing", derive(defmt::Format))]
pub struct Read {
    pub least: u32,
    pub most: u32,
    pub sum: u32,
    pub count: u32,
}

impl Read {
    pub fn mean(&self) -> u32 {
        self.sum.checked_div(self.count).unwrap_or(0)
    }
}

impl Span {
    pub const fn new() -> Self {
        Self {
            least: AtomicU32::new(u32::MAX),
            most: AtomicU32::new(0),
            sum: AtomicU32::new(0),
            count: AtomicU32::new(0),
        }
    }

    /// One writer only: a load and a store, no read-modify-write.
    fn add(&self, cycles: u32) {
        if cycles < self.least.load(Relaxed) {
            self.least.store(cycles, Relaxed);
        }
        if cycles > self.most.load(Relaxed) {
            self.most.store(cycles, Relaxed);
        }
        self.sum
            .store(self.sum.load(Relaxed).wrapping_add(cycles), Relaxed);
        self.count
            .store(self.count.load(Relaxed).wrapping_add(1), Relaxed);
    }

    fn reset(&self) {
        self.least.store(u32::MAX, Relaxed);
        self.most.store(0, Relaxed);
        self.sum.store(0, Relaxed);
        self.count.store(0, Relaxed);
    }

    pub fn read(&self) -> Read {
        Read {
            least: self.least.load(Relaxed),
            most: self.most.load(Relaxed),
            sum: self.sum.load(Relaxed),
            count: self.count.load(Relaxed),
        }
    }
}

impl Default for Span {
    fn default() -> Self {
        Self::new()
    }
}

/// One member's marks.
pub struct Marks {
    reset: AtomicBool,
    /// The entry of the last pass that found its event, and whether there is
    /// one since the last reset.
    served: AtomicU32,
    has_served: AtomicBool,
    /// From one served entry to the next.
    pub period: Span,
    /// From a served entry to the member's return.
    pub run: Span,
    /// The ADC port's kick: when it was last called.
    kicked: AtomicU32,
    /// From the kick to the sequence started; between starts; from the start
    /// to the end's entry.
    pub kick_to_start: Span,
    pub start_period: Span,
    pub conversion: Span,
    started: AtomicU32,
    has_started: AtomicBool,
}

impl Marks {
    pub const fn new() -> Self {
        Self {
            reset: AtomicBool::new(false),
            served: AtomicU32::new(0),
            has_served: AtomicBool::new(false),
            period: Span::new(),
            run: Span::new(),
            kicked: AtomicU32::new(0),
            kick_to_start: Span::new(),
            start_period: Span::new(),
            conversion: Span::new(),
            started: AtomicU32::new(0),
            has_started: AtomicBool::new(false),
        }
    }

    /// The bench: every span starts again at the writer's next mark.
    pub fn clear(&self) {
        self.reset.store(true, Relaxed);
    }

    fn apply_reset(&self) {
        if self.reset.swap(false, Relaxed) {
            for span in [
                &self.period,
                &self.run,
                &self.kick_to_start,
                &self.start_period,
                &self.conversion,
            ] {
                span.reset();
            }
            self.has_served.store(false, Relaxed);
            self.has_started.store(false, Relaxed);
        }
    }

    /// The member, at its return from a pass that found its event, which
    /// it entered at `entered`.
    #[inline(always)]
    pub fn served(&self, entered: u32) {
        #[cfg(feature = "prototype-timing")]
        {
            let left = cycles();
            self.apply_reset();
            if self.has_served.load(Relaxed) {
                self.period
                    .add(entered.wrapping_sub(self.served.load(Relaxed)));
            }
            self.served.store(entered, Relaxed);
            self.has_served.store(true, Relaxed);
            self.run.add(left.wrapping_sub(entered));
        }
        #[cfg(not(feature = "prototype-timing"))]
        let _ = entered;
    }

    /// The entry of the last pass that found its event: a firmware task's
    /// first line takes it from its own cycle count.
    pub fn last_served(&self) -> u32 {
        self.served.load(Relaxed)
    }

    /// The ADC port's kick.
    #[inline(always)]
    pub fn kicked(&self) {
        #[cfg(feature = "prototype-timing")]
        self.kicked.store(cycles(), Relaxed);
    }

    /// The ADC port's member, once a sequence started.
    #[inline(always)]
    pub fn started(&self) {
        #[cfg(feature = "prototype-timing")]
        {
            let now = cycles();
            self.apply_reset();
            self.kick_to_start
                .add(now.wrapping_sub(self.kicked.load(Relaxed)));
            if self.has_started.load(Relaxed) {
                self.start_period
                    .add(now.wrapping_sub(self.started.load(Relaxed)));
            }
            self.started.store(now, Relaxed);
            self.has_started.store(true, Relaxed);
        }
    }

    /// The ADC port's member, entered at `entered` for a sequence's end.
    #[inline(always)]
    pub fn ended(&self, entered: u32) {
        #[cfg(feature = "prototype-timing")]
        {
            self.apply_reset();
            if self.has_started.load(Relaxed) {
                self.conversion
                    .add(entered.wrapping_sub(self.started.load(Relaxed)));
            }
        }
        #[cfg(not(feature = "prototype-timing"))]
        let _ = entered;
    }
}

impl Default for Marks {
    fn default() -> Self {
        Self::new()
    }
}

/// `exti_line`'s member (E).
pub static EXTI: Marks = Marks::new();
/// `timer_tick`'s member (K).
pub static TICK: Marks = Marks::new();
/// `adc_scan`'s member (A).
pub static SCAN: Marks = Marks::new();
/// `adc_event_port`'s member and kick (P).
pub static ADC_PORT: Marks = Marks::new();

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_span_keeps_least_most_and_sum() {
        let span = Span::new();
        for cycles in [5, 2, 9] {
            span.add(cycles);
        }
        let read = span.read();
        assert_eq!((read.least, read.most, read.sum, read.count), (2, 9, 16, 3));
        assert_eq!(read.mean(), 5);
        span.reset();
        assert_eq!(span.read().count, 0);
        assert_eq!(Read::default().mean(), 0);
    }
}
