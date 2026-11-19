//! What both USB prototype benches share, on both boards: the echo the
//! firmware's task runs over its CDC class, the idle-time meter, and the
//! report. PROTOTYPE bench code: it compares variant D (`usb_device`,
//! deferred) with variant P (`usb_polled`, polled in the vector).
//!
//! The CDC class is the bench firmware's, from `usbd-serial`: the port
//! carries none (D6).

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::DWT;

/// The host's end marker (`usb_cdc_host.py`).
pub const END: &[u8] = b"BENCHEND";

/// The echo: what came in, waiting to go back out.
pub struct Echo {
    pending: [u8; 64],
    head: usize,
    len: usize,
    tail: [u8; 8],
    pub received: u32,
    pub echoed: u32,
    pub ended: bool,
}

impl Echo {
    pub const fn new() -> Self {
        Self {
            pending: [0; 64],
            head: 0,
            len: 0,
            tail: [0; 8],
            received: 0,
            echoed: 0,
            ended: false,
        }
    }

    /// What has moved so far: bytes received and bytes echoed.
    pub fn progress(&self) -> (u32, u32) {
        (self.received, self.echoed)
    }

    /// One step: read what the class has if nothing waits, then write what
    /// waits. `read` and `write` are the class's.
    pub fn step<T>(
        &mut self,
        io: &mut T,
        mut read: impl FnMut(&mut T, &mut [u8]) -> Option<usize>,
        mut write: impl FnMut(&mut T, &[u8]) -> Option<usize>,
    ) {
        for _ in 0..4 {
            if self.len == 0 {
                self.head = 0;
                match read(io, &mut self.pending) {
                    Some(count) if count > 0 => {
                        let count = count.min(self.pending.len());
                        self.len = count;
                        self.received += count as u32;
                        for &byte in &self.pending[..count] {
                            self.tail.rotate_left(1);
                            self.tail[7] = byte;
                            if self.tail == *END {
                                self.ended = true;
                            }
                        }
                    }
                    _ => return,
                }
            }
            let end = (self.head + self.len).min(self.pending.len());
            match write(io, &self.pending[self.head..end]) {
                Some(count) if count > 0 => {
                    let count = count.min(self.len);
                    self.head += count;
                    self.len -= count;
                    self.echoed += count as u32;
                }
                _ => return,
            }
        }
    }
}

impl Default for Echo {
    fn default() -> Self {
        Self::new()
    }
}

/// Cycles the idle loop has run, unpreempted: each pass adds the cycles
/// since the last one when that is short, so time spent in any vector or
/// task is left out.
pub static IDLE_CYCLES: AtomicU32 = AtomicU32::new(0);
/// A pass longer than this was preempted.
const PASS_LIMIT: u32 = 200;

/// The idle loop: never returns.
pub fn idle_meter() -> ! {
    let mut last = DWT::cycle_count();
    loop {
        let now = DWT::cycle_count();
        let pass = now.wrapping_sub(last);
        last = now;
        if pass < PASS_LIMIT {
            IDLE_CYCLES.fetch_add(pass, Ordering::Relaxed);
        }
    }
}

/// A window of the meter: CPU cycles and idle cycles at its start.
#[derive(Clone, Copy)]
pub struct Window {
    cycles: u32,
    idle: u32,
}

impl Window {
    pub fn start() -> Self {
        Self {
            cycles: DWT::cycle_count(),
            idle: IDLE_CYCLES.load(Ordering::Relaxed),
        }
    }

    /// The busy share since the start, in tenths of a percent, and the
    /// window's length in cycles. A window over 2^32 cycles wraps; the
    /// benches keep theirs to a few seconds at most.
    pub fn busy_permille(self) -> (u32, u32) {
        let cycles = DWT::cycle_count().wrapping_sub(self.cycles).max(1);
        let idle = IDLE_CYCLES.load(Ordering::Relaxed).wrapping_sub(self.idle);
        let busy = cycles.saturating_sub(idle) as u64;
        ((busy * 1000 / cycles as u64) as u32, cycles)
    }
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
