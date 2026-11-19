//! The bench's consumer, the same on every board: one task per port, each
//! owning a `uart_port::Handle` and naming no UART. A counter wrapping at 256
//! goes out in frames written in place, and every byte read in place is
//! checked against the next value.
//!
//! One port of each pair floods: once its main run is back it sends more
//! than its pool and FerroHAL's ring hold while its consumer holds every
//! buffer, then reads again. The other runs a tail after its main run: eight
//! frames of 4 bytes, eight of 64, then one of 280.

use ferroforge_stm32::uart::{FrameError, Handle, Outcome};
use ferrohal_stm32::uart::ChunkEnd;

/// Bytes per frame in the main run.
pub const FRAME: usize = 32;
/// The tail: (frames, length), one frame at a time.
pub const TAIL: [(u32, usize); 3] = [(8, 4), (8, 64), (1, LONG)];
/// The long frame: a MAVLink 2 frame's longest.
pub const LONG: usize = 280;
/// Bytes the flooding port sends after its hold, read after the gap.
pub const AFTER: u32 = 256;
/// The tail phase whose frames time "sent" against the wire.
const TIMED: u8 = 2;

/// Frames the port gave back, as the consumer checked them.
#[derive(Clone, Copy, Default)]
pub struct Back {
    pub frames: u32,
    pub failed: u32,
    /// Frames not back in the order sent: the first byte is not the next
    /// counter value.
    pub disorder: u32,
    pub bytes: u32,
}

/// One port's loopback.
#[derive(Clone, Copy, Default)]
pub struct Loop {
    /// Stop sending at this many bytes.
    pub limit: u32,
    /// Send but do not read: the consumer holds every buffer.
    pub hold: bool,
    pub sent: u32,
    pub read: u32,
    pub wrong: u32,
    /// Chunks that ended in a gap, and whether the next byte resynchronises.
    pub gaps: u32,
    pub resync: bool,
    /// Chunks that ended where the line went idle.
    pub idles: u32,
    pub chunks: u32,
    pub wakes: u32,
    /// Times the consumer found every frame back (`outstanding() == 0`).
    pub all_back: u32,
    /// The generation of the first chunk, and chunks of another before the
    /// first gap (none should be).
    pub generation: Option<u32>,
    pub generation_moved: u32,
    /// The generation after the gap advanced past the one before.
    pub generation_after_gap: bool,
    /// Stamps: the earliest and latest, chunks stamped earlier than the
    /// chunk before, and chunks stamped later than the consumer read them.
    pub first_us: u64,
    pub last_us: u64,
    pub backwards: u32,
    pub future: u32,
    pub expected: u8,
    /// After the main run, send `flood` more while holding.
    pub flood: u32,
    pub flooding: bool,
    /// `read`, `gaps` and `chunks` when the flood started, and chunks taken
    /// at the release, before reading on.
    pub before: (u32, u32, u32),
    /// Bytes sent and never to be read: everything unread when the gap was
    /// taken. `read` at that gap.
    pub missing: u32,
    pub read_at_gap: Option<u32>,
    /// Frames refused, and the last reason.
    pub refused: u32,
    pub last_refusal: Option<FrameError>,
    /// Run the tail after the main run.
    pub tail: bool,
    pub phase: u8,
    pub len: usize,
    /// `read`, `idles` and `chunks` when the long frame went out.
    pub long_from: (u32, u32, u32),
    pub back: Back,
    /// The timed phase: when its frame was submitted, in cycles, and the
    /// shortest and longest from submit to its outcome taken.
    pub submitted: u32,
    pub sent_cycles: (u32, u32),
    /// The same, from submit to the handle saying the line went quiet, and
    /// whether this frame's quiet is still to come.
    pub quiet_cycles: (u32, u32),
    pub awaiting_quiet: bool,
    /// Times the consumer saw the line quiet with every frame back.
    pub quiet: u32,
    /// The handle's count of lost buffers, as last read.
    pub lost: u32,
}

impl Loop {
    pub fn new(limit: u32) -> Self {
        Self {
            limit,
            len: FRAME,
            sent_cycles: (u32::MAX, 0),
            quiet_cycles: (u32::MAX, 0),
            first_us: u64::MAX,
            ..Self::default()
        }
    }

    /// Whether the consumer should now pause, outside any lock, and then
    /// call [`release`](Self::release).
    pub fn wants_pause(&self) -> bool {
        self.hold && self.sent >= self.limit
    }

    /// Ends the hold, and sends `AFTER` more bytes, which must come back
    /// after the gap.
    pub fn release(&mut self) {
        self.hold = false;
        self.limit += AFTER;
    }

    /// The long frame's bytes back, its frame ends and its chunks.
    pub fn long(&self) -> (u32, u32, u32) {
        if usize::from(self.phase) < TAIL.len() {
            return (0, 0, 0);
        }
        let (read, idles, chunks) = self.long_from;
        (self.read - read, self.idles - idles, self.chunks - chunks)
    }

    /// Whether the flood, the hold and the bytes after it are done.
    pub fn flood_done(&self) -> bool {
        self.read_at_gap.is_some_and(|at| self.read >= at + AFTER)
    }

    /// Whether the tail is done and every byte of it is back.
    pub fn tail_done(&self) -> bool {
        usize::from(self.phase) == TAIL.len() && self.read >= self.limit
    }

    fn next_phase(&mut self) -> bool {
        let Some(&(count, len)) = TAIL.get(usize::from(self.phase)) else {
            return false;
        };
        if !self.tail || self.read < self.limit {
            return false;
        }
        if len == LONG {
            self.long_from = (self.read, self.idles, self.chunks);
        }
        self.phase += 1;
        self.len = len;
        self.limit += count * len as u32;
        true
    }

    fn collect(&mut self, handle: &mut Handle, cycles: fn() -> u32) {
        while let Some(frame) = handle.sent() {
            self.back.frames += 1;
            if frame.outcome != Outcome::Sent {
                self.back.failed += 1;
            }
            let bytes = frame.bytes();
            if bytes.first() != Some(&(self.back.bytes as u8)) {
                self.back.disorder += 1;
            }
            self.back.bytes += bytes.len() as u32;
            if self.phase == TIMED && bytes.len() == TAIL[1].1 {
                let took = cycles().wrapping_sub(self.submitted);
                self.sent_cycles = (self.sent_cycles.0.min(took), self.sent_cycles.1.max(took));
                self.awaiting_quiet = true;
            }
        }
        if handle.drained() && handle.outstanding() == 0 {
            self.quiet += 1;
            if self.awaiting_quiet {
                self.awaiting_quiet = false;
                let took = cycles().wrapping_sub(self.submitted);
                self.quiet_cycles = (self.quiet_cycles.0.min(took), self.quiet_cycles.1.max(took));
            }
        }
    }

    /// One wake: take the frames back, read what is there (unless holding),
    /// then send what fits.
    pub fn step(&mut self, handle: &mut Handle, now_us: fn() -> u64, cycles: fn() -> u32) {
        self.wakes += 1;
        self.lost = handle.lost();
        self.collect(handle, cycles);
        if !self.hold {
            for _ in 0..64 {
                let Some(got) = handle.next_chunk() else {
                    break;
                };
                let now = now_us();
                if got.at_us > now {
                    self.future += 1;
                }
                if got.at_us < self.last_us {
                    self.backwards += 1;
                }
                self.first_us = self.first_us.min(got.at_us);
                self.last_us = self.last_us.max(got.at_us);
                let gap = matches!(got.chunk.end, ChunkEnd::Gap(_));
                match self.generation {
                    None => self.generation = Some(got.chunk.generation),
                    Some(g) if self.gaps == 0 && !gap && got.chunk.generation != g => {
                        self.generation_moved += 1;
                    }
                    Some(g) if self.gaps > 0 && got.chunk.generation > g => {
                        self.generation_after_gap = true;
                    }
                    Some(_) => {}
                }
                for &byte in got.bytes() {
                    if self.resync {
                        self.resync = false;
                    } else if byte != self.expected {
                        self.wrong += 1;
                    }
                    self.expected = byte.wrapping_add(1);
                }
                self.read += got.bytes().len() as u32;
                self.chunks += 1;
                match got.chunk.end {
                    ChunkEnd::Idle => self.idles += 1,
                    ChunkEnd::Full => {}
                    ChunkEnd::Gap(_) => {
                        self.gaps += 1;
                        self.resync = true;
                        if self.flooding && self.read_at_gap.is_none() {
                            self.read_at_gap = Some(self.read);
                            self.missing = self.sent - self.read;
                        }
                    }
                }
                // Dropping `got` returns the buffer to the port.
            }
        }
        if self.flood > 0 && !self.flooding && self.read >= self.limit {
            self.flooding = true;
            self.hold = true;
            self.before = (self.read, self.gaps, self.chunks);
            self.limit += self.flood;
        }
        for _ in 0..16 {
            if self.sent >= self.limit {
                if self.next_phase() {
                    continue;
                }
                self.collect(handle, cycles);
                if handle.outstanding() == 0 {
                    self.all_back += 1;
                }
                break;
            }
            // Frame by frame: the next goes out once the last is back, so
            // the line idles between frames as a protocol's would. The flood
            // sends without waiting.
            if !self.hold && self.sent > self.read + self.missing {
                break;
            }
            let len = self.len.min((self.limit - self.sent) as usize);
            let first = self.sent;
            match handle.frame(len) {
                Ok(mut frame) => {
                    for (i, byte) in frame.bytes_mut().iter_mut().enumerate() {
                        *byte = (first as usize + i) as u8;
                    }
                    self.submitted = cycles();
                    frame.submit();
                    self.sent += len as u32;
                }
                Err(refusal) => {
                    self.refused += 1;
                    self.last_refusal = Some(refusal);
                    break;
                }
            }
            if !self.hold {
                break;
            }
        }
    }
}

pub fn verdict(name: &str, pass: bool) -> u32 {
    if pass {
        defmt::info!("PASS {}", name);
        0
    } else {
        defmt::error!("FAIL {}", name);
        1
    }
}

/// Judges one port. `pool` is the bytes its receive pool holds; `frame_cycles`
/// one character's time on the wire, in CPU cycles.
pub fn report(name: &str, l: &Loop, lost_outcomes: u32, pool: u32, frame_cycles: u32) -> u32 {
    defmt::info!(
        "{}: sent {} read {} wrong {} chunks {} idle-ended {} gaps {} wakes {} all-back {} stamps {}..{} us",
        name,
        l.sent,
        l.read,
        l.wrong,
        l.chunks,
        l.idles,
        l.gaps,
        l.wakes,
        l.all_back,
        l.first_us,
        l.last_us
    );
    defmt::info!(
        "{} frames back {} failed {} out of order {} bytes {}; refused {} (last {}); buffers lost {}, outcomes lost {}",
        name,
        l.back.frames,
        l.back.failed,
        l.back.disorder,
        l.back.bytes,
        l.refused,
        match l.last_refusal {
            None => "none",
            Some(FrameError::Outstanding) => "Outstanding",
            Some(FrameError::TooLong) => "TooLong",
            Some(FrameError::Empty) => "Empty",
        },
        l.lost,
        lost_outcomes
    );
    let mut failed = 0;
    if l.flood == 0 {
        failed += verdict(
            "every byte came back once, in order, no gap [PORT-003, PORT-008]",
            l.read == l.limit && l.wrong == 0 && l.gaps == 0,
        );
    } else {
        let (read0, gaps0, _) = l.before;
        defmt::info!(
            "{} held: {} of {} flooded bytes read, {} gaps",
            name,
            l.read - read0,
            l.flood,
            l.gaps - gaps0
        );
        failed += verdict(
            "every byte came back once, in order, before the flood [PORT-003, PORT-008]",
            read0 >= l.limit - l.flood - AFTER && l.wrong == 0,
        );
        let at_gap = l.read_at_gap.unwrap_or(read0);
        failed += verdict(
            "the loss while held reached the consumer as a gap, the stream in order on both sides [PORT-006]",
            l.gaps > gaps0 && l.wrong == 0 && l.generation_after_gap && l.flood_done(),
        );
        failed += verdict(
            "while held, the bytes the pool took were kept and read first, the rest after the gap [PORT-007]",
            at_gap - read0 >= pool && l.read - at_gap >= AFTER,
        );
    }
    failed += verdict(
        "frame ends reached the consumer as idle chunks [PORT-005]",
        l.idles > 0,
    );
    failed += verdict(
        "no generation change before the first gap [PORT-005]",
        l.generation.is_some() && l.generation_moved == 0,
    );
    failed += verdict(
        "every stamp is from the monotonic, in order, no later than its read [PORT-005]",
        l.chunks > 0 && l.backwards == 0 && l.future == 0 && l.first_us <= l.last_us,
    );
    failed += verdict(
        "every frame came back sent, in order, every byte [PORT-009]",
        l.back.frames > 0
            && l.back.failed == 0
            && l.back.disorder == 0
            && l.back.bytes == l.sent
            && l.all_back > 0
            && lost_outcomes == 0,
    );
    failed += verdict("no buffer was lost [PORT-009]", l.lost == 0);
    if l.tail {
        let (back, ends, chunks) = l.long();
        defmt::info!(
            "{} long frame: {} of {} bytes back, {} frame ends, {} chunks",
            name,
            back,
            LONG,
            ends,
            chunks
        );
        failed += verdict(
            "the 280-byte frame went whole, one frame end [PORT-008]",
            back == LONG as u32 && ends == 1,
        );
        let wire = TAIL[1].1 as u32 * frame_cycles;
        let (shortest, longest) = l.sent_cycles;
        defmt::info!(
            "{} sent timing: a 64-byte frame takes {} cycles on the wire; submit to outcome {}..{} cycles, so the outcome came {}..{} cycles before the last stop bit",
            name,
            wire,
            shortest,
            longest,
            wire as i64 - longest as i64,
            wire as i64 - shortest as i64
        );
        let (q_short, q_long) = l.quiet_cycles;
        defmt::info!(
            "{} quiet timing: submit to the line quiet {}..{} cycles, {}..{} cycles after the last stop bit",
            name,
            q_short,
            q_long,
            q_short as i64 - wire as i64,
            q_long as i64 - wire as i64
        );
        failed += verdict(
            "the consumer learned the line went quiet only after each frame's last stop bit [PORT-012]",
            l.quiet > 0 && q_long > 0 && q_short >= wire && q_short >= l.sent_cycles.0,
        );
    }
    if l.flood > 0 {
        failed += verdict(
            "with every buffer out a frame was refused, nothing lost, sending went on [PORT-010]",
            l.refused > 0 && l.last_refusal == Some(FrameError::Outstanding) && l.sent == l.limit,
        );
    }
    failed
}
