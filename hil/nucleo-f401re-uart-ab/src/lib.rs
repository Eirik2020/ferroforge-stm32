//! PROTOTYPE: the one consumer every variant runs. It speaks FerroHAL's
//! chunk vocabulary (`Chunk`, `ChunkEnd`, frames, idle), which variant A
//! gets from the locked `Uart`, variant B from the port's `Handle` and
//! variant B2 from the port's `LentUartHandle`. A and B copy each chunk into
//! the consumer's buffer and each frame out of it; B2 reads and writes in
//! the lent buffers.

#![no_std]

use ferroforge_stm32::{
    uart_lent::{FrameError, LentUartHandle, Outcome},
    uart_queue::{Handle, SLOT, Split},
};
use ferrohal_stm32::uart::{
    Chunk, ChunkEnd, Instance, RxDma, RxPin, SendError, TxDma, TxPin, Uart, Wiring,
};

/// Bytes per frame sent in the main run.
pub const FRAME: usize = 32;

/// The tail after the main run, on the port that does not flood: frames of
/// 4 and 64 bytes, one at a time, then one long frame. A port that cannot
/// send it whole sends it in pieces, back to back.
pub const TAIL: [(u32, usize); 3] = [(8, 4), (8, 64), (1, LONG)];
/// The long frame.
pub const LONG: usize = 280;
/// Bytes the tail adds.
pub const TAIL_BYTES: u32 = 8 * 4 + 8 * 64 + LONG as u32;

/// Why a port refused a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    Send(SendError),
    Frame(FrameError),
}

impl Refusal {
    pub fn name(self) -> &'static str {
        match self {
            Self::Send(SendError::Full) => "Full",
            Self::Send(SendError::TooLarge) => "TooLarge",
            Self::Send(SendError::Halted) => "Halted",
            Self::Frame(FrameError::Outstanding) => "Outstanding",
            Self::Frame(FrameError::TooLong) => "TooLong",
            Self::Frame(FrameError::Empty) => "Empty",
        }
    }
}

/// Frames a B2 port gave back, as the consumer checked them (D15).
#[derive(Clone, Copy, Default)]
pub struct Back {
    pub frames: u32,
    pub failed: u32,
    /// Frames not back in the order sent: the first byte is not the next
    /// counter value.
    pub disorder: u32,
    /// Bytes of the frames back.
    pub bytes: u32,
}

/// What the consumer needs from a port.
pub trait Port {
    /// Hands the next chunk, its bytes and, where the port gives one, its
    /// arrival time to `f`.
    fn take<R>(&mut self, f: impl FnOnce(&[u8], Chunk, Option<u32>) -> R) -> Option<R>;
    /// Sends one frame of `len` bytes that `fill` writes.
    fn send(&mut self, len: usize, fill: impl FnOnce(&mut [u8])) -> Result<(), Refusal>;
    /// Takes the frames back (B2 only).
    fn collect(&mut self, back: &mut Back);
    /// Whether everything sent has gone.
    fn idle(&mut self, back: &mut Back) -> bool;
    /// The longest frame one `send` takes.
    fn longest(&self) -> usize;
}

/// A and B: the frame is built on the stack and copied in.
fn copy_send(
    len: usize,
    fill: impl FnOnce(&mut [u8]),
    send: impl FnOnce(&[u8]) -> Result<(), SendError>,
) -> Result<(), Refusal> {
    let mut frame = [0u8; SLOT];
    let frame = frame
        .get_mut(..len)
        .ok_or(Refusal::Send(SendError::TooLarge))?;
    fill(frame);
    send(frame).map_err(Refusal::Send)
}

impl<U, Tx, Rx, TxD, RxD> Port for Uart<Wiring<U, Tx, Rx, TxD, RxD>>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    fn take<R>(&mut self, f: impl FnOnce(&[u8], Chunk, Option<u32>) -> R) -> Option<R> {
        let mut buf = [0u8; 64];
        let chunk = match self.read_chunk(&mut buf) {
            Ok(chunk) => chunk?,
            Err(e) => Chunk {
                len: 0,
                end: ChunkEnd::Gap(e),
                generation: self.generation(),
            },
        };
        Some(f(&buf[..chunk.len], chunk, None))
    }
    fn send(&mut self, len: usize, fill: impl FnOnce(&mut [u8])) -> Result<(), Refusal> {
        copy_send(len, fill, |frame| self.try_send(frame))
    }
    fn collect(&mut self, _: &mut Back) {}
    fn idle(&mut self, _: &mut Back) -> bool {
        self.is_idle().unwrap_or(false)
    }
    fn longest(&self) -> usize {
        SLOT
    }
}

impl Port for Handle {
    fn take<R>(&mut self, f: impl FnOnce(&[u8], Chunk, Option<u32>) -> R) -> Option<R> {
        let mut buf = [0u8; 64];
        let (chunk, at) = self.read_chunk(&mut buf)?;
        Some(f(&buf[..chunk.len], chunk, Some(at)))
    }
    fn send(&mut self, len: usize, fill: impl FnOnce(&mut [u8])) -> Result<(), Refusal> {
        copy_send(len, fill, |frame| self.try_send(frame))
    }
    fn collect(&mut self, _: &mut Back) {}
    fn idle(&mut self, _: &mut Back) -> bool {
        self.is_idle()
    }
    fn longest(&self) -> usize {
        SLOT
    }
}

impl Port for LentUartHandle {
    fn take<R>(&mut self, f: impl FnOnce(&[u8], Chunk, Option<u32>) -> R) -> Option<R> {
        let got = self.next_chunk()?;
        Some(f(got.bytes(), got.chunk, Some(got.at)))
        // Dropping `got` returns the buffer to the port.
    }
    fn send(&mut self, len: usize, fill: impl FnOnce(&mut [u8])) -> Result<(), Refusal> {
        let mut frame = self.frame(len).map_err(Refusal::Frame)?;
        fill(frame.bytes_mut());
        frame.submit();
        Ok(())
    }
    fn collect(&mut self, back: &mut Back) {
        while let Some(frame) = self.sent() {
            back.frames += 1;
            if frame.outcome != Outcome::Sent {
                back.failed += 1;
            }
            let bytes = frame.bytes();
            if bytes.first() != Some(&(back.bytes as u8)) {
                back.disorder += 1;
            }
            back.bytes += bytes.len() as u32;
        }
    }
    fn idle(&mut self, back: &mut Back) -> bool {
        self.collect(back);
        self.outstanding() == 0
    }
    fn longest(&self) -> usize {
        LentUartHandle::longest(self)
    }
}

/// One port's loopback: a counter wrapping at 256 goes out in frames, and
/// every byte read is checked against the next value.
#[derive(Clone, Copy, Default)]
pub struct Loop {
    /// Stop sending at this many bytes.
    pub limit: u32,
    /// Send but do not read: the slow consumer.
    pub hold: bool,
    pub sent: u32,
    pub read: u32,
    pub wrong: u32,
    /// Chunks that ended in a gap: loss the consumer was told about.
    pub gaps: u32,
    /// Chunks that ended where the line went idle (frame ends, D13).
    pub idles: u32,
    pub chunks: u32,
    pub wakes: u32,
    /// Times the consumer found the port idle with everything sent (D15).
    pub all_sent: u32,
    /// Longest the consumer's step took, in cycles; in A, all of it is
    /// under the port's lock.
    pub longest_step: u32,
    /// Oldest chunk the consumer took, in cycles since the port took it
    /// (B and B2: A has no arrival time), before the flood.
    pub oldest: u32,
    /// The same from the flood on: after the stall.
    pub oldest_stall: u32,
    pub expected: u8,
    /// After `limit` bytes are back, send `flood` more without reading,
    /// pause, then read: the slow consumer.
    pub flood: u32,
    pub flooding: bool,
    /// `read` and `gaps` when the flood started.
    pub before: (u32, u32),
    /// Frames refused (D8), by the port or its handle.
    pub refused: u32,
    pub last_refusal: Option<Refusal>,
    /// Run the tail after the main run.
    pub tail: bool,
    /// Tail phases begun (0: still the main run).
    pub phase: u8,
    /// This phase's frame length.
    pub len: usize,
    /// Wakes when the main run's bytes were all back.
    pub main_wakes: u32,
    /// `read`, `idles` and `chunks` when the long frame went out.
    pub long_from: (u32, u32, u32),
    /// Frames back (B2).
    pub back: Back,
}

impl Loop {
    pub fn new(limit: u32) -> Self {
        Self {
            limit,
            len: FRAME,
            ..Self::default()
        }
    }

    /// Whether the consumer should now pause, outside any lock, and then
    /// call [`release`](Self::release).
    pub fn wants_pause(&self) -> bool {
        self.hold && self.sent >= self.limit
    }

    pub fn release(&mut self) {
        self.hold = false;
    }

    /// The long frame's bytes back, its frame ends and its chunks.
    pub fn long(&self) -> (u32, u32, u32) {
        if self.phase < TAIL.len() as u8 {
            return (0, 0, 0);
        }
        let (read, idles, chunks) = self.long_from;
        (self.read - read, self.idles - idles, self.chunks - chunks)
    }

    /// Starts the next tail phase, once everything sent is back.
    fn next_phase(&mut self) -> bool {
        let Some(&(count, len)) = TAIL.get(usize::from(self.phase)) else {
            return false;
        };
        if !self.tail || self.read < self.limit {
            return false;
        }
        if self.phase == 0 {
            self.main_wakes = self.wakes;
        }
        if len == LONG {
            self.long_from = (self.read, self.idles, self.chunks);
        }
        self.phase += 1;
        self.len = len;
        self.limit += count * len as u32;
        true
    }

    /// One wake: read what is there (unless holding), then send what fits.
    pub fn step<P: Port>(&mut self, port: &mut P, now: fn() -> u32) {
        self.wakes += 1;
        port.collect(&mut self.back);
        if !self.hold {
            for _ in 0..64 {
                let Some((chunk, ok, wrong)) = port.take(|bytes, chunk, at| {
                    if let Some(at) = at {
                        let age = now().wrapping_sub(at);
                        if self.flooding {
                            self.oldest_stall = self.oldest_stall.max(age);
                        } else {
                            self.oldest = self.oldest.max(age);
                        }
                    }
                    let mut wrong = 0;
                    for &byte in bytes {
                        if byte != self.expected {
                            wrong += 1;
                        }
                        self.expected = byte.wrapping_add(1);
                    }
                    (chunk, bytes.len(), wrong)
                }) else {
                    break;
                };
                self.chunks += 1;
                match chunk.end {
                    ChunkEnd::Idle => self.idles += 1,
                    ChunkEnd::Full => {}
                    ChunkEnd::Gap(_) => self.gaps += 1,
                }
                self.wrong += wrong;
                self.read += ok as u32;
            }
        }
        if self.flood > 0 && !self.flooding && self.read >= self.limit {
            self.flooding = true;
            self.hold = true;
            self.before = (self.read, self.gaps);
            self.limit += self.flood;
        }
        // The long frame goes in pieces, back to back, where it must.
        let burst = usize::from(self.phase) == TAIL.len() && self.len == LONG;
        for _ in 0..16 {
            // Frame by frame: the next goes out once the last is back, so
            // the line idles between frames as a protocol's would. The
            // flood sends without waiting.
            if self.sent >= self.limit {
                if self.next_phase() {
                    continue;
                }
                if port.idle(&mut self.back) {
                    self.all_sent += 1;
                }
                break;
            }
            if !self.hold && !burst && self.sent > self.read {
                break;
            }
            let len = self
                .len
                .min(port.longest())
                .min((self.limit - self.sent) as usize);
            let first = self.sent;
            match port.send(len, |frame| {
                for (i, byte) in frame.iter_mut().enumerate() {
                    *byte = (first as usize + i) as u8;
                }
            }) {
                Ok(()) => self.sent += len as u32,
                Err(refusal) => {
                    self.refused += 1;
                    self.last_refusal = Some(refusal);
                    break;
                }
            }
            if !self.hold && !burst {
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

/// The seven checks of the first prototype, and B2's per-frame and refusal
/// checks when `lent`.
pub fn report(variant: &str, name: &str, l: &Loop, lent: bool) -> u32 {
    defmt::info!(
        "{} {}: sent {} read {} wrong {} chunks {} idle-ended {} gaps {} wakes {} (main run {}) all-sent {} longest step {} cycles oldest chunk {} cycles, after the stall {} cycles",
        variant,
        name,
        l.sent,
        l.read,
        l.wrong,
        l.chunks,
        l.idles,
        l.gaps,
        l.wakes,
        l.main_wakes,
        l.all_sent,
        l.longest_step,
        l.oldest,
        l.oldest_stall
    );
    defmt::info!(
        "{} {} refused: {} frames, last {}; frames back {} failed {} out of order {} bytes {}",
        variant,
        name,
        l.refused,
        l.last_refusal.map_or("none", Refusal::name),
        l.back.frames,
        l.back.failed,
        l.back.disorder,
        l.back.bytes
    );
    let mut failed = 0;
    if l.flood == 0 {
        failed += verdict(
            "main run: every byte back, in order",
            l.read == l.limit && l.wrong == 0 && l.gaps == 0,
        );
    } else {
        let (read0, gaps0) = l.before;
        defmt::info!(
            "{} {} slow: {} of {} flooded bytes read, {} gaps reported to the consumer",
            variant,
            name,
            l.read - read0,
            l.flood,
            l.gaps - gaps0
        );
        failed += verdict(
            "main run: every byte back before the flood",
            read0 >= l.limit - l.flood,
        );
        failed += verdict(
            "slow: loss reported to the consumer as a gap",
            l.gaps > gaps0,
        );
    }
    failed += verdict("frame ends reach the consumer (idle chunks)", l.idles > 0);
    failed += verdict("the consumer learned everything was sent", l.all_sent > 0);
    if l.tail {
        let (back, ends, chunks) = l.long();
        defmt::info!(
            "{} {} long frame: {} of {} bytes back, {} frame ends, {} chunks",
            variant,
            name,
            back,
            LONG,
            ends,
            chunks
        );
        failed += verdict("long frame: every byte back", back == LONG as u32);
    }
    if lent {
        failed += verdict(
            "every frame back with its outcome, in order, sent",
            l.back.frames > 0
                && l.back.failed == 0
                && l.back.disorder == 0
                && l.back.bytes == l.sent,
        );
        if l.flood > 0 {
            failed += verdict(
                "refusal with every buffer out, usable after",
                l.refused > 0
                    && l.last_refusal == Some(Refusal::Frame(FrameError::Outstanding))
                    && l.sent == l.limit
                    && l.read >= l.before.0 + 256,
            );
        }
    }
    failed
}

/// The port's longest run and what it is made of, in cycles.
pub fn report_split(variant: &str, name: &str, s: &Split) {
    defmt::info!(
        "{} {} split: run {} ({} chunks, {} frames) on_interrupt {} read_chunk {} try_send {} wake {}; per chunk of 4/32/64/256 bytes {} {} {} {}",
        variant,
        name,
        s.run,
        s.run_chunks,
        s.run_frames,
        s.events,
        s.read_chunk,
        s.try_send,
        s.wake,
        s.per_chunk[0],
        s.per_chunk[1],
        s.per_chunk[2],
        s.per_chunk[3]
    );
}
