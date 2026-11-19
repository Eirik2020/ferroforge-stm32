//! The UART port: the port's members own one FerroHAL [`Uart`], and the
//! consumer owns a [`Handle`] that names no UART, wiring, vector or lock.
//!
//! Bytes cross between them in memory the firmware gives at boot, lent back
//! and forth by reference:
//!
//! - **Transmit: the consumer lends.** The handle keeps its transmit buffers.
//!   [`Handle::frame`] takes a free one long enough, or refuses
//!   ([`FrameError`]); the consumer writes the frame in place, and
//!   [`Frame::submit`] lends it to the port and pends the port. The port hands
//!   the bytes to FerroHAL's `try_send`, the one copy left, and keeps the
//!   frame in flight until FerroHAL gives its outcome back. The buffer then
//!   comes back through [`Handle::sent`], in the order submitted, with that
//!   [`Outcome`]. [`Handle::drained`] says when the line has gone quiet: the
//!   last frame's stop bit has left the wire, which comes up to two
//!   characters after its `Sent`.
//! - **Receive: the port lends.** The port takes a buffer from its pool and
//!   has FerroHAL's `read_chunk` copy the ring straight into it, the one copy
//!   left, and lends it to the consumer with FerroHAL's [`Chunk`] (its
//!   length, why it ended, its generation) and the time the port was served.
//!   [`Handle::next_chunk`] reads it in place; dropping the [`Received`]
//!   returns the buffer. With no buffer free the bytes stay in FerroHAL's
//!   ring and the port marks itself starved: the next release pends it. Loss
//!   happens only when FerroHAL's ring laps, and FerroHAL reports it as a gap
//!   chunk at its place in the stream.
//!
//! Four single-producer, single-consumer queues of references carry the
//! buffers: frames out and back, chunks out and back. Every handle has one
//! type whatever its UART and buffers, so `init` can give any handle to any
//! task. The port wakes whichever task holds the handle through a signal in
//! the handle, so nothing is spawned. Nothing here panics; a buffer a queue
//! could not take is counted ([`Handle::lost`]), never dropped silently.
//!
//! The port carries no protocol: a decoder or encoder sits behind the handle.
//!
//! Requirements: PORT-001, PORT-002, PORT-003, PORT-004, PORT-005, PORT-006,
//! PORT-007, PORT-008, PORT-009, PORT-010, PORT-011, PORT-012

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use ferrohal_stm32::uart::{
    Chunk, ChunkEnd, Error, Instance, RxDma, RxPin, SendError, TX_FRAMES, TxDma, TxOutcome, TxPin,
    Uart, UartRefused, Wiring,
};
use heapless::{
    Deque, Vec, VecView,
    spsc::{Consumer, Producer, Queue},
};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

/// The most frames the port keeps in FerroHAL's transmit queue: FerroHAL's
/// own depth.
pub const IN_FLIGHT: usize = TX_FRAMES;

/// The most chunks, and the most frames, one service moves, so a service is
/// bounded however fast the line runs.
const PER_SERVICE: usize = 16;

/// How a lent frame ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// FerroHAL's transmit stream finished it: every byte was handed to the
    /// USART. The last one or two may still be on the wire.
    Sent,
    /// A DMA error ended it; part of it may have gone out.
    Failed,
    /// FerroHAL dropped its queue first, by a timeout or a transmit stream
    /// that would not run; part of it may have gone out.
    Dropped,
    /// FerroHAL overwrote its outcome before the port took it, so how it
    /// ended is not known ([`Port::lost_outcomes`]). The port takes every
    /// outcome in each service, so this does not happen unless something is
    /// broken.
    Lost,
    /// FerroHAL refused it: only `TooLarge`, which the constructor's length
    /// check rules out.
    Refused(SendError),
}

/// Why [`Handle::frame`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// No transmit buffer the handle was given is that long.
    TooLong,
    /// Every buffer long enough is out: lent to the port, or back and not
    /// yet taken through [`Handle::sent`].
    Outstanding,
    /// A frame of no bytes.
    Empty,
}

/// A frame lent to the port. Its size does not depend on the frame's.
struct TxLent {
    buf: &'static mut [u8],
    len: u16,
}

/// A frame coming back.
struct TxBack {
    lent: TxLent,
    outcome: Outcome,
}

/// A chunk lent to the consumer.
struct RxLent {
    buf: &'static mut [u8],
    chunk: Chunk,
    at_us: u64,
}

/// What the port and its handle both see outside the queues.
struct Status {
    /// The port had bytes to read and no buffer: the next release pends it.
    starved: AtomicBool,
    /// The line has gone quiet: FerroHAL saw the last stop bit leave, and
    /// no frame is lent or in flight. Set by the port at each service.
    drained: AtomicBool,
    /// Buffers a queue could not take. Zero by construction, since each
    /// queue holds every buffer of its kind; counted so a fault is not
    /// hidden.
    lost: AtomicU32,
}

/// The memory the port and its handle share: the firmware's, given once to
/// [`uart_port::new`] (an `init` local), which splits it. `RX` and `TX` are queue slots:
/// the pool may hold at most `RX - 1` receive buffers and the handle at most
/// `TX - 1` transmit buffers, the queues' capacity.
pub struct Memory<const RX: usize, const TX: usize> {
    free: Vec<&'static mut [u8], TX>,
    frames: Queue<TxLent, TX>,
    sent: Queue<TxBack, TX>,
    chunks: Queue<RxLent, RX>,
    back: Queue<&'static mut [u8], RX>,
    status: Status,
    wake: Signal<()>,
}

impl<const RX: usize, const TX: usize> Memory<RX, TX> {
    pub const fn new() -> Self {
        Self {
            free: Vec::new(),
            frames: Queue::new(),
            sent: Queue::new(),
            chunks: Queue::new(),
            back: Queue::new(),
            status: Status {
                starved: AtomicBool::new(false),
                drained: AtomicBool::new(false),
                lost: AtomicU32::new(0),
            },
            wake: Signal::new(),
        }
    }
}

impl<const RX: usize, const TX: usize> Default for Memory<RX, TX> {
    fn default() -> Self {
        Self::new()
    }
}

/// The consumer's end of one UART port, owned by one task as a `#[local]`.
/// Every port's handle has this type, whatever its UART and buffers.
///
/// A frame's buffer stays out until its outcome is taken through
/// [`sent`](Self::sent): a consumer that never takes them runs out of
/// buffers, and [`frame`](Self::frame) refuses with
/// [`FrameError::Outstanding`]. It needs as many transmit buffers as frames
/// it wants in flight at once.
pub struct Handle {
    free: &'static mut VecView<&'static mut [u8]>,
    frames: Producer<'static, TxLent>,
    sent: Consumer<'static, TxBack>,
    chunks: Consumer<'static, RxLent>,
    back: Producer<'static, &'static mut [u8]>,
    status: &'static Status,
    wake: SignalReader<'static, ()>,
    /// Transmit buffers given at boot, and the longest of them.
    buffers: u8,
    longest: usize,
    kick: fn(),
}

impl Handle {
    /// Waits until the port has news: a chunk, a frame back, or a fault.
    /// Wakes coalesce: one wake may stand for several of them, so drain
    /// [`next_chunk`](Self::next_chunk) and [`sent`](Self::sent) after it.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// The oldest chunk received, read in the port's buffer. Dropping it
    /// returns the buffer to the port.
    pub fn next_chunk(&mut self) -> Option<Received<'_>> {
        let lent = self.chunks.dequeue()?;
        Some(Received {
            buf: Some(lent.buf),
            chunk: lent.chunk,
            at_us: lent.at_us,
            handle: self,
        })
    }

    /// Takes the shortest free transmit buffer that holds `len` bytes, for a
    /// frame of exactly `len` bytes written in place.
    pub fn frame(&mut self, len: usize) -> Result<Frame<'_>, FrameError> {
        if len == 0 {
            return Err(FrameError::Empty);
        }
        if len > self.longest {
            return Err(FrameError::TooLong);
        }
        let mut best: Option<(usize, usize)> = None;
        for (i, buf) in self.free.iter().enumerate() {
            let size = buf.len();
            if size >= len && best.is_none_or(|(_, b)| size < b) {
                best = Some((i, size));
            }
        }
        let (i, _) = best.ok_or(FrameError::Outstanding)?;
        let buf = self.remove(i).ok_or(FrameError::Outstanding)?;
        Ok(Frame {
            buf: Some(buf),
            // `len <= longest`, and the constructor keeps every buffer within
            // FerroHAL's transmit buffer, whose length fits a u16.
            len: u16::try_from(len).unwrap_or(u16::MAX),
            handle: self,
        })
    }

    /// The oldest frame back from the port, in its buffer, with its outcome.
    /// Frames come back in the order submitted. Dropping it frees the buffer.
    pub fn sent(&mut self) -> Option<SentFrame<'_>> {
        let back = self.sent.dequeue()?;
        Some(SentFrame {
            buf: Some(back.lent.buf),
            len: back.lent.len,
            outcome: back.outcome,
            handle: self,
        })
    }

    /// Transmit buffers not free: lent to the port, or back and not yet
    /// taken. Zero means every frame submitted has finished and been
    /// accounted for.
    pub fn outstanding(&self) -> u8 {
        let free = u8::try_from(self.free.len()).unwrap_or(u8::MAX);
        self.buffers.saturating_sub(free)
    }

    /// Whether the line has gone quiet: every frame submitted has finished,
    /// and FerroHAL saw the USART's transmission complete after the last, so
    /// its last stop bit has left the wire. The port wakes the handle when
    /// it becomes so. A frame's `Sent` comes up to two characters earlier.
    pub fn drained(&self) -> bool {
        self.status.drained.load(Ordering::Acquire)
    }

    /// Transmit buffers given at boot.
    pub fn buffers(&self) -> u8 {
        self.buffers
    }

    /// The longest frame this handle can send: its longest buffer.
    pub fn longest(&self) -> usize {
        self.longest
    }

    /// Buffers lost to a full queue or free list: zero unless something is
    /// broken.
    pub fn lost(&self) -> u32 {
        self.status.lost.load(Ordering::Relaxed)
    }

    /// Removes free buffer `i` without shifting the others.
    fn remove(&mut self, i: usize) -> Option<&'static mut [u8]> {
        let last = self.free.pop()?;
        if i == self.free.len() {
            return Some(last);
        }
        self.free
            .get_mut(i)
            .map(|slot| core::mem::replace(slot, last))
    }

    fn give_back(&mut self, buf: &'static mut [u8]) {
        if self.free.push(buf).is_err() {
            self.status.lost.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Returns a receive buffer to the port, and pends it if it was starved
    /// for one.
    fn release(&mut self, buf: &'static mut [u8]) {
        if self.back.enqueue(buf).is_err() {
            self.status.lost.fetch_add(1, Ordering::Relaxed);
        }
        if self.status.starved.swap(false, Ordering::AcqRel) {
            (self.kick)();
        }
    }
}

/// A received chunk, read in place. Dropping it returns the buffer.
pub struct Received<'h> {
    handle: &'h mut Handle,
    buf: Option<&'static mut [u8]>,
    /// Its length, why it ended and its generation, as FerroHAL gave them. A
    /// chunk that ended in `ChunkEnd::Gap` marks where bytes were lost or
    /// damaged.
    pub chunk: Chunk,
    /// When the port was served and took it from FerroHAL's ring: the
    /// firmware monotonic's time read in that interrupt, in microseconds
    /// since its epoch.
    pub at_us: u64,
}

impl Received<'_> {
    /// The chunk's bytes.
    pub fn bytes(&self) -> &[u8] {
        self.buf
            .as_deref()
            .and_then(|b| b.get(..self.chunk.len))
            .unwrap_or(&[])
    }
}

impl Drop for Received<'_> {
    fn drop(&mut self) {
        if let Some(buf) = self.buf.take() {
            self.handle.release(buf);
        }
    }
}

/// A frame being written in a lent buffer. Dropping it unsent frees the
/// buffer.
pub struct Frame<'h> {
    handle: &'h mut Handle,
    buf: Option<&'static mut [u8]>,
    len: u16,
}

impl Frame<'_> {
    /// The frame's bytes, to write in place.
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        let len = usize::from(self.len);
        self.buf
            .as_deref_mut()
            .and_then(|b| b.get_mut(..len))
            .unwrap_or(&mut [])
    }

    /// Lends the buffer to the port and pends it. Its outcome comes back
    /// through [`Handle::sent`].
    pub fn submit(mut self) {
        let Some(buf) = self.buf.take() else { return };
        // Not quiet any more, before the port can see the frame.
        self.handle.status.drained.store(false, Ordering::Release);
        if let Err(back) = self.handle.frames.enqueue(TxLent { buf, len: self.len }) {
            // Cannot happen: the queue holds every buffer the handle has.
            self.handle.give_back(back.buf);
            self.handle.status.lost.fetch_add(1, Ordering::Relaxed);
            return;
        }
        (self.handle.kick)();
    }
}

impl Drop for Frame<'_> {
    fn drop(&mut self) {
        if let Some(buf) = self.buf.take() {
            self.handle.give_back(buf);
        }
    }
}

/// A frame back from the port. Dropping it frees the buffer.
pub struct SentFrame<'h> {
    handle: &'h mut Handle,
    buf: Option<&'static mut [u8]>,
    len: u16,
    /// How it ended.
    pub outcome: Outcome,
}

impl SentFrame<'_> {
    /// The frame as it was submitted, to tell frames apart.
    pub fn bytes(&self) -> &[u8] {
        let len = usize::from(self.len);
        self.buf
            .as_deref()
            .and_then(|b| b.get(..len))
            .unwrap_or(&[])
    }
}

impl Drop for SentFrame<'_> {
    fn drop(&mut self) {
        if let Some(buf) = self.buf.take() {
            self.handle.give_back(buf);
        }
    }
}

/// The port's transmit side: the frames lent to it and those in FerroHAL's
/// queue.
struct TxEnd {
    frames: Consumer<'static, TxLent>,
    sent: Producer<'static, TxBack>,
    /// In FerroHAL's queue, oldest first: the order FerroHAL gives their
    /// outcomes back.
    in_flight: Deque<TxLent, IN_FLIGHT>,
    /// Frames whose outcome FerroHAL overwrote before it was taken.
    lost_outcomes: u32,
}

impl TxEnd {
    /// Returns the oldest frame in flight with FerroHAL's outcome for it. One
    /// `Lost(n)` stands for the oldest `n` frames. Returns the frames given
    /// back.
    fn settle(&mut self, outcome: TxOutcome, status: &Status) -> usize {
        let (frames, outcome) = match outcome {
            TxOutcome::Sent => (1, Outcome::Sent),
            TxOutcome::Failed => (1, Outcome::Failed),
            TxOutcome::Dropped => (1, Outcome::Dropped),
            TxOutcome::Lost(n) => {
                self.lost_outcomes = self.lost_outcomes.saturating_add(n);
                (n, Outcome::Lost)
            }
        };
        let mut returned = 0;
        for _ in 0..frames {
            let Some(lent) = self.in_flight.pop_front() else {
                break;
            };
            self.give_back(lent, outcome, status);
            returned += 1;
        }
        returned
    }

    fn give_back(&mut self, lent: TxLent, outcome: Outcome, status: &Status) {
        if self.sent.enqueue(TxBack { lent, outcome }).is_err() {
            // Cannot happen: the queue holds every buffer the handle has.
            status.lost.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// The port's receive side: its pool, as returned buffers and one spare.
struct RxEnd {
    chunks: Producer<'static, RxLent>,
    back: Consumer<'static, &'static mut [u8]>,
    /// A buffer taken for a chunk that was not complete yet.
    spare: Option<&'static mut [u8]>,
}

impl RxEnd {
    /// A free buffer, or `None` with the port marked starved. The flag is
    /// set before the queue is looked at, so a release racing this, at any
    /// priority, either is seen here or sees the flag and pends the port.
    fn take(&mut self, status: &Status) -> Option<&'static mut [u8]> {
        if let Some(buf) = self.spare.take() {
            return Some(buf);
        }
        status.starved.store(true, Ordering::Release);
        let buf = self.back.dequeue()?;
        status.starved.store(false, Ordering::Release);
        Some(buf)
    }

    /// Lends `buf` to the consumer, or keeps it as the spare when the queue
    /// is full (it holds the whole pool, so it is not).
    fn lend(&mut self, buf: &'static mut [u8], chunk: Chunk, at_us: u64, status: &Status) -> bool {
        match self.chunks.enqueue(RxLent { buf, chunk, at_us }) {
            Ok(()) => true,
            Err(back) => {
                self.spare = Some(back.buf);
                status.lost.fetch_add(1, Ordering::Relaxed);
                false
            }
        }
    }
}

/// A port its members serve with the time they read in the interrupt.
/// [`Port`] is the one implementation; the members are bounded on this
/// trait so one definition serves every UART.
pub trait Serve {
    /// Serves whatever is pending on any of the port's vectors, stamping
    /// what it hands the consumer with `at_us`.
    fn serve(&mut self, at_us: u64);
}

/// The port's own state, shared by its three members only: the UART and the
/// port's ends of the queues.
pub struct Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>,
    tx: TxEnd,
    rx: RxEnd,
    status: &'static Status,
    wake: SignalWriter<'static, ()>,
}

impl<U, Tx, Rx, TxD, RxD> Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    /// FerroHAL's UART, to read its health counters.
    pub fn uart(&self) -> &Uart<Wiring<U, Tx, Rx, TxD, RxD>> {
        &self.uart
    }

    /// Frames whose outcome FerroHAL overwrote before the port took it:
    /// zero unless something is broken.
    pub fn lost_outcomes(&self) -> u32 {
        self.tx.lost_outcomes
    }

    /// Gives back every frame FerroHAL has finished, with its outcome.
    fn settle(&mut self) -> usize {
        let mut returned = 0;
        // Bounded: FerroHAL keeps at most `TX_FRAMES` outcomes.
        for _ in 0..=TX_FRAMES {
            let Some(outcome) = self.uart.take_outcome() else {
                break;
            };
            returned += self.tx.settle(outcome, self.status);
        }
        returned
    }

    /// Lent frames into FerroHAL, in order. Returns the frames returned.
    fn pump_tx(&mut self) -> usize {
        let mut returned = 0;
        for _ in 0..PER_SERVICE {
            if self.tx.in_flight.is_full() {
                break;
            }
            let Some(lent) = self.tx.frames.peek() else {
                break;
            };
            let result = match lent.buf.get(..usize::from(lent.len)) {
                Some(bytes) => self.uart.try_send(bytes),
                None => Err(SendError::TooLarge),
            };
            let full = result == Err(SendError::Full);
            if !full && let Some(lent) = self.tx.frames.dequeue() {
                match result {
                    Ok(()) => {
                        // Room was checked above.
                        if let Err(lent) = self.tx.in_flight.push_back(lent) {
                            self.tx.give_back(lent, Outcome::Dropped, self.status);
                            returned += 1;
                        }
                    }
                    Err(SendError::Halted) => {
                        self.tx.give_back(lent, Outcome::Dropped, self.status);
                        returned += 1;
                    }
                    Err(refused) => {
                        self.tx
                            .give_back(lent, Outcome::Refused(refused), self.status);
                        returned += 1;
                    }
                }
            }
            // `try_send` serves FerroHAL's queue: a frame may have finished.
            returned += self.settle();
            if full {
                break;
            }
        }
        returned
    }

    /// The ring into lent buffers, whole chunks. Returns the chunks lent and
    /// the frames returned (`read_chunk` serves FerroHAL's transmit queue
    /// too).
    fn pump_rx(&mut self, at_us: u64) -> (usize, usize) {
        let (mut chunks, mut returned) = (0, 0);
        for _ in 0..PER_SERVICE {
            let Some(buf) = self.rx.take(self.status) else {
                break;
            };
            let result = self.uart.read_chunk(buf);
            returned += self.settle();
            let chunk = match result {
                Ok(Some(chunk)) => chunk,
                Ok(None) => {
                    self.rx.spare = Some(buf);
                    break;
                }
                // Only `Halted`: reported as a gap chunk, never dropped.
                Err(error) => Chunk {
                    len: 0,
                    end: ChunkEnd::Gap(error),
                    generation: self.uart.generation(),
                },
            };
            if !self.rx.lend(buf, chunk, at_us, self.status) {
                break;
            }
            chunks += 1;
            if chunk.end == ChunkEnd::Gap(Error::Halted) {
                break;
            }
        }
        (chunks, returned)
    }
}

impl<U, Tx, Rx, TxD, RxD> Serve for Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    fn serve(&mut self, at_us: u64) {
        let events = self.uart.on_interrupt();
        let mut returned = self.settle();
        returned += self.pump_tx();
        let (chunks, back) = self.pump_rx(at_us);
        returned += back;
        // Quiet: FerroHAL saw the last stop bit leave, and nothing is lent or
        // waiting to go.
        let drained =
            self.uart.tx_drained() && self.tx.in_flight.is_empty() && !self.tx.frames.ready();
        let was = self.status.drained.swap(drained, Ordering::AcqRel);
        if chunks > 0 || returned > 0 || events.fault || (drained && !was) {
            self.wake.write(());
        }
    }
}

/// What [`uart_port::new`] builds: the port's resource and the consumer's handle.
pub struct Resources<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    /// Shared by the port's three members only, bound as the group's `port`.
    pub port: Port<U, Tx, Rx, TxD, RxD>,
    /// The consumer's, as a `#[local]` of whichever task `init` gives it to.
    pub handle: Handle,
}

/// Why [`uart_port::new`] refused. Nothing is ever returned opened: a refusal gives back
/// the wiring and the buffers, so the firmware can use them elsewhere.
pub enum Refused<W> {
    /// FerroHAL refused the UART: its reason, with the wiring and buffers.
    Open(UartRefused<W>),
    /// A lent buffer the port cannot use, found before the UART was opened:
    /// in the receive pool (`receive`), one empty, of another length than
    /// the first, longer than half the receive ring, or one more than the
    /// queues hold; among the transmit buffers, one empty, longer than
    /// FerroHAL's transmit buffer, or one more than the queues hold. `len` is
    /// the offending buffer's length. The memory stays split, which is
    /// harmless: a refused port stops the firmware's boot.
    Buffer {
        receive: bool,
        len: usize,
        wiring: W,
        tx_buf: &'static mut [u8],
        rx_buf: &'static mut [u8],
    },
}

/// The port's ends and the handle, split from `memory`. A buffer it cannot
/// use comes back as `Err((receive, len))`.
#[allow(clippy::type_complexity)]
fn split_memory<const RX: usize, const TX: usize>(
    memory: &'static mut Memory<RX, TX>,
    rx_pool: &mut dyn Iterator<Item = &'static mut [u8]>,
    rx_longest: usize,
    tx_lent: &mut dyn Iterator<Item = &'static mut [u8]>,
    tx_longest: usize,
    kick: fn(),
) -> Result<
    (
        TxEnd,
        RxEnd,
        &'static Status,
        SignalWriter<'static, ()>,
        Handle,
    ),
    (bool, usize),
> {
    let Memory {
        free,
        frames,
        sent,
        chunks,
        back,
        status,
        wake,
    } = memory;
    let mut pool_len = None;
    for buf in rx_pool {
        let len = buf.len();
        if len == 0 || len > rx_longest || pool_len.is_some_and(|l| l != len) {
            return Err((true, len));
        }
        pool_len = Some(len);
        // The pool starts in the return queue, the port's source of buffers.
        if back.enqueue(buf).is_err() {
            return Err((true, len));
        }
    }
    let room = frames.capacity();
    let mut buffers = 0u8;
    let mut longest = 0;
    for buf in tx_lent {
        let len = buf.len();
        if len == 0 || len > tx_longest || usize::from(buffers) >= room {
            return Err((false, len));
        }
        if free.push(buf).is_err() {
            return Err((false, len));
        }
        buffers = buffers.saturating_add(1);
        longest = longest.max(len);
    }
    let status: &'static Status = status;
    let wake: &'static Signal<()> = wake;
    let (frames_in, frames_out) = frames.split();
    let (sent_in, sent_out) = sent.split();
    let (chunks_in, chunks_out) = chunks.split();
    let (back_in, back_out) = back.split();
    let (writer, reader) = wake.split();
    Ok((
        TxEnd {
            frames: frames_out,
            sent: sent_in,
            in_flight: Deque::new(),
            lost_outcomes: 0,
        },
        RxEnd {
            chunks: chunks_in,
            back: back_out,
            spare: None,
        },
        status,
        writer,
        Handle {
            free: free.as_mut_view(),
            frames: frames_in,
            sent: sent_out,
            chunks: chunks_out,
            back: back_in,
            status,
            wake: reader,
            buffers,
            longest,
            kick,
        },
    ))
}

/// The UART port group. A firmware selects it once per UART, binds the
/// port's resource, and gives the handle to any task:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::uart_port, shared = [port = usart1])]
/// mod usart1 {
///     #[task(binds = USART1, priority = 3)] fn on_uart;
///     #[task(binds = DMA2_STREAM2, priority = 3)] fn on_rx_dma;
///     #[task(binds = DMA2_STREAM7, priority = 3)] fn on_tx_dma;
/// }
/// // in init:
/// let built = uart_port::new(&mut rcc, wiring, &FRAME, tx_buf, rx_buf,
///     memory, &mut pool, &mut lent, usart1::kick)?;
/// ```
///
/// The members share `port`; one priority for all three makes RTIC's lock
/// among them free, but the firmware decides. Each stamps its service with
/// the firmware's monotonic, read in the interrupt.
#[ferroforge::group(
    slots = [instance: U, tx: Tx, rx: Rx, tx_dma: TxD, rx_dma: RxD],
    vectors = [on_uart = instance, on_rx_dma = rx_dma, on_tx_dma = tx_dma],
    shared = [port: Port<U, Tx, Rx, TxD, RxD>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [
        config: Config @ platform,
        rx_pool: &mut dyn Iterator<Item = &'static mut [u8]> @ software,
        tx_lent: &mut dyn Iterator<Item = &'static mut [u8]> @ software,
    ],
    memory = [memory: Memory],
    handles = [handle: Handle],
    pends = [kick = on_tx_dma],
    constructor = new(rcc, wiring, config, tx_buf, rx_buf, memory, rx_pool, tx_lent, kick),
)]
pub mod uart_port {
    // Private: a `pub use` here would include a group (FerroForge's
    // `group`). The types are `ferroforge_stm32::uart`'s.
    use super::{Memory, Port, Refused, Resources, Serve, split_memory};
    use ferrohal_stm32::{
        rcc::Rcc,
        uart::{Buffers, Config, Instance, RxDma, RxPin, TxDma, TxPin, Uart, Wiring},
    };

    /// Checks the lent buffers, then opens the UART on `wiring` and splits
    /// `memory`.
    ///
    /// - `tx_buf` is FerroHAL's transmit buffer and `rx_buf` its receive ring
    ///   (FerroHAL's `uart::Buffers` states their limits).
    /// - `rx_pool` is the port's receive pool: buffers of one length, each at
    ///   most half `rx_buf`, since a longer one would let a continuous stream
    ///   lap the ring before a chunk is taken.
    /// - `tx_lent` are the handle's transmit buffers, each at most `tx_buf`'s
    ///   length.
    /// - `kick` is the selection's generated pend of the transmit stream's
    ///   vector (the contract's `pends`).
    ///
    /// The buffers are checked before the UART is opened, so a refusal never
    /// carries an opened driver.
    #[allow(clippy::type_complexity, clippy::too_many_arguments)]
    pub fn new<U, Tx, Rx, TxD, RxD, const RX: usize, const TX: usize>(
        rcc: &mut Rcc,
        wiring: Wiring<U, Tx, Rx, TxD, RxD>,
        config: &Config,
        tx_buf: &'static mut [u8],
        rx_buf: &'static mut [u8],
        memory: &'static mut Memory<RX, TX>,
        rx_pool: &mut dyn Iterator<Item = &'static mut [u8]>,
        tx_lent: &mut dyn Iterator<Item = &'static mut [u8]>,
        kick: fn(),
    ) -> Result<Resources<U, Tx, Rx, TxD, RxD>, Refused<Wiring<U, Tx, Rx, TxD, RxD>>>
    where
        U: Instance,
        Tx: TxPin<U>,
        Rx: RxPin<U>,
        TxD: TxDma<U>,
        RxD: RxDma<U>,
    {
        let (tx_longest, rx_longest) = (tx_buf.len(), rx_buf.len() / 2);
        let (tx, rx, status, wake, handle) =
            match split_memory(memory, rx_pool, rx_longest, tx_lent, tx_longest, kick) {
                Ok(ends) => ends,
                Err((receive, len)) => {
                    return Err(Refused::Buffer {
                        receive,
                        len,
                        wiring,
                        tx_buf,
                        rx_buf,
                    });
                }
            };
        let buffers = Buffers {
            tx: tx_buf,
            rx: rx_buf,
        };
        let mut uart = Uart::new(rcc, wiring, config, buffers).map_err(Refused::Open)?;
        // The line's quiet comes by the USART's interrupt, which `on_uart`
        // serves; the port never moves the UART out of its members' reach.
        uart.set_tx_idle_interrupt(true);
        Ok(Resources {
            port: Port {
                uart,
                tx,
                rx,
                status,
                wake,
            },
            handle,
        })
    }

    /// The USART's own vector: idle line, line errors.
    #[ferroforge::task(shared = [port], bounds = [port: Serve], monotonic = Mono)]
    pub fn on_uart(mut cx: on_uart::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        cx.shared.port.lock(|port| port.serve(at_us));
    }

    /// The receive stream's vector: the ring's half and end, DMA errors.
    #[ferroforge::task(shared = [port], bounds = [port: Serve], monotonic = Mono)]
    pub fn on_rx_dma(mut cx: on_rx_dma::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        cx.shared.port.lock(|port| port.serve(at_us));
    }

    /// The transmit stream's vector: a frame finished, the next started.
    /// The handle pends it to pass a submitted frame on, or to serve a
    /// starved receive side.
    #[ferroforge::task(shared = [port], bounds = [port: Serve], monotonic = Mono)]
    pub fn on_tx_dma(mut cx: on_tx_dma::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        cx.shared.port.lock(|port| port.serve(at_us));
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::AtomicUsize;
    use std::{boxed::Box, vec, vec::Vec as StdVec};

    static KICKS: AtomicUsize = AtomicUsize::new(0);
    fn kick() {
        KICKS.fetch_add(1, Ordering::Relaxed);
    }

    fn leak(len: usize) -> &'static mut [u8] {
        Box::leak(vec![0u8; len].into_boxed_slice())
    }

    type Ends = (
        TxEnd,
        RxEnd,
        &'static Status,
        SignalWriter<'static, ()>,
        Handle,
    );

    /// Memory with a pool of `rx` receive buffers of 8 bytes and transmit
    /// buffers of the lengths given.
    fn ends(rx: usize, tx: &[usize]) -> Ends {
        let memory: &'static mut Memory<4, 4> = Box::leak(Box::new(Memory::new()));
        let mut pool = (0..rx).map(|_| leak(8));
        let mut lent = tx.iter().map(|&n| leak(n));
        split_memory(memory, &mut pool, 16, &mut lent, 64, kick).unwrap_or_else(|_| panic!())
    }

    #[test]
    fn a_frame_is_refused_until_a_buffer_is_free_and_comes_back_in_order() {
        let (mut tx, _rx, status, _w, mut h) = ends(1, &[16, 32]);
        assert_eq!(h.frame(0).err(), Some(FrameError::Empty));
        assert_eq!(h.frame(33).err(), Some(FrameError::TooLong));
        // The shortest that fits, written in place.
        let mut f = h.frame(10).unwrap();
        f.bytes_mut().copy_from_slice(b"0123456789");
        let before = KICKS.load(Ordering::Relaxed);
        f.submit();
        assert!(KICKS.load(Ordering::Relaxed) > before);
        let mut g = h.frame(20).unwrap();
        g.bytes_mut()[0] = b'B';
        g.submit();
        assert_eq!(h.frame(1).err(), Some(FrameError::Outstanding));
        assert_eq!(h.outstanding(), 2);
        // The port: both into FerroHAL, then FerroHAL gives one back sent
        // and one failed.
        for _ in 0..2 {
            let lent = tx.frames.dequeue().unwrap();
            assert!(tx.in_flight.push_back(lent).is_ok());
        }
        assert_eq!(tx.settle(TxOutcome::Sent, status), 1);
        assert_eq!(tx.settle(TxOutcome::Failed, status), 1);
        let first = h.sent().unwrap();
        assert_eq!(
            (first.outcome, first.bytes()),
            (Outcome::Sent, &b"0123456789"[..])
        );
        drop(first);
        let second = h.sent().unwrap();
        assert_eq!(second.outcome, Outcome::Failed);
        assert_eq!((second.bytes().len(), second.bytes()[0]), (20, b'B'));
        drop(second);
        assert_eq!(h.outstanding(), 0);
        assert_eq!(h.lost(), 0);
        // A frame dropped unsent frees its buffer.
        drop(h.frame(5).unwrap());
        assert_eq!(h.outstanding(), 0);
    }

    #[test]
    fn each_frame_takes_ferrohal_s_outcome_in_order_and_a_lost_outcome_is_counted() {
        let (mut tx, _rx, status, _w, mut h) = ends(1, &[8, 8, 8]);
        for _ in 0..3 {
            h.frame(4).unwrap().submit();
            let lent = tx.frames.dequeue().unwrap();
            assert!(tx.in_flight.push_back(lent).is_ok());
        }
        assert_eq!(tx.settle(TxOutcome::Dropped, status), 1);
        // One `Lost(1)` stands for the next frame.
        assert_eq!(tx.settle(TxOutcome::Lost(1), status), 1);
        assert_eq!(tx.lost_outcomes, 1);
        assert_eq!(tx.settle(TxOutcome::Sent, status), 1);
        // An outcome with nothing in flight gives nothing back.
        assert_eq!(tx.settle(TxOutcome::Lost(2), status), 0);
        let outcomes: StdVec<_> = core::iter::from_fn(|| h.sent().map(|s| s.outcome)).collect();
        assert_eq!(outcomes, [Outcome::Dropped, Outcome::Lost, Outcome::Sent]);
        assert_eq!(h.outstanding(), 0);
    }

    #[test]
    fn a_submitted_frame_ends_the_line_s_quiet_until_the_port_sees_it_again() {
        let (_tx, _rx, status, _w, mut h) = ends(1, &[8]);
        status.drained.store(true, Ordering::Release);
        assert!(h.drained());
        h.frame(4).unwrap().submit();
        assert!(!h.drained());
    }

    #[test]
    fn a_chunk_is_read_in_place_and_a_starved_port_is_pended_on_release() {
        let (_tx, mut rx, status, _w, mut h) = ends(2, &[8]);
        let chunk = Chunk {
            len: 3,
            end: ChunkEnd::Idle,
            generation: 4,
        };
        for byte in *b"ab" {
            let buf = rx.take(status).unwrap();
            buf[..3].fill(byte);
            assert!(rx.lend(buf, chunk, 9, status));
        }
        // Pool empty: the bytes stay in FerroHAL's ring, the port starved.
        assert!(rx.take(status).is_none());
        assert!(status.starved.load(Ordering::Acquire));
        let got = h.next_chunk().unwrap();
        assert_eq!((got.bytes(), got.chunk, got.at_us), (&b"aaa"[..], chunk, 9));
        let before = KICKS.load(Ordering::Relaxed);
        drop(got);
        assert!(KICKS.load(Ordering::Relaxed) > before);
        assert!(!status.starved.load(Ordering::Acquire));
        // The released buffer is the port's again; the other is still lent.
        assert!(rx.take(status).is_some());
        assert_eq!(h.next_chunk().unwrap().bytes(), b"bbb");
        assert!(h.next_chunk().is_none());
        assert_eq!(h.lost(), 0);
    }

    #[test]
    fn a_gap_chunk_reaches_the_consumer_at_its_place() {
        let (_tx, mut rx, status, _w, mut h) = ends(3, &[8]);
        let idle = Chunk {
            len: 2,
            end: ChunkEnd::Idle,
            generation: 0,
        };
        let gap = Chunk {
            len: 0,
            end: ChunkEnd::Gap(Error::BufferOverrun),
            generation: 1,
        };
        for chunk in [
            idle,
            gap,
            Chunk {
                generation: 1,
                ..idle
            },
        ] {
            let buf = rx.take(status).unwrap();
            assert!(rx.lend(buf, chunk, 0, status));
        }
        let ends: StdVec<_> =
            core::iter::from_fn(|| h.next_chunk().map(|c| (c.chunk.end, c.chunk.generation)))
                .collect();
        assert_eq!(
            ends,
            [
                (ChunkEnd::Idle, 0),
                (ChunkEnd::Gap(Error::BufferOverrun), 1),
                (ChunkEnd::Idle, 1)
            ]
        );
    }

    #[test]
    fn buffers_the_port_cannot_use_are_refused() {
        let memory = || -> &'static mut Memory<4, 4> { Box::leak(Box::new(Memory::new())) };
        let none = || core::iter::empty::<&'static mut [u8]>();
        // Receive: two lengths, longer than half the ring, more than fit.
        let mut pool = [leak(8), leak(4)].into_iter();
        assert!(matches!(
            split_memory(memory(), &mut pool, 16, &mut none(), 64, kick),
            Err((true, 4))
        ));
        let mut pool = [leak(17)].into_iter();
        assert!(matches!(
            split_memory(memory(), &mut pool, 16, &mut none(), 64, kick),
            Err((true, 17))
        ));
        let mut pool = (0..4).map(|_| leak(8));
        assert!(matches!(
            split_memory(memory(), &mut pool, 16, &mut none(), 64, kick),
            Err((true, 8))
        ));
        // Transmit: longer than FerroHAL's buffer, empty, more than fit.
        let mut lent = [leak(65)].into_iter();
        assert!(matches!(
            split_memory(memory(), &mut none(), 16, &mut lent, 64, kick),
            Err((false, 65))
        ));
        let mut lent = [leak(0)].into_iter();
        assert!(matches!(
            split_memory(memory(), &mut none(), 16, &mut lent, 64, kick),
            Err((false, 0))
        ));
        let mut lent = (0..4).map(|_| leak(8));
        assert!(matches!(
            split_memory(memory(), &mut none(), 16, &mut lent, 64, kick),
            Err((false, 8))
        ));
    }
}
