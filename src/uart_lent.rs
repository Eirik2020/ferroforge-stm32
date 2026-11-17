//! PROTOTYPE 2: the UART port with lent buffers, variant B2 of
//! `uart-port-prototype-2-design.md` (port design record, section 3). Not
//! product code; not merged.
//!
//! As variant B ([`uart_queue`](crate::uart_queue)), the port's three
//! members own FerroHAL's [`Uart`] and the consumer owns one
//! [`LentUartHandle`], naming no UART, wiring, vector or lock (D20). What
//! moves between them is references to memory the firmware gave at boot,
//! not bytes:
//!
//! - **Transmit, the consumer lends.** The handle keeps its transmit
//!   buffers. [`LentUartHandle::frame`] takes a free one, or refuses (D8);
//!   the frame is written in place and [`Frame::submit`] lends it to the
//!   port and pends it. The port passes it to FerroHAL's `try_send`, the one
//!   copy left, and keeps it in flight until FerroHAL counts it done. It
//!   then comes back, in order, with its [`Outcome`]: D15 per frame.
//! - **Receive, the port lends.** The port takes a buffer from its pool and
//!   has `read_chunk` copy FerroHAL's ring straight into it, the one copy
//!   left, then lends it to the consumer with the [`Chunk`] and the time.
//!   [`LentUartHandle::next_chunk`] reads it in place; dropping it returns
//!   the buffer. With no buffer free, the bytes stay in FerroHAL's ring and
//!   the port is starved: the next release pends it (D19).
//!
//! Four single-producer, single-consumer queues of references: frames out
//! and back, chunks out and back. No critical section of the port's own.
//! Nothing here panics.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use ferrohal_stm32::{
    Service,
    rcc::Rcc,
    uart::{
        Buffers, Chunk, ChunkEnd, Config, Error, Instance, RxDma, RxPin, SendError, TxDma, TxPin,
        Uart, UartRefused, Wiring,
    },
};
use heapless::{
    Deque, Vec, VecView,
    spsc::{Consumer, Producer, Queue},
};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

use crate::uart_queue::Split;

/// The most frames the port has in FerroHAL's transmit queue: FerroHAL's
/// own limit (`tx_ring::FRAMES`, private to FerroHAL at the pinned
/// revision), restated here.
pub const IN_FLIGHT: usize = 8;

/// How a lent frame ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// FerroHAL counted it sent.
    Sent,
    /// It was lost: a DMA error, or the transmitter halted first. Also given
    /// to every frame of a batch FerroHAL's counters cannot order (see
    /// [`LentUartPort::unsure`]): a frame is never reported sent unless it
    /// was.
    Failed(Error),
    /// FerroHAL refused it. Only `TooLarge`, which the constructor's length
    /// check rules out.
    Refused(SendError),
}

/// Why [`LentUartHandle::frame`] refused (D8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// No transmit buffer is that long.
    TooLong,
    /// Every buffer long enough is out: lent, or back and not yet taken.
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
    at: u32,
}

/// What the port and its handle both see outside the queues.
struct Status {
    /// The port had bytes to read and no buffer: the next release pends it.
    starved: AtomicBool,
    /// Buffers a queue could not take. Zero by construction (each queue
    /// holds every buffer of its kind); counted so a fault is not hidden.
    lost: AtomicU32,
}

/// The memory the port and its handle share, provided by the firmware (an
/// `init` local) and split once by [`new_lent`]. `RX` and `TX` slots: the
/// port takes at most `RX - 1` receive buffers and the handle `TX - 1`
/// transmit buffers, the queues' capacity.
pub struct LentUartMemory<const RX: usize, const TX: usize> {
    free: Vec<&'static mut [u8], TX>,
    frames: Queue<TxLent, TX>,
    sent: Queue<TxBack, TX>,
    chunks: Queue<RxLent, RX>,
    back: Queue<&'static mut [u8], RX>,
    status: Status,
    wake: Signal<()>,
}

impl<const RX: usize, const TX: usize> LentUartMemory<RX, TX> {
    pub const fn new() -> Self {
        Self {
            free: Vec::new(),
            frames: Queue::new(),
            sent: Queue::new(),
            chunks: Queue::new(),
            back: Queue::new(),
            status: Status {
                starved: AtomicBool::new(false),
                lost: AtomicU32::new(0),
            },
            wake: Signal::new(),
        }
    }
}

impl<const RX: usize, const TX: usize> Default for LentUartMemory<RX, TX> {
    fn default() -> Self {
        Self::new()
    }
}

/// The consumer's end of one UART port. Every UART's handle has this type,
/// whatever its buffers, so `init` can give any handle to any task (D12).
pub struct LentUartHandle {
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

impl LentUartHandle {
    /// Waits until the port has news: a chunk, a frame back, a fault.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// The next chunk, in the port's buffer. Dropping it returns the buffer.
    pub fn next_chunk(&mut self) -> Option<Received<'_>> {
        let lent = self.chunks.dequeue()?;
        Some(Received {
            buf: Some(lent.buf),
            chunk: lent.chunk,
            at: lent.at,
            handle: self,
        })
    }

    /// Takes the shortest free transmit buffer that holds `len` bytes, for
    /// a frame of exactly `len` bytes written in place.
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
            // `len <= longest`, and the constructor keeps every buffer
            // within FerroHAL's transmit buffer, whose length is a u16.
            len: u16::try_from(len).unwrap_or(u16::MAX),
            handle: self,
        })
    }

    /// The oldest frame back from the port, in its buffer, with its
    /// outcome. Frames come back in the order submitted. Dropping it frees
    /// the buffer.
    pub fn sent(&mut self) -> Option<SentFrame<'_>> {
        let back = self.sent.dequeue()?;
        Some(SentFrame {
            buf: Some(back.lent.buf),
            len: back.lent.len,
            outcome: back.outcome,
            handle: self,
        })
    }

    /// Transmit buffers not free: lent, or back and not yet taken.
    pub fn outstanding(&self) -> u8 {
        let free = u8::try_from(self.free.len()).unwrap_or(u8::MAX);
        self.buffers.saturating_sub(free)
    }

    /// Transmit buffers given at boot.
    pub fn buffers(&self) -> u8 {
        self.buffers
    }

    /// The longest frame this handle can send.
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

    /// Returns a receive buffer to the port, and pends it if it was
    /// starved for one.
    fn release(&mut self, buf: &'static mut [u8]) {
        if self.back.enqueue(buf).is_err() {
            self.status.lost.fetch_add(1, Ordering::Relaxed);
        }
        if self.status.starved.swap(false, Ordering::AcqRel) {
            (self.kick)();
        }
    }
}

/// A received chunk, read in place. Dropping it returns the buffer to the
/// port.
pub struct Received<'h> {
    handle: &'h mut LentUartHandle,
    buf: Option<&'static mut [u8]>,
    /// Its length, why it ended and its generation, as FerroHAL gave them.
    pub chunk: Chunk,
    /// When the port took it from FerroHAL's ring, in the firmware's clock.
    pub at: u32,
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

/// A frame being written in a lent buffer. Dropping it frees the buffer.
pub struct Frame<'h> {
    handle: &'h mut LentUartHandle,
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
    /// through [`LentUartHandle::sent`].
    pub fn submit(mut self) {
        let Some(buf) = self.buf.take() else { return };
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
    handle: &'h mut LentUartHandle,
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

/// The port's transmit side: the frames lent to it, those in FerroHAL's
/// queue, and FerroHAL's counters as last read.
struct TxEnd {
    frames: Consumer<'static, TxLent>,
    sent: Producer<'static, TxBack>,
    /// In FerroHAL's queue, oldest first: the order FerroHAL finishes them.
    in_flight: Deque<TxLent, IN_FLIGHT>,
    /// FerroHAL's `sent` and `dma_errors` at the last settle.
    seen: (u32, u32),
    unsure: u32,
}

impl TxEnd {
    /// Returns the frames FerroHAL has finished since the last settle.
    /// `queued` and `health` are FerroHAL's queue length and counters now.
    ///
    /// FerroHAL finishes frames in order, and `queued` counts the ones not
    /// yet finished, so the oldest `in_flight - queued` are done. Its
    /// counters say how each ended: a frame dropped without being counted
    /// (FerroHAL's halt clears its queue) failed `Halted`. When frames sent
    /// and failed are finished in one settle, which one failed is not
    /// known: all are reported failed and counted in `unsure`.
    fn settle(&mut self, queued: usize, sent: u32, failed: u32, status: &Status) -> usize {
        let mut sent_new = sent.wrapping_sub(self.seen.0);
        let mut failed_new = failed.wrapping_sub(self.seen.1);
        self.seen = (sent, failed);
        let finished = self.in_flight.len().saturating_sub(queued);
        let mixed = sent_new > 0 && failed_new > 0;
        let mut returned = 0;
        for _ in 0..finished {
            let Some(lent) = self.in_flight.pop_front() else {
                break;
            };
            let outcome = if mixed {
                self.unsure = self.unsure.saturating_add(1);
                Outcome::Failed(Error::Dma)
            } else if sent_new > 0 {
                sent_new -= 1;
                Outcome::Sent
            } else if failed_new > 0 {
                failed_new -= 1;
                Outcome::Failed(Error::Dma)
            } else {
                Outcome::Failed(Error::Halted)
            };
            self.give_back(lent, outcome, status);
            returned += 1;
        }
        returned
    }

    /// Returns every frame in flight as failed: the transmitter halted.
    fn halted(&mut self, status: &Status) -> usize {
        let mut returned = 0;
        while let Some(lent) = self.in_flight.pop_front() {
            self.give_back(lent, Outcome::Failed(Error::Halted), status);
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

    /// Lends `buf` to the consumer, or keeps it as the spare when the
    /// queue is full (it holds the whole pool, so it is not).
    fn lend(&mut self, buf: &'static mut [u8], chunk: Chunk, at: u32, status: &Status) -> bool {
        match self.chunks.enqueue(RxLent { buf, chunk, at }) {
            Ok(()) => true,
            Err(back) => {
                self.spare = Some(back.buf);
                status.lost.fetch_add(1, Ordering::Relaxed);
                false
            }
        }
    }
}

/// The port's own state: the UART and the port's ends of the queues.
pub struct LentUartPort<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>,
    tx: TxEnd,
    rx: RxEnd,
    status: &'static Status,
    wake: SignalWriter<'static, ()>,
    now: fn() -> u32,
    /// PROTOTYPE measurement: the longest run, split.
    split: Split,
}

impl<U, Tx, Rx, TxD, RxD> LentUartPort<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    /// FerroHAL's UART, for checks such as `health()`.
    pub fn uart(&self) -> &Uart<Wiring<U, Tx, Rx, TxD, RxD>> {
        &self.uart
    }

    /// PROTOTYPE measurement: what the longest runs are made of.
    pub fn split(&self) -> Split {
        self.split
    }

    /// Frames reported failed because FerroHAL finished a sent and a failed
    /// frame in one call, and its counters do not say which was which.
    pub fn unsure(&self) -> u32 {
        self.tx.unsure
    }

    fn settle(&mut self) -> usize {
        let health = self.uart.tx_health();
        self.tx.settle(
            self.uart.queued(),
            health.sent,
            health.dma_errors,
            self.status,
        )
    }

    /// Lent frames into FerroHAL, in order. Frames passed on, and frames
    /// returned.
    fn pump_tx(&mut self) -> (usize, usize) {
        let (mut frames, mut returned) = (0, 0);
        for _ in 0..16 {
            if self.tx.in_flight.is_full() {
                break;
            }
            let Some(lent) = self.tx.frames.peek() else {
                break;
            };
            let start = (self.now)();
            let result = match lent.buf.get(..usize::from(lent.len)) {
                Some(bytes) => self.uart.try_send(bytes),
                None => Err(SendError::TooLarge),
            };
            self.split.try_send = self.split.try_send.max((self.now)().wrapping_sub(start));
            let full = result == Err(SendError::Full);
            if !full && let Some(lent) = self.tx.frames.dequeue() {
                match result {
                    Ok(()) => {
                        frames += 1;
                        // Room was checked above.
                        if let Err(lent) = self.tx.in_flight.push_back(lent) {
                            self.tx
                                .give_back(lent, Outcome::Failed(Error::Halted), self.status);
                        }
                    }
                    Err(SendError::Halted) => {
                        self.tx
                            .give_back(lent, Outcome::Failed(Error::Halted), self.status);
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
        (frames, returned)
    }

    /// Ring into lent buffers, whole chunks. Chunks lent, and frames
    /// returned (`read_chunk` serves FerroHAL's transmit queue too).
    fn pump_rx(&mut self) -> (usize, usize) {
        let (mut chunks, mut returned) = (0, 0);
        for _ in 0..16 {
            let start = (self.now)();
            let Some(buf) = self.rx.take(self.status) else {
                break;
            };
            let read = (self.now)();
            let result = self.uart.read_chunk(buf);
            self.split.read_chunk = self.split.read_chunk.max((self.now)().wrapping_sub(read));
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
            if !self.rx.lend(buf, chunk, (self.now)(), self.status) {
                break;
            }
            self.split
                .chunk(chunk.len, (self.now)().wrapping_sub(start));
            chunks += 1;
            if chunk.end == ChunkEnd::Gap(Error::Halted) {
                break;
            }
        }
        (chunks, returned)
    }
}

impl<U, Tx, Rx, TxD, RxD> Service for LentUartPort<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    /// Nothing: the port wakes its consumer itself.
    type Event = ();

    fn on_interrupt(&mut self) {
        let start = (self.now)();
        let events = self.uart.on_interrupt();
        self.split.events = self.split.events.max((self.now)().wrapping_sub(start));
        let mut returned = self.settle();
        let (frames, back) = self.pump_tx();
        returned += back;
        // A halted transmitter keeps its queue and never finishes it; an
        // empty `try_send` says so without touching the queue.
        if !self.tx.in_flight.is_empty() && self.uart.try_send(&[]) == Err(SendError::Halted) {
            returned += self.tx.halted(self.status);
        }
        let (chunks, back) = self.pump_rx();
        returned += back;
        if chunks > 0 || returned > 0 || events.fault {
            let wake = (self.now)();
            self.wake.write(());
            self.split.wake = self.split.wake.max((self.now)().wrapping_sub(wake));
        }
        self.split
            .run((self.now)().wrapping_sub(start), chunks, frames);
    }
}

/// What [`new_lent`] builds: the port's resource and the consumer's handle.
pub struct LentUartResources<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    /// Shared by the port's three members only.
    pub port: LentUartPort<U, Tx, Rx, TxD, RxD>,
    /// The consumer's, as a `#[local]`, given to any task at boot.
    pub handle: LentUartHandle,
}

/// Why [`new_lent`] refused. Built once, at boot: its size does not matter.
#[allow(clippy::large_enum_variant)]
pub enum LentUartRefused<W, U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    /// FerroHAL refused the UART: its reason, with what it took.
    Open(UartRefused<W>),
    /// A buffer the port cannot use: a receive buffer empty, of another
    /// length than the first, longer than half the receive ring, or one
    /// more than the queues hold; a transmit buffer empty, longer than
    /// FerroHAL's transmit buffer, or one more than the queues hold. With
    /// the open UART, so the firmware can `free` it.
    Buffer {
        receive: bool,
        len: usize,
        uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>,
    },
}

/// The port's ends and the handle, from the memory alone: what
/// [`new_lent`] does once FerroHAL's UART is open. A buffer it cannot use
/// comes back as `Err((receive, len))`.
#[allow(clippy::type_complexity)]
fn split_memory<const RX: usize, const TX: usize>(
    memory: &'static mut LentUartMemory<RX, TX>,
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
        LentUartHandle,
    ),
    (bool, usize),
> {
    let LentUartMemory {
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
        buffers += 1;
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
            seen: (0, 0),
            unsure: 0,
        },
        RxEnd {
            chunks: chunks_in,
            back: back_out,
            spare: None,
        },
        status,
        writer,
        LentUartHandle {
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

/// Opens the UART on `wiring` and splits `memory`. `rx_pool` is the port's
/// receive pool, buffers of one length up to half `rx_buf` (a longer one
/// would let a continuous stream lap the ring before a chunk is taken);
/// `tx_lent` the handle's transmit buffers, each up to `tx_buf`'s length.
/// `kick` is the selection's generated pend of the transmit stream's
/// vector; `now` is the firmware's clock, read once per chunk.
#[allow(
    clippy::type_complexity,
    clippy::too_many_arguments,
    clippy::result_large_err
)]
pub fn new_lent<U, Tx, Rx, TxD, RxD, const RX: usize, const TX: usize>(
    rcc: &mut Rcc,
    wiring: Wiring<U, Tx, Rx, TxD, RxD>,
    config: &Config,
    tx_buf: &'static mut [u8],
    rx_buf: &'static mut [u8],
    memory: &'static mut LentUartMemory<RX, TX>,
    rx_pool: &mut dyn Iterator<Item = &'static mut [u8]>,
    tx_lent: &mut dyn Iterator<Item = &'static mut [u8]>,
    kick: fn(),
    now: fn() -> u32,
) -> Result<
    LentUartResources<U, Tx, Rx, TxD, RxD>,
    LentUartRefused<Wiring<U, Tx, Rx, TxD, RxD>, U, Tx, Rx, TxD, RxD>,
>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    let (tx_longest, rx_longest) = (tx_buf.len(), rx_buf.len() / 2);
    let uart = Uart::new(
        rcc,
        wiring,
        config,
        Buffers {
            tx: tx_buf,
            rx: rx_buf,
        },
    )
    .map_err(LentUartRefused::Open)?;
    match split_memory(memory, rx_pool, rx_longest, tx_lent, tx_longest, kick) {
        Ok((tx, rx, status, wake, handle)) => Ok(LentUartResources {
            port: LentUartPort {
                uart,
                tx,
                rx,
                status,
                wake,
                now,
                split: Split::default(),
            },
            handle,
        }),
        Err((receive, len)) => Err(LentUartRefused::Buffer { receive, len, uart }),
    }
}

/// The variant B2 group: variant B's members and contract, with lent
/// buffers. Members share `port`; one priority for all three makes RTIC's
/// lock among them free, but the firmware decides (D14). Nothing is
/// spawned: the port wakes whichever task holds the handle, which the
/// firmware gives to any task at boot (D12). In `init`:
/// `uart_lent::new_lent(.., memory, &mut pool, &mut lent, <selection>::kick, now)`.
#[ferroforge::group(
    slots = [instance: U, tx: Tx, rx: Rx, tx_dma: TxD, rx_dma: RxD],
    vectors = [l_on_uart = instance, l_on_rx_dma = rx_dma, l_on_tx_dma = tx_dma],
    shared = [port: LentUartPort<U, Tx, Rx, TxD, RxD>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [
        config: Config @ platform,
        rx_pool: &mut dyn Iterator<Item = &'static mut [u8]> @ software,
        tx_lent: &mut dyn Iterator<Item = &'static mut [u8]> @ software,
        now: fn() -> u32 @ platform,
    ],
    memory = [memory: LentUartMemory],
    handles = [handle: LentUartHandle],
    pends = [kick = l_on_tx_dma],
    constructor = new_lent(rcc, wiring, config, tx_buf, rx_buf, memory, rx_pool, tx_lent, kick, now),
)]
pub mod uart_lent_port {
    #[allow(unused_imports)]
    use super::{Config, LentUartHandle, LentUartMemory, LentUartPort, new_lent};
    pub use ferrohal_stm32::Service;

    /// The USART's own vector.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn l_on_uart(mut cx: l_on_uart::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }

    /// The receive stream's vector.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn l_on_rx_dma(mut cx: l_on_rx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }

    /// The transmit stream's vector, also pended by the handle.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn l_on_tx_dma(mut cx: l_on_tx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
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
        LentUartHandle,
    );

    /// Memory with a pool of `rx` receive buffers of 8 bytes and transmit
    /// buffers of the lengths given.
    fn ends(rx: usize, tx: &[usize]) -> Ends {
        let memory: &'static mut LentUartMemory<4, 4> = Box::leak(Box::new(LentUartMemory::new()));
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
        // The port: both into FerroHAL, then FerroHAL finishes one sent and
        // one failed in separate calls.
        for _ in 0..2 {
            let lent = tx.frames.dequeue().unwrap();
            assert!(tx.in_flight.push_back(lent).is_ok());
        }
        assert_eq!(tx.settle(2, 0, 0, status), 0);
        assert_eq!(tx.settle(1, 1, 0, status), 1);
        assert_eq!(tx.settle(0, 1, 1, status), 1);
        let first = h.sent().unwrap();
        assert_eq!(
            (first.outcome, first.bytes()),
            (Outcome::Sent, &b"0123456789"[..])
        );
        drop(first);
        let second = h.sent().unwrap();
        assert_eq!(second.outcome, Outcome::Failed(Error::Dma));
        assert_eq!((second.bytes().len(), second.bytes()[0]), (20, b'B'));
        drop(second);
        assert_eq!(h.outstanding(), 0);
        assert_eq!(h.lost(), 0);
        // A frame dropped unsent frees its buffer.
        drop(h.frame(5).unwrap());
        assert_eq!(h.outstanding(), 0);
    }

    #[test]
    fn frames_finished_together_are_never_reported_sent_unless_known() {
        let (mut tx, _rx, status, _w, mut h) = ends(1, &[8, 8, 8]);
        for _ in 0..3 {
            h.frame(4).unwrap().submit();
            let lent = tx.frames.dequeue().unwrap();
            assert!(tx.in_flight.push_back(lent).is_ok());
        }
        // Two finished in one call, one sent and one failed: unknown which.
        assert_eq!(tx.settle(1, 1, 1, status), 2);
        assert_eq!(tx.unsure, 2);
        // FerroHAL halted and cleared its queue: no count moved.
        assert_eq!(tx.settle(0, 1, 1, status), 1);
        let outcomes: StdVec<_> = core::iter::from_fn(|| h.sent().map(|s| s.outcome)).collect();
        assert_eq!(
            outcomes,
            [
                Outcome::Failed(Error::Dma),
                Outcome::Failed(Error::Dma),
                Outcome::Failed(Error::Halted)
            ]
        );
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
        assert_eq!((got.bytes(), got.chunk, got.at), (&b"aaa"[..], chunk, 9));
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
    fn buffers_the_port_cannot_use_are_refused() {
        let memory =
            || -> &'static mut LentUartMemory<4, 4> { Box::leak(Box::new(LentUartMemory::new())) };
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
