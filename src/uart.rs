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
//!   frame in flight until FerroHAL counts it finished. The buffer then comes
//!   back through [`Handle::sent`], in the order submitted, with its
//!   [`Outcome`].
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
//! Requirements: PORT-001, PORT-002, PORT-003, PORT-004, PORT-008, PORT-009,
//! PORT-010, PORT-011

use core::sync::atomic::Ordering;

use ferrohal_stm32::uart::{
    Error, Instance, RxDma, RxPin, SendError, TxDma, TxPin, Uart, UartRefused, Wiring,
};
use heapless::{
    Deque, Vec, VecView,
    spsc::{Consumer, Producer, Queue},
};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

pub use crate::uart_receive::Received;
use crate::uart_receive::{PER_SERVICE, RxEnd, RxQueues, RxSide, Status, as_chunk};

/// The most frames the port keeps in FerroHAL's transmit queue: FerroHAL's
/// own limit, private to it at the pinned revision, restated here.
pub const IN_FLIGHT: usize = 8;

/// How a lent frame ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// FerroHAL's transmit stream finished it: every byte was handed to the
    /// USART. The last one or two may still be on the wire.
    Sent,
    /// It was lost: a DMA error, or the transmitter halted first. Also given
    /// to each frame of a batch FerroHAL finished in one call with a sent and
    /// a failed frame among them, since FerroHAL's counters do not say which
    /// was which ([`Port::unsure`]): a frame is never reported sent unless it
    /// was.
    Failed(Error),
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

/// The memory the port and its handle share: the firmware's, given once to
/// [`uart_port::new`] (an `init` local), which splits it. `RX` and `TX` are queue slots:
/// the pool may hold at most `RX - 1` receive buffers and the handle at most
/// `TX - 1` transmit buffers, the queues' capacity.
pub struct Memory<const RX: usize, const TX: usize> {
    free: Vec<&'static mut [u8], TX>,
    frames: Queue<TxLent, TX>,
    sent: Queue<TxBack, TX>,
    rx: RxQueues<RX>,
    status: Status,
    wake: Signal<()>,
}

impl<const RX: usize, const TX: usize> Memory<RX, TX> {
    pub const fn new() -> Self {
        Self {
            free: Vec::new(),
            frames: Queue::new(),
            sent: Queue::new(),
            rx: RxQueues::new(),
            status: Status::new(),
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
    rx: RxSide,
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
        self.rx.next()
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
    /// `queued`, `sent` and `failed` are FerroHAL's queue length and counters
    /// now.
    ///
    /// FerroHAL finishes frames in order, and `queued` counts those not yet
    /// finished, so the oldest `in_flight - queued` are done. Its counters
    /// say how each ended: a frame dropped without being counted failed
    /// `Halted` (FerroHAL's halt clears its queue). When frames sent and
    /// failed finish in one settle, which one failed is not known: all are
    /// reported failed and counted in `unsure`.
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
                            self.tx
                                .give_back(lent, Outcome::Failed(Error::Halted), self.status);
                            returned += 1;
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
        returned
    }

    /// The ring into lent buffers, whole chunks. Returns the chunks lent and
    /// the frames returned (`read_chunk` serves FerroHAL's transmit queue
    /// too, so each read is followed by a settle).
    fn pump_rx(&mut self, at_us: u64) -> (usize, usize) {
        let (uart, tx, status) = (&mut self.uart, &mut self.tx, self.status);
        let mut returned = 0;
        let chunks = self.rx.pump(status, at_us, |buf| {
            let result = uart.read_chunk(buf);
            let health = uart.tx_health();
            returned += tx.settle(uart.queued(), health.sent, health.dma_errors, status);
            as_chunk(result, uart.generation())
        });
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
        // A halted transmitter keeps its queue and never finishes it; an
        // empty `try_send` says so without touching the queue.
        if !self.tx.in_flight.is_empty() && self.uart.try_send(&[]) == Err(SendError::Halted) {
            returned += self.tx.halted(self.status);
        }
        let (chunks, back) = self.pump_rx(at_us);
        returned += back;
        if chunks > 0 || returned > 0 || events.fault {
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
        rx,
        status,
        wake,
    } = memory;
    let status: &'static Status = status;
    let (rx_end, rx_side) = rx
        .split(status, rx_pool, rx_longest, kick)
        .map_err(|len| (true, len))?;
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
    let wake: &'static Signal<()> = wake;
    let (frames_in, frames_out) = frames.split();
    let (sent_in, sent_out) = sent.split();
    let (writer, reader) = wake.split();
    Ok((
        TxEnd {
            frames: frames_out,
            sent: sent_in,
            in_flight: Deque::new(),
            seen: (0, 0),
            unsure: 0,
        },
        rx_end,
        status,
        writer,
        Handle {
            free: free.as_mut_view(),
            frames: frames_in,
            sent: sent_out,
            rx: rx_side,
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
        let uart = Uart::new(rcc, wiring, config, buffers).map_err(Refused::Open)?;
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
    fn a_halted_transmitter_returns_every_frame_in_flight_failed() {
        let (mut tx, _rx, status, _w, mut h) = ends(1, &[8, 8]);
        for _ in 0..2 {
            h.frame(4).unwrap().submit();
            let lent = tx.frames.dequeue().unwrap();
            assert!(tx.in_flight.push_back(lent).is_ok());
        }
        assert_eq!(tx.halted(status), 2);
        let outcomes: StdVec<_> = core::iter::from_fn(|| h.sent().map(|s| s.outcome)).collect();
        assert_eq!(outcomes, [Outcome::Failed(Error::Halted); 2]);
        assert_eq!(h.outstanding(), 0);
    }

    #[test]
    fn buffers_the_port_cannot_use_are_refused() {
        let memory = || -> &'static mut Memory<4, 4> { Box::leak(Box::new(Memory::new())) };
        let none = || core::iter::empty::<&'static mut [u8]>();
        // Receive: the pool's own checks, reported as the receive side's.
        let mut pool = [leak(17)].into_iter();
        assert!(matches!(
            split_memory(memory(), &mut pool, 16, &mut none(), 64, kick),
            Err((true, 17))
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
