//! PROTOTYPE 2: the SPI port with lent buffers, variant R of
//! `spi-port-prototype-2-design.md` (port design record, section 3a). Not
//! product code; not merged.
//!
//! The established pattern for a shared SPI bus (Linux's `spi_message`,
//! Zephyr's RTIO, Betaflight's segments): the consumer owns the memory and
//! the queues hold references to it, not bytes.
//!
//! - Each device has buffers of the firmware's choosing, `&'static mut [u8]`
//!   up to the bus's `capacity()`, given at boot. Its [`LentHandle`] keeps
//!   the free ones.
//! - [`LentHandle::message`] takes a free buffer, or refuses with
//!   [`SubmitError::Outstanding`] when every one is out (D8). Parts are
//!   written straight into it.
//! - [`Message::submit`] moves the reference into the device's own
//!   single-producer, single-consumer queue and pends the port. The queue
//!   holds as many entries as the device has buffers.
//! - The port's run: a transfer that ended is released, what came in is
//!   copied over the lent buffer, and the buffer goes back through the
//!   device's return queue with a wake. Then, if the bus is free, it looks
//!   at every device's front entry and starts the highest priority, the
//!   oldest among equals.
//! - [`LentHandle::take`] gives the buffer back as a [`Taken`]; the read
//!   parts are read in place, and dropping it frees the buffer.
//!
//! Order: within one device, the order submitted; across devices, priority
//! then age. No critical section anywhere: each queue has one producer and
//! one consumer. Nothing here panics.

use core::sync::atomic::{AtomicBool, Ordering};

use ferrohal_stm32::{
    Service,
    rcc::Rcc,
    spi::{
        Buffers, Config, ConfigError, Error, Instance, MisoPin, MosiPin, Parts, SckPin, Spi,
        SpiRefused, SpiRxDma, SpiTxDma, Wiring,
    },
};
use heapless::{
    Vec, VecView,
    spsc::{Consumer, Producer, Queue},
};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

use crate::spi_queue::{ChipSelects, DEVICES, FILLER, Outcome, PARTS, Part, Stats, SubmitError};

/// A message lent to the port: the buffer and how to read it back. Its
/// size does not depend on the data's length.
struct Lent {
    buf: &'static mut [u8],
    len: u16,
    keep: [Part; PARTS],
    kept: u8,
    priority: u8,
    submitted: u32,
}

/// A lent message coming back.
struct Done {
    lent: Lent,
    outcome: Outcome,
    waited: u32,
    took: u32,
}

/// What the handle and the port both see of one device.
struct LentState {
    /// Set by the handle's `cancel`, cleared by the port when served.
    cancel: AtomicBool,
}

/// One device's memory: its free list, its two queues, its state and its
/// wake. `N` slots: the device takes at most `N - 1` buffers, the queues'
/// capacity.
pub struct LentDevice<const N: usize> {
    free: Vec<&'static mut [u8], N>,
    lent: Queue<Lent, N>,
    returned: Queue<Done, N>,
    state: LentState,
    wake: Signal<()>,
}

impl<const N: usize> LentDevice<N> {
    pub const fn new() -> Self {
        Self {
            free: Vec::new(),
            lent: Queue::new(),
            returned: Queue::new(),
            state: LentState {
                cancel: AtomicBool::new(false),
            },
            wake: Signal::new(),
        }
    }
}

impl<const N: usize> Default for LentDevice<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// The port's memory: `D` devices, `N` slots each.
pub struct LentMemory<const N: usize, const D: usize> {
    devices: [LentDevice<N>; D],
}

impl<const N: usize, const D: usize> LentMemory<N, D> {
    pub const fn new() -> Self {
        Self {
            devices: [const { LentDevice::new() }; D],
        }
    }
}

impl<const N: usize, const D: usize> Default for LentMemory<N, D> {
    fn default() -> Self {
        Self::new()
    }
}

/// The port's end of one device.
struct LentEnd {
    lent: Consumer<'static, Lent>,
    returned: Producer<'static, Done>,
    wake: SignalWriter<'static, ()>,
    state: &'static LentState,
}

/// The consumer's end of one SPI device: the same type for every device
/// and every port size, so `init` can give any handle to any task (D12).
pub struct LentHandle {
    free: &'static mut VecView<&'static mut [u8]>,
    lent: Producer<'static, Lent>,
    returned: Consumer<'static, Done>,
    state: &'static LentState,
    wake: SignalReader<'static, ()>,
    /// Buffers given at boot, and the longest of them.
    buffers: u8,
    longest: usize,
    /// The most the queues hold: the limit on `buffers`.
    room: usize,
    device: u8,
    kick: fn(),
    now: fn() -> u32,
    /// Buffers the free list could not take back. Zero by construction
    /// (it holds every buffer given); counted so a fault is not hidden.
    lost: u32,
}

/// Splits one device's memory into the port's end and a handle without
/// buffers.
fn split_device<const N: usize>(
    memory: &'static mut LentDevice<N>,
    device: u8,
    kick: fn(),
    now: fn() -> u32,
) -> (LentEnd, LentHandle) {
    let LentDevice {
        free,
        lent,
        returned,
        state,
        wake,
    } = memory;
    let state: &'static LentState = state;
    let wake: &'static Signal<()> = wake;
    let room = lent.capacity();
    let (lent_in, lent_out) = lent.split();
    let (back_in, back_out) = returned.split();
    let (writer, reader) = wake.split();
    (
        LentEnd {
            lent: lent_out,
            returned: back_in,
            wake: writer,
            state,
        },
        LentHandle {
            free: free.as_mut_view(),
            lent: lent_in,
            returned: back_out,
            state,
            wake: reader,
            buffers: 0,
            longest: 0,
            room,
            device,
            kick,
            now,
            lost: 0,
        },
    )
}

impl LentHandle {
    /// Takes the shortest free buffer that holds `len` bytes, for a
    /// message at `priority` (higher goes first across devices). Refused
    /// with `TooLong` when no buffer of this device is that long, and with
    /// `Outstanding` when every one that is is out (D8).
    pub fn message(&mut self, priority: u8, len: usize) -> Result<Message<'_>, SubmitError> {
        if len > self.longest {
            return Err(SubmitError::TooLong);
        }
        let mut best: Option<(usize, usize)> = None;
        for (i, buf) in self.free.iter().enumerate() {
            let size = buf.len();
            if size >= len && best.is_none_or(|(_, b)| size < b) {
                best = Some((i, size));
            }
        }
        let (i, _) = best.ok_or(SubmitError::Outstanding)?;
        let buf = self.remove(i).ok_or(SubmitError::Outstanding)?;
        Ok(Message {
            handle: self,
            buf: Some(buf),
            len: 0,
            keep: [Part::default(); PARTS],
            kept: 0,
            priority,
            refused: None,
        })
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
            self.lost = self.lost.saturating_add(1);
        }
    }

    /// The oldest message the port gave back, with its buffer.
    pub fn take(&mut self) -> Option<Taken<'_>> {
        let done = self.returned.dequeue()?;
        Some(Taken {
            buf: Some(done.lent.buf),
            len: done.lent.len,
            keep: done.lent.keep,
            kept: done.lent.kept,
            outcome: done.outcome,
            waited: done.waited,
            took: done.took,
            priority: done.lent.priority,
            handle: self,
        })
    }

    /// Waits until the port gives a message back.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// Asks the port to drop this device's queued messages and abort its
    /// running one; each comes back marked `Aborted`, with chip select
    /// released.
    pub fn cancel(&mut self) {
        self.state.cancel.store(true, Ordering::Release);
        (self.kick)();
    }

    /// Buffers not free: lent, or back and not yet taken.
    pub fn outstanding(&self) -> u8 {
        let free = u8::try_from(self.free.len()).unwrap_or(u8::MAX);
        self.buffers.saturating_sub(free)
    }

    /// Buffers given at boot.
    pub fn buffers(&self) -> u8 {
        self.buffers
    }

    /// This device's number on its port.
    pub fn device(&self) -> u8 {
        self.device
    }

    /// Buffers lost to a full free list: zero unless something is broken.
    pub fn lost(&self) -> u32 {
        self.lost
    }
}

/// One chip-select assertion being built in a lent buffer. Dropping it, or
/// a refused submit, frees the buffer.
pub struct Message<'h> {
    handle: &'h mut LentHandle,
    buf: Option<&'static mut [u8]>,
    len: u16,
    keep: [Part; PARTS],
    kept: u8,
    priority: u8,
    refused: Option<SubmitError>,
}

impl Message<'_> {
    /// Appends bytes to send; what comes back during them is not read.
    pub fn write(&mut self, bytes: &[u8]) -> &mut Self {
        self.append(Some(bytes), bytes.len(), false);
        self
    }

    /// Appends `n` filler bytes, and keeps what comes back during them.
    pub fn read(&mut self, n: usize) -> &mut Self {
        self.append(None, n, true);
        self
    }

    /// Appends bytes to send, and keeps what comes back during them.
    pub fn transfer(&mut self, bytes: &[u8]) -> &mut Self {
        self.append(Some(bytes), bytes.len(), true);
        self
    }

    /// Bytes on the wire so far.
    pub fn len(&self) -> usize {
        usize::from(self.len)
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The buffer's length: the most this message can hold.
    pub fn capacity(&self) -> usize {
        self.buf.as_deref().map_or(0, <[u8]>::len)
    }

    fn append(&mut self, src: Option<&[u8]>, n: usize, keep: bool) {
        if self.refused.is_some() || n == 0 {
            return;
        }
        let at = usize::from(self.len);
        let Some(end) = at.checked_add(n).filter(|&e| e <= usize::from(u16::MAX)) else {
            self.refused = Some(SubmitError::TooLong);
            return;
        };
        let Some(dst) = self.buf.as_deref_mut().and_then(|b| b.get_mut(at..end)) else {
            self.refused = Some(SubmitError::TooLong);
            return;
        };
        match src {
            Some(src) => dst.copy_from_slice(src),
            None => dst.fill(FILLER),
        }
        if keep {
            let Some(part) = self.keep.get_mut(usize::from(self.kept)) else {
                self.refused = Some(SubmitError::Parts);
                return;
            };
            // Both fit in u16: end <= u16::MAX.
            *part = Part {
                at: at as u16,
                len: n as u16,
            };
            self.kept += 1;
        }
        self.len = end as u16;
    }

    /// Lends the buffer to the port and pends it. Refused, with the buffer
    /// freed, when the message is malformed.
    pub fn submit(mut self) -> Result<(), SubmitError> {
        if let Some(refused) = self.refused {
            return Err(refused);
        }
        if self.len == 0 {
            return Err(SubmitError::Empty);
        }
        let Some(buf) = self.buf.take() else {
            return Err(SubmitError::Empty);
        };
        let lent = Lent {
            buf,
            len: self.len,
            keep: self.keep,
            kept: self.kept,
            priority: self.priority,
            submitted: (self.handle.now)(),
        };
        if let Err(back) = self.handle.lent.enqueue(lent) {
            // Cannot happen: the queue holds every buffer the device has.
            self.handle.give_back(back.buf);
            return Err(SubmitError::Full);
        }
        (self.handle.kick)();
        Ok(())
    }
}

impl Drop for Message<'_> {
    fn drop(&mut self) {
        if let Some(buf) = self.buf.take() {
            self.handle.give_back(buf);
        }
    }
}

/// A message back from the port, in its own buffer. Dropping it frees the
/// buffer.
pub struct Taken<'h> {
    handle: &'h mut LentHandle,
    buf: Option<&'static mut [u8]>,
    len: u16,
    keep: [Part; PARTS],
    kept: u8,
    /// How it ended.
    pub outcome: Outcome,
    /// Clock ticks from `submit` to the transfer's start (or to the drop,
    /// for a cancelled one).
    pub waited: u32,
    /// Clock ticks on the wire, start to end.
    pub took: u32,
    /// The message's priority, to tell messages apart.
    pub priority: u8,
}

impl Taken<'_> {
    /// What came back during kept part `i` (the read and transfer parts,
    /// counted from 0), in place; empty unless the outcome is `Done`.
    pub fn read(&self, i: usize) -> &[u8] {
        if self.outcome != Outcome::Done || i >= usize::from(self.kept) {
            return &[];
        }
        let Some(part) = self.keep.get(i) else {
            return &[];
        };
        let (at, len) = (usize::from(part.at), usize::from(part.len));
        self.buf
            .as_deref()
            .and_then(|b| b.get(at..at + len))
            .unwrap_or(&[])
    }

    /// How many parts kept what came back.
    pub fn parts(&self) -> usize {
        usize::from(self.kept)
    }

    /// Bytes on the wire.
    pub fn len(&self) -> usize {
        usize::from(self.len)
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Drop for Taken<'_> {
    fn drop(&mut self) {
        if let Some(buf) = self.buf.take() {
            self.handle.give_back(buf);
        }
    }
}

/// The transfer on the wire, with its buffer.
struct Running {
    device: u8,
    lent: Lent,
    started: u32,
}

/// The port's own state: FerroHAL's `Spi`, the chip selects, each device's
/// settings and queues.
pub struct LentPort<W: Parts, Cs> {
    spi: Spi<W>,
    cs: Cs,
    configs: [Option<Config>; DEVICES],
    ends: [Option<LentEnd>; DEVICES],
    running: Option<Running>,
    last: Option<u8>,
    now: fn() -> u32,
    stats: Stats,
    /// PROTOTYPE measurement: the longest `start` (FerroHAL's copy out and
    /// the DMA set-up) and the longest copy back, since the last clear.
    copies: (u32, u32),
    /// Transfers whose received bytes did not cover what was sent. Zero by
    /// construction; counted, and the message failed, so it is not hidden.
    short: u32,
}

impl<W: Parts, Cs: ChipSelects> LentPort<W, Cs> {
    /// FerroHAL's driver, for checks such as `health()` and `vectors()`.
    pub fn spi(&self) -> &Spi<W> {
        &self.spi
    }

    /// PROTOTYPE measurements; `lost` includes the short transfers.
    pub fn stats(&self) -> Stats {
        let mut stats = self.stats;
        stats.lost = stats.lost.saturating_add(self.short);
        stats
    }

    /// PROTOTYPE measurement: the longest `start` and copy back, in clock
    /// ticks, since [`clear_copies`](Self::clear_copies).
    pub fn copies(&self) -> (u32, u32) {
        self.copies
    }

    pub fn clear_copies(&mut self) {
        self.copies = (0, 0);
    }

    /// PROTOTYPE: device `device`'s chip select, as the port drives it.
    pub fn is_selected(&mut self, device: usize) -> Option<bool> {
        self.cs.is_selected(device)
    }

    fn deliver(&mut self, device: u8, done: Done) {
        let Some(end) = self
            .ends
            .get_mut(usize::from(device))
            .and_then(Option::as_mut)
        else {
            self.stats.lost = self.stats.lost.saturating_add(1);
            return;
        };
        if end.returned.enqueue(done).is_err() {
            self.stats.lost = self.stats.lost.saturating_add(1);
        }
        end.wake.write(());
    }

    /// The running transfer ended with `outcome`: release, copy back,
    /// return the buffer.
    fn finish(&mut self, outcome: Result<usize, Error>, aborted: bool) {
        let Some(run) = self.running.take() else {
            return;
        };
        let now = (self.now)();
        let d = usize::from(run.device);
        #[cfg(not(feature = "prototype-quiet"))]
        if crate::spi_queue::readback_wrong(&mut self.cs, d) {
            self.stats.cs_wrong = self.stats.cs_wrong.saturating_add(1);
        }
        self.cs.select(d, false);
        let lent = run.lent;
        let outcome = match (aborted, outcome) {
            (true, _) => Outcome::Aborted,
            (false, Err(error)) => Outcome::Failed(error),
            (false, Ok(_)) => {
                let start = (self.now)();
                let len = usize::from(lent.len);
                let copied = match (lent.buf.get_mut(..len), self.spi.received().get(..len)) {
                    (Some(dst), Some(src)) => {
                        dst.copy_from_slice(src);
                        true
                    }
                    _ => false,
                };
                let took = (self.now)().wrapping_sub(start);
                self.copies.1 = self.copies.1.max(took);
                if copied {
                    Outcome::Done
                } else {
                    self.short = self.short.saturating_add(1);
                    Outcome::Failed(Error::Empty)
                }
            }
        };
        let done = Done {
            waited: run.started.wrapping_sub(lent.submitted),
            took: now.wrapping_sub(run.started),
            outcome,
            lent,
        };
        self.deliver(run.device, done);
    }

    /// A lent message that never reached the wire.
    fn refuse(&mut self, device: u8, lent: Lent, outcome: Outcome) {
        let now = (self.now)();
        let done = Done {
            waited: now.wrapping_sub(lent.submitted),
            took: 0,
            outcome,
            lent,
        };
        self.deliver(device, done);
    }

    fn serve_cancels(&mut self) {
        for d in 0..Cs::COUNT.min(DEVICES) {
            let Some(end) = self.ends.get(d).and_then(Option::as_ref) else {
                continue;
            };
            if !end.state.cancel.swap(false, Ordering::AcqRel) {
                continue;
            }
            if self
                .running
                .as_ref()
                .is_some_and(|r| usize::from(r.device) == d)
            {
                self.spi.abort();
                self.finish(Err(Error::Busy), true);
            }
            // Bounded by the queue: each pass removes one entry.
            for _ in 0..u8::MAX {
                let Some(lent) = self
                    .ends
                    .get_mut(d)
                    .and_then(Option::as_mut)
                    .and_then(|e| e.lent.dequeue())
                else {
                    break;
                };
                self.refuse(d as u8, lent, Outcome::Aborted);
            }
        }
    }

    fn start_next(&mut self) {
        // Bounded: each pass takes one entry off a queue.
        for _ in 0..u8::MAX {
            if self.running.is_some() || self.spi.is_busy() {
                return;
            }
            let Some(d) = pick(self.ends.get(..Cs::COUNT).unwrap_or(&[])) else {
                return;
            };
            let Some(lent) = self
                .ends
                .get_mut(d)
                .and_then(Option::as_mut)
                .and_then(|e| e.lent.dequeue())
            else {
                return;
            };
            let device = d as u8;
            if self.last != Some(device) {
                let Some(config) = self.configs.get(d).copied().flatten() else {
                    self.refuse(device, lent, Outcome::Config(ConfigError::Frequency));
                    continue;
                };
                let start = (self.now)();
                let switched = self.spi.set_config(&config);
                let took = (self.now)().wrapping_sub(start);
                match switched {
                    Ok(sck) => {
                        self.last = Some(device);
                        self.stats.switches = self.stats.switches.saturating_add(1);
                        self.stats.longest_switch = self.stats.longest_switch.max(took);
                        if let Some(slot) = self.stats.sck.get_mut(d) {
                            *slot = sck.0;
                        }
                    }
                    Err(error) => {
                        self.last = None;
                        self.refuse(device, lent, Outcome::Config(error));
                        continue;
                    }
                }
            }
            self.cs.select(d, true);
            let started = (self.now)();
            let wire = lent.buf.get(..usize::from(lent.len)).unwrap_or(&[]);
            let started_ok = self.spi.start(wire);
            let took = (self.now)().wrapping_sub(started);
            self.copies.0 = self.copies.0.max(took);
            match started_ok {
                Ok(()) => {
                    self.running = Some(Running {
                        device,
                        lent,
                        started,
                    });
                    return;
                }
                Err(error) => {
                    self.cs.select(d, false);
                    self.refuse(device, lent, Outcome::Failed(error));
                }
            }
        }
    }
}

/// The device whose front entry goes next: the highest priority, the
/// oldest among equals, the lowest device among equal stamps. Looks, takes
/// nothing.
fn pick(ends: &[Option<LentEnd>]) -> Option<usize> {
    let mut best: Option<(usize, u8, u32)> = None;
    for (d, end) in ends.iter().enumerate() {
        let Some(front) = end.as_ref().and_then(|e| e.lent.peek()) else {
            continue;
        };
        let better = match best {
            None => true,
            Some((_, p, at)) => {
                front.priority > p
                    || (front.priority == p && crate::spi_queue::older(front.submitted, at))
            }
        };
        if better {
            best = Some((d, front.priority, front.submitted));
        }
    }
    best.map(|(d, _, _)| d)
}

impl<W: Parts, Cs: ChipSelects> Service for LentPort<W, Cs> {
    /// Nothing: the port wakes each device's holder itself.
    type Event = ();

    /// Ends a finished transfer, serves cancels, starts the next.
    fn on_interrupt(&mut self) {
        let start = (self.now)();
        if let Some(outcome) = self.spi.on_interrupt() {
            self.finish(outcome, false);
        }
        self.serve_cancels();
        self.start_next();
        let took = (self.now)().wrapping_sub(start);
        self.stats.longest_service = self.stats.longest_service.max(took);
    }
}

/// The handles, one per device, taken by number at boot.
pub struct LentHandles {
    handles: [Option<LentHandle>; DEVICES],
}

impl LentHandles {
    /// Device `device`'s handle, once.
    pub fn take(&mut self, device: usize) -> Option<LentHandle> {
        self.handles.get_mut(device).and_then(Option::take)
    }
}

/// What the constructor builds.
pub struct LentResources<W: Parts, Cs> {
    /// The member's shared resource.
    pub port: LentPort<W, Cs>,
    /// The consumers', each a `#[local]` of whichever task `init` picks.
    pub handles: LentHandles,
}

/// Why the constructor refused.
pub enum LentRefused<U, Sck, Miso, Mosi, TxD, RxD, Cs>
where
    U: Instance,
{
    /// FerroHAL refused the bus: its reason, with the wiring and buffers.
    Open(SpiRefused<U, Sck, Miso, Mosi, TxD, RxD>),
    /// There is not one config per chip select and per device's memory, or
    /// more than [`DEVICES`].
    Count(Cs),
    /// A device's settings: FerroHAL's reason, with the open driver and the
    /// pins, so the firmware can `free` it.
    Config {
        device: usize,
        error: ConfigError,
        spi: Spi<Wiring<U, Sck, Miso, Mosi, TxD, RxD>>,
        cs: Cs,
    },
    /// A lent buffer for a device that does not exist, longer than the
    /// bus's capacity, empty, or one more than the device's queues hold.
    Buffer {
        device: usize,
        len: usize,
        spi: Spi<Wiring<U, Sck, Miso, Mosi, TxD, RxD>>,
        cs: Cs,
    },
}

/// Builds the port: one config per chip select, device `i` on `cs.i` with
/// `configs[i]`, each applied once here so a device FerroHAL cannot serve
/// is refused at boot. `lent` gives each buffer to its device by number.
/// `kick` is the selection's generated pend of the member's vector; `now`
/// is the firmware's clock.
#[allow(
    clippy::type_complexity,
    clippy::too_many_arguments,
    clippy::result_large_err
)]
pub fn new_lent<U, Sck, Miso, Mosi, TxD, RxD, Cs, const N: usize, const D: usize>(
    rcc: &mut Rcc,
    wiring: Wiring<U, Sck, Miso, Mosi, TxD, RxD>,
    mut cs: Cs,
    configs: &[Config],
    tx_buf: &'static mut [u8],
    rx_buf: &'static mut [u8],
    memory: &'static mut LentMemory<N, D>,
    lent: &mut dyn Iterator<Item = (usize, &'static mut [u8])>,
    kick: fn(),
    now: fn() -> u32,
) -> Result<
    LentResources<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>,
    LentRefused<U, Sck, Miso, Mosi, TxD, RxD, Cs>,
>
where
    U: Instance,
    Sck: SckPin<U>,
    Miso: MisoPin<U>,
    Mosi: MosiPin<U>,
    TxD: SpiTxDma<U>,
    RxD: SpiRxDma<U>,
    Cs: ChipSelects,
{
    if configs.len() != Cs::COUNT || D != Cs::COUNT || Cs::COUNT > DEVICES {
        return Err(LentRefused::Count(cs));
    }
    let Some(first) = configs.first() else {
        return Err(LentRefused::Count(cs));
    };
    for d in 0..Cs::COUNT {
        cs.select(d, false);
    }
    let mut spi = Spi::new(
        rcc,
        wiring,
        first,
        Buffers {
            tx: tx_buf,
            rx: rx_buf,
        },
    )
    .map_err(LentRefused::Open)?;
    let mut stats = Stats::default();
    let mut table = [None; DEVICES];
    for (device, config) in configs.iter().enumerate().rev() {
        match spi.set_config(config) {
            Ok(sck) => {
                if let (Some(slot), Some(entry)) =
                    (stats.sck.get_mut(device), table.get_mut(device))
                {
                    *slot = sck.0;
                    *entry = Some(*config);
                }
            }
            Err(error) => {
                return Err(LentRefused::Config {
                    device,
                    error,
                    spi,
                    cs,
                });
            }
        }
    }
    // Applied last in reverse, so device 0's settings are in force.
    let mut handles = [const { None }; DEVICES];
    let mut ends = [const { None }; DEVICES];
    for (device, memory) in memory.devices.iter_mut().enumerate() {
        let (end, handle) = split_device(memory, device as u8, kick, now);
        if let (Some(e), Some(h)) = (ends.get_mut(device), handles.get_mut(device)) {
            *e = Some(end);
            *h = Some(handle);
        }
    }
    let capacity = spi.capacity();
    for (device, buf) in lent {
        let len = buf.len();
        let handle = handles.get_mut(device).and_then(Option::as_mut);
        let given = match handle {
            Some(h) if len > 0 && len <= capacity && usize::from(h.buffers) < h.room => {
                if h.free.push(buf).is_ok() {
                    h.buffers += 1;
                    h.longest = h.longest.max(len);
                    true
                } else {
                    false
                }
            }
            _ => false,
        };
        if !given {
            return Err(LentRefused::Buffer {
                device,
                len,
                spi,
                cs,
            });
        }
    }
    Ok(LentResources {
        port: LentPort {
            spi,
            cs,
            configs: table,
            ends,
            running: None,
            last: Some(0),
            now,
            stats,
            copies: (0, 0),
            short: 0,
        },
        handles: LentHandles { handles },
    })
}

/// R, the lent-buffer port. One member, on the receive stream's vector,
/// also pended by every handle. In `init`:
/// `spi_lent::new_lent(.., (cs0, cs1, ..), &configs, .., memory,
/// &mut [(0, buf_a), (1, buf_b)].into_iter(), <selection>::kick, now)`.
#[ferroforge::group(
    slots = [instance: U, sck: Sck, miso: Miso, mosi: Mosi, tx_dma: TxD, rx_dma: RxD, cs: Cs],
    vectors = [r_on_spi_rx_dma = rx_dma],
    shared = [port: LentPort<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [
        configs: &[Config] @ board,
        lent: &mut dyn Iterator<Item = (usize, &'static mut [u8])> @ software,
        now: fn() -> u32 @ platform,
    ],
    memory = [memory: LentMemory],
    handles = [handles: LentHandles],
    pends = [kick = r_on_spi_rx_dma],
    constructor = new_lent(rcc, wiring, cs, configs, tx_buf, rx_buf, memory, lent, kick, now),
)]
pub mod spi_lent_port {
    #[allow(unused_imports)]
    use super::{Config, LentHandles, LentMemory, LentPort, Wiring, new_lent};
    pub use ferrohal_stm32::Service;

    /// The receive stream's vector: a transfer's end, a submit, a cancel.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn r_on_spi_rx_dma(mut cx: r_on_spi_rx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::{AtomicU32, AtomicUsize};
    use std::{boxed::Box, vec::Vec as StdVec};

    static KICKS: AtomicUsize = AtomicUsize::new(0);
    fn kick() {
        KICKS.fetch_add(1, Ordering::Relaxed);
    }
    static CLOCK: AtomicU32 = AtomicU32::new(0);
    fn now() -> u32 {
        CLOCK.fetch_add(1, Ordering::Relaxed)
    }

    fn buf(len: usize) -> &'static mut [u8] {
        Box::leak(std::vec![0u8; len].into_boxed_slice())
    }

    /// One device with buffers of `sizes`, the port's end and the handle.
    fn device<const N: usize>(d: u8, sizes: &[usize]) -> (LentEnd, LentHandle) {
        let memory: &'static mut LentDevice<N> = Box::leak(Box::new(LentDevice::new()));
        let (end, mut h) = split_device(memory, d, kick, now);
        for &len in sizes {
            assert!(h.free.push(buf(len)).is_ok());
            h.buffers += 1;
            h.longest = h.longest.max(len);
        }
        (end, h)
    }

    /// What the port does at the end of a loopback transfer: the bytes
    /// come back as sent.
    fn echo(end: &mut LentEnd, outcome: Outcome) {
        let lent = end.lent.dequeue().unwrap();
        let done = Done {
            lent,
            outcome,
            waited: 1,
            took: 2,
        };
        assert!(end.returned.enqueue(done).is_ok());
    }

    #[test]
    fn a_message_is_built_in_place_and_read_back_in_place() {
        let (mut end, mut h) = device::<4>(0, &[16]);
        let mut m = h.message(3, 8).unwrap();
        m.write(&[0xA0]).write(&[1, 2]).read(3).transfer(&[7, 8]);
        assert_eq!((m.len(), m.capacity()), (8, 16));
        assert_eq!(m.submit(), Ok(()));
        {
            let front = end.lent.peek().unwrap();
            assert_eq!(
                &front.buf[..usize::from(front.len)],
                &[0xA0, 1, 2, FILLER, FILLER, FILLER, 7, 8]
            );
            assert_eq!((front.kept, front.priority), (2, 3));
        }
        echo(&mut end, Outcome::Done);
        let t = h.take().unwrap();
        assert_eq!(t.parts(), 2);
        assert_eq!(t.read(0), &[FILLER; 3]);
        assert_eq!(t.read(1), &[7, 8]);
        assert_eq!(t.read(2), &[] as &[u8]);
        assert_eq!((t.waited, t.took, t.priority, t.len()), (1, 2, 3, 8));
    }

    #[test]
    fn a_failed_message_reads_back_nothing() {
        let (mut end, mut h) = device::<4>(0, &[4]);
        let mut m = h.message(0, 0).unwrap();
        m.read(2);
        assert_eq!(m.submit(), Ok(()));
        echo(&mut end, Outcome::Aborted);
        let t = h.take().unwrap();
        assert_eq!(t.outcome, Outcome::Aborted);
        assert_eq!(t.read(0), &[] as &[u8]);
    }

    #[test]
    fn buffers_are_taken_lent_returned_and_freed() {
        let (mut end, mut h) = device::<4>(0, &[4, 4, 4]);
        assert_eq!((h.buffers(), h.outstanding()), (3, 0));
        for _ in 0..3 {
            let mut m = h.message(0, 4).unwrap();
            m.transfer(&[1, 2, 3, 4]);
            assert_eq!(m.submit(), Ok(()));
        }
        assert_eq!(h.outstanding(), 3);
        // Every buffer out: refused, and nothing lent changed (D8).
        assert_eq!(h.message(0, 1).err(), Some(SubmitError::Outstanding));
        echo(&mut end, Outcome::Done);
        // Back, not yet taken: still outstanding.
        assert_eq!(h.outstanding(), 3);
        drop(h.take());
        assert_eq!(h.outstanding(), 2);
        assert!(h.message(0, 4).is_ok());
        // The message above was dropped unsubmitted: its buffer is free.
        assert_eq!(h.outstanding(), 2);
        assert_eq!(h.lost(), 0);
    }

    #[test]
    fn a_malformed_message_is_refused_and_frees_its_buffer() {
        let (_end, mut h) = device::<4>(0, &[4, 64]);
        assert_eq!(h.message(0, 65).err(), Some(SubmitError::TooLong));
        assert_eq!(h.message(0, 0).unwrap().submit(), Err(SubmitError::Empty));
        // The shortest buffer that fits: 4 bytes, so a fifth overflows it.
        let mut m = h.message(0, 4).unwrap();
        assert_eq!(m.capacity(), 4);
        m.write(&[0; 5]);
        assert_eq!(m.submit(), Err(SubmitError::TooLong));
        let mut m = h.message(0, 64).unwrap();
        for _ in 0..=PARTS {
            m.read(1);
        }
        assert_eq!(m.submit(), Err(SubmitError::Parts));
        assert_eq!(h.outstanding(), 0);
    }

    #[test]
    fn the_shortest_free_buffer_that_fits_is_taken() {
        let (_end, mut h) = device::<5>(0, &[64, 4, 256, 4]);
        let a = h.message(0, 1).unwrap().capacity();
        assert_eq!(a, 4);
        let m = h.message(0, 65).unwrap();
        assert_eq!(m.capacity(), 256);
        drop(m);
        // Held at once: the two 4s, then the 64, then the 256.
        let mut sizes = StdVec::new();
        let mut held = StdVec::new();
        for _ in 0..4 {
            let mut m = h.message(0, 1).unwrap();
            sizes.push(m.capacity());
            m.write(&[1]);
            held.push(m.submit());
        }
        assert_eq!(sizes, [4, 4, 64, 256]);
        assert!(held.iter().all(Result::is_ok));
        assert_eq!(h.message(0, 1).err(), Some(SubmitError::Outstanding));
    }

    #[test]
    fn within_a_device_order_is_as_submitted() {
        let (mut end, mut h) = device::<5>(0, &[4; 4]);
        for p in [1, 5, 5, 9] {
            let mut m = h.message(p, 1).unwrap();
            m.write(&[p]);
            assert_eq!(m.submit(), Ok(()));
        }
        let order: StdVec<_> = core::iter::from_fn(|| end.lent.dequeue())
            .map(|l| l.priority)
            .collect();
        assert_eq!(order, [1, 5, 5, 9]);
    }

    /// `pick` over devices whose front entries are `fronts` (priority,
    /// submit stamp).
    fn pick_of(fronts: &[Option<(u8, u32)>]) -> Option<usize> {
        let mut ends = StdVec::new();
        let mut handles = StdVec::new();
        for (d, front) in fronts.iter().enumerate() {
            let (end, mut h) = device::<3>(d as u8, &[4]);
            if let Some((priority, submitted)) = *front {
                let lent = Lent {
                    buf: h.remove(0).unwrap(),
                    len: 1,
                    keep: [Part::default(); PARTS],
                    kept: 0,
                    priority,
                    submitted,
                };
                assert!(h.lent.enqueue(lent).is_ok());
            }
            ends.push(Some(end));
            handles.push(h);
        }
        pick(&ends)
    }

    #[test]
    fn across_devices_priority_then_age_then_device() {
        assert_eq!(pick_of(&[None, None]), None);
        assert_eq!(pick_of(&[Some((1, 0)), Some((9, 5))]), Some(1));
        assert_eq!(pick_of(&[Some((5, 7)), Some((5, 3)), None]), Some(1));
        assert_eq!(pick_of(&[Some((5, 3)), Some((5, 3))]), Some(0));
        // Across the clock's wrap: u32::MAX was stamped before 2.
        assert_eq!(pick_of(&[Some((5, 2)), Some((5, u32::MAX))]), Some(1));
    }

    #[test]
    fn cancel_flags_and_kicks() {
        let (end, mut h) = device::<3>(2, &[4]);
        let before = KICKS.load(Ordering::Relaxed);
        h.cancel();
        assert!(end.state.cancel.load(Ordering::Acquire));
        assert!(KICKS.load(Ordering::Relaxed) > before);
        assert_eq!(h.device(), 2);
    }

    #[test]
    fn a_submit_kicks_the_port() {
        let (_end, mut h) = device::<3>(0, &[4]);
        let before = KICKS.load(Ordering::Relaxed);
        let mut m = h.message(0, 1).unwrap();
        m.write(&[1]);
        assert_eq!(m.submit(), Ok(()));
        assert!(KICKS.load(Ordering::Relaxed) > before);
    }

    #[test]
    fn the_entry_is_small_whatever_the_data_length() {
        // A reference, a length, the part ranges, the priority, the stamp.
        let entry = core::mem::size_of::<Lent>();
        assert!(entry <= 64, "entry {entry} bytes");
    }
}
