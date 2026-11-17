//! PROTOTYPE: the SPI port comparison of the port design record, section 3a
//! (`spi-port-prototype-design.md`). Not product code; not merged.
//!
//! Two ports on FerroHAL's single-owner [`Spi`], with one consumer type for
//! both, [`SpiHandle`]:
//!
//! - **S, single device** ([`spi_single_port`]): one chip select, a
//!   one-place lock-free slot between the handle and the port. No queue, no
//!   priority, no settings switch.
//! - **M, multi device** ([`spi_multi_port`]): up to [`DEVICES`] chip
//!   selects, each with its own bus settings, and one fixed-size priority
//!   queue for the bus. A full queue refuses (D8). The port always starts the
//!   highest priority, oldest first among equals, and never interrupts a
//!   running transfer.
//!
//! In both, a [`Transaction`] is one chip-select assertion sent as one
//! continuous full-duplex transfer; its parts are concatenated. Bytes are
//! copied into the transaction, so no buffer is shared between consumer and
//! port. Each result returns through the handle's own lock-free queue, with
//! how long it waited and how long it took (D7). `submit` reserves a result
//! slot, so no result is lost: when a device has as many transactions
//! outstanding as its result queue holds, `submit` refuses.
//!
//! The port's one member serves the receive stream's vector, which FerroHAL
//! says ends a transfer, and which the handle pends after a submit or a
//! cancel (the group's generated pend). Nothing here panics.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use critical_section::Mutex;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use ferrohal_stm32::{
    Service,
    rcc::Rcc,
    spi::{
        Buffers, Config, ConfigError, Error, Instance, MisoPin, MosiPin, Parts, SckPin, Spi,
        SpiRefused, SpiRxDma, SpiTxDma, Wiring,
    },
};
use heapless::spsc::{Consumer, Producer, Queue};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

/// The longest transaction, in bytes on the wire.
pub const MAX: usize = 64;
/// The most parts a transaction keeps the received bytes of.
pub const PARTS: usize = 8;
/// The most devices one port serves.
pub const DEVICES: usize = 8;
/// What a read part clocks out, as FerroHAL's `SpiBus::read` does.
pub const FILLER: u8 = 0x00;

/// Why a `submit` was refused. Each leaves everything as it was (D8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubmitError {
    /// The port's queue (M) or slot (S) is full.
    Full,
    /// The parts add up to more than [`MAX`] bytes.
    TooLong,
    /// More than [`PARTS`] parts keep what they receive.
    Parts,
    /// Nothing to send.
    Empty,
    /// The device has as many transactions outstanding as its result queue
    /// holds; take a result first.
    Outstanding,
}

/// How a transaction ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Every byte went out and came in.
    Done,
    /// FerroHAL refused the start or reported a fault (D16: never hidden).
    Failed(Error),
    /// FerroHAL refused the device's bus settings.
    Config(ConfigError),
    /// Cancelled: dropped from the queue, or aborted on the wire.
    Aborted,
}

/// One kept part's place in the transaction.
#[derive(Clone, Copy, Default)]
pub(crate) struct Part {
    pub(crate) at: u16,
    pub(crate) len: u16,
}

/// One chip-select assertion: parts appended back to back, sent as one
/// continuous transfer.
#[derive(Clone, Copy)]
pub struct Transaction {
    bytes: [u8; MAX],
    len: u16,
    keep: [Part; PARTS],
    kept: u8,
    priority: u8,
    refused: Option<SubmitError>,
}

impl Transaction {
    /// An empty transaction. Higher `priority` goes first on a multi-device
    /// port; a single-device port ignores it.
    pub const fn new(priority: u8) -> Self {
        Self {
            bytes: [0; MAX],
            len: 0,
            keep: [Part { at: 0, len: 0 }; PARTS],
            kept: 0,
            priority,
            refused: None,
        }
    }

    /// Appends bytes to send; what comes back during them is dropped.
    pub fn write(&mut self, bytes: &[u8]) -> &mut Self {
        self.append(Some(bytes), bytes.len(), false);
        self
    }

    /// Appends `n` filler bytes and keeps what comes back during them.
    pub fn read(&mut self, n: usize) -> &mut Self {
        self.append(None, n, true);
        self
    }

    /// Appends bytes to send and keeps what comes back during them.
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

    pub fn priority(&self) -> u8 {
        self.priority
    }

    fn append(&mut self, src: Option<&[u8]>, n: usize, keep: bool) {
        if self.refused.is_some() || n == 0 {
            return;
        }
        let at = usize::from(self.len);
        let Some(dst) = self.bytes.get_mut(at..at + n) else {
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
            // Both fit in u16: at + n <= MAX.
            *part = Part {
                at: at as u16,
                len: n as u16,
            };
            self.kept += 1;
        }
        self.len = (at + n) as u16;
    }

    fn check(&self) -> Result<(), SubmitError> {
        if let Some(refused) = self.refused {
            return Err(refused);
        }
        if self.len == 0 {
            return Err(SubmitError::Empty);
        }
        Ok(())
    }

    fn wire(&self) -> &[u8] {
        self.bytes.get(..usize::from(self.len)).unwrap_or(&[])
    }
}

/// What came back for one transaction.
#[derive(Clone, Copy)]
pub struct SpiResult {
    bytes: [u8; MAX],
    len: u16,
    /// How it ended.
    pub outcome: Outcome,
    /// Clock ticks from `submit` to the transfer's start (or to the drop,
    /// for a cancelled one): how long it was queued.
    pub waited: u32,
    /// Clock ticks on the wire, start to end.
    pub took: u32,
    /// The transaction's priority, to tell results apart.
    pub priority: u8,
}

impl SpiResult {
    /// What came back during the read and transfer parts, in order;
    /// empty unless the outcome is `Done`.
    pub fn bytes(&self) -> &[u8] {
        self.bytes.get(..usize::from(self.len)).unwrap_or(&[])
    }
}

/// A transaction in the port's queue or slot.
#[derive(Clone, Copy)]
struct Entry {
    txn: Transaction,
    device: u8,
    submitted: u32,
    seq: u32,
}

/// What the handle and the port both see of one device.
struct DeviceState {
    /// Transactions submitted and not yet taken as results.
    outstanding: AtomicU8,
    /// Set by the handle's `cancel`, cleared by the port when served.
    cancel: AtomicBool,
}

/// One device's memory: its result queue, its state and its wake. `R`
/// slots hold `R - 1` results, and bound what it may have outstanding.
pub struct DeviceMemory<const R: usize> {
    results: Queue<SpiResult, R>,
    state: DeviceState,
    wake: Signal<()>,
}

impl<const R: usize> DeviceMemory<R> {
    pub const fn new() -> Self {
        Self {
            results: Queue::new(),
            state: DeviceState {
                outstanding: AtomicU8::new(0),
                cancel: AtomicBool::new(false),
            },
            wake: Signal::new(),
        }
    }
}

impl<const R: usize> Default for DeviceMemory<R> {
    fn default() -> Self {
        Self::new()
    }
}

/// The port's end of one device.
struct DeviceEnd {
    results: Producer<'static, SpiResult>,
    wake: SignalWriter<'static, ()>,
    state: &'static DeviceState,
}

/// Splits one device's memory into the port's end and a handle.
fn split_device<const R: usize>(
    memory: &'static mut DeviceMemory<R>,
    device: u8,
    route: Route,
    kick: fn(),
    now: fn() -> u32,
) -> (DeviceEnd, SpiHandle) {
    let DeviceMemory {
        results,
        state,
        wake,
    } = memory;
    let state: &'static DeviceState = state;
    let wake: &'static Signal<()> = wake;
    let capacity = u8::try_from(results.capacity()).unwrap_or(u8::MAX);
    let (producer, consumer) = results.split();
    let (writer, reader) = wake.split();
    (
        DeviceEnd {
            results: producer,
            wake: writer,
            state,
        },
        SpiHandle {
            results: consumer,
            state,
            capacity,
            wake: reader,
            route,
            device,
            kick,
            now,
        },
    )
}

/// The multi-device port's shared priority queue. `Q` entries; a push and
/// a pop each take one critical section, whose length the queue measures.
pub struct PriorityQueue<const Q: usize> {
    inner: Mutex<RefCell<Slots<Q>>>,
}

struct Slots<const Q: usize> {
    entries: [Option<Entry>; Q],
    /// PROTOTYPE measurement: the longest push and pop, in clock ticks,
    /// with interrupts off.
    longest_push: u32,
    longest_pop: u32,
    /// The next entry's sequence number: equal priorities run oldest first.
    seq: u32,
}

impl<const Q: usize> PriorityQueue<Q> {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(Slots {
                entries: [const { None }; Q],
                longest_push: 0,
                longest_pop: 0,
                seq: 0,
            })),
        }
    }
}

impl<const Q: usize> Default for PriorityQueue<Q> {
    fn default() -> Self {
        Self::new()
    }
}

/// What the handle and the port need of the shared queue, whatever its size.
trait SharedQueue: Sync {
    /// Into the first free entry, or the entry back when full.
    fn push(&self, entry: Entry, now: fn() -> u32) -> Result<(), Entry>;
    /// The highest priority, oldest among equals; only `device`'s when given.
    fn pop(&self, device: Option<u8>, now: fn() -> u32) -> Option<Entry>;
    /// Longest push and pop, in clock ticks with interrupts off.
    fn longest(&self) -> (u32, u32);
}

impl<const Q: usize> SharedQueue for PriorityQueue<Q> {
    fn push(&self, entry: Entry, now: fn() -> u32) -> Result<(), Entry> {
        critical_section::with(|cs| {
            let start = now();
            let mut slots = self.inner.borrow_ref_mut(cs);
            let seq = slots.seq;
            let pushed = match slots.entries.iter_mut().find(|e| e.is_none()) {
                Some(free) => {
                    *free = Some(Entry { seq, ..entry });
                    Ok(())
                }
                None => Err(entry),
            };
            if pushed.is_ok() {
                slots.seq = seq.wrapping_add(1);
            }
            let took = now().wrapping_sub(start);
            slots.longest_push = slots.longest_push.max(took);
            pushed
        })
    }

    fn pop(&self, device: Option<u8>, now: fn() -> u32) -> Option<Entry> {
        critical_section::with(|cs| {
            let start = now();
            let mut slots = self.inner.borrow_ref_mut(cs);
            let mut best: Option<(usize, u8, u32)> = None;
            for (i, e) in slots.entries.iter().enumerate() {
                let Some(e) = e else { continue };
                if device.is_some_and(|d| d != e.device) {
                    continue;
                }
                let better = match best {
                    None => true,
                    Some((_, p, seq)) => {
                        e.txn.priority > p || (e.txn.priority == p && older(e.seq, seq))
                    }
                };
                if better {
                    best = Some((i, e.txn.priority, e.seq));
                }
            }
            let popped = best.and_then(|(i, _, _)| slots.entries.get_mut(i).and_then(Option::take));
            let took = now().wrapping_sub(start);
            slots.longest_pop = slots.longest_pop.max(took);
            popped
        })
    }

    fn longest(&self) -> (u32, u32) {
        critical_section::with(|cs| {
            let slots = self.inner.borrow_ref(cs);
            (slots.longest_push, slots.longest_pop)
        })
    }
}

/// Whether sequence number `a` was given before `b`, across a wrap.
pub(crate) fn older(a: u32, b: u32) -> bool {
    (b.wrapping_sub(a) as i32) > 0
}

/// Where a handle puts what it submits.
enum Route {
    /// S: the port's one-place slot.
    Single(Producer<'static, Entry>),
    /// M: the bus's priority queue.
    Shared(&'static dyn SharedQueue),
}

/// The consumer's end of one SPI device. The same type on either port and
/// for every device, so `init` can give any handle to any task (D12).
pub struct SpiHandle {
    results: Consumer<'static, SpiResult>,
    state: &'static DeviceState,
    capacity: u8,
    wake: SignalReader<'static, ()>,
    route: Route,
    device: u8,
    kick: fn(),
    now: fn() -> u32,
}

impl SpiHandle {
    /// An empty transaction at `priority`, to fill and submit.
    pub fn transaction(&self, priority: u8) -> Transaction {
        Transaction::new(priority)
    }

    /// Queues `txn` and pends the port. Refused, with nothing changed, when
    /// the queue is full, the transaction is malformed, or the device has
    /// as many results outstanding as its result queue holds.
    pub fn submit(&mut self, txn: Transaction) -> Result<(), SubmitError> {
        txn.check()?;
        let state = self.state;
        let capacity = self.capacity;
        state
            .outstanding
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                (n < capacity).then_some(n + 1)
            })
            .map_err(|_| SubmitError::Outstanding)?;
        // The shared queue numbers the entry as it takes it.
        let entry = Entry {
            txn,
            device: self.device,
            submitted: (self.now)(),
            seq: 0,
        };
        let pushed = match &mut self.route {
            Route::Single(slot) => slot.enqueue(entry).map_err(|_| ()),
            Route::Shared(queue) => queue.push(entry, self.now).map_err(|_| ()),
        };
        if pushed.is_err() {
            state.outstanding.fetch_sub(1, Ordering::AcqRel);
            return Err(SubmitError::Full);
        }
        (self.kick)();
        Ok(())
    }

    /// The oldest result not yet taken.
    pub fn take(&mut self) -> Option<SpiResult> {
        let result = self.results.dequeue()?;
        self.state.outstanding.fetch_sub(1, Ordering::AcqRel);
        Some(result)
    }

    /// Waits until the port has a result for this device.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// Asks the port to drop this device's queued transactions and abort
    /// its running one; each returns as a result marked `Aborted`, with
    /// chip select released.
    pub fn cancel(&mut self) {
        self.state.cancel.store(true, Ordering::Release);
        (self.kick)();
    }

    /// Transactions submitted and not yet taken.
    pub fn outstanding(&self) -> u8 {
        self.state.outstanding.load(Ordering::Acquire)
    }

    /// This device's number on its port.
    pub fn device(&self) -> u8 {
        self.device
    }
}

/// The chip-select pins of a port's devices, active low, numbered from 0:
/// one pin for S, a tuple of up to [`DEVICES`] pins for M, each its own type.
pub trait ChipSelects {
    /// How many devices.
    const COUNT: usize;
    /// Drives device `device`'s line: low when `selected`.
    fn select(&mut self, device: usize, selected: bool);
    /// Reads back the level the port drives (the output register):
    /// `Some(true)` when selected. PROTOTYPE: for the bench's checks.
    fn is_selected(&mut self, device: usize) -> Option<bool>;
}

fn drive<P: OutputPin>(pin: &mut P, selected: bool) {
    let _ = if selected {
        pin.set_low()
    } else {
        pin.set_high()
    };
}

fn read<P: StatefulOutputPin>(pin: &mut P) -> Option<bool> {
    pin.is_set_low().ok()
}

macro_rules! chip_selects {
    ($count:literal; $($index:tt $pin:ident),+) => {
        impl<$($pin: OutputPin + StatefulOutputPin),+> ChipSelects for ($($pin,)+) {
            const COUNT: usize = $count;
            fn select(&mut self, device: usize, selected: bool) {
                match device {
                    $($index => drive(&mut self.$index, selected),)+
                    _ => {}
                }
            }
            fn is_selected(&mut self, device: usize) -> Option<bool> {
                match device {
                    $($index => read(&mut self.$index),)+
                    _ => None,
                }
            }
        }
    };
}

chip_selects!(1; 0 A);
chip_selects!(2; 0 A, 1 B);
chip_selects!(3; 0 A, 1 B, 2 C);
chip_selects!(4; 0 A, 1 B, 2 C, 3 D);
chip_selects!(5; 0 A, 1 B, 2 C, 3 D, 4 E);
chip_selects!(6; 0 A, 1 B, 2 C, 3 D, 4 E, 5 F);
chip_selects!(7; 0 A, 1 B, 2 C, 3 D, 4 E, 5 F, 6 G);
chip_selects!(8; 0 A, 1 B, 2 C, 3 D, 4 E, 5 F, 6 G, 7 H);

/// PROTOTYPE check: whether, straight after device `d`'s transfer ended,
/// its own line was not low or another line was not high.
#[cfg_attr(feature = "prototype-quiet", allow(dead_code))]
pub(crate) fn readback_wrong<Cs: ChipSelects>(cs: &mut Cs, d: usize) -> bool {
    let mut wrong = cs.is_selected(d) != Some(true);
    for other in 0..Cs::COUNT {
        if other != d && cs.is_selected(other) != Some(false) {
            wrong = true;
        }
    }
    wrong
}

/// Where the port takes what to send next.
enum Source {
    Single(Consumer<'static, Entry>),
    Shared(&'static dyn SharedQueue),
}

impl Source {
    fn next(&mut self, now: fn() -> u32) -> Option<Entry> {
        match self {
            Self::Single(slot) => slot.dequeue(),
            Self::Shared(queue) => queue.pop(None, now),
        }
    }

    fn next_of(&mut self, device: u8, now: fn() -> u32) -> Option<Entry> {
        match self {
            // The slot holds one device's entries only.
            Self::Single(slot) => slot.dequeue(),
            Self::Shared(queue) => queue.pop(Some(device), now),
        }
    }
}

/// The transfer on the wire.
#[derive(Clone, Copy)]
struct Running {
    device: u8,
    keep: [Part; PARTS],
    kept: u8,
    priority: u8,
    submitted: u32,
    started: u32,
}

/// PROTOTYPE measurements, in clock ticks unless named otherwise.
#[derive(Clone, Copy, Default, Debug)]
pub struct Stats {
    /// Longest `on_interrupt`.
    pub longest_service: u32,
    /// Bus settings switches, and the longest one.
    pub switches: u32,
    pub longest_switch: u32,
    /// The SCK FerroHAL reported after each device's last switch, in Hz.
    pub sck: [u32; DEVICES],
    /// Transfers that ended with their own device not selected, or another
    /// device selected, read straight after the end.
    pub cs_wrong: u32,
    /// Results the port could not queue. Zero by construction (`submit`
    /// reserves a slot); counted so a broken reservation is not hidden.
    pub lost: u32,
}

/// The port's own state: FerroHAL's `Spi`, the chip selects, each device's
/// settings and result end, and where transactions come from.
pub struct Port<W: Parts, Cs> {
    spi: Spi<W>,
    cs: Cs,
    configs: [Option<Config>; DEVICES],
    ends: [Option<DeviceEnd>; DEVICES],
    source: Source,
    running: Option<Running>,
    last: Option<u8>,
    now: fn() -> u32,
    stats: Stats,
}

impl<W: Parts, Cs: ChipSelects> Port<W, Cs> {
    /// FerroHAL's driver, for checks such as `health()` and `vectors()`.
    pub fn spi(&self) -> &Spi<W> {
        &self.spi
    }

    /// PROTOTYPE measurements.
    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// PROTOTYPE measurement: longest push and pop of M's queue, in clock
    /// ticks with interrupts off; zero for S.
    pub fn queue_longest(&self) -> (u32, u32) {
        match &self.source {
            Source::Single(_) => (0, 0),
            Source::Shared(queue) => queue.longest(),
        }
    }

    /// PROTOTYPE: device `device`'s chip select, as the port drives it.
    pub fn is_selected(&mut self, device: usize) -> Option<bool> {
        self.cs.is_selected(device)
    }

    fn deliver(&mut self, device: u8, result: SpiResult) {
        let Some(end) = self
            .ends
            .get_mut(usize::from(device))
            .and_then(Option::as_mut)
        else {
            self.stats.lost = self.stats.lost.saturating_add(1);
            return;
        };
        if end.results.enqueue(result).is_err() {
            self.stats.lost = self.stats.lost.saturating_add(1);
        }
        end.wake.write(());
    }

    /// The running transfer ended with `outcome`: release, then report.
    fn finish(&mut self, outcome: Result<usize, Error>, aborted: bool) {
        let Some(run) = self.running.take() else {
            return;
        };
        let now = (self.now)();
        let d = usize::from(run.device);
        // PROTOTYPE check: own line low, every other line high, read
        // straight after the end and before the release. Compiled out by
        // `prototype-quiet`, to measure the port's run without it.
        #[cfg(not(feature = "prototype-quiet"))]
        if readback_wrong(&mut self.cs, d) {
            self.stats.cs_wrong = self.stats.cs_wrong.saturating_add(1);
        }
        self.cs.select(d, false);
        let mut result = SpiResult {
            bytes: [0; MAX],
            len: 0,
            outcome: Outcome::Done,
            waited: run.started.wrapping_sub(run.submitted),
            took: now.wrapping_sub(run.started),
            priority: run.priority,
        };
        match (aborted, outcome) {
            (true, _) => result.outcome = Outcome::Aborted,
            (false, Err(error)) => result.outcome = Outcome::Failed(error),
            (false, Ok(_)) => {
                let received = self.spi.received();
                let mut at = 0;
                for part in run.keep.iter().take(usize::from(run.kept)) {
                    let (from, len) = (usize::from(part.at), usize::from(part.len));
                    if let (Some(dst), Some(src)) = (
                        result.bytes.get_mut(at..at + len),
                        received.get(from..from + len),
                    ) {
                        dst.copy_from_slice(src);
                        at += len;
                    }
                }
                // at <= MAX.
                result.len = at as u16;
            }
        }
        self.deliver(run.device, result);
    }

    /// A queued entry that never reached the wire.
    fn refuse(&mut self, entry: &Entry, outcome: Outcome) {
        let now = (self.now)();
        let result = SpiResult {
            bytes: [0; MAX],
            len: 0,
            outcome,
            waited: now.wrapping_sub(entry.submitted),
            took: 0,
            priority: entry.txn.priority,
        };
        self.deliver(entry.device, result);
    }

    fn serve_cancels(&mut self) {
        for d in 0..Cs::COUNT.min(DEVICES) {
            let Some(end) = self.ends.get(d).and_then(Option::as_ref) else {
                continue;
            };
            if !end.state.cancel.swap(false, Ordering::AcqRel) {
                continue;
            }
            if self.running.is_some_and(|r| usize::from(r.device) == d) {
                self.spi.abort();
                self.finish(Err(Error::Busy), true);
            }
            // Bounded by the queue: each pass removes one entry.
            for _ in 0..u8::MAX {
                let Some(entry) = self.source.next_of(d as u8, self.now) else {
                    break;
                };
                self.refuse(&entry, Outcome::Aborted);
            }
        }
    }

    fn start_next(&mut self) {
        // Bounded: each pass takes one entry off the queue.
        for _ in 0..u8::MAX {
            if self.running.is_some() || self.spi.is_busy() {
                return;
            }
            let Some(entry) = self.source.next(self.now) else {
                return;
            };
            let d = entry.device;
            if self.last != Some(d) {
                let Some(config) = self.configs.get(usize::from(d)).copied().flatten() else {
                    self.refuse(&entry, Outcome::Config(ConfigError::Frequency));
                    continue;
                };
                let start = (self.now)();
                let switched = self.spi.set_config(&config);
                let took = (self.now)().wrapping_sub(start);
                match switched {
                    Ok(sck) => {
                        self.last = Some(d);
                        self.stats.switches = self.stats.switches.saturating_add(1);
                        self.stats.longest_switch = self.stats.longest_switch.max(took);
                        if let Some(slot) = self.stats.sck.get_mut(usize::from(d)) {
                            *slot = sck.0;
                        }
                    }
                    Err(error) => {
                        self.last = None;
                        self.refuse(&entry, Outcome::Config(error));
                        continue;
                    }
                }
            }
            self.cs.select(usize::from(d), true);
            let started = (self.now)();
            match self.spi.start(entry.txn.wire()) {
                Ok(()) => {
                    self.running = Some(Running {
                        device: d,
                        keep: entry.txn.keep,
                        kept: entry.txn.kept,
                        priority: entry.txn.priority,
                        submitted: entry.submitted,
                        started,
                    });
                    return;
                }
                Err(error) => {
                    self.cs.select(usize::from(d), false);
                    self.refuse(&entry, Outcome::Failed(error));
                }
            }
        }
    }
}

impl<W: Parts, Cs: ChipSelects> Service for Port<W, Cs> {
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

/// S's memory: the one-place slot and the device's.
pub struct SingleMemory<const R: usize> {
    slot: Queue<Entry, 2>,
    device: DeviceMemory<R>,
}

impl<const R: usize> SingleMemory<R> {
    pub const fn new() -> Self {
        Self {
            slot: Queue::new(),
            device: DeviceMemory::new(),
        }
    }
}

impl<const R: usize> Default for SingleMemory<R> {
    fn default() -> Self {
        Self::new()
    }
}

/// M's memory: the bus's queue, `Q` entries, and `N` devices' memory.
pub struct MultiMemory<const Q: usize, const R: usize, const N: usize> {
    queue: PriorityQueue<Q>,
    devices: [DeviceMemory<R>; N],
}

impl<const Q: usize, const R: usize, const N: usize> MultiMemory<Q, R, N> {
    pub const fn new() -> Self {
        Self {
            queue: PriorityQueue::new(),
            devices: [const { DeviceMemory::new() }; N],
        }
    }
}

impl<const Q: usize, const R: usize, const N: usize> Default for MultiMemory<Q, R, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// M's handles, one per device, taken by number at boot.
pub struct Handles {
    handles: [Option<SpiHandle>; DEVICES],
}

impl Handles {
    /// Device `device`'s handle, once.
    pub fn take(&mut self, device: usize) -> Option<SpiHandle> {
        self.handles.get_mut(device).and_then(Option::take)
    }
}

/// What S's constructor builds.
pub struct SingleResources<W: Parts, Cs> {
    /// The member's shared resource.
    pub port: Port<W, (Cs,)>,
    /// The consumer's, as a `#[local]`, given to any task at boot.
    pub handle: SpiHandle,
}

/// What M's constructor builds.
pub struct MultiResources<W: Parts, Cs> {
    pub port: Port<W, Cs>,
    pub handles: Handles,
}

/// Why M's constructor refused.
pub enum MultiRefused<U, Sck, Miso, Mosi, TxD, RxD, Cs>
where
    U: Instance,
{
    /// FerroHAL refused the bus: its reason, with the wiring and buffers.
    Open(SpiRefused<U, Sck, Miso, Mosi, TxD, RxD>),
    /// There is not one config per chip select, or more than [`DEVICES`].
    Count(Cs),
    /// A device's settings: FerroHAL's reason, with the open driver and the
    /// pins, so the firmware can `free` it.
    Config {
        device: usize,
        error: ConfigError,
        spi: Spi<Wiring<U, Sck, Miso, Mosi, TxD, RxD>>,
        cs: Cs,
    },
}

/// Builds S: opens the SPI on `wiring` with `config`, deselects `cs`, and
/// splits `memory`. `kick` is the selection's generated pend of the
/// member's vector; `now` is the firmware's clock.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn new_single<U, Sck, Miso, Mosi, TxD, RxD, Cs, const R: usize>(
    rcc: &mut Rcc,
    wiring: Wiring<U, Sck, Miso, Mosi, TxD, RxD>,
    cs: Cs,
    config: &Config,
    tx_buf: &'static mut [u8],
    rx_buf: &'static mut [u8],
    memory: &'static mut SingleMemory<R>,
    kick: fn(),
    now: fn() -> u32,
) -> Result<
    SingleResources<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>,
    SpiRefused<U, Sck, Miso, Mosi, TxD, RxD>,
>
where
    U: Instance,
    Sck: SckPin<U>,
    Miso: MisoPin<U>,
    Mosi: MosiPin<U>,
    TxD: SpiTxDma<U>,
    RxD: SpiRxDma<U>,
    Cs: OutputPin + StatefulOutputPin,
{
    let mut cs = (cs,);
    cs.select(0, false);
    let spi = Spi::new(
        rcc,
        wiring,
        config,
        Buffers {
            tx: tx_buf,
            rx: rx_buf,
        },
    )?;
    let SingleMemory { slot, device } = memory;
    let (producer, consumer) = slot.split();
    let (end, handle) = split_device(device, 0, Route::Single(producer), kick, now);
    let mut configs = [None; DEVICES];
    configs[0] = Some(*config);
    let mut ends = [const { None }; DEVICES];
    ends[0] = Some(end);
    let mut stats = Stats::default();
    stats.sck[0] = spi.frequency().0;
    Ok(SingleResources {
        port: Port {
            spi,
            cs,
            configs,
            ends,
            source: Source::Single(consumer),
            running: None,
            last: Some(0),
            now,
            stats,
        },
        handle,
    })
}

/// Builds M: one config per chip select, device `i` on `cs.i` with
/// `configs[i]`. Every config is applied once here, so a device FerroHAL
/// cannot serve is refused at boot, not at its first transfer. The refusal
/// carries the open driver back, so it is large; it happens once, at boot.
#[allow(
    clippy::type_complexity,
    clippy::too_many_arguments,
    clippy::result_large_err
)]
pub fn new_multi<U, Sck, Miso, Mosi, TxD, RxD, Cs, const Q: usize, const R: usize, const N: usize>(
    rcc: &mut Rcc,
    wiring: Wiring<U, Sck, Miso, Mosi, TxD, RxD>,
    mut cs: Cs,
    configs: &[Config],
    tx_buf: &'static mut [u8],
    rx_buf: &'static mut [u8],
    memory: &'static mut MultiMemory<Q, R, N>,
    kick: fn(),
    now: fn() -> u32,
) -> Result<
    MultiResources<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>,
    MultiRefused<U, Sck, Miso, Mosi, TxD, RxD, Cs>,
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
    if configs.len() != Cs::COUNT || N != Cs::COUNT || Cs::COUNT > DEVICES {
        return Err(MultiRefused::Count(cs));
    }
    let Some(first) = configs.first() else {
        return Err(MultiRefused::Count(cs));
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
    .map_err(MultiRefused::Open)?;
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
                return Err(MultiRefused::Config {
                    device,
                    error,
                    spi,
                    cs,
                });
            }
        }
    }
    // Applied last in reverse, so device 0's settings are in force.
    let MultiMemory { queue, devices } = memory;
    let queue: &'static PriorityQueue<Q> = queue;
    let mut handles = [const { None }; DEVICES];
    let mut ends = [const { None }; DEVICES];
    for (device, memory) in devices.iter_mut().enumerate() {
        let (end, handle) = split_device(memory, device as u8, Route::Shared(queue), kick, now);
        if let (Some(e), Some(h)) = (ends.get_mut(device), handles.get_mut(device)) {
            *e = Some(end);
            *h = Some(handle);
        }
    }
    Ok(MultiResources {
        port: Port {
            spi,
            cs,
            configs: table,
            ends,
            source: Source::Shared(queue),
            running: None,
            last: Some(0),
            now,
            stats,
        },
        handles: Handles { handles },
    })
}

/// S, the single-device port. One member, on the receive stream's vector,
/// also pended by the handle. In `init`:
/// `spi_queue::new_single(.., memory, <selection>::kick, now)`.
#[ferroforge::group(
    slots = [instance: U, sck: Sck, miso: Miso, mosi: Mosi, tx_dma: TxD, rx_dma: RxD, cs: Cs],
    vectors = [s_on_spi_rx_dma = rx_dma],
    shared = [port: Port<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, (Cs,)>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [config: Config @ board, now: fn() -> u32 @ platform],
    memory = [memory: SingleMemory],
    handles = [handle: SpiHandle],
    pends = [kick = s_on_spi_rx_dma],
    constructor = new_single(rcc, wiring, cs, config, tx_buf, rx_buf, memory, kick, now),
)]
pub mod spi_single_port {
    #[allow(unused_imports)]
    use super::{Config, Port, SingleMemory, SpiHandle, Wiring, new_single};
    pub use ferrohal_stm32::Service;

    /// The receive stream's vector: a transfer's end, a submit, a cancel.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn s_on_spi_rx_dma(mut cx: s_on_spi_rx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }
}

/// M, the multi-device port. One member, as S's.
/// `spi_queue::new_multi(.., (cs0, cs1, ..), &configs, .., memory,
/// <selection>::kick, now)`.
#[ferroforge::group(
    slots = [instance: U, sck: Sck, miso: Miso, mosi: Mosi, tx_dma: TxD, rx_dma: RxD, cs: Cs],
    vectors = [m_on_spi_rx_dma = rx_dma],
    shared = [port: Port<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [configs: &[Config] @ board, now: fn() -> u32 @ platform],
    memory = [memory: MultiMemory],
    handles = [handles: Handles],
    pends = [kick = m_on_spi_rx_dma],
    constructor = new_multi(rcc, wiring, cs, configs, tx_buf, rx_buf, memory, kick, now),
)]
pub mod spi_multi_port {
    #[allow(unused_imports)]
    use super::{Config, Handles, MultiMemory, Port, Wiring, new_multi};
    pub use ferrohal_stm32::Service;

    /// The receive stream's vector: a transfer's end, a submit, a cancel.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn m_on_spi_rx_dma(mut cx: m_on_spi_rx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::{AtomicU32, AtomicUsize};
    use std::{boxed::Box, vec::Vec};

    static KICKS: AtomicUsize = AtomicUsize::new(0);
    fn kick() {
        KICKS.fetch_add(1, Ordering::Relaxed);
    }
    static CLOCK: AtomicU32 = AtomicU32::new(0);
    fn now() -> u32 {
        CLOCK.fetch_add(1, Ordering::Relaxed)
    }

    #[test]
    fn parts_are_appended_and_only_kept_parts_are_listed() {
        let mut t = Transaction::new(3);
        t.write(&[0xA0]).write(&[1, 2]).read(3).transfer(&[7, 8]);
        assert_eq!(t.wire(), &[0xA0, 1, 2, FILLER, FILLER, FILLER, 7, 8]);
        assert_eq!(t.kept, 2);
        assert_eq!((t.keep[0].at, t.keep[0].len), (3, 3));
        assert_eq!((t.keep[1].at, t.keep[1].len), (6, 2));
        assert_eq!(t.check(), Ok(()));
        assert_eq!(t.priority(), 3);
    }

    #[test]
    fn a_malformed_transaction_is_refused_whole() {
        assert_eq!(Transaction::new(0).check(), Err(SubmitError::Empty));
        let mut long = Transaction::new(0);
        long.write(&[0; MAX]).write(&[1]);
        assert_eq!(long.check(), Err(SubmitError::TooLong));
        assert_eq!(long.len(), MAX);
        let mut parts = Transaction::new(0);
        for _ in 0..=PARTS {
            parts.read(1);
        }
        assert_eq!(parts.check(), Err(SubmitError::Parts));
    }

    fn entry(device: u8, priority: u8) -> Entry {
        let mut txn = Transaction::new(priority);
        txn.write(&[device, priority]);
        Entry {
            txn,
            device,
            submitted: 0,
            seq: 0,
        }
    }

    #[test]
    fn the_queue_pops_highest_priority_then_oldest_and_refuses_when_full() {
        let q: &'static PriorityQueue<4> = Box::leak(Box::new(PriorityQueue::new()));
        assert!(q.push(entry(0, 1), now).is_ok());
        assert!(q.push(entry(1, 5), now).is_ok());
        assert!(q.push(entry(2, 5), now).is_ok());
        assert!(q.push(entry(3, 9), now).is_ok());
        // Full: the entry comes back, and nothing queued changed.
        let back = q.push(entry(3, 200), now).err().map(|e| e.txn.priority);
        assert_eq!(back, Some(200));
        let order: Vec<_> = core::iter::from_fn(|| q.pop(None, now))
            .map(|e| (e.device, e.txn.priority))
            .collect();
        assert_eq!(order, [(3, 9), (1, 5), (2, 5), (0, 1)]);
        // Usable again after.
        assert!(q.push(entry(0, 0), now).is_ok());
        assert_eq!(q.pop(None, now).map(|e| e.device), Some(0));
        let (push, pop) = q.longest();
        assert!(push > 0 && pop > 0);
    }

    #[test]
    fn equal_priorities_keep_arrival_order_across_freed_slots() {
        let q: &'static PriorityQueue<3> = Box::leak(Box::new(PriorityQueue::new()));
        for d in 0..3 {
            assert!(q.push(entry(d, 4), now).is_ok());
        }
        assert_eq!(q.pop(None, now).map(|e| e.device), Some(0));
        // Lands in slot 0, but is the youngest.
        assert!(q.push(entry(7, 4), now).is_ok());
        let order: Vec<_> = core::iter::from_fn(|| q.pop(None, now))
            .map(|e| e.device)
            .collect();
        assert_eq!(order, [1, 2, 7]);
        assert!(older(u32::MAX, 0));
        assert!(!older(0, u32::MAX));
    }

    #[test]
    fn a_cancel_drains_only_its_device() {
        let q: &'static PriorityQueue<4> = Box::leak(Box::new(PriorityQueue::new()));
        for (d, p) in [(0, 1), (1, 2), (0, 3), (1, 4)] {
            assert!(q.push(entry(d, p), now).is_ok());
        }
        let mut source = Source::Shared(q);
        let dropped: Vec<_> = core::iter::from_fn(|| source.next_of(0, now))
            .map(|e| e.txn.priority)
            .collect();
        assert_eq!(dropped, [3, 1]);
        let left: Vec<_> = core::iter::from_fn(|| source.next(now))
            .map(|e| (e.device, e.txn.priority))
            .collect();
        assert_eq!(left, [(1, 4), (1, 2)]);
    }

    fn result(priority: u8) -> SpiResult {
        SpiResult {
            bytes: [priority; MAX],
            len: 1,
            outcome: Outcome::Done,
            waited: 0,
            took: 0,
            priority,
        }
    }

    #[test]
    fn submit_reserves_a_result_slot_so_no_result_is_lost() {
        let memory: &'static mut MultiMemory<8, 3, 1> = Box::leak(Box::new(MultiMemory::new()));
        let MultiMemory { queue, devices } = memory;
        let queue: &'static PriorityQueue<8> = queue;
        let [device] = devices;
        let (mut end, mut handle) = split_device(device, 0, Route::Shared(queue), kick, now);
        let mut t = handle.transaction(1);
        t.read(2);
        let before = KICKS.load(Ordering::Relaxed);
        // Three result slots hold two: two outstanding, the third refused.
        assert_eq!(handle.submit(t), Ok(()));
        assert_eq!(handle.submit(t), Ok(()));
        assert_eq!(handle.submit(t), Err(SubmitError::Outstanding));
        assert_eq!(handle.outstanding(), 2);
        assert!(KICKS.load(Ordering::Relaxed) >= before + 2);
        assert!(end.results.enqueue(result(1)).is_ok());
        assert_eq!(
            handle.take().map(|r| r.bytes().to_vec()),
            Some([1].to_vec())
        );
        assert_eq!(handle.outstanding(), 1);
        assert_eq!(handle.submit(t), Ok(()));
        assert_eq!(handle.take().map(|r| r.priority), None);
        // A malformed transaction reserves nothing.
        assert_eq!(handle.submit(Transaction::new(0)), Err(SubmitError::Empty));
        assert_eq!(handle.outstanding(), 2);
    }

    #[test]
    fn a_full_queue_refuses_and_releases_the_reservation() {
        let memory: &'static mut MultiMemory<2, 8, 2> = Box::leak(Box::new(MultiMemory::new()));
        let MultiMemory { queue, devices } = memory;
        let queue: &'static PriorityQueue<2> = queue;
        let [a, b] = devices;
        let (_ea, mut ha) = split_device(a, 0, Route::Shared(queue), kick, now);
        let (_eb, mut hb) = split_device(b, 1, Route::Shared(queue), kick, now);
        let mut t = ha.transaction(0);
        t.write(&[1]);
        assert_eq!(ha.submit(t), Ok(()));
        assert_eq!(hb.submit(t), Ok(()));
        assert_eq!(hb.submit(t), Err(SubmitError::Full));
        assert_eq!((ha.outstanding(), hb.outstanding()), (1, 1));
        assert_eq!(queue.pop(None, now).map(|e| e.device), Some(0));
        assert_eq!(hb.submit(t), Ok(()));
    }

    #[test]
    fn the_single_slot_holds_one_and_cancel_flags_and_kicks() {
        let memory: &'static mut SingleMemory<4> = Box::leak(Box::new(SingleMemory::new()));
        let SingleMemory { slot, device } = memory;
        let (producer, mut consumer) = slot.split();
        let (end, mut handle) = split_device(device, 0, Route::Single(producer), kick, now);
        let mut t = handle.transaction(9);
        t.transfer(&[5, 6]);
        assert_eq!(handle.submit(t), Ok(()));
        assert_eq!(handle.submit(t), Err(SubmitError::Full));
        assert_eq!(handle.outstanding(), 1);
        assert_eq!(
            consumer.dequeue().map(|e| e.txn.wire().to_vec()),
            Some([5, 6].to_vec())
        );
        let before = KICKS.load(Ordering::Relaxed);
        handle.cancel();
        assert!(end.state.cancel.load(Ordering::Acquire));
        assert!(KICKS.load(Ordering::Relaxed) > before);
        assert_eq!(handle.device(), 0);
    }

    struct Pin(bool);
    impl embedded_hal::digital::ErrorType for Pin {
        type Error = core::convert::Infallible;
    }
    impl OutputPin for Pin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0 = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0 = true;
            Ok(())
        }
    }
    impl StatefulOutputPin for Pin {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self.0)
        }
        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.0)
        }
    }

    #[test]
    fn chip_selects_are_numbered_from_zero() {
        let mut cs = (Pin(true), Pin(true), Pin(true), Pin(true));
        assert_eq!(<(Pin, Pin, Pin, Pin) as ChipSelects>::COUNT, 4);
        cs.select(2, true);
        assert_eq!(
            (0..5).map(|d| cs.is_selected(d)).collect::<Vec<_>>(),
            [Some(false), Some(false), Some(true), Some(false), None]
        );
        cs.select(2, false);
        cs.select(9, true);
        assert!((0..4).all(|d| cs.is_selected(d) == Some(false)));
    }
}
