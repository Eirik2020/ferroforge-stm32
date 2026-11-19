//! The SPI port: one FerroHAL [`Spi`] master shared by one or more devices,
//! each behind its own chip select, served from the one vector the SPI's
//! receive stream raises.
//!
//! The consumer owns its memory and lends it to the port, the pattern a
//! shared SPI bus is built on elsewhere (Linux's `spi_message`, Zephyr's
//! RTIO): the queues hold references to buffers, never bytes.
//!
//! - Each device is given buffers at boot, `&'static mut [u8]` of the
//!   firmware's lengths, up to the bus's capacity. Its [`SpiHandle`] keeps
//!   the free ones.
//! - [`SpiHandle::message`] takes the shortest free buffer that holds the
//!   length the consumer states, or refuses (D8). The parts of one
//!   chip-select assertion are written straight into it.
//! - [`Message::submit`] moves the reference into the device's own
//!   single-producer, single-consumer queue and pends the port's vector.
//! - The port's member ends a finished transfer, releases its chip select
//!   inside the interrupt, copies what came in over the lent buffer, and
//!   gives the buffer back through the device's return queue with a wake.
//!   Then, if the bus is free, it starts the front entry of the device whose
//!   front entry has the highest priority, the earliest submitted among
//!   equals, switching the bus's settings when the device changes.
//! - [`SpiHandle::take`] gives the buffer back as a [`Taken`], read in place;
//!   dropping it frees the buffer.
//!
//! Order: one device's messages run in the order submitted; priority orders
//! devices against each other. No queue here turns interrupts off: each has
//! one producer and one consumer. The wake is `rtic-sync`'s `Signal`, which
//! keeps a short critical section of its own.
//!
//! What goes over the bus, a device's registers and framing, is the
//! firmware's: the port moves bytes and frames them with chip select (D6).
//! Nothing here panics.
//!
//! Requirements: PORT-201, PORT-202, PORT-203, PORT-204, PORT-205, PORT-206,
//! PORT-207, PORT-208, PORT-209

/// The SPI port group. One member, on the receive stream's vector, which
/// every handle also pends. A firmware selects it once per bus:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::spi_port, shared = [port = spi1])]
/// mod spi1 {
///     #[task(binds = DMA2_STREAM0, priority = 4)] fn on_spi_rx_dma;
/// }
/// ```
///
/// and in `init`: `spi_port::new(&mut rcc, wiring, (cs0, cs1), &configs,
/// tx_buf, rx_buf, memory, &mut [(0, buf_a), (1, buf_b)].into_iter(),
/// spi1::kick, now)`, then moves each handle from `handles.take(i)` into the
/// task it chooses.
#[ferroforge::group(
    slots = [instance: U, sck: Sck, miso: Miso, mosi: Mosi, tx_dma: TxD, rx_dma: RxD, cs: Cs],
    vectors = [on_spi_rx_dma = rx_dma],
    shared = [port: SpiPort<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [
        configs: &[Config] @ board,
        lent: &mut dyn Iterator<Item = (usize, &'static mut [u8])> @ software,
        now: fn() -> u32 @ platform,
    ],
    memory = [memory: Memory],
    handles = [handles: Handles],
    pends = [kick = on_spi_rx_dma],
    constructor = new(rcc, wiring, cs, configs, tx_buf, rx_buf, memory, lent, kick, now),
)]
pub mod spi_port {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    pub use ferrohal_stm32::{
        Service,
        spi::{Config, ConfigError, Error, Wiring},
    };
    use ferrohal_stm32::{
        rcc::Rcc,
        spi::{
            Buffers, Instance, MisoPin, MosiPin, Parts, SckPin, Spi, SpiRefused, SpiRxDma, SpiTxDma,
        },
        time::Hertz,
    };
    use heapless::{
        Vec, VecView,
        spsc::{Consumer, Producer, Queue},
    };
    use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

    /// The most parts of one message whose received bytes are kept.
    pub const PARTS: usize = 8;
    /// The most devices one port serves.
    pub const DEVICES: usize = 8;
    /// What a read part clocks out, as FerroHAL's `SpiBus::read` does.
    pub const FILLER: u8 = 0x00;

    /// Why a message was refused. Each refusal leaves the device as it was
    /// (D8); the buffer, if one was taken, is free again.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum SubmitError {
        /// No buffer of this device is that long, or the parts outgrew the one
        /// the message took.
        TooLong,
        /// Every buffer long enough is out: lent, or back and not yet taken.
        Outstanding,
        /// More than [`PARTS`] parts keep what they receive.
        Parts,
        /// Nothing to send.
        Empty,
        /// The device's queue was full. The constructor gives a device no more
        /// buffers than its queue holds, so this does not happen; it is a
        /// refusal, not a loss, if it does.
        Full,
    }

    /// How a message ended.
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

    /// One kept part's place in the message.
    #[derive(Clone, Copy, Default)]
    struct Part {
        at: u16,
        len: u16,
    }

    /// Whether ticket `a` was issued before ticket `b`, across a wrap.
    fn earlier(a: u32, b: u32) -> bool {
        (b.wrapping_sub(a) as i32) > 0
    }

    /// The chip-select pins of a port's devices, active low, numbered from 0:
    /// a tuple of one to [`DEVICES`] pins, each its own type.
    pub trait ChipSelects {
        /// How many devices.
        const COUNT: usize;
        /// Drives device `device`'s line: low when `selected`.
        fn select(&mut self, device: usize, selected: bool);
        /// The level the port drives on device `device`'s line, read from the
        /// output register: `Some(true)` when selected.
        fn is_selected(&mut self, device: usize) -> Option<bool>;
    }

    fn drive<P: OutputPin>(pin: &mut P, selected: bool) {
        let _ = if selected {
            pin.set_low()
        } else {
            pin.set_high()
        };
    }

    fn level<P: StatefulOutputPin>(pin: &mut P) -> Option<bool> {
        pin.is_set_low().ok()
    }

    macro_rules! chip_selects {
        ($count:literal; $($index:tt $pin:ident),+) => {
            impl<$($pin: StatefulOutputPin),+> ChipSelects for ($($pin,)+) {
                const COUNT: usize = $count;
                fn select(&mut self, device: usize, selected: bool) {
                    match device {
                        $($index => drive(&mut self.$index, selected),)+
                        _ => {}
                    }
                }
                fn is_selected(&mut self, device: usize) -> Option<bool> {
                    match device {
                        $($index => level(&mut self.$index),)+
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

    /// A message lent to the port: the buffer and how to read it back. Its
    /// size does not depend on the data's length.
    struct Lent {
        buf: &'static mut [u8],
        len: u16,
        keep: [Part; PARTS],
        kept: u8,
        priority: u8,
        /// The port-wide order of submission.
        ticket: u32,
        /// The firmware's clock at the submit.
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
    struct DeviceState {
        /// Set by the handle's `cancel`, cleared by the port when served.
        cancel: AtomicBool,
        /// The ticket the next submit would have drawn when `cancel` was set:
        /// the port drops only what was submitted before it.
        cancel_before: AtomicU32,
    }

    /// One device's memory: its free list, its two queues, its state and its
    /// wake. A queue of `N` slots holds `N - 1` entries, so the device takes at
    /// most `N - 1` buffers.
    pub struct DeviceMemory<const N: usize> {
        free: Vec<&'static mut [u8], N>,
        lent: Queue<Lent, N>,
        returned: Queue<Done, N>,
        state: DeviceState,
        wake: Signal<()>,
    }

    impl<const N: usize> DeviceMemory<N> {
        /// Empty memory, for a `static` or `init(local = [..])`.
        pub const fn new() -> Self {
            Self {
                free: Vec::new(),
                lent: Queue::new(),
                returned: Queue::new(),
                state: DeviceState {
                    cancel: AtomicBool::new(false),
                    cancel_before: AtomicU32::new(0),
                },
                wake: Signal::new(),
            }
        }
    }

    impl<const N: usize> Default for DeviceMemory<N> {
        fn default() -> Self {
            Self::new()
        }
    }

    /// The port's memory: `D` devices of `N` slots each, and the port-wide
    /// order of submission.
    pub struct Memory<const N: usize, const D: usize> {
        devices: [DeviceMemory<N>; D],
        tickets: AtomicU32,
    }

    impl<const N: usize, const D: usize> Memory<N, D> {
        /// Empty memory, for a `static` or `init(local = [..])`.
        pub const fn new() -> Self {
            Self {
                devices: [const { DeviceMemory::new() }; D],
                tickets: AtomicU32::new(0),
            }
        }
    }

    impl<const N: usize, const D: usize> Default for Memory<N, D> {
        fn default() -> Self {
            Self::new()
        }
    }

    /// The port's end of one device.
    struct DeviceEnd {
        lent: Consumer<'static, Lent>,
        returned: Producer<'static, Done>,
        wake: SignalWriter<'static, ()>,
        state: &'static DeviceState,
    }

    /// The consumer's end of one SPI device: the same type for every device
    /// and every port size, so `init` can give any handle to any task (D12).
    pub struct SpiHandle {
        free: &'static mut VecView<&'static mut [u8]>,
        lent: Producer<'static, Lent>,
        returned: Consumer<'static, Done>,
        state: &'static DeviceState,
        wake: SignalReader<'static, ()>,
        tickets: &'static AtomicU32,
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
        memory: &'static mut DeviceMemory<N>,
        tickets: &'static AtomicU32,
        device: u8,
        kick: fn(),
        now: fn() -> u32,
    ) -> (DeviceEnd, SpiHandle) {
        let DeviceMemory {
            free,
            lent,
            returned,
            state,
            wake,
        } = memory;
        let state: &'static DeviceState = state;
        let wake: &'static Signal<()> = wake;
        let room = lent.capacity();
        let (lent_in, lent_out) = lent.split();
        let (back_in, back_out) = returned.split();
        let (writer, reader) = wake.split();
        (
            DeviceEnd {
                lent: lent_out,
                returned: back_in,
                wake: writer,
                state,
            },
            SpiHandle {
                free: free.as_mut_view(),
                lent: lent_in,
                returned: back_out,
                state,
                wake: reader,
                tickets,
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

    impl SpiHandle {
        /// Takes the shortest free buffer that holds `len` bytes, for a message
        /// at `priority` (higher goes first across devices). Refused with
        /// `TooLong` when no buffer of this device is that long, and with
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

        /// The oldest message the port gave back, in its buffer. It borrows the
        /// handle until dropped.
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

        /// Waits until the port gives a message back. Wakes coalesce: after a
        /// wake, take until `take` gives `None`.
        pub async fn wait(&mut self) {
            self.wake.wait().await;
        }

        /// Asks the port to drop this device's messages submitted before this
        /// call and to abort the one on the wire; each comes back `Aborted`,
        /// with chip select released. A message submitted after the call runs.
        pub fn cancel(&mut self) {
            let before = self.tickets.load(Ordering::Acquire);
            self.state.cancel_before.store(before, Ordering::Relaxed);
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

        /// The longest buffer given at boot: the longest message.
        pub fn longest(&self) -> usize {
            self.longest
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
        handle: &'h mut SpiHandle,
        buf: Option<&'static mut [u8]>,
        len: u16,
        keep: [Part; PARTS],
        kept: u8,
        priority: u8,
        refused: Option<SubmitError>,
    }

    impl Message<'_> {
        /// Appends bytes to send; what comes back during them is not kept.
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

        /// Whether nothing has been appended.
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
            // Both fit: `end` is at most u16::MAX.
            let (at, n, end) = (at as u16, n as u16, end as u16);
            if keep {
                let Some(part) = self.keep.get_mut(usize::from(self.kept)) else {
                    self.refused = Some(SubmitError::Parts);
                    return;
                };
                *part = Part { at, len: n };
                self.kept += 1;
            }
            self.len = end;
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
                ticket: self.handle.tickets.fetch_add(1, Ordering::AcqRel),
                submitted: (self.handle.now)(),
            };
            if let Err(back) = self.handle.lent.enqueue(lent) {
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
        handle: &'h mut SpiHandle,
        buf: Option<&'static mut [u8]>,
        len: u16,
        keep: [Part; PARTS],
        kept: u8,
        /// How it ended.
        pub outcome: Outcome,
        /// Clock ticks from the submit to the transfer's start, or to the drop
        /// of a message that never started.
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

        /// Whether the message had no bytes; never true for a submitted one.
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

    /// The port's counts, for a firmware's diagnostics.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct Health {
        /// Bus settings switches between devices.
        pub switches: u32,
        /// The longest settings switch, in clock ticks.
        pub longest_switch: u32,
        /// The longest run of the port's member, in clock ticks.
        pub longest_run: u32,
        /// Messages the port could not give back, or whose received bytes did
        /// not cover what was sent. Zero by construction; counted so a fault is
        /// not hidden.
        pub lost: u32,
    }

    /// The port's own state: FerroHAL's `Spi`, the chip selects, each device's
    /// settings and queues. The member's shared resource.
    pub struct SpiPort<W: Parts, Cs> {
        spi: Spi<W>,
        cs: Cs,
        configs: [Option<Config>; DEVICES],
        frequencies: [Option<Hertz>; DEVICES],
        ends: [Option<DeviceEnd>; DEVICES],
        running: Option<Running>,
        last: Option<u8>,
        now: fn() -> u32,
        health: Health,
    }

    impl<W: Parts, Cs: ChipSelects> SpiPort<W, Cs> {
        /// FerroHAL's driver, for its `health()` and `vectors()`.
        pub fn spi(&self) -> &Spi<W> {
            &self.spi
        }

        /// The port's counts.
        pub fn health(&self) -> Health {
            self.health
        }

        /// The SCK FerroHAL set for device `device`'s settings.
        pub fn frequency(&self, device: usize) -> Option<Hertz> {
            self.frequencies.get(device).copied().flatten()
        }

        /// Device `device`'s chip select, as the port drives it.
        pub fn is_selected(&mut self, device: usize) -> Option<bool> {
            self.cs.is_selected(device)
        }

        /// Whether a transfer is on the wire.
        pub fn is_busy(&self) -> bool {
            self.running.is_some() || self.spi.is_busy()
        }

        fn lose(&mut self) {
            self.health.lost = self.health.lost.saturating_add(1);
        }

        fn deliver(&mut self, device: u8, done: Done) {
            let Some(end) = self
                .ends
                .get_mut(usize::from(device))
                .and_then(Option::as_mut)
            else {
                self.lose();
                return;
            };
            let queued = end.returned.enqueue(done).is_ok();
            end.wake.write(());
            if !queued {
                self.lose();
            }
        }

        /// The running transfer ended with `outcome`: release, copy back,
        /// return the buffer.
        fn finish(&mut self, outcome: Result<usize, Error>, aborted: bool) {
            let Some(run) = self.running.take() else {
                return;
            };
            let now = (self.now)();
            self.cs.select(usize::from(run.device), false);
            let lent = run.lent;
            let outcome = match (aborted, outcome) {
                (true, _) => Outcome::Aborted,
                (false, Err(error)) => Outcome::Failed(error),
                (false, Ok(_)) => {
                    let len = usize::from(lent.len);
                    match (lent.buf.get_mut(..len), self.spi.received().get(..len)) {
                        (Some(dst), Some(src)) => {
                            dst.copy_from_slice(src);
                            Outcome::Done
                        }
                        _ => {
                            self.lose();
                            Outcome::Failed(Error::Empty)
                        }
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
                let Some(state) = self.ends.get(d).and_then(Option::as_ref).map(|e| e.state) else {
                    continue;
                };
                if !state.cancel.swap(false, Ordering::AcqRel) {
                    continue;
                }
                let before = state.cancel_before.load(Ordering::Relaxed);
                if self
                    .running
                    .as_ref()
                    .is_some_and(|r| usize::from(r.device) == d && earlier(r.lent.ticket, before))
                {
                    self.spi.abort();
                    self.finish(Err(Error::Busy), true);
                }
                // Bounded by the queue: each pass removes one entry.
                for _ in 0..u8::MAX {
                    let Some(end) = self.ends.get_mut(d).and_then(Option::as_mut) else {
                        break;
                    };
                    if !end
                        .lent
                        .peek()
                        .is_some_and(|front| earlier(front.ticket, before))
                    {
                        break;
                    }
                    let Some(lent) = end.lent.dequeue() else {
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
                // `pick` returns an index below `DEVICES`.
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
                        Ok(_) => {
                            self.last = Some(device);
                            self.health.switches = self.health.switches.saturating_add(1);
                            self.health.longest_switch = self.health.longest_switch.max(took);
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
                match self.spi.start(wire) {
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
    /// earliest submitted among equals. Looks, takes nothing.
    fn pick(ends: &[Option<DeviceEnd>]) -> Option<usize> {
        let mut best: Option<(usize, u8, u32)> = None;
        for (d, end) in ends.iter().enumerate() {
            let Some(front) = end.as_ref().and_then(|e| e.lent.peek()) else {
                continue;
            };
            let better = match best {
                None => true,
                Some((_, p, ticket)) => {
                    front.priority > p || (front.priority == p && earlier(front.ticket, ticket))
                }
            };
            if better {
                best = Some((d, front.priority, front.ticket));
            }
        }
        best.map(|(d, _, _)| d)
    }

    impl<W: Parts, Cs: ChipSelects> Service for SpiPort<W, Cs> {
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
            self.health.longest_run = self.health.longest_run.max(took);
        }
    }

    /// The handles, one per device, taken by number at boot.
    pub struct Handles {
        handles: [Option<SpiHandle>; DEVICES],
    }

    impl Handles {
        /// Device `device`'s handle, once.
        pub fn take(&mut self, device: usize) -> Option<SpiHandle> {
            self.handles.get_mut(device).and_then(Option::take)
        }
    }

    /// What [`new`] builds.
    pub struct Resources<W: Parts, Cs> {
        /// The member's shared resource.
        pub port: SpiPort<W, Cs>,
        /// The consumers': each a `#[local]` of whichever task `init` picks.
        pub handles: Handles,
    }

    /// Why [`new`] refused. Each refusal hands back what it was given that is
    /// not a reference.
    pub enum PortRefused<U, Sck, Miso, Mosi, TxD, RxD, Cs>
    where
        U: Instance,
    {
        /// There is not one config per chip select and per device's memory,
        /// none at all, or more than [`DEVICES`].
        Count {
            wiring: Wiring<U, Sck, Miso, Mosi, TxD, RxD>,
            cs: Cs,
            tx_buf: &'static mut [u8],
            rx_buf: &'static mut [u8],
        },
        /// FerroHAL refused the bus: its reason, with the wiring and buffers.
        Open {
            refused: SpiRefused<U, Sck, Miso, Mosi, TxD, RxD>,
            cs: Cs,
        },
        /// A device's settings: FerroHAL's reason, with the open driver and the
        /// pins, so the firmware can `free` it.
        Config {
            device: usize,
            error: ConfigError,
            spi: Spi<Wiring<U, Sck, Miso, Mosi, TxD, RxD>>,
            cs: Cs,
        },
        /// A lent buffer for a device that does not exist, longer than the
        /// bus's capacity, empty, or one more than the device's queue holds.
        Buffer {
            device: usize,
            len: usize,
            spi: Spi<Wiring<U, Sck, Miso, Mosi, TxD, RxD>>,
            cs: Cs,
        },
    }

    /// Builds the port: device `i` on chip select `cs.i` with `configs[i]`,
    /// each applied once here so a device FerroHAL cannot serve is refused at
    /// boot. `tx_buf` and `rx_buf` are FerroHAL's DMA buffers; the shorter sets
    /// the longest message. `lent` gives each consumer buffer to its device by
    /// number. `kick` is the selection's generated pend of the member's vector;
    /// `now` is the firmware's clock, for the messages' times.
    #[allow(
        clippy::type_complexity,
        clippy::too_many_arguments,
        clippy::result_large_err
    )]
    pub fn new<U, Sck, Miso, Mosi, TxD, RxD, Cs, const N: usize, const D: usize>(
        rcc: &mut Rcc,
        wiring: Wiring<U, Sck, Miso, Mosi, TxD, RxD>,
        mut cs: Cs,
        configs: &[Config],
        tx_buf: &'static mut [u8],
        rx_buf: &'static mut [u8],
        memory: &'static mut Memory<N, D>,
        lent: &mut dyn Iterator<Item = (usize, &'static mut [u8])>,
        kick: fn(),
        now: fn() -> u32,
    ) -> Result<
        Resources<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>,
        PortRefused<U, Sck, Miso, Mosi, TxD, RxD, Cs>,
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
        let first = configs.first().copied();
        let counted = configs.len() == Cs::COUNT && D == Cs::COUNT && Cs::COUNT <= DEVICES;
        let Some(first) = first.filter(|_| counted) else {
            return Err(PortRefused::Count {
                wiring,
                cs,
                tx_buf,
                rx_buf,
            });
        };
        for d in 0..Cs::COUNT {
            cs.select(d, false);
        }
        let mut spi = match Spi::new(
            rcc,
            wiring,
            &first,
            Buffers {
                tx: tx_buf,
                rx: rx_buf,
            },
        ) {
            Ok(spi) => spi,
            Err(refused) => return Err(PortRefused::Open { refused, cs }),
        };
        let mut configs_table = [None; DEVICES];
        let mut frequencies = [None; DEVICES];
        // In reverse, so device 0's settings are in force at the end.
        for (device, config) in configs.iter().enumerate().rev() {
            match spi.set_config(config) {
                Ok(sck) => {
                    if let (Some(entry), Some(f)) =
                        (configs_table.get_mut(device), frequencies.get_mut(device))
                    {
                        *entry = Some(*config);
                        *f = Some(sck);
                    }
                }
                Err(error) => {
                    return Err(PortRefused::Config {
                        device,
                        error,
                        spi,
                        cs,
                    });
                }
            }
        }
        let Memory { devices, tickets } = memory;
        let tickets: &'static AtomicU32 = tickets;
        let mut handles = [const { None }; DEVICES];
        let mut ends = [const { None }; DEVICES];
        for (device, memory) in devices.iter_mut().enumerate() {
            // `D` is at most `DEVICES`, checked above.
            let (end, handle) = split_device(memory, tickets, device as u8, kick, now);
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
                return Err(PortRefused::Buffer {
                    device,
                    len,
                    spi,
                    cs,
                });
            }
        }
        Ok(Resources {
            port: SpiPort {
                spi,
                cs,
                configs: configs_table,
                frequencies,
                ends,
                running: None,
                last: Some(0),
                now,
                health: Health::default(),
            },
            handles: Handles { handles },
        })
    }

    /// The receive stream's vector: a transfer's end, a submit, a cancel.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn on_spi_rx_dma(mut cx: on_spi_rx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }

    #[cfg(test)]
    mod tests {
        extern crate std;
        use super::*;
        use core::sync::atomic::AtomicUsize;
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

        fn tickets() -> &'static AtomicU32 {
            Box::leak(Box::new(AtomicU32::new(0)))
        }

        /// One device with buffers of `sizes`, the port's end and the handle.
        fn device<const N: usize>(
            d: u8,
            sizes: &[usize],
            tickets: &'static AtomicU32,
        ) -> (DeviceEnd, SpiHandle) {
            let memory: &'static mut DeviceMemory<N> = Box::leak(Box::new(DeviceMemory::new()));
            let (end, mut h) = split_device(memory, tickets, d, kick, now);
            for &len in sizes {
                assert!(h.free.push(buf(len)).is_ok());
                h.buffers += 1;
                h.longest = h.longest.max(len);
            }
            (end, h)
        }

        /// What the port does at the end of a loopback transfer: the bytes
        /// come back as sent.
        fn echo(end: &mut DeviceEnd, outcome: Outcome) {
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
            let (mut end, mut h) = device::<4>(0, &[16], tickets());
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
            let (mut end, mut h) = device::<4>(0, &[4], tickets());
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
            let (mut end, mut h) = device::<4>(0, &[4, 4, 4], tickets());
            assert_eq!((h.buffers(), h.outstanding(), h.longest()), (3, 0, 4));
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
            let (_end, mut h) = device::<4>(0, &[4, 64], tickets());
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
            let (_end, mut h) = device::<5>(0, &[64, 4, 256, 4], tickets());
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
            let (mut end, mut h) = device::<5>(0, &[4; 4], tickets());
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

        #[test]
        fn submits_draw_tickets_in_order_across_devices() {
            let shared = tickets();
            let (mut e0, mut h0) = device::<3>(0, &[4], shared);
            let (mut e1, mut h1) = device::<3>(1, &[4], shared);
            for h in [&mut h1, &mut h0] {
                let mut m = h.message(0, 1).unwrap();
                m.write(&[1]);
                assert_eq!(m.submit(), Ok(()));
            }
            let t1 = e1.lent.dequeue().unwrap().ticket;
            let t0 = e0.lent.dequeue().unwrap().ticket;
            assert!(earlier(t1, t0));
        }

        /// `pick` over devices whose front entries are `fronts` (priority,
        /// ticket).
        fn pick_of(fronts: &[Option<(u8, u32)>]) -> Option<usize> {
            let mut ends = StdVec::new();
            let mut handles = StdVec::new();
            for (d, front) in fronts.iter().enumerate() {
                let (end, mut h) = device::<3>(d as u8, &[4], tickets());
                if let Some((priority, ticket)) = *front {
                    let lent = Lent {
                        buf: h.remove(0).unwrap(),
                        len: 1,
                        keep: [Part::default(); PARTS],
                        kept: 0,
                        priority,
                        ticket,
                        submitted: 0,
                    };
                    assert!(h.lent.enqueue(lent).is_ok());
                }
                ends.push(Some(end));
                handles.push(h);
            }
            pick(&ends)
        }

        #[test]
        fn across_devices_priority_then_submission_then_device() {
            assert_eq!(pick_of(&[None, None]), None);
            assert_eq!(pick_of(&[Some((1, 0)), Some((9, 5))]), Some(1));
            assert_eq!(pick_of(&[Some((5, 7)), Some((5, 3)), None]), Some(1));
            assert_eq!(pick_of(&[Some((5, 3)), Some((5, 3))]), Some(0));
            // Across the counter's wrap: u32::MAX was issued before 2.
            assert_eq!(pick_of(&[Some((5, 2)), Some((5, u32::MAX))]), Some(1));
        }

        #[test]
        fn cancel_marks_what_came_before_and_kicks() {
            let shared = tickets();
            let (end, mut h) = device::<4>(2, &[4, 4], shared);
            let mut m = h.message(0, 1).unwrap();
            m.write(&[1]);
            assert_eq!(m.submit(), Ok(()));
            let before = KICKS.load(Ordering::Relaxed);
            h.cancel();
            assert!(KICKS.load(Ordering::Relaxed) > before);
            let mut m = h.message(0, 1).unwrap();
            m.write(&[2]);
            assert_eq!(m.submit(), Ok(()));
            assert!(end.state.cancel.load(Ordering::Acquire));
            let cut = end.state.cancel_before.load(Ordering::Relaxed);
            let mut queued = StdVec::new();
            let mut lent = end.lent;
            while let Some(l) = lent.dequeue() {
                queued.push(earlier(l.ticket, cut));
            }
            // The first was submitted before the cancel, the second after.
            assert_eq!(queued, [true, false]);
            assert_eq!(h.device(), 2);
        }

        #[test]
        fn a_submit_kicks_the_port() {
            let (_end, mut h) = device::<3>(0, &[4], tickets());
            let before = KICKS.load(Ordering::Relaxed);
            let mut m = h.message(0, 1).unwrap();
            m.write(&[1]);
            assert_eq!(m.submit(), Ok(()));
            assert!(KICKS.load(Ordering::Relaxed) > before);
        }

        #[test]
        fn the_entry_is_small_whatever_the_data_length() {
            // A reference, a length, the part ranges, the priority, a ticket and
            // a stamp.
            let entry = core::mem::size_of::<Lent>();
            assert!(entry <= 64, "entry {entry} bytes");
        }
    }
}
