//! PROTOTYPE: the timer lane frame port, variant F of
//! `lane-port-prototype-design.md` (port design record, section 3e, D28).
//! Not product code; not merged.
//!
//! The port is handed a frame per lane and plays it; on a two-way lane it
//! then receives, and it switches from sending to listening itself, in the
//! interrupt that sees the last lane end.
//!
//! - The port's task owns FerroHAL's waveform generator; the consumer never
//!   names or locks it. The consumer holds a [`FrameHandle`], as on the SPI
//!   port (D22).
//! - [`FrameHandle::exchange`] takes the consumer's lent buffers, or refuses
//!   with [`SubmitError::Outstanding`] while an exchange is out (D8): one
//!   exchange runs at a time. The frames are written straight into the
//!   lent buffer, one share per lane.
//! - [`Exchange::submit`] moves the buffers into the port's one-slot queue
//!   and pends the port. The port loads every lane that has a frame (the
//!   lane's own DMA buffer takes a copy: the one copy left on send) and
//!   plays them from one start.
//! - When the last lane ends, the port calls `listen` in that same
//!   interrupt if the exchange asks for a receive. When the receive ends, it
//!   copies each lane's samples into the lent receive buffer (the one copy
//!   left on receive), calls `speak`, and gives the buffers back with the
//!   outcome and the times through the handle.
//! - Any fault stops the generator, leaves every output on at its inactive
//!   level, and comes back as the outcome. A frame is played once.
//!
//! No critical section: the queues each have one producer and one
//! consumer. Nothing here panics.

use core::sync::atomic::{AtomicBool, Ordering};

use ferrohal_stm32::{
    Service,
    time::Hertz,
    timer::{Awg, ChannelTimer, Error, Lanes},
};
use heapless::spsc::{Consumer, Producer, Queue};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

/// The most lanes a port serves: FerroHAL's generator takes up to four a
/// timer, on two timers.
pub const MAX_LANES: usize = 8;

/// What the port needs of FerroHAL's generator: its own calls, unchanged.
/// [`Awg`] implements it; the host tests use a stand-in.
pub trait Generator {
    /// How many lanes.
    fn lanes(&self) -> usize;
    /// [`Awg::load`].
    fn load(&mut self, lane: usize, samples: &[u32]) -> Result<(), Error>;
    /// [`Awg::play`].
    fn play(&mut self) -> Result<(), Error>;
    /// [`Awg::listen`].
    fn listen(&mut self, rate: Hertz, count: usize) -> Result<Hertz, Error>;
    /// [`Awg::samples`].
    fn samples(&self, lane: usize) -> &[u16];
    /// [`Awg::speak`].
    fn speak(&mut self) -> Result<(), Error>;
    /// [`Awg::on_interrupt`].
    fn serve(&mut self) -> Option<Result<u8, Error>>;
    /// [`Awg::abort`].
    fn abort(&mut self);
    /// [`Awg::is_busy`].
    fn is_busy(&self) -> bool;
}

impl<A: ChannelTimer, B: ChannelTimer, LA: Lanes<A>, LB: Lanes<B>> Generator for Awg<A, B, LA, LB> {
    fn lanes(&self) -> usize {
        Awg::lanes(self)
    }
    fn load(&mut self, lane: usize, samples: &[u32]) -> Result<(), Error> {
        Awg::load(self, lane, samples)
    }
    fn play(&mut self) -> Result<(), Error> {
        Awg::play(self)
    }
    fn listen(&mut self, rate: Hertz, count: usize) -> Result<Hertz, Error> {
        Awg::listen(self, rate, count)
    }
    fn samples(&self, lane: usize) -> &[u16] {
        Awg::samples(self, lane)
    }
    fn speak(&mut self) -> Result<(), Error> {
        Awg::speak(self)
    }
    fn serve(&mut self) -> Option<Result<u8, Error>> {
        Awg::on_interrupt(self)
    }
    fn abort(&mut self) {
        Awg::abort(self)
    }
    fn is_busy(&self) -> bool {
        Awg::is_busy(self)
    }
}

/// Why an exchange was refused before it reached the port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubmitError {
    /// An exchange is out: submitted, or back and not yet taken.
    Outstanding,
    /// No lane has a frame.
    Empty,
    /// A frame or a receive longer than the lane's share of the lent
    /// buffer, or a lane the port does not have.
    TooLong,
    /// The port's queue refused the exchange. Not reachable with one
    /// exchange out; counted so a fault is not hidden.
    Full,
}

/// How an exchange ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Every frame was played; no receive was asked for.
    Sent,
    /// Every frame was played and every lane read its samples.
    Received,
    /// Cancelled by the consumer; the outputs are at their inactive level.
    Aborted,
    /// FerroHAL refused or failed; the generator was stopped, and the
    /// outputs are at their inactive level.
    Failed(Error),
}

/// When the exchange's steps happened, on the firmware's clock (`now`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Times {
    /// `submit` ran.
    pub submitted: u32,
    /// `play` returned.
    pub played: u32,
    /// The run that saw the last lane end began.
    pub frame_end: u32,
    /// `listen` returned.
    pub listening: u32,
    /// The run that saw the receive end began.
    pub received: u32,
    /// The receive rate the timer gave.
    pub rate: u32,
}

/// One exchange lent to the port.
struct Lent {
    frames: &'static mut [u32],
    received: &'static mut [u16],
    lens: [u16; MAX_LANES],
    receive: Option<(Hertz, u16)>,
    times: Times,
}

/// An exchange coming back.
struct Done {
    lent: Lent,
    outcome: Outcome,
}

/// The port's memory: a one-slot queue each way, the cancel flag and the
/// wake.
pub struct FrameMemory {
    lent: Queue<Lent, 2>,
    returned: Queue<Done, 2>,
    cancel: AtomicBool,
    wake: Signal<()>,
}

impl FrameMemory {
    pub const fn new() -> Self {
        Self {
            lent: Queue::new(),
            returned: Queue::new(),
            cancel: AtomicBool::new(false),
            wake: Signal::new(),
        }
    }
}

impl Default for FrameMemory {
    fn default() -> Self {
        Self::new()
    }
}

/// A span of cycle counts: least, most, sum and count.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Span {
    pub least: u32,
    pub most: u32,
    pub sum: u64,
    pub count: u32,
}

impl Span {
    pub fn add(&mut self, value: u32) {
        if self.count == 0 || value < self.least {
            self.least = value;
        }
        self.most = self.most.max(value);
        self.sum = self.sum.saturating_add(u64::from(value));
        self.count = self.count.saturating_add(1);
    }

    /// The mean, rounded down; 0 when empty.
    pub fn mean(&self) -> u32 {
        if self.count == 0 {
            0
        } else {
            u32::try_from(self.sum / u64::from(self.count)).unwrap_or(u32::MAX)
        }
    }
}

/// What the port has measured, on the firmware's clock.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// From the start of the run that saw the last lane end to `listen`
    /// having returned.
    pub turn: Span,
    /// `listen` alone.
    pub listen: Span,
    /// Loading every lane and playing.
    pub start: Span,
    /// One run of the port, whatever it did.
    pub run: Span,
    /// Samples copied into lane buffers by `load`, and out of them into
    /// lent buffers.
    pub copied_out: u32,
    pub copied_in: u32,
    /// Exchanges the return queue refused: zero unless something is broken.
    pub lost: u32,
}

/// The consumer's end of the port: the same type for every generator, so
/// `init` can give it to any task (D12).
pub struct FrameHandle {
    free: Option<(&'static mut [u32], &'static mut [u16])>,
    lent: Producer<'static, Lent>,
    returned: Consumer<'static, Done>,
    cancel: &'static AtomicBool,
    wake: SignalReader<'static, ()>,
    lanes: u8,
    kick: fn(),
    now: fn() -> u32,
    /// Refused by the port's queue; see [`SubmitError::Full`].
    full: u32,
}

impl FrameHandle {
    /// Takes the lent buffers for a new exchange. Refused while an exchange
    /// is out (D8).
    pub fn exchange(&mut self) -> Result<Exchange<'_>, SubmitError> {
        let (frames, received) = self.free.take().ok_or(SubmitError::Outstanding)?;
        Ok(Exchange {
            handle: self,
            buffers: Some((frames, received)),
            lens: [0; MAX_LANES],
            receive: None,
        })
    }

    /// The exchange the port gave back, with its buffers.
    pub fn take(&mut self) -> Option<Returned<'_>> {
        let done = self.returned.dequeue()?;
        Some(Returned {
            handle: self,
            lent: Some(done.lent),
            outcome: done.outcome,
        })
    }

    /// Waits until the port gives an exchange back.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// Asks the port to stop the exchange that runs; it comes back marked
    /// `Aborted`.
    pub fn cancel(&mut self) {
        self.cancel.store(true, Ordering::Release);
        (self.kick)();
    }

    /// How many lanes the port has.
    pub fn lanes(&self) -> usize {
        usize::from(self.lanes)
    }

    /// Exchanges the port's queue refused: zero unless something is broken.
    pub fn full(&self) -> u32 {
        self.full
    }
}

/// One exchange being built in the lent buffers. Dropping it, or a refused
/// submit, frees them.
pub struct Exchange<'h> {
    handle: &'h mut FrameHandle,
    buffers: Option<(&'static mut [u32], &'static mut [u16])>,
    lens: [u16; MAX_LANES],
    receive: Option<(Hertz, u16)>,
}

/// One lane's share of a lent buffer of `total`.
fn share(total: usize, lanes: u8) -> usize {
    total / usize::from(lanes.max(1))
}

impl Exchange<'_> {
    /// Lane `lane`'s frame, `len` samples, to be written in place. `None`
    /// for a lane the port does not have or a frame longer than the lane's
    /// share.
    pub fn frame(&mut self, lane: usize, len: usize) -> Option<&mut [u32]> {
        let lanes = self.handle.lanes;
        if lane >= usize::from(lanes) {
            return None;
        }
        let (frames, _) = self.buffers.as_mut()?;
        let stride = share(frames.len(), lanes);
        let slot = frames.get_mut(lane * stride..)?.get_mut(..stride)?;
        let frame = slot.get_mut(..len)?;
        *self.lens.get_mut(lane)? = u16::try_from(len).ok()?;
        Some(frame)
    }

    /// After the frames, every lane reads `count` samples at `rate`.
    pub fn receive(&mut self, rate: Hertz, count: usize) -> Result<(), SubmitError> {
        let lanes = self.handle.lanes;
        let room = self
            .buffers
            .as_ref()
            .map_or(0, |(_, received)| share(received.len(), lanes));
        if count == 0 || count > room {
            return Err(SubmitError::TooLong);
        }
        let count = u16::try_from(count).map_err(|_| SubmitError::TooLong)?;
        self.receive = Some((rate, count));
        Ok(())
    }

    /// Lends the buffers to the port and pends it.
    pub fn submit(mut self) -> Result<(), SubmitError> {
        if self.lens.iter().all(|&len| len == 0) {
            return Err(SubmitError::Empty);
        }
        let Some((frames, received)) = self.buffers.take() else {
            return Err(SubmitError::Outstanding);
        };
        let lent = Lent {
            frames,
            received,
            lens: self.lens,
            receive: self.receive,
            times: Times {
                submitted: (self.handle.now)(),
                ..Times::default()
            },
        };
        match self.handle.lent.enqueue(lent) {
            Ok(()) => {
                (self.handle.kick)();
                Ok(())
            }
            Err(lent) => {
                self.handle.full = self.handle.full.saturating_add(1);
                self.buffers = Some((lent.frames, lent.received));
                Err(SubmitError::Full)
            }
        }
    }
}

impl Drop for Exchange<'_> {
    fn drop(&mut self) {
        if let Some(buffers) = self.buffers.take() {
            self.handle.free = Some(buffers);
        }
    }
}

/// An exchange the port gave back. Dropping it frees the buffers.
pub struct Returned<'h> {
    handle: &'h mut FrameHandle,
    lent: Option<Lent>,
    outcome: Outcome,
}

impl Returned<'_> {
    pub fn outcome(&self) -> Outcome {
        self.outcome
    }

    pub fn times(&self) -> Times {
        self.lent
            .as_ref()
            .map_or(Times::default(), |lent| lent.times)
    }

    /// What lane `lane` read, as whole port samples; empty after a send
    /// only, or a failure.
    pub fn received(&self, lane: usize) -> &[u16] {
        let Some(lent) = self.lent.as_ref() else {
            return &[];
        };
        let count = match (self.outcome, lent.receive) {
            (Outcome::Received, Some((_, count))) => usize::from(count),
            _ => 0,
        };
        let stride = share(lent.received.len(), self.handle.lanes);
        lent.received
            .get(lane * stride..)
            .and_then(|slot| slot.get(..count))
            .unwrap_or(&[])
    }
}

impl Drop for Returned<'_> {
    fn drop(&mut self) {
        if let Some(lent) = self.lent.take() {
            self.handle.free = Some((lent.frames, lent.received));
        }
    }
}

/// The handle, taken once at boot.
pub struct FrameHandles {
    handle: Option<FrameHandle>,
}

impl FrameHandles {
    pub fn take(&mut self) -> Option<FrameHandle> {
        self.handle.take()
    }
}

/// Where an exchange is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    Sending,
    Listening,
}

/// The port's resource: the generator, the port's ends of the queues, and
/// the exchange that runs.
pub struct FramePort<G> {
    generator: G,
    lent: Consumer<'static, Lent>,
    returned: Producer<'static, Done>,
    wake: SignalWriter<'static, ()>,
    cancel: &'static AtomicBool,
    running: Option<Lent>,
    phase: Phase,
    now: fn() -> u32,
    stats: Stats,
}

impl<G: Generator> FramePort<G> {
    /// The generator, for the bench's own reads.
    pub fn generator(&self) -> &G {
        &self.generator
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    pub fn clear_stats(&mut self) {
        self.stats = Stats::default();
    }

    /// Whether an exchange runs.
    pub fn is_busy(&self) -> bool {
        self.phase != Phase::Idle
    }

    /// Gives the running exchange back with `outcome`.
    fn finish(&mut self, outcome: Outcome) {
        self.phase = Phase::Idle;
        if let Some(lent) = self.running.take() {
            if self.returned.enqueue(Done { lent, outcome }).is_err() {
                self.stats.lost = self.stats.lost.saturating_add(1);
            }
            self.wake.write(());
        }
    }

    /// Stops the generator after a fault or a cancel: every output at its
    /// inactive level and on again, since an abort while listening leaves
    /// them released.
    fn stop(&mut self, outcome: Outcome) {
        self.generator.abort();
        let _ = self.generator.speak();
        self.finish(outcome);
    }

    /// Loads every lane that has a frame and plays them from one start.
    fn start(&mut self, lent: Lent, entered: u32) {
        self.running = Some(lent);
        let lanes = self.generator.lanes().min(MAX_LANES);
        let mut failed = None;
        if let Some(lent) = self.running.as_ref() {
            let stride = share(lent.frames.len(), u8::try_from(lanes).unwrap_or(1));
            for lane in 0..lanes {
                let len = usize::from(lent.lens.get(lane).copied().unwrap_or(0));
                if len == 0 {
                    continue;
                }
                let frame = lent
                    .frames
                    .get(lane * stride..)
                    .and_then(|slot| slot.get(..len));
                let Some(frame) = frame else {
                    failed = Some(Error::TooLarge);
                    break;
                };
                if let Err(error) = self.generator.load(lane, frame) {
                    failed = Some(error);
                    break;
                }
                self.stats.copied_out = self
                    .stats
                    .copied_out
                    .saturating_add(u32::try_from(len).unwrap_or(u32::MAX));
            }
        }
        if let Some(error) = failed {
            self.stop(Outcome::Failed(error));
            return;
        }
        if let Err(error) = self.generator.play() {
            self.stop(Outcome::Failed(error));
            return;
        }
        let played = (self.now)();
        self.stats.start.add(played.wrapping_sub(entered));
        if let Some(lent) = self.running.as_mut() {
            lent.times.played = played;
        }
        self.phase = Phase::Sending;
    }

    /// The last lane ended its frame: listen in this same run, or give the
    /// exchange back.
    fn sent(&mut self, entered: u32) {
        let receive = self.running.as_mut().and_then(|lent| {
            lent.times.frame_end = entered;
            lent.receive
        });
        let Some((rate, count)) = receive else {
            self.finish(Outcome::Sent);
            return;
        };
        let before = (self.now)();
        match self.generator.listen(rate, usize::from(count)) {
            Ok(got) => {
                let after = (self.now)();
                self.stats.turn.add(after.wrapping_sub(entered));
                self.stats.listen.add(after.wrapping_sub(before));
                if let Some(lent) = self.running.as_mut() {
                    lent.times.listening = after;
                    lent.times.rate = got.0;
                }
                self.phase = Phase::Listening;
            }
            Err(error) => self.stop(Outcome::Failed(error)),
        }
    }

    /// Every lane has read its samples: copy them out and speak again.
    fn received(&mut self, entered: u32) {
        let lanes = self.generator.lanes().min(MAX_LANES);
        let mut copied = 0usize;
        if let Some(lent) = self.running.as_mut() {
            lent.times.received = entered;
            let count = lent.receive.map_or(0, |(_, count)| usize::from(count));
            let stride = share(lent.received.len(), u8::try_from(lanes).unwrap_or(1));
            for lane in 0..lanes {
                let read = self.generator.samples(lane);
                let to = lent
                    .received
                    .get_mut(lane * stride..)
                    .and_then(|slot| slot.get_mut(..count));
                if let (Some(to), Some(read)) = (to, read.get(..count)) {
                    to.copy_from_slice(read);
                    copied += count;
                }
            }
        }
        self.stats.copied_in = self
            .stats
            .copied_in
            .saturating_add(u32::try_from(copied).unwrap_or(u32::MAX));
        match self.generator.speak() {
            Ok(()) => self.finish(Outcome::Received),
            Err(error) => self.stop(Outcome::Failed(error)),
        }
    }
}

impl<G: Generator> Service for FramePort<G> {
    type Event = ();

    /// A lane's stream ended, the consumer submitted, or it cancelled.
    fn on_interrupt(&mut self) {
        let entered = (self.now)();
        if self.phase != Phase::Idle {
            match self.generator.serve() {
                Some(Err(error)) => self.stop(Outcome::Failed(error)),
                Some(Ok(_)) if !self.generator.is_busy() => match self.phase {
                    Phase::Sending => self.sent(entered),
                    Phase::Listening => self.received(entered),
                    Phase::Idle => {}
                },
                _ => {}
            }
        }
        if self.cancel.swap(false, Ordering::AcqRel) {
            if self.phase != Phase::Idle {
                self.stop(Outcome::Aborted);
            }
            if let Some(lent) = self.lent.dequeue() {
                self.running = Some(lent);
                self.finish(Outcome::Aborted);
            }
        }
        if self.phase == Phase::Idle
            && self.running.is_none()
            && let Some(lent) = self.lent.dequeue()
        {
            self.start(lent, entered);
        }
        let ran = (self.now)().wrapping_sub(entered);
        self.stats.run.add(ran);
    }
}

/// What the constructor builds.
pub struct FrameResources<G> {
    /// The member's shared resource.
    pub port: FramePort<G>,
    /// The consumer's, a `#[local]` of whichever task `init` picks.
    pub handles: FrameHandles,
}

/// Builds the port around the generator the firmware made in `init`
/// (lanes, timers, polarity and outputs are FerroHAL's), with the
/// consumer's lent buffers: `frames` and `received`, each split evenly
/// between the lanes. Infallible: a frame or receive too long for its share
/// is refused when the exchange is built.
pub fn new_frame<G: Generator>(
    generator: G,
    memory: &'static mut FrameMemory,
    frames: &'static mut [u32],
    received: &'static mut [u16],
    kick: fn(),
    now: fn() -> u32,
) -> FrameResources<G> {
    let FrameMemory {
        lent,
        returned,
        cancel,
        wake,
    } = memory;
    let cancel: &'static AtomicBool = cancel;
    let (lent_in, lent_out) = lent.split();
    let (back_in, back_out) = returned.split();
    let (writer, reader) = wake.split();
    let lanes = u8::try_from(generator.lanes().min(MAX_LANES)).unwrap_or(1);
    FrameResources {
        port: FramePort {
            generator,
            lent: lent_out,
            returned: back_in,
            wake: writer,
            cancel,
            running: None,
            phase: Phase::Idle,
            now,
            stats: Stats::default(),
        },
        handles: FrameHandles {
            handle: Some(FrameHandle {
                free: Some((frames, received)),
                lent: lent_in,
                returned: back_out,
                cancel,
                wake: reader,
                lanes,
                kick,
                now,
                full: 0,
            }),
        },
    }
}

/// F, the frame port. One member, declared once per lane's stream vector,
/// also pended by the handle. In `init`:
/// `lanes_frame::new_frame(generator, memory, frames, received,
/// <selection>::kick, now)`.
#[ferroforge::group(
    slots = [generator: G],
    vectors = [f_on_lane_dma = [generator]],
    shared = [port: FramePort<G>],
    config = [
        frames: &'static mut [u32] @ software,
        received: &'static mut [u16] @ software,
        now: fn() -> u32 @ platform,
    ],
    memory = [memory: FrameMemory],
    handles = [handles: FrameHandles],
    pends = [kick = f_on_lane_dma],
    constructor = new_frame(generator, memory, frames, received, kick, now),
)]
pub mod lane_frame_port {
    #[allow(unused_imports)]
    use super::{FrameHandles, FrameMemory, FramePort, new_frame};
    pub use ferrohal_stm32::Service;

    /// A lane's stream vector: a lane's end, a submit, a cancel.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn f_on_lane_dma(mut cx: f_on_lane_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::{AtomicU32, AtomicUsize};
    use std::{boxed::Box, string::ToString, vec, vec::Vec};

    static KICKS: AtomicUsize = AtomicUsize::new(0);
    fn kick() {
        KICKS.fetch_add(1, Ordering::Relaxed);
    }
    static CLOCK: AtomicU32 = AtomicU32::new(0);
    fn now() -> u32 {
        CLOCK.fetch_add(1, Ordering::Relaxed)
    }

    /// A stand-in generator: records the calls, and ends a play or a
    /// listen when the test says so.
    #[derive(Default)]
    struct Fake {
        lanes: usize,
        calls: Vec<&'static str>,
        loaded: Vec<(usize, Vec<u32>)>,
        busy: bool,
        listening: bool,
        outputs_off: bool,
        /// What the next `serve` returns.
        next: Option<Result<u8, Error>>,
        refuse_load: Option<Error>,
        read: Vec<u16>,
    }

    impl Generator for Fake {
        fn lanes(&self) -> usize {
            self.lanes
        }
        fn load(&mut self, lane: usize, samples: &[u32]) -> Result<(), Error> {
            self.calls.push("load");
            if let Some(error) = self.refuse_load {
                return Err(error);
            }
            self.loaded.push((lane, samples.to_vec()));
            Ok(())
        }
        fn play(&mut self) -> Result<(), Error> {
            self.calls.push("play");
            self.busy = true;
            Ok(())
        }
        fn listen(&mut self, rate: Hertz, count: usize) -> Result<Hertz, Error> {
            self.calls.push("listen");
            if self.busy || self.listening {
                return Err(Error::Busy);
            }
            self.listening = true;
            self.outputs_off = true;
            self.busy = true;
            self.read = (0..count as u16).collect();
            Ok(rate)
        }
        fn samples(&self, _lane: usize) -> &[u16] {
            if self.busy { &[] } else { &self.read }
        }
        fn speak(&mut self) -> Result<(), Error> {
            self.calls.push("speak");
            if self.busy {
                return Err(Error::Busy);
            }
            self.listening = false;
            self.outputs_off = false;
            Ok(())
        }
        fn serve(&mut self) -> Option<Result<u8, Error>> {
            let next = self.next.take();
            if let Some(Ok(_)) = next {
                self.busy = false;
            }
            if let Some(Err(_)) = next {
                self.abort();
            }
            next
        }
        fn abort(&mut self) {
            self.calls.push("abort");
            self.busy = false;
            if self.listening {
                // As FerroHAL's: the outputs stay released.
                self.listening = false;
            }
        }
        fn is_busy(&self) -> bool {
            self.busy
        }
    }

    fn port(lanes: usize) -> (FramePort<Fake>, FrameHandle) {
        let memory: &'static mut FrameMemory = Box::leak(Box::new(FrameMemory::new()));
        let frames: &'static mut [u32] = Box::leak(vec![0u32; 8 * lanes].into_boxed_slice());
        let received: &'static mut [u16] = Box::leak(vec![0u16; 16 * lanes].into_boxed_slice());
        let fake = Fake {
            lanes,
            ..Fake::default()
        };
        let mut built = new_frame(fake, memory, frames, received, kick, now);
        let handle = built.handles.take().unwrap();
        assert!(built.handles.take().is_none());
        (built.port, handle)
    }

    fn submit(h: &mut FrameHandle, frames: &[&[u32]], receive: Option<usize>) {
        let mut x = h.exchange().unwrap();
        for (lane, frame) in frames.iter().enumerate() {
            x.frame(lane, frame.len()).unwrap().copy_from_slice(frame);
        }
        if let Some(count) = receive {
            x.receive(Hertz::mhz(2), count).unwrap();
        }
        x.submit().unwrap();
    }

    #[test]
    fn a_send_only_exchange_plays_once_and_comes_back_sent() {
        let (mut p, mut h) = port(2);
        submit(&mut h, &[&[1, 2, 3], &[4, 5]], None);
        p.on_interrupt();
        assert_eq!(p.generator.calls, ["load", "load", "play"]);
        assert_eq!(p.generator.loaded, [(0, vec![1, 2, 3]), (1, vec![4, 5])]);
        assert!(h.take().is_none());
        // A run with no lane ended changes nothing.
        p.on_interrupt();
        assert!(h.take().is_none());
        p.generator.next = Some(Ok(0b11));
        p.on_interrupt();
        let r = h.take().unwrap();
        assert_eq!(r.outcome(), Outcome::Sent);
        assert!(r.received(0).is_empty());
        drop(r);
        // Played exactly once: a further run with nothing queued plays nothing.
        p.on_interrupt();
        assert_eq!(
            p.generator.calls.iter().filter(|c| **c == "play").count(),
            1
        );
        assert_eq!(p.stats().copied_out, 5);
        assert!(h.exchange().is_ok());
    }

    #[test]
    fn the_port_listens_in_the_run_that_sees_the_last_lane_end() {
        let (mut p, mut h) = port(1);
        submit(&mut h, &[&[7, 8]], Some(5));
        p.on_interrupt();
        p.generator.next = Some(Ok(0b1));
        p.on_interrupt();
        // Listening started in the same run as the end, with no return yet.
        assert_eq!(p.generator.calls, ["load", "play", "listen"]);
        assert!(p.generator.listening);
        assert!(h.take().is_none());
        assert_eq!(p.stats().turn.count, 1);
        p.generator.next = Some(Ok(0b1));
        p.on_interrupt();
        assert_eq!(p.generator.calls.last(), Some(&"speak"));
        assert!(!p.generator.outputs_off);
        let r = h.take().unwrap();
        assert_eq!(r.outcome(), Outcome::Received);
        assert_eq!(r.received(0), &[0, 1, 2, 3, 4]);
        let t = r.times();
        assert!(t.submitted < t.played && t.played < t.frame_end);
        assert!(t.frame_end < t.listening && t.listening < t.received);
        assert_eq!(t.rate, 2_000_000);
        drop(r);
        assert_eq!(p.stats().copied_in, 5);
    }

    #[test]
    fn a_second_exchange_is_refused_while_one_is_out() {
        let (mut p, mut h) = port(1);
        submit(&mut h, &[&[1]], None);
        assert_eq!(h.exchange().err(), Some(SubmitError::Outstanding));
        p.on_interrupt();
        p.generator.next = Some(Ok(1));
        p.on_interrupt();
        // Back but not taken: still out.
        assert_eq!(h.exchange().err(), Some(SubmitError::Outstanding));
        drop(h.take());
        assert!(h.exchange().is_ok());
    }

    #[test]
    fn an_exchange_is_checked_before_it_is_lent() {
        let (_p, mut h) = port(2);
        let mut x = h.exchange().unwrap();
        // Each lane's share is 8 samples of frame and 16 of receive.
        assert!(x.frame(0, 9).is_none());
        assert!(x.frame(2, 1).is_none());
        assert_eq!(x.receive(Hertz::mhz(1), 17), Err(SubmitError::TooLong));
        assert_eq!(x.receive(Hertz::mhz(1), 0), Err(SubmitError::TooLong));
        assert!(x.frame(1, 8).is_some());
        drop(x);
        let x = h.exchange().unwrap();
        assert_eq!(x.submit(), Err(SubmitError::Empty));
        // The refused exchange freed the buffers.
        assert!(h.exchange().is_ok());
    }

    #[test]
    fn a_refused_load_stops_the_generator_and_comes_back_failed() {
        let (mut p, mut h) = port(1);
        p.generator.refuse_load = Some(Error::TooLarge);
        submit(&mut h, &[&[1, 2]], Some(4));
        p.on_interrupt();
        assert_eq!(p.generator.calls, ["load", "abort", "speak"]);
        let r = h.take().unwrap();
        assert_eq!(r.outcome(), Outcome::Failed(Error::TooLarge));
    }

    #[test]
    fn a_cancel_mid_receive_leaves_the_outputs_on_and_comes_back_aborted() {
        let (mut p, mut h) = port(1);
        submit(&mut h, &[&[1, 2]], Some(4));
        p.on_interrupt();
        p.generator.next = Some(Ok(1));
        p.on_interrupt();
        assert!(p.generator.listening);
        h.cancel();
        p.on_interrupt();
        assert!(!p.generator.outputs_off);
        assert!(!p.generator.busy);
        let r = h.take().unwrap();
        assert_eq!(r.outcome(), Outcome::Aborted);
        assert!(r.received(0).is_empty());
        drop(r);
        // The next exchange runs.
        submit(&mut h, &[&[3]], None);
        p.on_interrupt();
        p.generator.next = Some(Ok(1));
        p.on_interrupt();
        assert_eq!(h.take().unwrap().outcome(), Outcome::Sent);
    }

    #[test]
    fn a_lane_fault_mid_frame_comes_back_failed() {
        let (mut p, mut h) = port(1);
        submit(&mut h, &[&[1, 2]], Some(4));
        p.on_interrupt();
        p.generator.next = Some(Err(Error::Dma));
        p.on_interrupt();
        assert!(!p.generator.calls.contains(&"listen"));
        assert_eq!(h.take().unwrap().outcome(), Outcome::Failed(Error::Dma));
    }

    #[test]
    fn lane_frame_port_states_its_contract() {
        let found = crate::tests::groups(include_str!("lanes_frame.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in lanes_frame.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "lane_frame_port");
        let contract = &group.arguments.contract;
        let [vector] = contract.vectors.as_slice() else {
            panic!("one vector role");
        };
        assert_eq!(vector.member, "f_on_lane_dma");
        assert!(vector.is_repeated());
        let pends: Vec<_> = contract
            .pends
            .iter()
            .map(|p| (p.name.to_string(), p.member.to_string()))
            .collect();
        assert_eq!(pends, [("kick".to_string(), "f_on_lane_dma".to_string())]);
    }

    /// The constructor's type, for the type checks below.
    type Build<G> = fn(
        G,
        &'static mut FrameMemory,
        &'static mut [u32],
        &'static mut [u16],
        fn(),
        fn() -> u32,
    ) -> FrameResources<G>;

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn the_frame_port_builds_on_f401_tim1_lanes() {
        use ferrohal_stm32::{
            chip::TIM1,
            dma::Stream,
            gpio::{Alternate, Pin},
            timer::{CompareLane, ComplementaryLaneWiring, LaneWiring},
        };
        // TIM1 CH4 on PA11, CH2 on PA9, CH1N on PB13 and CH3N on PB1 (AF1,
        // DS10086 table 9), over DMA2 streams 4, 2, 1 and 6, channel 6
        // (RM0368 table 29).
        type L0 =
            CompareLane<TIM1, 4, LaneWiring<TIM1, 4, Pin<'A', 11, Alternate<1>>, Stream<2, 4>>>;
        type L1 =
            CompareLane<TIM1, 2, LaneWiring<TIM1, 2, Pin<'A', 9, Alternate<1>>, Stream<2, 2>>>;
        type L2 = CompareLane<
            TIM1,
            1,
            ComplementaryLaneWiring<TIM1, 1, Pin<'B', 13, Alternate<1>>, Stream<2, 1>>,
        >;
        type L3 = CompareLane<
            TIM1,
            3,
            ComplementaryLaneWiring<TIM1, 3, Pin<'B', 1, Alternate<1>>, Stream<2, 6>>,
        >;
        type One = Awg<TIM1, TIM1, (L0,), ()>;
        type Two = Awg<TIM1, TIM1, (L0, L1), ()>;
        type Four = Awg<TIM1, TIM1, (L0, L1, L2, L3), ()>;
        let one: Build<One> = new_frame;
        let two: Build<Two> = new_frame;
        let four: Build<Four> = new_frame;
        let _ = (one, two, four);
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn the_frame_port_builds_on_h753_tim1_lanes() {
        use ferrohal_stm32::{
            chip::TIM1,
            dma::Stream,
            gpio::{Alternate, Pin},
            timer::{CompareLane, ComplementaryLaneWiring, LaneWiring},
        };
        // TIM1 CH2 on PE11 and CH3N on PE12 (AF1, DS12117 table 10), over
        // DMA2 streams 0 and 1 through DMAMUX1 (RM0433 section 17.3).
        type L0 =
            CompareLane<TIM1, 2, LaneWiring<TIM1, 2, Pin<'E', 11, Alternate<1>>, Stream<2, 0>>>;
        type L1 = CompareLane<
            TIM1,
            3,
            ComplementaryLaneWiring<TIM1, 3, Pin<'E', 12, Alternate<1>>, Stream<2, 1>>,
        >;
        let two: Build<Awg<TIM1, TIM1, (L0, L1), ()>> = new_frame;
        let _ = two;
    }
}
