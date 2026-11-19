//! The lane frame port (D28, D30 to D33; `docs/design/lane-port.md`): a
//! frame of samples per lane is played from lent memory and, on two-way
//! lanes, the port then receives into lent memory, switching from sending to
//! listening itself, in the interrupt that sees the last lane end.
//!
//! - The port's task owns FerroHAL's waveform generator; the consumer never
//!   names or locks it. The consumer holds a [`FrameHandle`], as on the SPI
//!   port (D22).
//! - [`FrameHandle::exchange`] opens an exchange, or refuses with
//!   [`SubmitError::Outstanding`] while one is out (D8): one exchange runs at
//!   a time. The frames are written straight into lent memory, one share
//!   per lane, split once at construction.
//! - [`Exchange::submit`] checks and closes every frame there, on the
//!   consumer's side ([`Frame::new`]: no top-to-zero step, the closing 0
//!   written), moves the shares into the one exchange, which lives in the
//!   port's memory, passes a reference to it through the port's one-slot
//!   queue (the exchange itself is never copied, D33) and pends the port's
//!   start member. The port hands each frame to its lane, which plays it in
//!   place: nothing is copied on send.
//! - When the last lane ends, the port calls `listen` in that same
//!   interrupt if the exchange asks for a receive. The lanes read into the
//!   lent receive shares themselves; with a shared reader, only the reader
//!   lane's share is written, and [`Returned::received`] gives each lane its
//!   reader's port words. Nothing is copied on receive. The port calls
//!   `speak`, and gives the shares back with the outcome and the times.
//! - Any fault stops the generator, leaves every output on at its inactive
//!   level, and comes back as the outcome. A frame is played once.
//!
//! - The port keeps statistics on the firmware's clock (`now`): the turn,
//!   the listen, the start and its parts, the runs. They are the contract's
//!   longest runs, per chip, in `docs/design/lane-port.md`.
//!
//! No critical section: the queues each have one producer and one
//! consumer. Nothing here panics.
//!
//! Requirements: PORT-001, PORT-002, PORT-501, PORT-502, PORT-503, PORT-504, PORT-505, PORT-506, PORT-507, PORT-508, PORT-509

use core::sync::atomic::{AtomicBool, Ordering};

use ferrohal_stm32::{
    Service,
    resource::Refused,
    time::Hertz,
    timer::{Awg, ChannelTimer, Error, Frame, FrameLimits, Lanes},
};
use heapless::spsc::{Consumer, Producer, Queue};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

/// The most lanes a port serves: FerroHAL's generator takes up to four a
/// timer, on two timers.
pub const MAX_LANES: usize = 8;

/// A refused lend of a frame, or of samples memory, with it handed back.
pub type FrameRefused = (Error, Frame);
pub type SamplesRefused = (Error, &'static mut [u16]);

/// What the port needs of FerroHAL's generator: its own calls, unchanged.
/// [`Awg`] implements it; the host tests use a stand-in.
pub trait Generator {
    /// How many lanes.
    fn lanes(&self) -> usize;
    /// [`Awg::frame_limits`].
    fn limits(&self, lane: usize) -> Option<FrameLimits>;
    /// [`Awg::load`].
    fn load(&mut self, lane: usize, frame: Frame) -> Result<(), FrameRefused>;
    /// [`Awg::take_frame`].
    fn take_frame(&mut self, lane: usize) -> Option<Frame>;
    /// [`Awg::lend_samples`].
    fn lend(&mut self, lane: usize, samples: &'static mut [u16]) -> Result<(), SamplesRefused>;
    /// [`Awg::take_samples`].
    fn take_samples(&mut self, lane: usize) -> Option<&'static mut [u16]>;
    /// [`Awg::reader_of`].
    fn reader_of(&self, lane: usize) -> usize;
    /// [`Awg::play`].
    fn play(&mut self) -> Result<(), Error>;
    /// [`Awg::prepare_listen`], unless [`Awg::prepared`] already matches:
    /// the rate the read will run at. Off the turn-around.
    fn prepare(&mut self, rate: Hertz, count: usize) -> Result<Hertz, Error>;
    /// [`Awg::listen`]: the prepared read, and nothing else.
    fn listen(&mut self) -> Result<(), Error>;
    /// [`Awg::speak`].
    fn speak(&mut self) -> Result<(), Error>;
    /// [`Awg::on_interrupt`].
    fn serve(&mut self) -> Option<Result<u8, Error>>;
    /// [`Awg::listen_on_end`]: listen inside
    /// the `serve` that sees the frame end. Whether it will.
    fn listen_on_end(&mut self) -> bool {
        false
    }
    /// [`Awg::is_listening`].
    fn is_listening(&self) -> bool {
        false
    }
    /// [`Awg::abort`].
    fn abort(&mut self);
    /// [`Awg::is_busy`].
    fn is_busy(&self) -> bool;
}

impl<A: ChannelTimer, B: ChannelTimer, LA: Lanes<A>, LB: Lanes<B>> Generator for Awg<A, B, LA, LB> {
    fn lanes(&self) -> usize {
        Awg::lanes(self)
    }
    fn limits(&self, lane: usize) -> Option<FrameLimits> {
        Awg::frame_limits(self, lane)
    }
    fn load(&mut self, lane: usize, frame: Frame) -> Result<(), FrameRefused> {
        Awg::load(self, lane, frame).map_err(Refused::into_inner)
    }
    fn take_frame(&mut self, lane: usize) -> Option<Frame> {
        Awg::take_frame(self, lane)
    }
    fn lend(&mut self, lane: usize, samples: &'static mut [u16]) -> Result<(), SamplesRefused> {
        Awg::lend_samples(self, lane, samples).map_err(Refused::into_inner)
    }
    fn take_samples(&mut self, lane: usize) -> Option<&'static mut [u16]> {
        Awg::take_samples(self, lane)
    }
    fn reader_of(&self, lane: usize) -> usize {
        Awg::reader_of(self, lane)
    }
    fn play(&mut self) -> Result<(), Error> {
        Awg::play(self)
    }
    fn prepare(&mut self, rate: Hertz, count: usize) -> Result<Hertz, Error> {
        match Awg::prepared(self) {
            Some((r, c, got)) if r == rate && c == count => Ok(got),
            _ => Awg::prepare_listen(self, rate, count),
        }
    }
    fn listen(&mut self) -> Result<(), Error> {
        Awg::listen(self)
    }
    fn speak(&mut self) -> Result<(), Error> {
        Awg::speak(self)
    }
    fn serve(&mut self) -> Option<Result<u8, Error>> {
        Awg::on_interrupt(self)
    }
    fn listen_on_end(&mut self) -> bool {
        Awg::listen_on_end(self)
    }
    fn is_listening(&self) -> bool {
        Awg::is_listening(self)
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
    /// A frame [`Frame::new`] refused: a duty near the top before a 0 or at
    /// the end (`TopToZero`).
    Frame(Error),
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

/// A lane's frame share while lent: checked and closed, or unused.
enum Share {
    Free(&'static mut [u32]),
    Frame(Frame),
}

impl Share {
    fn into_words(self) -> &'static mut [u32] {
        match self {
            Share::Free(words) => words,
            Share::Frame(frame) => frame.into_words(),
        }
    }
}

/// One exchange lent to the port: every lane's shares. It lives in the
/// port's memory ([`FrameMemory`]) and stays there: the queues carry a
/// reference to it, so an exchange crosses each way without its 300 bytes
/// being copied (D33).
struct Lent {
    frames: [Option<Share>; MAX_LANES],
    received: [Option<&'static mut [u16]>; MAX_LANES],
    /// For each lane, the lane whose share holds its read.
    reader_of: [u8; MAX_LANES],
    receive: Option<(Hertz, u16)>,
    times: Times,
}

impl Lent {
    const fn new() -> Self {
        Self {
            frames: [const { None }; MAX_LANES],
            received: [const { None }; MAX_LANES],
            reader_of: [0; MAX_LANES],
            receive: None,
            times: Times {
                submitted: 0,
                played: 0,
                frame_end: 0,
                listening: 0,
                received: 0,
                rate: 0,
            },
        }
    }
}

/// An exchange coming back: the exchange itself stays in the port's memory.
struct Done {
    lent: &'static mut Lent,
    outcome: Outcome,
}

/// The port's memory: a one-slot queue each way, the cancel flag and the
/// wake.
pub struct FrameMemory {
    lent: Queue<&'static mut Lent, 2>,
    returned: Queue<Done, 2>,
    /// The one exchange (D8), handed from side to side by reference.
    exchange: Lent,
    cancel: AtomicBool,
    wake: Signal<()>,
}

impl FrameMemory {
    pub const fn new() -> Self {
        Self {
            lent: Queue::new(),
            returned: Queue::new(),
            exchange: Lent::new(),
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
    /// Lending every share, preparing, loading every lane and playing.
    pub start: Span,
    /// The run that sees the last lane end, whole: the turn and taking the
    /// frames back.
    pub sent: Span,
    /// The run that sees the receive end, whole: taking the samples back,
    /// `speak`, and giving the exchange back.
    pub end: Span,
    /// One run of the port, whatever it did.
    pub run: Span,
    /// Samples copied by the port: zero by construction with lent memory,
    /// kept so the bench reports it.
    pub copied_out: u32,
    pub copied_in: u32,
    /// Exchanges the return queue refused: zero unless something is broken.
    pub lost: u32,
    /// Shares a lane did not give back (its stream would not stop).
    pub stuck: u32,
    /// `start` by part.
    pub parts: StartParts,
    /// The turn by part.
    pub turn_parts: TurnParts,
}

/// Where the turn goes before `listen`, on the firmware's clock.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TurnParts {
    /// `serve` in the run that sees the frame end (with listen at the end,
    /// the listen too).
    pub serve: Span,
    /// From `serve` returning to `listen` being called.
    pub between: Span,
    /// The port's runs while the frame is sent, counted per exchange at
    /// the turn, that run included.
    pub runs: Span,
    /// From the entry of the first of those runs to the entry of the one
    /// that turns.
    pub lead: Span,
    /// The port's runs while listening, per exchange, the one that ends
    /// the receive included.
    pub listening_runs: Span,
    /// Turns made inside `serve` (listen at the end).
    pub early: u32,
}

/// Where an exchange's start spends its time, on the firmware's
/// clock. Each span runs from the end of the one before.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StartParts {
    /// From the run's entry to `start`: `serve`, and any end of the exchange
    /// before it in the same run.
    pub before: Span,
    /// Of `before`: from the run's entry to the queue's dequeue.
    pub check: Span,
    /// Of `before`: the dequeue, the exchange moved into `running`.
    pub dequeue: Span,
    /// Lending every receive share.
    pub lend: Span,
    /// `prepare`: both timers' rates and each reader's transfer.
    pub prepare: Span,
    /// Each lane's `load`, by lane.
    pub load: [Span; MAX_LANES],
    /// Recording each lane's reader.
    pub reader: Span,
    /// `play`: staging and starting the timers.
    pub play: Span,
    /// `start` whole, for exchanges with a receive and without.
    pub with_receive: Span,
    pub without_receive: Span,
    /// Starts in a run that also ended an exchange.
    pub after_end: u32,
}

/// The consumer's end of the port: the same type for every generator, so
/// `init` can give it to any task (D12).
pub struct FrameHandle {
    /// Every lane's frame share and receive share, while not lent.
    frames: [Option<&'static mut [u32]>; MAX_LANES],
    received: [Option<&'static mut [u16]>; MAX_LANES],
    /// What each lane's frames are checked against at submit.
    limits: [Option<FrameLimits>; MAX_LANES],
    out: bool,
    /// The exchange, while the handle holds it.
    slot: Option<&'static mut Lent>,
    lent: Producer<'static, &'static mut Lent>,
    returned: Consumer<'static, Done>,
    cancel: &'static AtomicBool,
    wake: SignalReader<'static, ()>,
    lanes: u8,
    kick: fn(),
    now: fn() -> u32,
    /// Refused by the port's queue; see [`SubmitError::Full`].
    full: u32,
    /// `submit`, from entry to the pend: checking, closing and lending.
    submit: Span,
}

impl FrameHandle {
    /// Opens a new exchange. Refused while an exchange is out (D8).
    pub fn exchange(&mut self) -> Result<Exchange<'_>, SubmitError> {
        if self.out {
            return Err(SubmitError::Outstanding);
        }
        Ok(Exchange {
            handle: self,
            lens: [0; MAX_LANES],
            receive: None,
        })
    }

    /// The exchange the port gave back, with its shares.
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

    /// What `submit` has cost, on the firmware's clock.
    pub fn submit_stats(&self) -> Span {
        self.submit
    }

    pub fn clear_submit_stats(&mut self) {
        self.submit = Span::default();
    }
}

/// One exchange being built in the lent shares. Dropping it, or a refused
/// submit, leaves them with the handle.
pub struct Exchange<'h> {
    handle: &'h mut FrameHandle,
    lens: [u16; MAX_LANES],
    receive: Option<(Hertz, u16)>,
}

impl Exchange<'_> {
    /// Lane `lane`'s frame, `len` samples, to be written in place. `None`
    /// for a lane the port does not have, or a frame that leaves no room in
    /// the lane's share for the closing 0.
    pub fn frame(&mut self, lane: usize, len: usize) -> Option<&mut [u32]> {
        if lane >= usize::from(self.handle.lanes) {
            return None;
        }
        let share = self.handle.frames.get_mut(lane)?.as_deref_mut()?;
        if len >= share.len() {
            return None;
        }
        *self.lens.get_mut(lane)? = u16::try_from(len).ok()?;
        share.get_mut(..len)
    }

    /// After the frames, every lane reads `count` samples at `rate`.
    pub fn receive(&mut self, rate: Hertz, count: usize) -> Result<(), SubmitError> {
        let lanes = usize::from(self.handle.lanes);
        let room = self
            .handle
            .received
            .iter()
            .take(lanes)
            .map(|share| share.as_ref().map_or(0, |share| share.len()))
            .min()
            .unwrap_or(0);
        if count == 0 || count > room {
            return Err(SubmitError::TooLong);
        }
        let count = u16::try_from(count).map_err(|_| SubmitError::TooLong)?;
        self.receive = Some((rate, count));
        Ok(())
    }

    /// Checks and closes every frame in place, lends the shares to the port
    /// and pends it. A refused frame refuses the whole exchange, with every
    /// share left with the handle.
    pub fn submit(self) -> Result<(), SubmitError> {
        let entered = (self.handle.now)();
        let handle = self.handle;
        if self.lens.iter().all(|&len| len == 0) {
            return Err(SubmitError::Empty);
        }
        let mut frames: [Option<Share>; MAX_LANES] = Default::default();
        let mut refused = None;
        for (lane, slot) in frames.iter_mut().enumerate() {
            let Some(words) = handle.frames.get_mut(lane).and_then(Option::take) else {
                continue;
            };
            let len = usize::from(self.lens.get(lane).copied().unwrap_or(0));
            if len == 0 || refused.is_some() {
                *slot = Some(Share::Free(words));
                continue;
            }
            let Some(limits) = handle.limits.get(lane).copied().flatten() else {
                refused = Some(SubmitError::TooLong);
                *slot = Some(Share::Free(words));
                continue;
            };
            match Frame::new(words, len, limits) {
                Ok(frame) => *slot = Some(Share::Frame(frame)),
                Err(no) => {
                    let (error, words) = no.into_inner();
                    refused = Some(SubmitError::Frame(error));
                    *slot = Some(Share::Free(words));
                }
            }
        }
        if let Some(error) = refused {
            for (lane, share) in frames.into_iter().enumerate() {
                if let (Some(to), Some(share)) = (handle.frames.get_mut(lane), share) {
                    *to = Some(share.into_words());
                }
            }
            return Err(error);
        }
        let mut received: [Option<&'static mut [u16]>; MAX_LANES] = Default::default();
        for (to, from) in received.iter_mut().zip(handle.received.iter_mut()) {
            *to = from.take();
        }
        let Some(lent) = handle.slot.take() else {
            // The exchange is still out: `out` said so first.
            for (lane, share) in frames.into_iter().enumerate() {
                if let (Some(to), Some(share)) = (handle.frames.get_mut(lane), share) {
                    *to = Some(share.into_words());
                }
            }
            for (to, from) in handle.received.iter_mut().zip(received) {
                *to = from;
            }
            return Err(SubmitError::Outstanding);
        };
        lent.frames = frames;
        lent.received = received;
        lent.reader_of = core::array::from_fn(|lane| lane as u8);
        lent.receive = self.receive;
        lent.times = Times {
            submitted: entered,
            ..Times::default()
        };
        match handle.lent.enqueue(lent) {
            Ok(()) => {
                handle.out = true;
                handle.submit.add((handle.now)().wrapping_sub(entered));
                (handle.kick)();
                Ok(())
            }
            Err(lent) => {
                handle.full = handle.full.saturating_add(1);
                handle.give_back(lent);
                Err(SubmitError::Full)
            }
        }
    }
}

impl FrameHandle {
    /// Puts an exchange's shares back with the handle, and the exchange.
    fn give_back(&mut self, lent: &'static mut Lent) {
        for (lane, share) in lent.frames.iter_mut().enumerate() {
            if let (Some(to), Some(share)) = (self.frames.get_mut(lane), share.take()) {
                *to = Some(share.into_words());
            }
        }
        for (to, from) in self.received.iter_mut().zip(lent.received.iter_mut()) {
            if let Some(share) = from.take() {
                *to = Some(share);
            }
        }
        self.slot = Some(lent);
    }
}

/// An exchange the port gave back. Dropping it frees the shares.
pub struct Returned<'h> {
    handle: &'h mut FrameHandle,
    lent: Option<&'static mut Lent>,
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

    /// What lane `lane` read, as whole port samples: its reader's share,
    /// which other lanes on that timer and port share. Empty after a send
    /// only, or a failure.
    pub fn received(&self, lane: usize) -> &[u16] {
        let Some(lent) = self.lent.as_ref() else {
            return &[];
        };
        let count = match (self.outcome, lent.receive) {
            (Outcome::Received, Some((_, count))) => usize::from(count),
            _ => 0,
        };
        let reader = usize::from(lent.reader_of.get(lane).copied().unwrap_or(0));
        lent.received
            .get(reader)
            .and_then(Option::as_deref)
            .and_then(|share| share.get(..count))
            .unwrap_or(&[])
    }
}

impl Drop for Returned<'_> {
    fn drop(&mut self) {
        if let Some(lent) = self.lent.take() {
            self.handle.give_back(lent);
            self.handle.out = false;
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
    lent: Consumer<'static, &'static mut Lent>,
    returned: Producer<'static, Done>,
    wake: SignalWriter<'static, ()>,
    cancel: &'static AtomicBool,
    running: Option<&'static mut Lent>,
    phase: Phase,
    now: fn() -> u32,
    stats: Stats,
    /// `serve` listens at the frame's end.
    early: bool,
    /// Runs this exchange, while sending and while listening,
    /// and the first sending run's entry.
    sending_runs: u32,
    listening_runs: u32,
    first_entry: u32,
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

    fn lanes(&self) -> usize {
        self.generator.lanes().min(MAX_LANES)
    }

    /// Takes every frame the lanes hold back into the running exchange.
    fn take_frames(&mut self) {
        let lanes = self.lanes();
        let Some(lent) = self.running.as_mut() else {
            return;
        };
        for (lane, slot) in lent.frames.iter_mut().enumerate().take(lanes) {
            if slot.is_none() {
                match self.generator.take_frame(lane) {
                    Some(frame) => *slot = Some(Share::Frame(frame)),
                    None => self.stats.stuck = self.stats.stuck.saturating_add(1),
                }
            }
        }
    }

    /// Takes every receive share the lanes hold back into the running
    /// exchange.
    fn take_samples(&mut self) {
        let lanes = self.lanes();
        let Some(lent) = self.running.as_mut() else {
            return;
        };
        for (lane, slot) in lent.received.iter_mut().enumerate().take(lanes) {
            if slot.is_none() {
                match self.generator.take_samples(lane) {
                    Some(samples) => *slot = Some(samples),
                    None => self.stats.stuck = self.stats.stuck.saturating_add(1),
                }
            }
        }
    }

    /// Gives the running exchange back with `outcome`, every share taken
    /// back first.
    fn finish(&mut self, outcome: Outcome) {
        self.phase = Phase::Idle;
        self.take_frames();
        self.take_samples();
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

    /// Lends every share, prepares the receive, loads every lane that has a
    /// frame and plays them from one start.
    fn start(&mut self, entered: u32, after_end: bool) {
        let lanes = self.lanes();
        let mut failed = None;
        let mut mark = (self.now)();
        self.stats.parts.before.add(mark.wrapping_sub(entered));
        if after_end {
            self.stats.parts.after_end = self.stats.parts.after_end.saturating_add(1);
        }
        // The exchange is worked on in place, in `running`: moved once, out
        // of the queue.
        let Some(lent) = self.running.as_mut() else {
            return;
        };
        if lent.receive.is_some() {
            for (lane, slot) in lent.received.iter_mut().enumerate().take(lanes) {
                let Some(samples) = slot.take() else {
                    failed = Some(Error::TooLarge);
                    break;
                };
                if let Err((error, samples)) = self.generator.lend(lane, samples) {
                    *slot = Some(samples);
                    failed = Some(error);
                    break;
                }
            }
        }
        let now = (self.now)();
        self.stats.parts.lend.add(now.wrapping_sub(mark));
        mark = now;
        // The listen is computed before the frame, outside the
        // turn-around.
        if failed.is_none()
            && let Some((rate, count)) = lent.receive
        {
            match self.generator.prepare(rate, usize::from(count)) {
                Ok(got) => lent.times.rate = got.0,
                Err(error) => failed = Some(error),
            }
        }
        let now = (self.now)();
        self.stats.parts.prepare.add(now.wrapping_sub(mark));
        mark = now;
        if failed.is_none() {
            for (lane, slot) in lent.frames.iter_mut().enumerate().take(lanes) {
                if !matches!(slot, Some(Share::Frame(_))) {
                    continue;
                }
                let Some(Share::Frame(frame)) = slot.take() else {
                    continue;
                };
                if let Err((error, frame)) = self.generator.load(lane, frame) {
                    *slot = Some(Share::Frame(frame));
                    failed = Some(error);
                    break;
                }
                let now = (self.now)();
                if let Some(span) = self.stats.parts.load.get_mut(lane) {
                    span.add(now.wrapping_sub(mark));
                }
                mark = now;
            }
        }
        if failed.is_none() {
            for (lane, reader) in lent.reader_of.iter_mut().enumerate().take(lanes) {
                *reader = u8::try_from(self.generator.reader_of(lane)).unwrap_or(0);
            }
        }
        let now = (self.now)();
        self.stats.parts.reader.add(now.wrapping_sub(mark));
        mark = now;
        let receive = lent.receive.is_some();
        if let Some(error) = failed {
            self.stop(Outcome::Failed(error));
            return;
        }
        self.early = receive && self.generator.listen_on_end();
        self.sending_runs = 0;
        self.listening_runs = 0;
        if let Err(error) = self.generator.play() {
            self.stop(Outcome::Failed(error));
            return;
        }
        let played = (self.now)();
        self.stats.parts.play.add(played.wrapping_sub(mark));
        self.stats.start.add(played.wrapping_sub(entered));
        if receive {
            self.stats
                .parts
                .with_receive
                .add(played.wrapping_sub(entered));
        } else {
            self.stats
                .parts
                .without_receive
                .add(played.wrapping_sub(entered));
        }
        if let Some(lent) = self.running.as_mut() {
            lent.times.played = played;
        }
        self.phase = Phase::Sending;
    }

    /// The last lane ended its frame: listen in this same run, or give the
    /// exchange back.
    fn sent(&mut self, entered: u32, served: u32) {
        self.stats.turn_parts.runs.add(self.sending_runs);
        self.stats
            .turn_parts
            .lead
            .add(entered.wrapping_sub(self.first_entry));
        self.stats
            .turn_parts
            .serve
            .add(served.wrapping_sub(entered));
        if self.early && self.generator.is_listening() {
            // `serve` listened already.
            self.stats.turn_parts.early = self.stats.turn_parts.early.saturating_add(1);
            self.stats.turn.add(served.wrapping_sub(entered));
            if let Some(lent) = self.running.as_mut() {
                lent.times.frame_end = entered;
                lent.times.listening = served;
            }
            self.phase = Phase::Listening;
            self.stats.sent.add((self.now)().wrapping_sub(entered));
            return;
        }
        let receive = self.running.as_mut().and_then(|lent| {
            lent.times.frame_end = entered;
            lent.receive
        });
        if receive.is_none() {
            self.finish(Outcome::Sent);
            return;
        }
        let before = (self.now)();
        self.stats
            .turn_parts
            .between
            .add(before.wrapping_sub(served));
        match self.generator.listen() {
            Ok(()) => {
                let after = (self.now)();
                self.stats.turn.add(after.wrapping_sub(entered));
                self.stats.listen.add(after.wrapping_sub(before));
                if let Some(lent) = self.running.as_mut() {
                    lent.times.listening = after;
                }
                self.phase = Phase::Listening;
                // The frames come back with the exchange: a reader lane's
                // frame stays with it while its read runs.
                self.stats.sent.add((self.now)().wrapping_sub(entered));
            }
            Err(error) => self.stop(Outcome::Failed(error)),
        }
    }

    /// Every lane has read into its share: speak again, and give the
    /// exchange back.
    fn received(&mut self, entered: u32) {
        self.stats
            .turn_parts
            .listening_runs
            .add(self.listening_runs);
        if let Some(lent) = self.running.as_mut() {
            lent.times.received = entered;
        }
        match self.generator.speak() {
            Ok(()) => self.finish(Outcome::Received),
            Err(error) => self.stop(Outcome::Failed(error)),
        }
        self.stats.end.add((self.now)().wrapping_sub(entered));
    }
}

impl<G: Generator> Service for FramePort<G> {
    type Event = ();

    /// A lane's stream ended, the consumer submitted, or it cancelled.
    fn on_interrupt(&mut self) {
        let entered = (self.now)();
        let busy = self.phase != Phase::Idle;
        if self.phase != Phase::Idle {
            match self.phase {
                Phase::Sending => {
                    if self.sending_runs == 0 {
                        self.first_entry = entered;
                    }
                    self.sending_runs = self.sending_runs.saturating_add(1);
                }
                Phase::Listening => self.listening_runs = self.listening_runs.saturating_add(1),
                Phase::Idle => {}
            }
            let served = self.generator.serve();
            let after = (self.now)();
            match served {
                Some(Err(error)) => self.stop(Outcome::Failed(error)),
                Some(Ok(_))
                    if self.phase == Phase::Sending
                        && self.early
                        && self.generator.is_listening() =>
                {
                    self.sent(entered, after)
                }
                Some(Ok(_)) if !self.generator.is_busy() => match self.phase {
                    Phase::Sending => self.sent(entered, after),
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
        if self.phase == Phase::Idle && self.running.is_none() {
            let checked = (self.now)();
            self.running = self.lent.dequeue();
            if self.running.is_some() {
                let dequeued = (self.now)();
                self.stats.parts.check.add(checked.wrapping_sub(entered));
                self.stats.parts.dequeue.add(dequeued.wrapping_sub(checked));
                self.start(entered, busy);
            }
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

/// Splits `memory` into `lanes` equal shares, once.
fn split<T>(mut memory: &'static mut [T], lanes: usize) -> [Option<&'static mut [T]>; MAX_LANES] {
    let stride = memory.len() / lanes.max(1);
    let mut shares: [Option<&'static mut [T]>; MAX_LANES] = Default::default();
    for share in shares.iter_mut().take(lanes) {
        let (head, tail) = core::mem::take(&mut memory).split_at_mut(stride);
        *share = Some(head);
        memory = tail;
    }
    shares
}

/// Builds the port around the generator the firmware made in `init`
/// (lanes, timers, polarity and outputs are FerroHAL's), with the
/// consumer's lent memory: `frames` and `received`, each split once,
/// evenly, between the lanes. A lane's frame share holds its longest frame
/// and the closing 0. Infallible: a frame or receive too long for its
/// share is refused when the exchange is built.
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
        exchange,
        cancel,
        wake,
    } = memory;
    let cancel: &'static AtomicBool = cancel;
    let (lent_in, lent_out) = lent.split();
    let (back_in, back_out) = returned.split();
    let (writer, reader) = wake.split();
    let count = generator.lanes().min(MAX_LANES);
    let lanes = u8::try_from(count).unwrap_or(1);
    let limits = core::array::from_fn(|lane| generator.limits(lane));
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
            early: false,
            sending_runs: 0,
            listening_runs: 0,
            first_entry: 0,
        },
        handles: FrameHandles {
            handle: Some(FrameHandle {
                frames: split(frames, count),
                received: split(received, count),
                limits,
                out: false,
                slot: Some(exchange),
                lent: lent_in,
                returned: back_out,
                cancel,
                wake: reader,
                lanes,
                kick,
                now,
                full: 0,
                submit: Span::default(),
            }),
        },
    }
}

/// F, the frame port. One member, declared once per lane's stream vector,
/// also pended by the handle. In `init`:
/// `lanes_frame::new_frame(generator, memory, frames, received,
/// <selection>::kick, now)`.
#[ferroforge::group(
    slots = [generator: G, start: S],
    vectors = [f_on_lane_dma = [generator], f_on_start = start],
    shared = [port: FramePort<G>],
    config = [
        frames: &'static mut [u32] @ software,
        received: &'static mut [u16] @ software,
        now: fn() -> u32 @ platform,
    ],
    memory = [memory: FrameMemory],
    handles = [handles: FrameHandles],
    pends = [kick = f_on_start],
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

    /// The start vector, which the handle pends: a submit, a cancel.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn f_on_start(mut cx: f_on_start::Context) {
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

    /// A stand-in generator: records the calls, holds what it is lent, and
    /// ends a play or a listen when the test says so.
    #[derive(Default)]
    struct Fake {
        lanes: usize,
        calls: Vec<&'static str>,
        loaded: Vec<(usize, Vec<u32>)>,
        held: Vec<Option<Frame>>,
        lent: Vec<Option<&'static mut [u16]>>,
        busy: bool,
        sending: bool,
        listening: bool,
        outputs_off: bool,
        prepared: Option<(Hertz, usize)>,
        /// Every lane shares lane 0's read.
        shared: bool,
        /// What the next `serve` returns.
        next: Option<Result<u8, Error>>,
        refuse_load: Option<Error>,
    }

    const LIMITS: FrameLimits = FrameLimits::new(100, 5);

    impl Generator for Fake {
        fn lanes(&self) -> usize {
            self.lanes
        }
        fn limits(&self, lane: usize) -> Option<FrameLimits> {
            (lane < self.lanes).then_some(LIMITS)
        }
        fn load(&mut self, lane: usize, frame: Frame) -> Result<(), FrameRefused> {
            self.calls.push("load");
            if let Some(error) = self.refuse_load {
                return Err((error, frame));
            }
            self.loaded.push((lane, frame.duties().to_vec()));
            self.held.resize_with(self.lanes, || None);
            self.held[lane] = Some(frame);
            Ok(())
        }
        fn take_frame(&mut self, lane: usize) -> Option<Frame> {
            if self.sending {
                return None;
            }
            self.held.get_mut(lane)?.take()
        }
        fn lend(&mut self, lane: usize, samples: &'static mut [u16]) -> Result<(), SamplesRefused> {
            self.lent.resize_with(self.lanes, || None);
            self.lent[lane] = Some(samples);
            Ok(())
        }
        fn take_samples(&mut self, lane: usize) -> Option<&'static mut [u16]> {
            if self.busy {
                return None;
            }
            self.lent.get_mut(lane)?.take()
        }
        fn reader_of(&self, lane: usize) -> usize {
            if self.shared { 0 } else { lane }
        }
        fn play(&mut self) -> Result<(), Error> {
            self.calls.push("play");
            self.busy = true;
            self.sending = true;
            Ok(())
        }
        fn prepare(&mut self, rate: Hertz, count: usize) -> Result<Hertz, Error> {
            self.prepared = Some((rate, count));
            Ok(rate)
        }
        fn listen(&mut self) -> Result<(), Error> {
            self.calls.push("listen");
            if self.busy || self.listening {
                return Err(Error::Busy);
            }
            let (_, count) = self.prepared.ok_or(Error::NotPrepared)?;
            self.listening = true;
            self.outputs_off = true;
            self.busy = true;
            for lane in 0..self.lanes {
                if self.reader_of(lane) != lane {
                    continue;
                }
                if let Some(Some(share)) = self.lent.get_mut(lane) {
                    for (i, s) in share.iter_mut().take(count).enumerate() {
                        *s = i as u16 + 100 * lane as u16;
                    }
                }
            }
            Ok(())
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
                self.sending = false;
            }
            if let Some(Err(_)) = next {
                self.abort();
            }
            next
        }
        fn abort(&mut self) {
            self.calls.push("abort");
            self.busy = false;
            self.sending = false;
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
        // Played in place: nothing copied, every frame back with the handle.
        assert_eq!(p.stats().copied_out, 0);
        assert_eq!(p.stats().stuck, 0);
        let mut x = h.exchange().unwrap();
        // The closing 0 was written in the share, after the frame.
        assert_eq!(x.frame(0, 4).unwrap(), &[1, 2, 3, 0]);
        assert!(x.frame(1, 7).is_some());
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
        assert_eq!(p.stats().sent.count, 1);
        let t = r.times();
        assert!(t.submitted < t.played && t.played < t.frame_end);
        assert!(t.frame_end < t.listening && t.listening < t.received);
        assert_eq!(t.rate, 2_000_000);
        drop(r);
        assert_eq!(p.stats().copied_in, 0);
        assert_eq!(p.stats().stuck, 0);
        assert_eq!(p.stats().end.count, 1);
    }

    #[test]
    fn lanes_that_share_a_reader_get_its_share() {
        let (mut p, mut h) = port(2);
        p.generator.shared = true;
        submit(&mut h, &[&[7], &[8]], Some(3));
        p.on_interrupt();
        p.generator.next = Some(Ok(0b11));
        p.on_interrupt();
        p.generator.next = Some(Ok(0b1));
        p.on_interrupt();
        let r = h.take().unwrap();
        assert_eq!(r.outcome(), Outcome::Received);
        assert_eq!(r.received(0), &[0, 1, 2]);
        assert_eq!(r.received(1), &[0, 1, 2]);
    }

    #[test]
    fn a_frame_stepping_from_the_top_to_zero_is_refused_at_submit() {
        let (mut p, mut h) = port(2);
        let mut x = h.exchange().unwrap();
        x.frame(0, 2).unwrap().copy_from_slice(&[10, 20]);
        // 96 is within 5 counts of the period's end (100), then 0.
        x.frame(1, 3).unwrap().copy_from_slice(&[96, 0, 10]);
        assert_eq!(x.submit(), Err(SubmitError::Frame(Error::TopToZero)));
        let mut x = h.exchange().unwrap();
        x.frame(0, 1).unwrap().copy_from_slice(&[96]);
        assert_eq!(x.submit(), Err(SubmitError::Frame(Error::TopToZero)));
        // Nothing reached the port, and every share is still the handle's.
        p.on_interrupt();
        assert!(p.generator.calls.is_empty());
        let mut x = h.exchange().unwrap();
        assert!(x.frame(0, 7).is_some());
        assert!(x.frame(1, 7).is_some());
        // The exchange ends here unsubmitted: it only borrowed the handle.
        let mut x = h.exchange().unwrap();
        x.frame(0, 1).unwrap().copy_from_slice(&[94]);
        assert_eq!(x.submit(), Ok(()));
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
        // Each lane's share is 8 words of frame (7 duties and the closing
        // 0) and 16 samples of receive.
        assert!(x.frame(0, 8).is_none());
        assert!(x.frame(2, 1).is_none());
        assert_eq!(x.receive(Hertz::mhz(1), 17), Err(SubmitError::TooLong));
        assert_eq!(x.receive(Hertz::mhz(1), 0), Err(SubmitError::TooLong));
        assert!(x.frame(1, 7).is_some());
        // The exchange ends here unsubmitted: it only borrowed the handle.
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
        drop(r);
        // The refused frame and the lent receive share came back.
        assert_eq!(p.stats().stuck, 0);
        let mut x = h.exchange().unwrap();
        assert!(x.frame(0, 7).is_some());
        assert!(x.receive(Hertz::mhz(1), 16).is_ok());
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
        let [lanes, start] = contract.vectors.as_slice() else {
            panic!("two vector roles");
        };
        // One member serves every lane's stream vector; the handle pends
        // the other, which serves one vector, the start's.
        assert_eq!(lanes.member, "f_on_lane_dma");
        assert!(lanes.is_repeated());
        assert_eq!(start.member, "f_on_start");
        assert!(!start.is_repeated());
        let pends: Vec<_> = contract
            .pends
            .iter()
            .map(|p| (p.name.to_string(), p.member.to_string()))
            .collect();
        assert_eq!(pends, [("kick".to_string(), "f_on_start".to_string())]);
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
