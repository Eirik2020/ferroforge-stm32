//! PROTOTYPE: the timer lane loopback checks of `lane-port-prototype-design.md`,
//! shared by both boards' benches and both variants: T (today's
//! `compare_lanes`, the firmware's task switching to listen) and F (the frame
//! port, `lanes_frame`, switching in its own interrupt). Each bench includes
//! this file with `#[path]` and implements [`Port`] for its variant,
//! [`Responder`] for its responder lane and, on the H753, [`Witness`] for its
//! pin sampler.
//!
//! The loopback: lane 0's pin is linked through a series resistor to the
//! responder, a lane on a second timer that the lane timer's start starts
//! through the trigger route. Armed before each exchange, the responder holds
//! its line inactive for the frame and `d` periods more, then plays
//! [`REPLY`]. Lane 0, listening by then, reads it; the reply is compared
//! sample for sample, with no protocol decoder. The other lanes have no link
//! and must read their idle level.
//!
//! Every lane idles high (active low), with a pull-up on its pin.

pub use ferroforge_stm32::lanes_frame::Span;
use ferrohal_stm32::{time::Hertz, timer::recover};

/// Samples in a frame: DShot's sixteen bits, as duties. The frame's last
/// sample is always [`LAST`], so its last edge sits at a fixed point.
pub const FRAME: usize = 16;
/// The frame's last sample, in sixteenths of a period.
pub const LAST: u32 = 8;
/// The reply, in sixteenths: it starts and ends with a pulse, and its long
/// and short pulses differ by half a period.
pub const REPLY: [u32; 8] = [12, 4, 12, 12, 4, 4, 12, 4];
/// The longest delay, in periods: 60 us at 600 kHz.
pub const D_MAX: usize = 36;
/// Receive samples a period of the send rate. The reply's short pulse is a
/// quarter period, so at 4 a period it lasts one capture and a receive
/// catches it or not by phase, as the first H753 run showed; at 8 every
/// pulse spans at least one whole capture.
pub const RX_PER_PERIOD: u32 = 8;
/// A receive long enough for the latest reply, and some.
pub const RX_COUNT: usize = RX_PER_PERIOD as usize * (D_MAX + REPLY.len() + 6);
/// Exchanges at each delay.
pub const EXCHANGES: u32 = 1000;
/// The longest frame the benches give a lane: its DMA buffer holds one more,
/// the 0 FerroHAL appends.
pub const LONG: usize = 500;
pub const LANE_BUFFER: usize = LONG + 1;
/// The longest receive: a lane's samples buffer.
pub const LONG_RX: usize = 1024;
/// The responder's waveform at the longest delay, with FerroHAL's 0.
pub const RESPONDER_BUFFER: usize = FRAME + D_MAX + REPLY.len() + 2;
/// Witness captures a period of the send rate.
pub const WITNESS_PER_PERIOD: u32 = 8;
/// Enough for the longest frame and the lead from its submit to its play,
/// which on the F407 with four lanes in F is over 80 us.
pub const WITNESS_BUFFER: usize = WITNESS_PER_PERIOD as usize * (LONG + 120);

/// How an exchange ended, as the bench sees it.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Outcome {
    Sent,
    Received,
    Aborted,
    Failed,
    /// Refused before it started.
    Refused,
}

/// When an exchange's steps happened, in core cycles.
#[derive(Clone, Copy, Default)]
pub struct Took {
    pub submitted: u32,
    pub played: u32,
    pub frame_end: u32,
    pub listening: u32,
    pub received: u32,
    /// When the bench saw it come back.
    pub seen: u32,
}

/// What a variant measures of itself, in core cycles.
#[derive(Clone, Copy, Default)]
pub struct PortStats {
    /// From the start of the run that saw the last lane end to `listen`
    /// having returned.
    pub turn: Span,
    /// `listen` alone.
    pub listen: Span,
    /// One run of the lanes' member.
    pub run: Span,
    /// T: each time the firmware's task holds the generator's lock. F: none.
    pub hold: Span,
    /// Loading every lane and playing.
    pub start: Span,
}

/// Each lane's samples, as an exchange brought them back.
pub trait Received {
    fn lane(&self, lane: usize) -> &[u16];
}

/// An exchange with nothing read.
pub struct Nothing;

impl Received for Nothing {
    fn lane(&self, _lane: usize) -> &[u16] {
        &[]
    }
}

/// One variant, as the checks drive it.
pub trait Port {
    fn lanes(&self) -> usize;
    fn period(&mut self) -> u32;
    fn sample_rate(&mut self) -> Hertz;
    /// Starts one exchange: `frames[lane]` on each lane, then, with
    /// `receive`, every lane reads `count` samples at `rate`.
    fn submit(&mut self, frames: &[&[u32]], receive: Option<(Hertz, usize)>) -> bool;
    /// Once the exchange is back, hands it to `read` with each lane's
    /// samples, and returns true.
    fn poll(&mut self, read: &mut dyn FnMut(Outcome, &Took, &dyn Received)) -> bool;
    fn cancel(&mut self);
    fn stats(&mut self) -> PortStats;
    fn clear_stats(&mut self);
}

/// The responder lane, owned by the bench.
pub trait Responder {
    /// Arms `wave` to start with the lane timer.
    fn arm(&mut self, wave: &[u32]) -> bool;
    /// The responder timer's period in its own counts. It plays at the
    /// lanes' rate, but its timer may count a different clock (on the F407,
    /// TIM3 counts APB1's 84 MHz against TIM1's 168 MHz), so its waveform is
    /// built in these counts, not the lanes'.
    fn period(&self) -> u32;
    /// Waits for the waveform to end and stops the timer; whether it ended.
    fn finish(&mut self) -> bool;
    /// Stops it wherever it is, line inactive.
    fn abort(&mut self);
}

/// The independent pin sampler, where the board has one.
pub trait Witness {
    fn rate(&self) -> Hertz;
    fn start(&mut self, count: usize) -> bool;
    fn wait(&mut self) -> Option<&[u16]>;
}

/// The F401 has no timer left for one.
pub struct NoWitness;

impl Witness for NoWitness {
    fn rate(&self) -> Hertz {
        Hertz(0)
    }
    fn start(&mut self, _count: usize) -> bool {
        false
    }
    fn wait(&mut self) -> Option<&[u16]> {
        None
    }
}

/// What the checks need to know of the board.
pub struct Board {
    pub name: &'static str,
    pub variant: &'static str,
    /// Core cycles a microsecond.
    pub per_us: u32,
    /// Each lane's pin, on the port its stream reads.
    pub pins: &'static [u8],
    /// Each lane's pin on the witness's port, if it reads it.
    pub witnessed: &'static [Option<u8>],
    /// Lanes on complementary outputs (CHxN), by bit.
    pub complementary: u8,
}

/// Lane `lane`'s frame for exchange `index`: a 1 is 12/16 of the period, a
/// 0 is 6/16, and the last sample is [`LAST`]/16.
pub fn frame(index: u32, lane: usize, period: u32) -> [u32; FRAME] {
    let bits = (index as u16).wrapping_mul(0x9e37) ^ (0xa5c3u16.rotate_left(lane as u32 * 4));
    core::array::from_fn(|i| {
        if i == FRAME - 1 {
            period * LAST / 16
        } else if bits & (0x8000 >> i) != 0 {
            period * 12 / 16
        } else {
            period * 6 / 16
        }
    })
}

/// The responder's waveform for a delay of `d` periods: inactive for the
/// frame and `d` periods, then the reply.
pub fn responder_wave(d: usize, period: u32, out: &mut [u32; RESPONDER_BUFFER]) -> &[u32] {
    let len = FRAME + d + REPLY.len();
    for (i, slot) in out.iter_mut().enumerate().take(len) {
        *slot = match i.checked_sub(FRAME + d) {
            Some(r) => period * REPLY[r] / 16,
            None => 0,
        };
    }
    &out[..len]
}

/// The time from the frame's last edge to the reply's first, for delay `d`,
/// in nanoseconds: the rest of the last period, then `d` periods.
pub fn gap_ns(d: usize, rate: Hertz) -> u32 {
    let period_ns = 1_000_000_000u64 / u64::from(rate.0.max(1));
    let rest = period_ns * u64::from(16 - LAST) / 16;
    (rest + period_ns * d as u64) as u32
}

/// Whether `samples` hold the whole reply on `pin`: exactly its eight low
/// pulses, each as long as its sample within a capture and a half. A reply
/// that began before the first capture has a short first pulse, so it is
/// not whole.
pub fn whole(samples: &[u16], pin: u8, rx: Hertz, rate: Hertz, period: u32) -> bool {
    let mut want = [0u32; REPLY.len()];
    for (w, r) in want.iter_mut().zip(REPLY) {
        *w = period * r / 16;
    }
    widths_match(samples, pin, rx, rate, period, &want)
}

/// Whether the low pulses on `pin` are exactly `want`, one pulse a sample,
/// each pulse's width within a capture and a half of its sample's. Each
/// pulse is measured on its own, so a capture rate a little off the one
/// asked for costs a fraction of a capture a pulse, never a drift that
/// grows along the frame as `recover`'s windows do. Every sample in `want`
/// has to be between 0 and a whole period, exclusive, so that each is one
/// pulse.
pub fn widths_match(
    samples: &[u16],
    pin: u8,
    captures: Hertz,
    rate: Hertz,
    period: u32,
    want: &[u32],
) -> bool {
    first_mismatch(samples, pin, captures, rate, period, want).is_none()
}

/// Where [`widths_match`] fails: the pulse's index, its width in captures
/// (0 for a pulse missing at the end, `u32::MAX` for one too many or a line
/// left low) and the sample it was held against; `None` if it matches.
pub fn first_mismatch(
    samples: &[u16],
    pin: u8,
    captures: Hertz,
    rate: Hertz,
    period: u32,
    want: &[u32],
) -> Option<(usize, u32, u32)> {
    let Some(mask) = 1u16.checked_shl(u32::from(pin)) else {
        return Some((0, u32::MAX, 0));
    };
    // A width in captures, against a sample in counts: width / (captures /
    // rate) periods against sample / period periods.
    let scale = u64::from(period) * u64::from(rate.0);
    let per = u64::from(captures.0);
    let tolerance = scale * 3 / 2;
    let mut index = 0;
    let mut begun = None;
    for (i, s) in samples.iter().enumerate() {
        match (s & mask == 0, begun) {
            (true, None) => begun = Some(i),
            (false, Some(b)) => {
                begun = None;
                let Some(&w) = want.get(index) else {
                    return Some((index, u32::MAX, 0));
                };
                let width = (i - b) as u64;
                if (width * scale).abs_diff(u64::from(w) * per) > tolerance {
                    return Some((index, width as u32, w));
                }
                index += 1;
            }
            _ => {}
        }
    }
    match (begun, want.get(index)) {
        (Some(_), _) => Some((index, u32::MAX, want.get(index).copied().unwrap_or(0))),
        (None, Some(&w)) => Some((index, 0, w)),
        (None, None) => None,
    }
}

/// Prints what a lane's receive read on `pin`, for the margin sweep's
/// diagnosis: its length, its low pulses, and what `recover` makes of them
/// against the reply `whole` expects.
pub fn show_receive(d: usize, samples: &[u16], pin: u8, rx: Hertz, rate: Hertz, period: u32) {
    let per16 = (u64::from(rx.0) * 16 / u64::from(rate.0.max(1))) as u32;
    let mut p = [(0u32, 0u32); REPLY.len() + 4];
    let n = pulses(samples, pin, &mut p);
    let mut out = [0u32; REPLY.len() + 4];
    let found = recover(samples, pin, true, per16, period, &mut out);
    let want: [u32; REPLY.len()] = core::array::from_fn(|i| period * REPLY[i] / 16);
    defmt::info!(
        "d {} receive: {} captures at {} Hz, first {=u16:#x}, {} low pulses on pin {}: {}; recover found {} (want {}): {}; want {}",
        d,
        samples.len(),
        rx.0,
        samples.first().copied().unwrap_or(0),
        n,
        pin,
        p[..n.min(p.len())],
        found,
        REPLY.len(),
        out[..found.min(out.len())],
        want
    );
}

/// Whether every sample has `pin` high: the line idle.
pub fn idle(samples: &[u16], pin: u8) -> bool {
    !samples.is_empty() && samples.iter().all(|s| s & (1 << pin) != 0)
}

/// The low pulses on `pin`, as (first low capture, first high after), up to
/// `out`'s length; returns how many.
pub fn pulses(samples: &[u16], pin: u8, out: &mut [(u32, u32)]) -> usize {
    let mut n = 0;
    let mut start = None;
    for (i, s) in samples.iter().enumerate() {
        let low = s & (1 << pin) == 0;
        match (low, start) {
            (true, None) => start = Some(i as u32),
            (false, Some(begun)) => {
                if let Some(slot) = out.get_mut(n) {
                    *slot = (begun, i as u32);
                }
                n += 1;
                start = None;
            }
            _ => {}
        }
    }
    if let Some(begun) = start {
        if let Some(slot) = out.get_mut(n) {
            *slot = (begun, samples.len() as u32);
        }
        n += 1;
    }
    n
}

/// Prints the check's verdict; 1 for a failure, to count them.
pub fn verdict(name: &str, pass: bool) -> u32 {
    if pass {
        defmt::info!("PASS {}", name);
        0
    } else {
        defmt::error!("FAIL {}", name);
        1
    }
}

/// Spins until `done`, for at most `cycles`; whether it got there.
pub fn spin(now: fn() -> u32, cycles: u32, mut done: impl FnMut() -> bool) -> bool {
    let start = now();
    loop {
        if done() {
            return true;
        }
        if now().wrapping_sub(start) > cycles {
            return false;
        }
    }
}

/// One exchange, waited for; `None` if it did not come back in 5 ms (it is
/// then cancelled).
fn exchange<P: Port>(
    port: &mut P,
    now: fn() -> u32,
    per_us: u32,
    frames: &[&[u32]],
    receive: Option<(Hertz, usize)>,
    read: &mut dyn FnMut(Outcome, &Took, &dyn Received),
) -> Option<Outcome> {
    if !port.submit(frames, receive) {
        return Some(Outcome::Refused);
    }
    let mut outcome = None;
    let mut seen = |o: Outcome, t: &Took, s: &dyn Received| {
        outcome = Some(o);
        read(o, t, s);
    };
    if spin(now, 5_000 * per_us, || port.poll(&mut seen)) {
        return outcome;
    }
    port.cancel();
    let _ = spin(now, 1_000 * per_us, || port.poll(&mut |_, _, _| {}));
    None
}

fn span_us(name: &str, s: &Span, per_us: u32) {
    defmt::info!(
        "{}: least {} cycles, mean {}, most {} ({} ns most), over {}",
        name,
        s.least,
        s.mean(),
        s.most,
        u64::from(s.most) * 1000 / u64::from(per_us.max(1)),
        s.count
    );
}

/// The frames of exchange `index`, one per lane, into `store`.
fn frames_of(index: u32, lanes: usize, period: u32, store: &mut [[u32; FRAME]; 8]) {
    for (lane, slot) in store.iter_mut().enumerate().take(lanes) {
        *slot = frame(index, lane, period);
    }
}

/// Runs every check; returns how many failed.
pub fn run<P: Port, R: Responder, W: Witness>(
    board: &Board,
    port: &mut P,
    responder: &mut R,
    witness: &mut W,
    now: fn() -> u32,
) -> u32 {
    let per_us = board.per_us;
    let lanes = port.lanes();
    let period = port.period();
    let rate = port.sample_rate();
    let rx = Hertz(rate.0 * RX_PER_PERIOD);
    defmt::info!(
        "{} {}: {} lane(s), {} Hz, period {} counts, receive asked at {} Hz",
        board.name,
        board.variant,
        lanes,
        rate.0,
        period,
        rx.0
    );
    let mut failed = 0u32;
    let mut store = [[0u32; FRAME]; 8];
    let mut wave = [0u32; RESPONDER_BUFFER];

    // Before any receive: what each witnessed lane puts on its pin, as a
    // baseline for the finding at the end.
    if witness.start(WITNESS_PER_PERIOD as usize * (FRAME + 40)) {
        frames_of(2, lanes, period, &mut store);
        let refs: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
        let sent = exchange(port, now, per_us, &refs[..lanes], None, &mut |_, _, _| {});
        if let Some(samples) = witness.wait() {
            let mut p = [(0u32, 0u32); 2 * FRAME];
            for (lane, w) in board.witnessed.iter().enumerate().take(lanes) {
                if let Some(pin) = w {
                    let n = pulses(samples, *pin, &mut p);
                    defmt::info!(
                        "before any receive, lane {}: {} pulses on its pin of {} ({})",
                        lane,
                        n,
                        FRAME,
                        sent
                    );
                }
            }
        }
    }

    // The sweep: D from 60 us down, one period a step, EXCHANGES at each.
    port.clear_stats();
    let mut smallest_whole: Option<usize> = None;
    let mut broken_from: Option<usize> = None;
    let mut first_block = PortStats::default();
    let mut cycle = Span::default();
    let mut idle_faults = 0u32;
    let mut lost = 0u32;
    let mut wire_gaps = [0u32; D_MAX + 1];
    for d in (0..=D_MAX).rev() {
        let mut whole_count = 0u32;
        let mut bad_outcome = 0u32;
        for index in 0..EXCHANGES {
            frames_of(index, lanes, period, &mut store);
            let refs: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
            if !responder.arm(responder_wave(d, responder.period(), &mut wave)) {
                bad_outcome += 1;
                continue;
            }
            let sampled =
                index == 0 && witness.start(WITNESS_PER_PERIOD as usize * (FRAME + D_MAX + 40));
            let mut ok = false;
            let mut idle_ok = true;
            let mut took = Took::default();
            let mut got = Outcome::Failed;
            let outcome = exchange(
                port,
                now,
                per_us,
                &refs[..lanes],
                Some((rx, RX_COUNT)),
                &mut |o, t, s| {
                    got = o;
                    took = *t;
                    if o == Outcome::Received {
                        ok = whole(s.lane(0), board.pins[0], rx, rate, period);
                        if index == 0 && (d == D_MAX || d == D_MAX / 2) {
                            show_receive(d, s.lane(0), board.pins[0], rx, rate, period);
                        }
                        for lane in 1..lanes {
                            idle_ok &= idle(s.lane(lane), board.pins[lane]);
                        }
                    }
                },
            );
            let ended = responder.finish();
            if outcome.is_none() {
                lost += 1;
            }
            if got != Outcome::Received || !ended {
                bad_outcome += 1;
                responder.abort();
            }
            whole_count += u32::from(ok);
            idle_faults += u32::from(!idle_ok);
            if d == D_MAX && got == Outcome::Received {
                cycle.add(took.seen.wrapping_sub(took.submitted));
            }
            if sampled && let Some(samples) = witness.wait() {
                if let Some(pin) = board.witnessed.first().copied().flatten() {
                    let mut p = [(0u32, 0u32); FRAME + REPLY.len() + 4];
                    let n = pulses(samples, pin, &mut p);
                    if d == D_MAX || d == D_MAX / 2 {
                        let shown = n.min(p.len());
                        defmt::info!(
                            "d {} witness: {} low pulses on pin {} at {} Hz (begin, end in captures); after the frame: {}",
                            d,
                            n,
                            pin,
                            witness.rate().0,
                            p[FRAME.min(shown)..shown]
                        );
                    }
                    // The reply is the last eight pulses: anything between
                    // the frame and the reply (a turn-around glitch) is not
                    // the reply's first edge.
                    if n >= FRAME + REPLY.len() && n <= p.len() {
                        let captures = p[n - REPLY.len()].0.saturating_sub(p[FRAME - 1].1);
                        wire_gaps[d] = (u64::from(captures) * 1_000_000_000
                            / u64::from(witness.rate().0.max(1)))
                            as u32;
                    }
                }
            }
        }
        if d == D_MAX {
            first_block = port.stats();
        }
        defmt::info!(
            "d {} ({} ns frame end to reply, set; {} ns on the wire): {} of {} read whole, {} not received",
            d,
            gap_ns(d, rate),
            wire_gaps[d],
            whole_count,
            EXCHANGES,
            bad_outcome
        );
        if whole_count == EXCHANGES && broken_from.is_none() {
            smallest_whole = Some(d);
        } else if broken_from.is_none() {
            broken_from = Some(d);
        }
    }
    match (smallest_whole, broken_from) {
        (Some(d), _) => defmt::info!(
            "margin: smallest delay read whole every time d = {} ({} ns set, {} ns on the wire); first short d = {}; margin to 30 us = {} ns",
            d,
            gap_ns(d, rate),
            wire_gaps[d],
            broken_from.map_or(-1, |b| b as i32),
            30_000i64 - i64::from(gap_ns(d, rate))
        ),
        (None, _) => defmt::error!("margin: no delay read whole every time"),
    }
    failed += verdict(
        "a reply 30 us after the frame is read whole, 1000 of 1000 [lane prototype]",
        smallest_whole.is_some_and(|d| gap_ns(d, rate) <= 30_000),
    );
    failed += verdict(
        "every other lane read its idle level in every exchange [lane prototype]",
        idle_faults == 0,
    );
    failed += verdict("every exchange came back [lane prototype]", lost == 0);

    // Finding 4, the short low pulse after the frame: the same exchange with
    // a receive, but the responder never armed. A pulse that stays is the
    // lane's own, from its switch to listening; one that goes was the
    // responder's.
    if witness.start(WITNESS_PER_PERIOD as usize * (FRAME + D_MAX + 40)) {
        frames_of(0, lanes, period, &mut store);
        let refs: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
        let outcome = exchange(
            port,
            now,
            per_us,
            &refs[..lanes],
            Some((rx, RX_COUNT)),
            &mut |_, _, _| {},
        );
        if let Some(samples) = witness.wait()
            && let Some(Some(pin)) = board.witnessed.first().copied()
        {
            let mut p = [(0u32, 0u32); FRAME + REPLY.len() + 4];
            let n = pulses(samples, pin, &mut p);
            let shown = n.min(p.len());
            defmt::info!(
                "responder unarmed, {}: {} low pulses on pin {} (the frame has {}); after the frame: {}",
                outcome,
                n,
                pin,
                FRAME,
                p[FRAME.min(shown)..shown]
            );
        }
    }
    span_us("turn-around in code", &first_block.turn, per_us);
    span_us("listen alone", &first_block.listen, per_us);
    span_us("load and play", &first_block.start, per_us);
    span_us("lanes' member run", &first_block.run, per_us);
    span_us("consumer's lock hold (T)", &first_block.hold, per_us);
    span_us("exchange, submit to seen, with receive", &cycle, per_us);

    // Send only: the rate, and two frames back to back.
    let mut send_cycle = Span::default();
    let mut gaps = Span::default();
    let mut last_end: Option<u32> = None;
    for index in 0..EXCHANGES {
        frames_of(index, lanes, period, &mut store);
        let refs: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
        let mut took = Took::default();
        let outcome = exchange(port, now, per_us, &refs[..lanes], None, &mut |_, t, _| {
            took = *t
        });
        if outcome == Some(Outcome::Sent) {
            send_cycle.add(took.seen.wrapping_sub(took.submitted));
            if let Some(end) = last_end {
                gaps.add(took.played.wrapping_sub(end));
            }
            last_end = Some(took.frame_end);
        } else {
            last_end = None;
        }
    }
    span_us("exchange, send only, submit to seen", &send_cycle, per_us);
    span_us("back to back: frame end to next play", &gaps, per_us);
    failed += verdict(
        "1000 send-only exchanges, each sent once [lane prototype]",
        send_cycle.count == EXCHANGES,
    );
    if witness.start(WITNESS_PER_PERIOD as usize * (2 * FRAME + 120)) {
        frames_of(1, lanes, period, &mut store);
        let first: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
        let wr = witness.rate();
        let a = exchange(port, now, per_us, &first[..lanes], None, &mut |_, _, _| {});
        let b = exchange(port, now, per_us, &first[..lanes], None, &mut |_, _, _| {});
        if let (Some(Outcome::Sent), Some(Outcome::Sent), Some(samples), Some(Some(pin))) =
            (a, b, witness.wait(), board.witnessed.first().copied())
        {
            let mut p = [(0u32, 0u32); 2 * FRAME + 4];
            let n = pulses(samples, pin, &mut p);
            if n == 2 * FRAME {
                let captures = p[FRAME].0.saturating_sub(p[FRAME - 1].1);
                defmt::info!(
                    "back to back on the wire: {} ns from the first frame's last edge to the second's first; line idle between",
                    u64::from(captures) * 1_000_000_000 / u64::from(wr.0.max(1))
                );
            }
            failed += verdict(
                "back to back, the wire shows two whole frames and nothing else [lane prototype]",
                n == 2 * FRAME,
            );
        } else {
            failed += verdict(
                "back to back, both frames sent and sampled [lane prototype]",
                false,
            );
        }
    }

    // The longest frame and the longest receive.
    {
        let mut long = [0u32; LONG];
        for (i, s) in long.iter_mut().enumerate() {
            *s = period * if i % 3 == 0 { 12 } else { 6 } / 16;
        }
        let mut refs: [&[u32]; 8] = [&[]; 8];
        refs[0] = &long;
        let sampled = witness.start(WITNESS_PER_PERIOD as usize * (LONG + 120));
        let mut took = Took::default();
        let outcome = exchange(port, now, per_us, &refs[..1], None, &mut |_, t, _| {
            took = *t
        });
        let played_ns =
            u64::from(took.frame_end.wrapping_sub(took.played)) * 1000 / u64::from(per_us);
        defmt::info!(
            "long frame: {} samples, {} ns from play to the end's run, {}",
            LONG,
            played_ns,
            outcome
        );
        failed += verdict(
            "a frame of 500 samples plays once and ends [lane prototype]",
            outcome == Some(Outcome::Sent),
        );
        let wr = witness.rate();
        if sampled && let Some(samples) = witness.wait() {
            let pin = board.witnessed.first().copied().flatten().unwrap_or(0);
            let mismatch = first_mismatch(samples, pin, wr, rate, period, &long);
            if let Some((index, width, want)) = mismatch {
                let mut p = [(0u32, 0u32); LONG + 4];
                let n = pulses(samples, pin, &mut p);
                let from = index.saturating_sub(2).min(n);
                let to = (index + 3).min(n).min(p.len());
                defmt::warn!(
                    "long frame on the wire: pulse {} of {} is {} captures against a sample of {} counts ({} captures a period at {} Hz); {} pulses seen; pulses {} to {}: {}",
                    index,
                    LONG,
                    width,
                    want,
                    wr.0 / rate.0.max(1),
                    wr.0,
                    n,
                    from,
                    to,
                    p[from..to]
                );
            }
            let right = mismatch.is_none();
            failed += verdict(
                "the 500-sample frame reads back whole on the wire [lane prototype]",
                right,
            );
        }
        let short = [period * 4 / 16];
        let none: [&[u32]; 8] = [&short; 8];
        let mut count = 0usize;
        let outcome = exchange(
            port,
            now,
            per_us,
            &none[..lanes],
            Some((rx, LONG_RX)),
            &mut |_, _, s| {
                count = s.lane(0).len();
            },
        );
        defmt::info!(
            "longest receive: {} samples asked, {} read, {}",
            LONG_RX,
            count,
            outcome
        );
        failed += verdict(
            "a receive of 1024 samples on every lane comes back whole [lane prototype]",
            outcome == Some(Outcome::Received) && count == LONG_RX,
        );
    }

    // Faults: cancelled mid-frame and mid-receive; the next exchange works.
    {
        let mut long = [0u32; LONG];
        long.fill(period * 8 / 16);
        let mut refs: [&[u32]; 8] = [&[]; 8];
        refs[0] = &long;
        let mut results = [None; 2];
        for (which, receive) in [(0usize, None), (1, Some((rx, LONG_RX)))] {
            if !port.submit(&refs[..1], receive) {
                continue;
            }
            // Mid-frame: 100 us in. Mid-receive: after the frame (FRAME is
            // 500 samples, ~833 us), 200 us into the receive.
            let wait_us = if which == 0 { 100 } else { 1_033 };
            let _ = spin(now, wait_us * per_us, || false);
            port.cancel();
            let mut got = None;
            let _ = spin(now, 2_000 * per_us, || {
                port.poll(&mut |o, _, _| got = Some(o))
            });
            results[which] = got;
            // The witness watches the 40 periods after the cancel came back,
            // so the line it judges is the one the cancel left.
            let sampled = witness.start(WITNESS_PER_PERIOD as usize * 40);
            if sampled
                && let Some(samples) = witness.wait()
                && let Some(Some(pin)) = board.witnessed.first().copied()
            {
                let tail = samples;
                failed += verdict(
                    if which == 0 {
                        "after a cancel mid-frame, lane 0's line is idle [lane prototype]"
                    } else {
                        "after a cancel mid-receive, lane 0's line is idle [lane prototype]"
                    },
                    idle(tail, pin),
                );
            }
            // The next exchange works.
            frames_of(7, lanes, period, &mut store);
            let next: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
            let mut ok = false;
            let armed = responder.arm(responder_wave(D_MAX, responder.period(), &mut wave));
            let _ = exchange(
                port,
                now,
                per_us,
                &next[..lanes],
                Some((rx, RX_COUNT)),
                &mut |o, _, s| {
                    ok =
                        o == Outcome::Received && whole(s.lane(0), board.pins[0], rx, rate, period);
                },
            );
            let _ = responder.finish();
            failed += verdict(
                if which == 0 {
                    "after a cancel mid-frame, the next exchange reads its reply whole [lane prototype]"
                } else {
                    "after a cancel mid-receive, the next exchange reads its reply whole [lane prototype]"
                },
                armed && ok,
            );
        }
        defmt::info!(
            "cancel mid-frame: {}; mid-receive: {}",
            results[0],
            results[1]
        );
        failed += verdict(
            "a cancel mid-frame and mid-receive each come back aborted [lane prototype]",
            results == [Some(Outcome::Aborted), Some(Outcome::Aborted)],
        );
    }

    // A frame is played exactly once, and a complementary lane after a
    // receive (a finding about FerroHAL, not a verdict on the port).
    if witness.start(WITNESS_PER_PERIOD as usize * (3 * FRAME + 40)) {
        frames_of(3, lanes, period, &mut store);
        let refs: [&[u32]; 8] = core::array::from_fn(|l| &store[l][..]);
        let sent = exchange(port, now, per_us, &refs[..lanes], None, &mut |_, _, _| {});
        let _ = spin(now, 100 * per_us, || false);
        if let (Some(samples), Some(Some(pin))) = (witness.wait(), board.witnessed.first().copied())
        {
            let mut p = [(0u32, 0u32); 3 * FRAME];
            let n = pulses(samples, pin, &mut p);
            failed += verdict(
                "a frame is played exactly once: 16 pulses on the wire in three frames' time [lane prototype]",
                sent == Some(Outcome::Sent) && n == FRAME,
            );
            for (lane, w) in board.witnessed.iter().enumerate().skip(1).take(lanes - 1) {
                if let Some(pin) = w {
                    let n = pulses(samples, *pin, &mut p);
                    let kind = if board.complementary & (1 << lane) != 0 {
                        "complementary"
                    } else {
                        "plain"
                    };
                    defmt::info!(
                        "FINDING lane {} ({}), after receives: {} pulses on its pin of {} sent",
                        lane,
                        kind,
                        n,
                        FRAME
                    );
                }
            }
        }
    }

    failed
}
