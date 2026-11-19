//! The SPI port's checks, shared by every bench that runs them: each bench
//! crate's `lib.rs` includes this file. The checks name only the port's own
//! types, through the handles a consumer holds and the port's shared
//! resource, read under its lock.
//!
//! Two kinds of bus run them. On a loopback (MOSI into MISO) a transfer part
//! comes back as sent and a read part as the filler; `Loop` checks that. On
//! a real device the bench checks the device's answers itself.
//!
//! Each check prints `PASS` or `FAIL` with the requirements it verifies in
//! brackets, and returns 1 for a failure, so a bench can count them.

use ferroforge_stm32::spi_port::{
    ChipSelects, FILLER, Outcome, SpiHandle, SpiPort, SubmitError, Taken,
};
use ferrohal_stm32::spi::Parts;

/// The firmware's clock: CPU cycles.
pub type Now = fn() -> u32;

/// The longest message any check sends on a loopback, and the length of a
/// "long" transfer: at the slowest SCK it holds the bus for over 1.5 ms.
pub const LONG: usize = 64;

/// One part of a message.
#[derive(Clone, Copy)]
pub enum Seg<'a> {
    /// Bytes to send; what comes back is not kept.
    W(&'a [u8]),
    /// Filler bytes to send; what comes back is kept.
    R(usize),
    /// Bytes to send; what comes back is kept.
    T(&'a [u8]),
}

/// Bytes on the wire for `segs`.
pub fn wire_len(segs: &[Seg]) -> usize {
    segs.iter()
        .map(|s| match s {
            Seg::W(b) | Seg::T(b) => b.len(),
            Seg::R(n) => *n,
        })
        .sum()
}

/// Builds a message from `segs` at `priority` and submits it.
pub fn send(h: &mut SpiHandle, priority: u8, segs: &[Seg]) -> Result<(), SubmitError> {
    let mut m = h.message(priority, wire_len(segs))?;
    for s in segs {
        match *s {
            Seg::W(b) => m.write(b),
            Seg::R(n) => m.read(n),
            Seg::T(b) => m.transfer(b),
        };
    }
    m.submit()
}

/// What the checks keep of a message back: its outcome and whether its
/// kept parts, in order, are exactly the expected bytes, compared in place.
#[derive(Clone, Copy)]
pub struct Got {
    pub outcome: Outcome,
    pub matched: bool,
    /// Bytes kept.
    pub kept: usize,
    pub waited: u32,
    pub took: u32,
    pub priority: u8,
}

/// Compares a message back with `expect`, then frees its buffer.
pub fn judge(t: Taken<'_>, expect: &[u8]) -> Got {
    let mut at = 0;
    let mut matched = true;
    for i in 0..t.parts() {
        let part = t.read(i);
        matched &= expect.get(at..at + part.len()) == Some(part);
        at += part.len();
    }
    matched &= at == expect.len();
    Got {
        outcome: t.outcome,
        matched,
        kept: at,
        waited: t.waited,
        took: t.took,
        priority: t.priority,
    }
}

/// The oldest message back, judged against `expect`.
pub fn take(h: &mut SpiHandle, expect: &[u8]) -> Option<Got> {
    h.take().map(|t| judge(t, expect))
}

/// The next message back, waiting for the port's wake.
pub async fn next(h: &mut SpiHandle, expect: &[u8]) -> Got {
    loop {
        if let Some(r) = take(h, expect) {
            return r;
        }
        h.wait().await;
    }
}

/// The next message back, polled for at most `ticks`: for checks where a
/// message that never comes back must fail, not hang.
pub fn poll(h: &mut SpiHandle, expect: &[u8], now: Now, ticks: u32) -> Option<Got> {
    let start = now();
    loop {
        if let Some(r) = take(h, expect) {
            return Some(r);
        }
        if now().wrapping_sub(start) > ticks {
            return None;
        }
    }
}

/// What a bench reads of the port, under its lock.
pub trait View {
    fn health(&mut self) -> ferroforge_stm32::spi_port::Health;
    fn bus(&mut self) -> ferrohal_stm32::spi::Health;
    fn selected(&mut self, device: usize) -> Option<bool>;
    fn devices(&mut self) -> usize;
    fn frequency(&mut self, device: usize) -> u32;
    fn vectors_enabled(&mut self) -> bool;
}

impl<M, W, Cs> View for M
where
    M: rtic::Mutex<T = SpiPort<W, Cs>>,
    W: Parts,
    Cs: ChipSelects,
{
    fn health(&mut self) -> ferroforge_stm32::spi_port::Health {
        self.lock(|p| p.health())
    }
    fn bus(&mut self) -> ferrohal_stm32::spi::Health {
        self.lock(|p| p.spi().health())
    }
    fn selected(&mut self, device: usize) -> Option<bool> {
        self.lock(|p| p.is_selected(device))
    }
    fn devices(&mut self) -> usize {
        Cs::COUNT
    }
    fn frequency(&mut self, device: usize) -> u32 {
        self.lock(|p| p.frequency(device).map_or(0, |f| f.0))
    }
    fn vectors_enabled(&mut self) -> bool {
        self.lock(|p| p.spi().vectors().verify().is_ok())
    }
}

/// Cuts `pool` into lent buffers: `(device, length)` in order, as the
/// constructor takes them. Stops early if the pool runs out, which the
/// constructor's own count then shows.
pub fn carve(pool: &'static mut [u8], sizes: &'static [(usize, usize)]) -> Carve {
    Carve {
        pool,
        sizes: sizes.iter(),
    }
}

pub struct Carve {
    pool: &'static mut [u8],
    sizes: core::slice::Iter<'static, (usize, usize)>,
}

impl Iterator for Carve {
    type Item = (usize, &'static mut [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        let &(device, len) = self.sizes.next()?;
        let pool = core::mem::take(&mut self.pool);
        if len > pool.len() {
            return None;
        }
        let (buf, rest) = pool.split_at_mut(len);
        self.pool = rest;
        Some((device, buf))
    }
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

/// Four bytes that differ from one tag to the next.
pub fn tagged(tag: u32) -> [u8; 4] {
    let b = tag.to_le_bytes();
    [b[0], b[1] ^ 0x5A, b[2], b[3] ^ 0xA5]
}

/// Minimum, maximum and sum of a series.
#[derive(Clone, Copy, Default)]
pub struct Spread {
    pub min: u32,
    pub max: u32,
    pub sum: u64,
    pub n: u32,
}

impl Spread {
    pub fn add(&mut self, v: u32) {
        if self.n == 0 || v < self.min {
            self.min = v;
        }
        self.max = self.max.max(v);
        self.sum += u64::from(v);
        self.n += 1;
    }
    pub fn mean(&self) -> u32 {
        if self.n == 0 {
            0
        } else {
            (self.sum / u64::from(self.n)) as u32
        }
    }
}

/// `n` loopback transfers of `len` bytes one at a time, each awaited:
/// submit to start (`waited`, stamped by the port) and submit to the
/// message in the consumer's hands, in cycles. Returns (to start, to
/// result, wrong).
async fn timed(h: &mut SpiHandle, now: Now, n: u32, len: usize) -> (Spread, Spread, u32) {
    let mut to_start = Spread::default();
    let mut to_result = Spread::default();
    let mut wrong = 0u32;
    let mut bytes = [0u8; 256];
    let len = len.min(bytes.len());
    for i in 0..n {
        for (k, chunk) in bytes[..len].chunks_mut(4).enumerate() {
            let t = tagged(i.wrapping_mul(64).wrapping_add(k as u32));
            chunk.copy_from_slice(&t[..chunk.len()]);
        }
        let expect = &bytes[..len];
        let Ok(mut m) = h.message(0, len) else {
            wrong += 1;
            continue;
        };
        m.transfer(expect);
        let t0 = now();
        if m.submit().is_err() {
            wrong += 1;
            continue;
        }
        let r = next(h, expect).await;
        let t1 = now();
        if r.outcome != Outcome::Done || !r.matched {
            wrong += 1;
        }
        to_start.add(r.waited);
        to_result.add(t1.wrapping_sub(t0));
    }
    (to_start, to_result, wrong)
}

/// 100 loopback transfers at each of `lens`, in increasing order, run
/// before any other check, so the port's longest run so far covers that
/// length and the shorter ones.
pub async fn length(h: &mut SpiHandle, view: &mut impl View, now: Now, lens: &[usize]) -> u32 {
    let mut failed = 0;
    for &len in lens {
        let (to_start, to_result, wrong) = timed(h, now, 100, len).await;
        let run = view.health().longest_run;
        defmt::info!(
            "length {} bytes, cycles over 100: submit to start min {} mean {} max {}; submit to result min {} mean {} max {}; longest port run so far {}",
            len,
            to_start.min,
            to_start.mean(),
            to_start.max,
            to_result.min,
            to_result.mean(),
            to_result.max,
            run
        );
        failed += verdict(
            "length: every message came back done with the bytes clocked in [PORT-201]",
            wrong == 0 && to_result.n == 100,
        );
    }
    failed
}

/// `n` 4-byte loopback transfers, timed.
pub async fn latency(h: &mut SpiHandle, now: Now, n: u32) -> u32 {
    let (to_start, to_result, wrong) = timed(h, now, n, 4).await;
    defmt::info!(
        "latency over {} messages of 4 bytes, cycles: submit to start min {} mean {} max {}; submit to result taken min {} mean {} max {}",
        n,
        to_start.min,
        to_start.mean(),
        to_start.max,
        to_result.min,
        to_result.mean(),
        to_result.max
    );
    verdict(
        "latency: every message came back done, in order, with its bytes [PORT-201]",
        wrong == 0 && to_result.n == n,
    )
}

/// `n` probes on `bus` one at a time, each awaited and timed.
pub async fn latency_probe(h: &mut SpiHandle, bus: Bus, now: Now, n: u32) -> u32 {
    let mut to_start = Spread::default();
    let mut to_result = Spread::default();
    let mut wrong = 0u32;
    for i in 0..n {
        let t0 = now();
        if probe(h, bus, 0, i).is_err() {
            wrong += 1;
            continue;
        }
        let r = next_probe(h, bus, i).await;
        if r.outcome != Outcome::Done || !r.matched {
            wrong += 1;
        }
        to_start.add(r.waited);
        to_result.add(now().wrapping_sub(t0));
    }
    defmt::info!(
        "latency over {} probes, cycles: submit to start min {} mean {} max {}; submit to result taken min {} mean {} max {}",
        n,
        to_start.min,
        to_start.mean(),
        to_start.max,
        to_result.min,
        to_result.mean(),
        to_result.max
    );
    verdict(
        "latency: every message came back done, in order, with the data expected [PORT-201]",
        wrong == 0 && to_result.n == n,
    )
}

/// Write, write, read, transfer in one message on a loopback: only the
/// read and transfer parts come back, in place; a write-only message comes
/// back done with nothing kept.
pub async fn multipart(h: &mut SpiHandle) -> u32 {
    let segs = [Seg::W(&[0xA0]), Seg::W(&[1, 2]), Seg::R(3), Seg::T(&[7, 8])];
    let wire = wire_len(&segs);
    let expected = [FILLER, FILLER, FILLER, 7, 8];
    let sent = send(h, 0, &segs);
    let r = next(h, &expected).await;
    defmt::info!(
        "multi-part: {} bytes on the wire, {} kept, as expected {}",
        wire,
        r.kept,
        r.matched
    );
    let sent2 = send(h, 0, &[Seg::W(&[1, 2, 3])]);
    let r2 = next(h, &[]).await;
    verdict(
        "multi-part: one message, only the read and transfer parts kept, in place [PORT-209]",
        sent.is_ok() && wire == 8 && r.outcome == Outcome::Done && r.matched,
    ) + verdict(
        "write-only: done, nothing kept [PORT-209]",
        sent2.is_ok() && r2.outcome == Outcome::Done && r2.kept == 0,
    )
}

/// A long transfer on `h`'s device: its line low and every other high
/// while it runs, all high once it is back.
pub async fn chip_select(h: &mut SpiHandle, view: &mut impl View) -> u32 {
    let d = usize::from(h.device());
    let sent = send(h, 0, &[Seg::W(&[0x55; LONG])]);
    // The port runs above this task, so the transfer started inside submit.
    let own = view.selected(d);
    let devices = view.devices();
    let others = (0..devices).all(|o| o == d || view.selected(o) == Some(false));
    let r = next(h, &[]).await;
    let after = (0..devices).all(|o| view.selected(o) == Some(false));
    defmt::info!(
        "chip select on device {}: while running own {} others high {}; once back, all high {}",
        d,
        own,
        others,
        after
    );
    verdict(
        "chip select: own line low and no other while running, all high once back [PORT-202]",
        sent.is_ok() && own == Some(true) && others && after && r.outcome == Outcome::Done,
    )
}

/// What is on a device's bus, which decides what a probe message is and
/// what comes back from it.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Bus {
    /// MOSI into MISO: a four-byte transfer comes back as sent.
    Loop,
    /// A real device: a one-byte register read, the read bit and the
    /// register, then one byte that must be `answer`.
    Register { register: u8, answer: u8 },
    /// Nothing answers: a four-byte write, nothing kept.
    Blind,
}

/// Submits probe `tag` on `bus` at `priority`.
pub fn probe(h: &mut SpiHandle, bus: Bus, priority: u8, tag: u32) -> Result<(), SubmitError> {
    let bytes = tagged(tag);
    match bus {
        Bus::Loop => send(h, priority, &[Seg::T(&bytes)]),
        Bus::Register { register, .. } => send(h, priority, &[Seg::W(&[0x80 | register]), Seg::R(1)]),
        Bus::Blind => send(h, priority, &[Seg::W(&bytes)]),
    }
}

/// What probe `tag` on `bus` keeps: the bytes and their count.
pub fn expected(bus: Bus, tag: u32) -> ([u8; 4], usize) {
    match bus {
        Bus::Loop => (tagged(tag), 4),
        Bus::Register { answer, .. } => ([answer, 0, 0, 0], 1),
        Bus::Blind => ([0; 4], 0),
    }
}

/// The next message back on `h`, judged as probe `tag` on `bus`.
pub async fn next_probe(h: &mut SpiHandle, bus: Bus, tag: u32) -> Got {
    let (e, n) = expected(bus, tag);
    next(h, &e[..n]).await
}

/// The next message back on `h`, polled, judged as probe `tag` on `bus`.
pub fn poll_probe(h: &mut SpiHandle, bus: Bus, tag: u32, now: Now, ticks: u32) -> Option<Got> {
    let (e, n) = expected(bus, tag);
    poll(h, &e[..n], now, ticks)
}

/// One long transfer on the wire, then probes until the device's buffers
/// are all out: the next is refused `Outstanding`, nothing lent is lost,
/// and a message runs after.
pub async fn full(h: &mut SpiHandle, bus: Bus, now: Now) -> u32 {
    let first = send(h, 0, &[Seg::W(&[0x33; LONG])]);
    let mut accepted = 0u32;
    let mut refused = None;
    for i in 0..64u32 {
        match probe(h, bus, 0, 1000 + i) {
            Ok(()) => accepted += 1,
            Err(e) => {
                refused = Some(e);
                break;
            }
        }
    }
    let mut good = 0u32;
    let mut last_waited = 0u32;
    let r = next(h, &[]).await;
    good += u32::from(first.is_ok() && r.outcome == Outcome::Done);
    for i in 0..accepted {
        let r = next_probe(h, bus, 1000 + i).await;
        if r.outcome == Outcome::Done && r.matched {
            good += 1;
        }
        last_waited = r.waited;
    }
    let after = probe(h, bus, 0, 9).is_ok()
        && poll_probe(h, bus, 9, now, 100_000_000).is_some_and(|r| r.matched);
    let expected = u32::from(h.buffers()) - 1;
    defmt::info!(
        "full: {} lent behind a running transfer, then refused {}; {} of {} came back done and in order; the last waited {} cycles",
        accepted,
        named(refused),
        good,
        accepted + 1,
        last_waited
    );
    verdict(
        "full: refused Outstanding with every buffer out, nothing lent lost, usable after [PORT-205]",
        accepted == expected
            && refused == Some(SubmitError::Outstanding)
            && good == accepted + 1
            && after,
    )
}

/// A long low-priority transfer running, then two medium probes and one
/// high, in that order. `roles` picks the handle in `hs` for low, medium,
/// medium and high, and `buses` says what is on each handle's bus.
/// `across`: the expected end order is priority across devices (low, high,
/// medium, medium); otherwise submission order within one device (low,
/// medium, medium, high). The mediums are told apart by the order they come
/// back on their device.
pub fn priority(
    hs: &mut [&mut SpiHandle],
    buses: &[Bus],
    roles: [usize; 4],
    now: Now,
    across: bool,
) -> u32 {
    let plan: [u8; 4] = [1, 5, 5, 9];
    let mut stamps = [0u32; 4];
    let mut sent = true;
    for (k, &p) in plan.iter().enumerate() {
        let (Some(h), Some(&bus)) = (hs.get_mut(roles[k]), buses.get(roles[k])) else {
            return verdict("priority: handles", false);
        };
        stamps[k] = now();
        sent &= if k == 0 {
            send(h, p, &[Seg::W(&[0x11; LONG])])
        } else {
            probe(h, bus, p, 2000 + k as u32)
        }
        .is_ok();
    }
    let mut ends = [0u32; 4];
    let mut seen = [false; 4];
    let mut waited_high = 0;
    let mut ok = true;
    let start = now();
    while seen.iter().any(|s| !s) && now().wrapping_sub(start) < 200_000_000 {
        for (i, h) in hs.iter_mut().enumerate() {
            let Some(t) = h.take() else {
                continue;
            };
            let k = match t.priority {
                1 => 0,
                9 => 3,
                _ if !seen[1] => 1,
                _ => 2,
            };
            let bus = buses.get(i).copied().unwrap_or(Bus::Blind);
            let r = if k == 0 {
                judge(t, &[])
            } else {
                let (e, n) = expected(bus, 2000 + k as u32);
                judge(t, &e[..n])
            };
            ok &= r.outcome == Outcome::Done && r.matched && !seen[k];
            seen[k] = true;
            if k == 3 {
                waited_high = r.waited;
            }
            // The end, on the consumer's clock: submit + waited + took.
            ends[k] = stamps[k].wrapping_add(r.waited).wrapping_add(r.took);
        }
    }
    let since = |k: usize| ends[k].wrapping_sub(stamps[0]) as i32;
    let order_ok = if across {
        since(0) < since(3) && since(3) < since(1) && since(1) < since(2)
    } else {
        since(0) < since(1) && since(1) < since(2) && since(2) < since(3)
    };
    defmt::info!(
        "priority: ends in cycles from the low's submit: low {} high {} medium {} medium {}; the high waited {} cycles",
        since(0),
        since(3),
        since(1),
        since(2),
        waited_high
    );
    verdict(
        if across {
            "priority across devices: low (running), then high, then the mediums as submitted [PORT-204]"
        } else {
            "order within one device: as submitted (low, medium, medium, high) [PORT-203]"
        },
        sent && ok && seen.iter().all(|s| *s) && order_ok,
    )
}

/// Two devices with different settings, alternating 100 times: every
/// probe comes back intact, each device's time on the wire fits its own
/// SCK, and the bus switched at every change of device. `bits` is the
/// probe's length on the wire on each bus.
pub async fn switch(
    a: &mut SpiHandle,
    a_bus: Bus,
    b: &mut SpiHandle,
    b_bus: Bus,
    view: &mut impl View,
    cpu_hz: u32,
) -> u32 {
    let before = view.health().switches;
    let mut took = [u32::MAX; 2];
    let mut wrong = 0u32;
    for i in 0..100u32 {
        for (k, (h, bus)) in [(&mut *a, a_bus), (&mut *b, b_bus)].into_iter().enumerate() {
            let sent = probe(h, bus, 0, 3000 + i);
            let r = next_probe(h, bus, 3000 + i).await;
            if sent.is_err() || r.outcome != Outcome::Done || !r.matched {
                wrong += 1;
            }
            took[k] = took[k].min(r.took);
        }
    }
    let health = view.health();
    let switches = health.switches - before;
    let bits = |bus: Bus| match bus {
        Bus::Register { .. } => 16u64,
        _ => 32,
    };
    // Bits on the wire over the time taken: an upper bound on the SCK.
    let rate = |t: u32, bus: Bus| (bits(bus) * u64::from(cpu_hz) / u64::from(t.max(1))) as u32;
    let (da, db) = (usize::from(a.device()), usize::from(b.device()));
    let (sa, sb) = (view.frequency(da), view.frequency(db));
    let (ra, rb) = (rate(took[0], a_bus), rate(took[1], b_bus));
    defmt::info!(
        "switch: {} switches in 200 alternating messages, longest {} cycles; device {} SCK {} Hz ({} bit/s on the wire), device {} SCK {} Hz ({} bit/s)",
        switches,
        health.longest_switch,
        da,
        sa,
        ra,
        db,
        sb,
        rb
    );
    // The wire's rate is below the SCK (the gaps between bytes) and above
    // a quarter of it (the port's start and end inside `took`).
    let fits = |sck: u32, r: u32| r <= sck && r > sck / 4;
    verdict(
        "switch: each device's own settings in force for its messages, data intact [PORT-208]",
        // 200 alternating messages change device at least 199 times.
        wrong == 0 && switches >= 199 && fits(sa, ra) && fits(sb, rb),
    )
}

/// A transfer running and `queued` probes behind it on `h`, one probe on
/// `other`; then `h.cancel()` and one more probe on `h` straight after:
/// every message from before the cancel comes back aborted with the line
/// high, the one after runs, and `other`'s is untouched. `h` needs
/// `queued + 2` buffers.
#[allow(clippy::too_many_arguments)]
pub async fn cancel(
    h: &mut SpiHandle,
    bus: Bus,
    other: &mut SpiHandle,
    other_bus: Bus,
    view: &mut impl View,
    now: Now,
    queued: u32,
) -> u32 {
    let d = usize::from(h.device());
    let mut sent = send(h, 0, &[Seg::W(&[0x77; LONG])]).is_ok();
    for i in 0..queued {
        sent &= probe(h, bus, 0, 5000 + i).is_ok();
    }
    sent &= probe(other, other_bus, 0, 5100).is_ok();
    let running = view.selected(d) == Some(true);
    h.cancel();
    let after_sent = probe(h, bus, 0, 5200).is_ok();
    let mut aborted = 0u32;
    for _ in 0..=queued {
        if poll(h, &[], now, 50_000_000).is_some_and(|r| r.outcome == Outcome::Aborted) {
            aborted += 1;
        }
    }
    let after = after_sent
        && poll_probe(h, bus, 5200, now, 50_000_000)
            .is_some_and(|r| r.outcome == Outcome::Done && r.matched);
    let released = view.selected(d) == Some(false);
    let other_ok = poll_probe(other, other_bus, 5100, now, 50_000_000)
        .is_some_and(|r| r.outcome == Outcome::Done && r.matched);
    defmt::info!(
        "cancel: running {}; {} of {} aborted; line high after {}; the other device's done {}; the message submitted after the cancel done {}",
        running,
        aborted,
        queued + 1,
        released,
        other_ok,
        after
    );
    verdict(
        "cancel: running and queued aborted, chip select high, the other device untouched, the next message runs [PORT-206]",
        sent && running && aborted == queued + 1 && released && other_ok && after && h.outstanding() == 0,
    )
}

/// Device `a` holds every buffer it has, behind a long transfer on the
/// wire; then device `b` submits a probe, which is accepted and comes back.
pub async fn isolation(a: &mut SpiHandle, a_bus: Bus, b: &mut SpiHandle, b_bus: Bus, now: Now) -> u32 {
    let first = send(a, 0, &[Seg::W(&[0x44; LONG])]);
    let mut held = 0u32;
    let mut a_refused = None;
    for i in 0..64u32 {
        match probe(a, a_bus, 0, 6000 + i) {
            Ok(()) => held += 1,
            Err(e) => {
                a_refused = Some(e);
                break;
            }
        }
    }
    let b_sent = probe(b, b_bus, 0, 6100);
    let mut a_good = u32::from(
        first.is_ok() && poll(a, &[], now, 50_000_000).is_some_and(|r| r.outcome == Outcome::Done),
    );
    for i in 0..held {
        if poll_probe(a, a_bus, 6000 + i, now, 50_000_000)
            .is_some_and(|r| r.outcome == Outcome::Done && r.matched)
        {
            a_good += 1;
        }
    }
    let b_back = b_sent.is_ok()
        && poll_probe(b, b_bus, 6100, now, 50_000_000).is_some_and(|r| r.matched);
    defmt::info!(
        "isolation: device {} held {} behind a running transfer, then refused {}; device {} then submitted: {}; back: device {} {} of {}, device {} {}",
        a.device(),
        held,
        named(a_refused),
        b.device(),
        named(b_sent.err()),
        a.device(),
        a_good,
        held + 1,
        b.device(),
        b_back
    );
    verdict(
        "isolation: a device with every buffer out does not stop another's message [PORT-205]",
        b_back && a_refused == Some(SubmitError::Outstanding) && a_good == held + 1,
    )
}

/// A refusal's name for the log; "none" when accepted.
pub fn named(e: Option<SubmitError>) -> &'static str {
    match e {
        None => "none (accepted)",
        Some(SubmitError::Full) => "Full",
        Some(SubmitError::TooLong) => "TooLong",
        Some(SubmitError::Parts) => "Parts",
        Some(SubmitError::Empty) => "Empty",
        Some(SubmitError::Outstanding) => "Outstanding",
    }
}

/// Keeps up to three probes outstanding on `h` for `n` messages, checking
/// each. Run from tasks at different priorities at once, it submits from
/// preempting contexts. Returns (done, wrong, refused and retried).
pub async fn stress(h: &mut SpiHandle, bus: Bus, n: u32, tag: u32) -> (u32, u32, u32) {
    let (mut sent, mut done, mut wrong, mut retried) = (0u32, 0u32, 0u32, 0u32);
    while done < n {
        while sent < n && h.outstanding() < 3 {
            match probe(h, bus, (tag & 7) as u8, tag * 100_000 + sent) {
                Ok(()) => sent += 1,
                Err(_) => {
                    retried += 1;
                    break;
                }
            }
        }
        let r = next_probe(h, bus, tag * 100_000 + done).await;
        if r.outcome != Outcome::Done || !r.matched {
            wrong += 1;
        }
        done += 1;
    }
    (done, wrong, retried)
}

/// What every bench checks last: the port's and FerroHAL's own counts.
pub fn closing(view: &mut impl View) -> u32 {
    let h = view.health();
    let bus = view.bus();
    defmt::info!(
        "port: longest run {} cycles; switches {} longest {} cycles; lost {}; FerroHAL transfers {} dma_errors {} overruns {} mode_faults {} aborts {}",
        h.longest_run,
        h.switches,
        h.longest_switch,
        h.lost,
        bus.transfers,
        bus.dma_errors,
        bus.overruns,
        bus.mode_faults,
        bus.aborts
    );
    verdict(
        "port: no message lost, no bus fault [PORT-201]",
        h.lost == 0 && bus.dma_errors == 0 && bus.overruns == 0 && bus.mode_faults == 0,
    )
}
