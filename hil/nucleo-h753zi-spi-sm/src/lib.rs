//! PROTOTYPE: the consumer side of the SPI port comparison, shared by every
//! image of the bench (S, M with one device and with four, R with one
//! device and with four). The checks name only a [`Device`], which both
//! consumer types implement: the first prototype's `SpiHandle` and the
//! second's `LentHandle`. The port is read through [`View`] for the
//! measurements a consumer would not make.
//!
//! The loop is the board's: MOSI into MISO, so a transfer part comes back
//! as sent and a read part as the filler.

#![no_std]

use core::future::Future;

use ferroforge_stm32::spi_lent::{LentHandle, LentPort};
use ferroforge_stm32::spi_queue::{
    ChipSelects, FILLER, MAX, Outcome, Port, SpiHandle, Stats, SubmitError,
};
use ferrohal_stm32::spi::{Health, Parts};

/// The firmware's clock: CPU cycles.
pub type Now = fn() -> u32;

/// One part of a transaction.
#[derive(Clone, Copy)]
pub enum Seg<'a> {
    /// Bytes to send; what comes back is dropped.
    W(&'a [u8]),
    /// Filler bytes to send; what comes back is kept.
    R(usize),
    /// Bytes to send; what comes back is kept.
    T(&'a [u8]),
}

fn wire_len(segs: &[Seg]) -> usize {
    segs.iter()
        .map(|s| match s {
            Seg::W(b) | Seg::T(b) => b.len(),
            Seg::R(n) => *n,
        })
        .sum()
}

/// What the checks keep of a result: the consumer's handling of it is the
/// comparison against what it expected, in place.
#[derive(Clone, Copy)]
pub struct Got {
    pub outcome: Outcome,
    /// The kept parts, in order, are exactly the expected bytes.
    pub matched: bool,
    /// Bytes kept.
    pub kept: usize,
    pub waited: u32,
    pub took: u32,
    pub priority: u8,
}

/// The consumer's side of one SPI device, as the checks use it.
pub trait Device {
    /// A transaction built and not yet submitted.
    type Built<'a>
    where
        Self: 'a;
    /// Builds a transaction from `segs` at `priority`.
    fn build<'a>(&'a mut self, priority: u8, segs: &[Seg]) -> Self::Built<'a>;
    /// Submits what `build` made.
    fn submit(built: Self::Built<'_>) -> Result<(), SubmitError>;
    /// The oldest result, compared with `expect`, and released.
    fn take(&mut self, expect: &[u8]) -> Option<Got>;
    fn wait(&mut self) -> impl Future<Output = ()>;
    fn cancel(&mut self);
    fn outstanding(&self) -> u8;
    fn device(&self) -> u8;

    fn send(&mut self, priority: u8, segs: &[Seg]) -> Result<(), SubmitError> {
        let built = self.build(priority, segs);
        Self::submit(built)
    }
}

impl Device for SpiHandle {
    type Built<'a> = (&'a mut SpiHandle, ferroforge_stm32::spi_queue::Transaction);

    fn build<'a>(&'a mut self, priority: u8, segs: &[Seg]) -> Self::Built<'a> {
        let mut t = self.transaction(priority);
        for s in segs {
            match *s {
                Seg::W(b) => t.write(b),
                Seg::R(n) => t.read(n),
                Seg::T(b) => t.transfer(b),
            };
        }
        (self, t)
    }

    fn submit((h, t): Self::Built<'_>) -> Result<(), SubmitError> {
        h.submit(t)
    }

    fn take(&mut self, expect: &[u8]) -> Option<Got> {
        let r = SpiHandle::take(self)?;
        Some(Got {
            outcome: r.outcome,
            matched: r.bytes() == expect,
            kept: r.bytes().len(),
            waited: r.waited,
            took: r.took,
            priority: r.priority,
        })
    }

    async fn wait(&mut self) {
        SpiHandle::wait(self).await
    }

    fn cancel(&mut self) {
        SpiHandle::cancel(self)
    }

    fn outstanding(&self) -> u8 {
        SpiHandle::outstanding(self)
    }

    fn device(&self) -> u8 {
        SpiHandle::device(self)
    }
}

impl Device for LentHandle {
    type Built<'a> = Result<ferroforge_stm32::spi_lent::Message<'a>, SubmitError>;

    fn build<'a>(&'a mut self, priority: u8, segs: &[Seg]) -> Self::Built<'a> {
        let mut m = self.message(priority, wire_len(segs))?;
        for s in segs {
            match *s {
                Seg::W(b) => m.write(b),
                Seg::R(n) => m.read(n),
                Seg::T(b) => m.transfer(b),
            };
        }
        Ok(m)
    }

    fn submit(built: Self::Built<'_>) -> Result<(), SubmitError> {
        built?.submit()
    }

    fn take(&mut self, expect: &[u8]) -> Option<Got> {
        let t = LentHandle::take(self)?;
        // In place: each kept part against its slice of `expect`.
        let mut at = 0;
        let mut matched = true;
        for i in 0..t.parts() {
            let part = t.read(i);
            matched &= expect.get(at..at + part.len()) == Some(part);
            at += part.len();
        }
        matched &= at == expect.len();
        Some(Got {
            outcome: t.outcome,
            matched,
            kept: at,
            waited: t.waited,
            took: t.took,
            priority: t.priority,
        })
    }

    async fn wait(&mut self) {
        LentHandle::wait(self).await
    }

    fn cancel(&mut self) {
        LentHandle::cancel(self)
    }

    fn outstanding(&self) -> u8 {
        LentHandle::outstanding(self)
    }

    fn device(&self) -> u8 {
        LentHandle::device(self)
    }
}

/// What the bench reads of either port.
pub trait PortView {
    fn stats(&self) -> Stats;
    /// M's queue critical sections; zero where there is none.
    fn queue_longest(&self) -> (u32, u32);
    fn selected(&mut self, device: usize) -> Option<bool>;
    fn health(&self) -> Health;
    fn devices(&self) -> usize;
    /// R's longest `start` and copy back since the last clear; None for M.
    fn copies(&self) -> Option<(u32, u32)>;
    fn clear_copies(&mut self);
}

impl<W: Parts, Cs: ChipSelects> PortView for Port<W, Cs> {
    fn stats(&self) -> Stats {
        Port::stats(self)
    }
    fn queue_longest(&self) -> (u32, u32) {
        Port::queue_longest(self)
    }
    fn selected(&mut self, device: usize) -> Option<bool> {
        self.is_selected(device)
    }
    fn health(&self) -> Health {
        self.spi().health()
    }
    fn devices(&self) -> usize {
        Cs::COUNT
    }
    fn copies(&self) -> Option<(u32, u32)> {
        None
    }
    fn clear_copies(&mut self) {}
}

impl<W: Parts, Cs: ChipSelects> PortView for LentPort<W, Cs> {
    fn stats(&self) -> Stats {
        LentPort::stats(self)
    }
    fn queue_longest(&self) -> (u32, u32) {
        (0, 0)
    }
    fn selected(&mut self, device: usize) -> Option<bool> {
        self.is_selected(device)
    }
    fn health(&self) -> Health {
        self.spi().health()
    }
    fn devices(&self) -> usize {
        Cs::COUNT
    }
    fn copies(&self) -> Option<(u32, u32)> {
        Some(LentPort::copies(self))
    }
    fn clear_copies(&mut self) {
        LentPort::clear_copies(self)
    }
}

/// What the bench reads of a port, under its lock.
pub trait View {
    fn stats(&mut self) -> Stats;
    fn queue_longest(&mut self) -> (u32, u32);
    fn selected(&mut self, device: usize) -> Option<bool>;
    fn health(&mut self) -> Health;
    fn devices(&mut self) -> usize;
    fn copies(&mut self) -> Option<(u32, u32)>;
    fn clear_copies(&mut self);
}

impl<M, P> View for M
where
    M: rtic::Mutex<T = P>,
    P: PortView,
{
    fn stats(&mut self) -> Stats {
        self.lock(|p| p.stats())
    }
    fn queue_longest(&mut self) -> (u32, u32) {
        self.lock(|p| p.queue_longest())
    }
    fn selected(&mut self, device: usize) -> Option<bool> {
        self.lock(|p| p.selected(device))
    }
    fn health(&mut self) -> Health {
        self.lock(|p| p.health())
    }
    fn devices(&mut self) -> usize {
        self.lock(|p| p.devices())
    }
    fn copies(&mut self) -> Option<(u32, u32)> {
        self.lock(|p| p.copies())
    }
    fn clear_copies(&mut self) {
        self.lock(|p| p.clear_copies())
    }
}

/// Cuts `pool` into lent buffers: `(device, length)` in order, as R's
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

/// The next result, waiting for the port's wake.
pub async fn next<H: Device>(h: &mut H, expect: &[u8]) -> Got {
    loop {
        if let Some(r) = h.take(expect) {
            return r;
        }
        h.wait().await;
    }
}

/// The next result, polled for at most `ticks`: for checks where a result
/// that never comes must fail, not hang.
pub fn poll<H: Device>(h: &mut H, expect: &[u8], now: Now, ticks: u32) -> Option<Got> {
    let start = now();
    loop {
        if let Some(r) = h.take(expect) {
            return Some(r);
        }
        if now().wrapping_sub(start) > ticks {
            return None;
        }
    }
}

/// The same bytes back, for a transfer part sent round the loop.
fn tagged(tag: u32) -> [u8; 4] {
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

/// `n` transfers of `len` bytes one at a time, each awaited: submit to
/// start (`waited`, stamped by the port) and submit to the result in the
/// consumer's hands, in cycles. The clock starts after the transaction is
/// built, as at the first prototype. Returns (to start, to result, wrong).
async fn timed<H: Device>(h: &mut H, now: Now, n: u32, len: usize) -> (Spread, Spread, u32) {
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
        let built = h.build(0, &[Seg::T(expect)]);
        let t0 = now();
        if H::submit(built).is_err() {
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

/// `n` 4-byte transfers: the first prototype's latency check.
pub async fn latency<H: Device>(variant: &str, h: &mut H, now: Now, n: u32) -> u32 {
    let (to_start, to_result, wrong) = timed(h, now, n, 4).await;
    defmt::info!(
        "{} latency over {} transfers of 4 bytes, cycles: submit to start min {} mean {} max {}; submit to result taken min {} mean {} max {}",
        variant,
        n,
        to_start.min,
        to_start.mean(),
        to_start.max,
        to_result.min,
        to_result.mean(),
        to_result.max
    );
    verdict(
        "latency: every transfer came back, in order, with its bytes",
        wrong == 0 && to_result.n == n,
    )
}

/// 100 transfers at each of `lens`, in increasing order, run before any
/// other check: the two timings, the port's longest run so far (a maximum
/// since boot, so at each length it covers that length and the shorter
/// ones) and, on R, the longest `start` and copy back at that length.
pub async fn length<H: Device>(
    variant: &str,
    h: &mut H,
    view: &mut impl View,
    now: Now,
    lens: &[usize],
) -> u32 {
    let mut failed = 0;
    for &len in lens {
        view.clear_copies();
        let (to_start, to_result, wrong) = timed(h, now, 100, len).await;
        let service = view.stats().longest_service;
        let (start, back) = view.copies().unwrap_or((0, 0));
        defmt::info!(
            "{} length {} bytes, cycles over 100: submit to start min {} mean {} max {}; submit to result min {} mean {} max {}; longest port run so far {}; longest start {} copy back {} (0: not measured)",
            variant,
            len,
            to_start.min,
            to_start.mean(),
            to_start.max,
            to_result.min,
            to_result.mean(),
            to_result.max,
            service,
            start,
            back
        );
        failed += verdict(
            "length: every transfer came back with its bytes",
            wrong == 0 && to_result.n == 100,
        );
    }
    failed
}

/// Write, write, read, transfer in one transaction: only the read and
/// transfer parts come back, in place.
pub async fn multipart<H: Device>(h: &mut H) -> u32 {
    let segs = [Seg::W(&[0xA0]), Seg::W(&[1, 2]), Seg::R(3), Seg::T(&[7, 8])];
    let wire = wire_len(&segs);
    let expected = [FILLER, FILLER, FILLER, 7, 8];
    let _ = h.send(0, &segs);
    let r = next(h, &expected).await;
    defmt::info!(
        "multi-part: {} bytes on the wire, {} came back, as expected {}",
        wire,
        r.kept,
        r.matched
    );
    let _ = h.send(0, &[Seg::W(&[1, 2, 3])]);
    let r2 = next(h, &[]).await;
    verdict(
        "multi-part: one transaction, only the read and transfer parts returned",
        wire == 8 && r.outcome == Outcome::Done && r.matched,
    ) + verdict(
        "write-only: done, nothing returned",
        r2.outcome == Outcome::Done && r2.kept == 0,
    )
}

/// A 64-byte transfer on `device`: its line low and every other high while
/// it runs, all high after.
pub async fn chip_select<H: Device>(h: &mut H, view: &mut impl View) -> u32 {
    let d = usize::from(h.device());
    let _ = h.send(0, &[Seg::W(&[0x55; MAX])]);
    // The port runs above this task, so the transfer started inside submit;
    // 64 bytes at the slowest SCK take over 1.5 ms.
    let own = view.selected(d);
    let mut others = true;
    let devices = view.devices();
    for o in 0..devices {
        if o != d && view.selected(o) != Some(false) {
            others = false;
        }
    }
    let r = next(h, &[]).await;
    let after = (0..devices).all(|o| view.selected(o) == Some(false));
    let wrong = view.stats().cs_wrong;
    defmt::info!(
        "chip select on device {}: while running own {} others high {}; after, all high {}; port's readback at each end wrong {} times",
        d,
        own,
        others,
        after,
        wrong
    );
    verdict(
        "chip select: own line low and no other while running, all high after",
        own == Some(true) && others && after && r.outcome == Outcome::Done,
    )
}

/// One long transfer on the wire, then `capacity` short ones queued; the
/// next is refused with `refusal` (M: `Full`, the queue; R: `Outstanding`,
/// the device's buffers), nothing queued is lost, and a submit works after.
pub async fn full<H: Device>(h: &mut H, now: Now, capacity: u32, refusal: SubmitError) -> u32 {
    let _ = h.send(0, &[Seg::W(&[0x33; MAX])]);
    let mut accepted = 0u32;
    let mut refused = None;
    for i in 0..capacity + 4 {
        match h.send(0, &[Seg::T(&tagged(1000 + i))]) {
            Ok(()) => accepted += 1,
            Err(e) => {
                refused = Some(e);
                break;
            }
        }
    }
    let mut good = 0u32;
    let mut last_waited = 0u32;
    let first = next(h, &[]).await;
    good += u32::from(first.outcome == Outcome::Done);
    for i in 0..accepted {
        let r = next(h, &tagged(1000 + i)).await;
        if r.outcome == Outcome::Done && r.matched {
            good += 1;
        }
        last_waited = r.waited;
    }
    let after = h.send(0, &[Seg::T(&[9])]).is_ok()
        && poll(h, &[9], now, 100_000_000).is_some_and(|r| r.matched);
    defmt::info!(
        "full: {} queued behind a running transfer, then refused as expected {}; {} of {} came back done and in order; the last waited {} cycles (D7)",
        accepted,
        refused == Some(refusal),
        good,
        accepted + 1,
        last_waited
    );
    verdict(
        "full: refused at the limit with the expected error, nothing queued lost, usable after",
        accepted == capacity && refused == Some(refusal) && good == accepted + 1 && after,
    )
}

/// A long low-priority transfer running, then two medium and one high
/// submitted, in that order. `roles` picks the handle in `hs` for low,
/// medium, medium and high (all one handle on a one-device port).
/// `across` names the expected end order: true for priority across
/// devices (low, high, medium, medium), false for submission order within
/// one device (low, medium, medium, high).
pub fn priority<H: Device>(hs: &mut [&mut H], roles: [usize; 4], now: Now, across: bool) -> u32 {
    let plan: [(u8, u32); 4] = [(1, 0), (5, 2000), (5, 2001), (9, 2002)];
    let mut stamps = [0u32; 4];
    for (k, &(p, tag)) in plan.iter().enumerate() {
        let Some(h) = hs.get_mut(roles[k]) else {
            return verdict("priority: handles", false);
        };
        stamps[k] = now();
        let _ = if k == 0 {
            h.send(p, &[Seg::W(&[0x11; MAX])])
        } else {
            h.send(p, &[Seg::T(&tagged(tag))])
        };
    }
    let mut ends = [0u32; 4];
    let mut seen = [false; 4];
    let mut waited_high = 0;
    let mut ok = true;
    let start = now();
    while seen.iter().any(|s| !s) && now().wrapping_sub(start) < 200_000_000 {
        for h in hs.iter_mut() {
            let Some(r) = h.take(&tagged(2000)) else {
                continue;
            };
            let k = match r.priority {
                1 => 0,
                9 => 3,
                _ if r.matched => 1,
                _ => 2,
            };
            ok &= r.outcome == Outcome::Done && !seen[k];
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
            "priority: low (running) then high then the mediums in arrival order"
        } else {
            "order within one device: as submitted (low, medium, medium, high), priority not applied"
        },
        ok && seen.iter().all(|s| *s) && order_ok,
    )
}

/// Two devices with different settings, alternating: the bytes come back
/// on each, and each device's time on the wire matches its own SCK.
pub async fn switch<H: Device>(a: &mut H, b: &mut H, view: &mut impl View, cpu_hz: u32) -> u32 {
    let before = view.stats().switches;
    let mut took = [u32::MAX; 2];
    let mut wrong = 0u32;
    for i in 0..100u32 {
        for (k, h) in [&mut *a, &mut *b].into_iter().enumerate() {
            let mut bytes = [0u8; 8];
            bytes[..4].copy_from_slice(&tagged(3000 + i));
            bytes[4..].copy_from_slice(&tagged(4000 + i));
            let _ = h.send(0, &[Seg::T(&bytes)]);
            let r = next(h, &bytes).await;
            if r.outcome != Outcome::Done || !r.matched {
                wrong += 1;
            }
            took[k] = took[k].min(r.took);
        }
    }
    let s = view.stats();
    let switches = s.switches - before;
    // 64 bits on the wire: an upper bound on the SCK in force.
    let rate = |t: u32| (64u64 * u64::from(cpu_hz) / u64::from(t.max(1))) as u32;
    let (da, db) = (usize::from(a.device()), usize::from(b.device()));
    defmt::info!(
        "switch: {} switches in 200 alternating transfers, longest {} cycles; device {} SCK {} Hz (8 bytes in {} cycles, {} bit/s), device {} SCK {} Hz (8 bytes in {} cycles, {} bit/s)",
        switches,
        s.longest_switch,
        da,
        s.sck[da],
        took[0],
        rate(took[0]),
        db,
        s.sck[db],
        took[1],
        rate(took[1])
    );
    let (sa, sb) = (s.sck[da], s.sck[db]);
    // The wire's rate is below the SCK (the gaps between bytes) and above
    // half of it.
    let fits = |sck: u32, r: u32| r <= sck && r > sck / 2;
    verdict(
        "switch: settings changed with the device, bytes intact, each at its own SCK",
        // The first transfer goes to the device already in force: 200
        // alternating transfers change device 199 times.
        wrong == 0 && switches >= 199 && fits(sa, rate(took[0])) && fits(sb, rate(took[1])),
    )
}

/// A transfer running and `queued` behind it on `h`, one on `other` too;
/// `h.cancel()` returns all of `h`'s as aborted with its line high, leaves
/// `other`'s alone, and the bus runs again after.
pub async fn cancel<H: Device>(
    h: &mut H,
    other: Option<&mut H>,
    view: &mut impl View,
    now: Now,
    queued: u32,
) -> u32 {
    let d = usize::from(h.device());
    let _ = h.send(0, &[Seg::W(&[0x77; MAX])]);
    for i in 0..queued {
        let _ = h.send(0, &[Seg::T(&tagged(5000 + i))]);
    }
    let mut other = other;
    if let Some(o) = other.as_deref_mut() {
        let _ = o.send(0, &[Seg::T(&[0xC4])]);
    }
    let running = view.selected(d) == Some(true);
    h.cancel();
    let mut aborted = 0u32;
    for _ in 0..=queued {
        if poll(h, &[], now, 50_000_000).is_some_and(|r| r.outcome == Outcome::Aborted) {
            aborted += 1;
        }
    }
    let released = view.selected(d) == Some(false);
    let other_ok = match other {
        Some(o) => poll(o, &[0xC4], now, 50_000_000)
            .is_some_and(|r| r.outcome == Outcome::Done && r.matched),
        None => true,
    };
    let after = h.send(0, &[Seg::T(&[0x3C])]).is_ok() && next(h, &[0x3C]).await.matched;
    defmt::info!(
        "cancel: running {}; {} of {} aborted; line high after {}; the other device's done {}; runs again {}",
        running,
        aborted,
        queued + 1,
        released,
        other_ok,
        after
    );
    verdict(
        "cancel: queued and running aborted, chip select high, others untouched, bus usable",
        running && aborted == queued + 1 && released && other_ok && after && h.outstanding() == 0,
    )
}

/// Device `a` holds as many transactions as it can, behind a long one on
/// the wire; then device `b` submits one. `isolated`: whether `b` is
/// expected to be accepted (R: each device has its own queue) or refused
/// `Full` (M: one queue for the bus). Everything accepted comes back.
pub async fn isolation<H: Device>(a: &mut H, b: &mut H, now: Now, isolated: bool) -> u32 {
    let _ = a.send(0, &[Seg::W(&[0x44; MAX])]);
    let mut held = 0u32;
    let mut a_refused = None;
    for i in 0..64u32 {
        match a.send(0, &[Seg::T(&tagged(6000 + i))]) {
            Ok(()) => held += 1,
            Err(e) => {
                a_refused = Some(e);
                break;
            }
        }
    }
    let b_sent = b.send(0, &[Seg::T(&[0xB1])]);
    let mut a_good =
        u32::from(poll(a, &[], now, 50_000_000).is_some_and(|r| r.outcome == Outcome::Done));
    for i in 0..held {
        if poll(a, &tagged(6000 + i), now, 50_000_000)
            .is_some_and(|r| r.outcome == Outcome::Done && r.matched)
        {
            a_good += 1;
        }
    }
    let b_back = match b_sent {
        Ok(()) => poll(b, &[0xB1], now, 50_000_000).is_some_and(|r| r.matched),
        Err(_) => false,
    };
    defmt::info!(
        "isolation: device {} held {} queued behind a running transfer, then refused {}; device {} then submitted: {}; came back: device {} {} of {}, device {} {}",
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
    let expected_b = if isolated {
        b_sent.is_ok() && b_back
    } else {
        b_sent == Err(SubmitError::Full)
    };
    verdict(
        if isolated {
            "isolation: a full device does not stop another's submit"
        } else {
            "isolation (recorded): a device filling the shared queue refuses another's submit"
        },
        expected_b && a_refused.is_some() && a_good == held + 1,
    )
}

/// A refusal's name for the log; "none" when accepted.
fn named(e: Option<SubmitError>) -> &'static str {
    match e {
        None => "none (accepted)",
        Some(SubmitError::Full) => "Full",
        Some(SubmitError::TooLong) => "TooLong",
        Some(SubmitError::Parts) => "Parts",
        Some(SubmitError::Empty) => "Empty",
        Some(SubmitError::Outstanding) => "Outstanding",
    }
}

/// Keeps up to three transactions outstanding on `h` for `n` transfers,
/// checking each: run from tasks at different priorities at once, it
/// submits from preempting contexts. Returns
/// (done, wrong, refused-and-retried).
pub async fn stress<H: Device>(h: &mut H, n: u32, tag: u32) -> (u32, u32, u32) {
    let (mut sent, mut done, mut wrong, mut retried) = (0u32, 0u32, 0u32, 0u32);
    while done < n {
        while sent < n && h.outstanding() < 3 {
            match h.send((tag & 7) as u8, &[Seg::T(&tagged(tag * 100_000 + sent))]) {
                Ok(()) => sent += 1,
                Err(_) => {
                    retried += 1;
                    break;
                }
            }
        }
        let r = next(h, &tagged(tag * 100_000 + done)).await;
        if r.outcome != Outcome::Done || !r.matched {
            wrong += 1;
        }
        done += 1;
    }
    (done, wrong, retried)
}

/// What every image checks last: the port's own counts.
pub fn closing(variant: &str, view: &mut impl View) -> u32 {
    let s = view.stats();
    let (push, pop) = view.queue_longest();
    let health = view.health();
    defmt::info!(
        "{} port: longest service {} cycles; queue critical section push {} pop {} cycles; switches {} longest {} cycles; chip select wrong {}; results lost {}; FerroHAL transfers {} dma_errors {} overruns {} mode_faults {} aborts {}",
        variant,
        s.longest_service,
        push,
        pop,
        s.switches,
        s.longest_switch,
        s.cs_wrong,
        s.lost,
        health.transfers,
        health.dma_errors,
        health.overruns,
        health.mode_faults,
        health.aborts
    );
    verdict(
        "port: chip select right at every end, no result lost, no bus fault",
        s.cs_wrong == 0
            && s.lost == 0
            && health.dma_errors == 0
            && health.overruns == 0
            && health.mode_faults == 0,
    )
}
