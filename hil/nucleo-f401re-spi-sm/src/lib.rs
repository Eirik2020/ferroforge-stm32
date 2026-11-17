//! PROTOTYPE: the consumer side of the SPI port comparison, shared by every
//! image of the bench (S, M with one device, M with four). It names only
//! `SpiHandle` and what comes back; the port is read through [`View`] for
//! the measurements a consumer would not make.
//!
//! The loop is the board's: MOSI into MISO, so a transfer part comes back
//! as sent and a read part as the filler.

#![no_std]

use ferroforge_stm32::spi_queue::{
    ChipSelects, FILLER, MAX, Outcome, Port, SpiHandle, SpiResult, Stats, SubmitError,
};
use ferrohal_stm32::spi::{Health, Parts};

/// The firmware's clock: CPU cycles.
pub type Now = fn() -> u32;

/// What the bench reads of a port, under its lock.
pub trait View {
    fn stats(&mut self) -> Stats;
    fn queue_longest(&mut self) -> (u32, u32);
    fn selected(&mut self, device: usize) -> Option<bool>;
    fn health(&mut self) -> Health;
    fn devices(&self) -> usize;
}

impl<M, W, Cs> View for M
where
    M: rtic::Mutex<T = Port<W, Cs>>,
    W: Parts,
    Cs: ChipSelects,
{
    fn stats(&mut self) -> Stats {
        self.lock(|p| p.stats())
    }
    fn queue_longest(&mut self) -> (u32, u32) {
        self.lock(|p| p.queue_longest())
    }
    fn selected(&mut self, device: usize) -> Option<bool> {
        self.lock(|p| p.is_selected(device))
    }
    fn health(&mut self) -> Health {
        self.lock(|p| p.spi().health())
    }
    fn devices(&self) -> usize {
        Cs::COUNT
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
pub async fn next(h: &mut SpiHandle) -> SpiResult {
    loop {
        if let Some(r) = h.take() {
            return r;
        }
        h.wait().await;
    }
}

/// The next result, polled for at most `ticks`: for checks where a result
/// that never comes must fail, not hang.
pub fn poll(h: &mut SpiHandle, now: Now, ticks: u32) -> Option<SpiResult> {
    let start = now();
    loop {
        if let Some(r) = h.take() {
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

/// `n` 4-byte transfers one at a time, each awaited: submit to start
/// (`waited`, stamped by the port) and submit to the result in the
/// consumer's hands, in cycles.
pub async fn latency(variant: &str, h: &mut SpiHandle, now: Now, n: u32) -> u32 {
    let mut to_start = Spread::default();
    let mut to_result = Spread::default();
    let mut wrong = 0u32;
    let mut refused = 0u32;
    for i in 0..n {
        let bytes = tagged(i);
        let mut t = h.transaction(0);
        t.transfer(&bytes);
        let t0 = now();
        if h.submit(t).is_err() {
            refused += 1;
            continue;
        }
        let r = next(h).await;
        let t1 = now();
        if r.outcome != Outcome::Done || r.bytes() != bytes {
            wrong += 1;
        }
        to_start.add(r.waited);
        to_result.add(t1.wrapping_sub(t0));
    }
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
        wrong == 0 && refused == 0 && to_result.n == n,
    )
}

/// Write, write, read, transfer in one transaction: only the read and
/// transfer parts come back, in place.
pub async fn multipart(h: &mut SpiHandle) -> u32 {
    let mut t = h.transaction(0);
    t.write(&[0xA0]).write(&[1, 2]).read(3).transfer(&[7, 8]);
    let wire = t.len();
    let _ = h.submit(t);
    let r = next(h).await;
    let expected = [FILLER, FILLER, FILLER, 7, 8];
    defmt::info!(
        "multi-part: {} bytes on the wire, {} came back: {=[u8]}",
        wire,
        r.bytes().len(),
        r.bytes()
    );
    let mut w = h.transaction(0);
    w.write(&[1, 2, 3]);
    let _ = h.submit(w);
    let r2 = next(h).await;
    verdict(
        "multi-part: one transaction, only the read and transfer parts returned",
        wire == 8 && r.outcome == Outcome::Done && r.bytes() == expected,
    ) + verdict(
        "write-only: done, nothing returned",
        r2.outcome == Outcome::Done && r2.bytes().is_empty(),
    )
}

/// A 64-byte transfer on `device`: its line low and every other high while
/// it runs, all high after.
pub async fn chip_select(h: &mut SpiHandle, view: &mut impl View) -> u32 {
    let d = usize::from(h.device());
    let mut t = h.transaction(0);
    t.write(&[0x55; MAX]);
    let _ = h.submit(t);
    // The port runs above this task, so the transfer started inside submit;
    // 64 bytes at the slowest SCK take over 1.5 ms.
    let own = view.selected(d);
    let mut others = true;
    for o in 0..view.devices() {
        if o != d && view.selected(o) != Some(false) {
            others = false;
        }
    }
    let r = next(h).await;
    let after = (0..view.devices()).all(|o| view.selected(o) == Some(false));
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
/// next is refused `Full`, nothing queued is lost, and a submit works after.
pub async fn full(h: &mut SpiHandle, now: Now, capacity: u32) -> u32 {
    let mut long = h.transaction(0);
    long.write(&[0x33; MAX]);
    let _ = h.submit(long);
    let mut accepted = 0u32;
    let mut refusal = None;
    for i in 0..capacity + 4 {
        let mut t = h.transaction(0);
        t.transfer(&tagged(1000 + i));
        match h.submit(t) {
            Ok(()) => accepted += 1,
            Err(e) => {
                refusal = Some(e);
                break;
            }
        }
    }
    let mut good = 0u32;
    let mut last_waited = 0u32;
    let mut first = true;
    for i in 0..=accepted {
        let r = next(h).await;
        if first {
            first = false;
            good += u32::from(r.outcome == Outcome::Done);
            continue;
        }
        if r.outcome == Outcome::Done && r.bytes() == tagged(1000 + i - 1) {
            good += 1;
        }
        last_waited = r.waited;
    }
    let mut again = h.transaction(0);
    again.transfer(&[9]);
    let after =
        h.submit(again).is_ok() && poll(h, now, 100_000_000).is_some_and(|r| r.bytes() == [9]);
    defmt::info!(
        "full: {} queued behind a running transfer, then refused {}; {} of {} came back done and in order; the last waited {} cycles (D7)",
        accepted,
        refusal.map(|e| e == SubmitError::Full),
        good,
        accepted + 1,
        last_waited
    );
    verdict(
        "full: refused with Full at the queue's size, nothing queued lost, usable after",
        accepted == capacity && refusal == Some(SubmitError::Full) && good == accepted + 1 && after,
    )
}

/// A long low-priority transfer running, then two medium and one high
/// submitted, in that order: they must finish low, high, medium, medium.
/// `roles` picks the handle in `hs` for low, medium, medium and high (all
/// one handle on a one-device port).
pub fn priority(hs: &mut [&mut SpiHandle], roles: [usize; 4], now: Now) -> u32 {
    let plan: [(u8, u32); 4] = [(1, 0), (5, 2000), (5, 2001), (9, 2002)];
    let mut stamps = [0u32; 4];
    for (k, &(p, tag)) in plan.iter().enumerate() {
        let Some(h) = hs.get_mut(roles[k]) else {
            return verdict("priority: handles", false);
        };
        let mut t = h.transaction(p);
        if k == 0 {
            t.write(&[0x11; MAX]);
        } else {
            t.transfer(&tagged(tag));
        }
        stamps[k] = now();
        let _ = h.submit(t);
    }
    let mut ends = [0u32; 4];
    let mut seen = [false; 4];
    let mut waited_high = 0;
    let mut ok = true;
    let start = now();
    while seen.iter().any(|s| !s) && now().wrapping_sub(start) < 200_000_000 {
        for h in hs.iter_mut() {
            let Some(r) = h.take() else { continue };
            let k = match r.priority {
                1 => 0,
                9 => 3,
                _ if r.bytes() == tagged(2000) => 1,
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
    let order_ok = since(0) < since(3) && since(3) < since(1) && since(1) < since(2);
    defmt::info!(
        "priority: ends in cycles from the low's submit: low {} high {} medium {} medium {}; the high waited {} cycles",
        since(0),
        since(3),
        since(1),
        since(2),
        waited_high
    );
    verdict(
        "priority: low (running) then high then the mediums in arrival order",
        ok && seen.iter().all(|s| *s) && order_ok,
    )
}

/// Two devices with different settings, alternating: the bytes come back
/// on each, and each device's time on the wire matches its own SCK.
pub async fn switch(
    a: &mut SpiHandle,
    b: &mut SpiHandle,
    view: &mut impl View,
    cpu_hz: u32,
) -> u32 {
    let before = view.stats().switches;
    let mut took = [u32::MAX; 2];
    let mut wrong = 0u32;
    for i in 0..100u32 {
        for (k, h) in [&mut *a, &mut *b].into_iter().enumerate() {
            let mut t = h.transaction(0);
            let mut bytes = [0u8; 8];
            bytes[..4].copy_from_slice(&tagged(3000 + i));
            bytes[4..].copy_from_slice(&tagged(4000 + i));
            t.transfer(&bytes);
            let _ = h.submit(t);
            let r = next(h).await;
            if r.outcome != Outcome::Done || r.bytes() != bytes {
                wrong += 1;
            }
            took[k] = took[k].min(r.took);
        }
    }
    let s = view.stats();
    let switches = s.switches - before;
    // 64 bits on the wire: an upper bound on the SCK in force.
    let rate = |t: u32| (64u64 * u64::from(cpu_hz) / u64::from(t.max(1))) as u32;
    defmt::info!(
        "switch: {} switches in 200 alternating transfers, longest {} cycles; device {} SCK {} Hz (8 bytes in {} cycles, {} bit/s), device {} SCK {} Hz (8 bytes in {} cycles, {} bit/s)",
        switches,
        s.longest_switch,
        a.device(),
        s.sck[usize::from(a.device())],
        took[0],
        rate(took[0]),
        b.device(),
        s.sck[usize::from(b.device())],
        took[1],
        rate(took[1])
    );
    let (sa, sb) = (
        s.sck[usize::from(a.device())],
        s.sck[usize::from(b.device())],
    );
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
pub async fn cancel(
    h: &mut SpiHandle,
    other: Option<&mut SpiHandle>,
    view: &mut impl View,
    now: Now,
    queued: u32,
) -> u32 {
    let d = usize::from(h.device());
    let mut long = h.transaction(0);
    long.write(&[0x77; MAX]);
    let _ = h.submit(long);
    for i in 0..queued {
        let mut t = h.transaction(0);
        t.transfer(&tagged(5000 + i));
        let _ = h.submit(t);
    }
    let mut other = other;
    if let Some(o) = other.as_deref_mut() {
        let mut t = o.transaction(0);
        t.transfer(&[0xC4]);
        let _ = o.submit(t);
    }
    let running = view.selected(d) == Some(true);
    h.cancel();
    let mut aborted = 0u32;
    for _ in 0..=queued {
        if poll(h, now, 50_000_000).is_some_and(|r| r.outcome == Outcome::Aborted) {
            aborted += 1;
        }
    }
    let released = view.selected(d) == Some(false);
    let other_ok = match other {
        Some(o) => poll(o, now, 50_000_000)
            .is_some_and(|r| r.outcome == Outcome::Done && r.bytes() == [0xC4]),
        None => true,
    };
    let mut again = h.transaction(0);
    again.transfer(&[0x3C]);
    let after = h.submit(again).is_ok() && next(h).await.bytes() == [0x3C];
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

/// Keeps up to three transactions outstanding on `h` for `n` transfers,
/// checking each: run from tasks at different priorities at once, it
/// pushes into the shared queue from preempting contexts. Returns
/// (done, wrong, refused-and-retried).
pub async fn stress(h: &mut SpiHandle, n: u32, tag: u32) -> (u32, u32, u32) {
    let (mut sent, mut done, mut wrong, mut retried) = (0u32, 0u32, 0u32, 0u32);
    while done < n {
        while sent < n && h.outstanding() < 3 {
            let mut t = h.transaction((tag & 7) as u8);
            t.transfer(&tagged(tag * 100_000 + sent));
            match h.submit(t) {
                Ok(()) => sent += 1,
                Err(_) => {
                    retried += 1;
                    break;
                }
            }
        }
        let r = next(h).await;
        if r.outcome != Outcome::Done || r.bytes() != tagged(tag * 100_000 + done) {
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
