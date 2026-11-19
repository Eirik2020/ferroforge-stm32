//! The receive side both UART ports share: a pool of buffers the port lends
//! to its consumer, filled from FerroHAL's ring.
//!
//! In each service the port takes a free pool buffer and has FerroHAL's
//! `read_chunk` copy the ring into it, the one copy left, then lends it with
//! FerroHAL's [`Chunk`] and the time the service read. The consumer reads it
//! in place; dropping the [`Received`] returns the buffer. With no buffer
//! free the bytes stay in FerroHAL's ring and the port marks itself starved:
//! the release pends it. Bytes are lost only when the ring laps, and FerroHAL
//! then reports a gap, which reaches the consumer as a chunk ending in
//! `ChunkEnd::Gap` at its place in the stream.
//!
//! Two single-producer, single-consumer queues of references carry the
//! buffers: chunks out, buffers back. Nothing here panics; a buffer a queue
//! cannot take is counted, never dropped silently.
//!
//! Requirements: PORT-005, PORT-006, PORT-007

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use ferrohal_stm32::uart::{Chunk, ChunkEnd, Error};
use heapless::spsc::{Consumer, Producer, Queue};

/// The most chunks one service moves, so a service is bounded however fast
/// the line runs.
pub(crate) const PER_SERVICE: usize = 16;

/// A chunk lent to the consumer.
pub(crate) struct RxLent {
    buf: &'static mut [u8],
    chunk: Chunk,
    at_us: u64,
}

/// What a port and its handle both see outside the queues.
pub(crate) struct Status {
    /// The port had bytes to read and no buffer: the next release pends it.
    pub(crate) starved: AtomicBool,
    /// Buffers a queue could not take. Zero by construction, since each
    /// queue holds every buffer of its kind; counted so a fault is not
    /// hidden.
    pub(crate) lost: AtomicU32,
}

impl Status {
    pub(crate) const fn new() -> Self {
        Self {
            starved: AtomicBool::new(false),
            lost: AtomicU32::new(0),
        }
    }

    pub(crate) fn count_lost(&self) {
        self.lost.fetch_add(1, Ordering::Relaxed);
    }
}

/// The receive queues, part of a port's memory. `RX` slots: the pool holds
/// at most `RX - 1` buffers.
pub(crate) struct RxQueues<const RX: usize> {
    chunks: Queue<RxLent, RX>,
    back: Queue<&'static mut [u8], RX>,
}

impl<const RX: usize> RxQueues<RX> {
    pub(crate) const fn new() -> Self {
        Self {
            chunks: Queue::new(),
            back: Queue::new(),
        }
    }
}

/// The consumer's receive end, inside its handle.
pub(crate) struct RxSide {
    chunks: Consumer<'static, RxLent>,
    back: Producer<'static, &'static mut [u8]>,
    status: &'static Status,
    kick: fn(),
}

impl RxSide {
    /// The oldest chunk received, read in the port's buffer.
    pub(crate) fn next(&mut self) -> Option<Received<'_>> {
        let lent = self.chunks.dequeue()?;
        Some(Received {
            buf: Some(lent.buf),
            chunk: lent.chunk,
            at_us: lent.at_us,
            side: self,
        })
    }

    /// Returns a buffer to the port, and pends it if it was starved for one.
    fn release(&mut self, buf: &'static mut [u8]) {
        if self.back.enqueue(buf).is_err() {
            self.status.count_lost();
        }
        if self.status.starved.swap(false, Ordering::AcqRel) {
            (self.kick)();
        }
    }
}

/// A received chunk, read in place. Dropping it returns the buffer.
pub struct Received<'h> {
    side: &'h mut RxSide,
    buf: Option<&'static mut [u8]>,
    /// Its length, why it ended and its generation, as FerroHAL gave them. A
    /// chunk that ended in `ChunkEnd::Gap` marks where bytes were lost or
    /// damaged.
    pub chunk: Chunk,
    /// When the port was served and took it from FerroHAL's ring: the
    /// firmware monotonic's time read in that interrupt, in microseconds
    /// since its epoch.
    pub at_us: u64,
}

impl Received<'_> {
    /// The chunk's bytes.
    pub fn bytes(&self) -> &[u8] {
        self.buf
            .as_deref()
            .and_then(|b| b.get(..self.chunk.len))
            .unwrap_or(&[])
    }
}

impl Drop for Received<'_> {
    fn drop(&mut self) {
        if let Some(buf) = self.buf.take() {
            self.side.release(buf);
        }
    }
}

/// The port's receive end: its pool, as returned buffers and one spare.
pub(crate) struct RxEnd {
    chunks: Producer<'static, RxLent>,
    back: Consumer<'static, &'static mut [u8]>,
    /// A buffer taken for a chunk that was not complete yet.
    spare: Option<&'static mut [u8]>,
}

impl RxEnd {
    /// A free buffer, or `None` with the port marked starved. The flag is
    /// set before the queue is looked at, so a release racing this, at any
    /// priority, either is seen here or sees the flag and pends the port.
    fn take(&mut self, status: &Status) -> Option<&'static mut [u8]> {
        if let Some(buf) = self.spare.take() {
            return Some(buf);
        }
        status.starved.store(true, Ordering::Release);
        let buf = self.back.dequeue()?;
        status.starved.store(false, Ordering::Release);
        Some(buf)
    }

    /// Lends `buf` to the consumer, or keeps it as the spare when the queue
    /// is full (it holds the whole pool, so it is not).
    fn lend(&mut self, buf: &'static mut [u8], chunk: Chunk, at_us: u64, status: &Status) -> bool {
        match self.chunks.enqueue(RxLent { buf, chunk, at_us }) {
            Ok(()) => true,
            Err(back) => {
                self.spare = Some(back.buf);
                status.count_lost();
                false
            }
        }
    }

    /// FerroHAL's ring into lent buffers, whole chunks; returns the chunks
    /// lent. `read` is FerroHAL's `read_chunk` into the buffer, its error
    /// already turned into a gap chunk: `None` means no chunk is complete.
    pub(crate) fn pump(
        &mut self,
        status: &Status,
        at_us: u64,
        mut read: impl FnMut(&mut [u8]) -> Option<Chunk>,
    ) -> usize {
        let mut chunks = 0;
        for _ in 0..PER_SERVICE {
            let Some(buf) = self.take(status) else {
                break;
            };
            let Some(chunk) = read(buf) else {
                self.spare = Some(buf);
                break;
            };
            if !self.lend(buf, chunk, at_us, status) {
                break;
            }
            chunks += 1;
            if chunk.end == ChunkEnd::Gap(Error::Halted) {
                break;
            }
        }
        chunks
    }
}

/// FerroHAL's `read_chunk` result as `pump` takes it: an error, which is only
/// ever `Halted`, becomes a gap chunk, never dropped.
pub(crate) fn as_chunk(result: Result<Option<Chunk>, Error>, generation: u32) -> Option<Chunk> {
    match result {
        Ok(chunk) => chunk,
        Err(error) => Some(Chunk {
            len: 0,
            end: ChunkEnd::Gap(error),
            generation,
        }),
    }
}

impl<const RX: usize> RxQueues<RX> {
    /// Puts the pool in the return queue, the port's source of buffers, and
    /// splits the queues. A buffer the port cannot use comes back as
    /// `Err(len)`: one empty, of another length than the first, longer than
    /// `longest` (half the receive ring), or one more than the queue holds.
    pub(crate) fn split(
        &'static mut self,
        status: &'static Status,
        pool: &mut dyn Iterator<Item = &'static mut [u8]>,
        longest: usize,
        kick: fn(),
    ) -> Result<(RxEnd, RxSide), usize> {
        let RxQueues { chunks, back } = self;
        let mut pool_len = None;
        for buf in pool {
            let len = buf.len();
            if len == 0 || len > longest || pool_len.is_some_and(|l| l != len) {
                return Err(len);
            }
            pool_len = Some(len);
            if back.enqueue(buf).is_err() {
                return Err(len);
            }
        }
        let (chunks_in, chunks_out) = chunks.split();
        let (back_in, back_out) = back.split();
        Ok((
            RxEnd {
                chunks: chunks_in,
                back: back_out,
                spare: None,
            },
            RxSide {
                chunks: chunks_out,
                back: back_in,
                status,
                kick,
            },
        ))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::AtomicUsize;
    use std::{boxed::Box, vec, vec::Vec};

    static KICKS: AtomicUsize = AtomicUsize::new(0);
    fn kick() {
        KICKS.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn leak(len: usize) -> &'static mut [u8] {
        Box::leak(vec![0u8; len].into_boxed_slice())
    }

    /// A pool of `n` buffers of 8 bytes on a 16-byte ring.
    fn ends(n: usize) -> (RxEnd, RxSide, &'static Status) {
        let queues: &'static mut RxQueues<4> = Box::leak(Box::new(RxQueues::new()));
        let status: &'static Status = Box::leak(Box::new(Status::new()));
        let mut pool = (0..n).map(|_| leak(8));
        let Ok((end, side)) = queues.split(status, &mut pool, 8, kick) else {
            panic!("the pool fits");
        };
        (end, side, status)
    }

    fn chunk(len: usize, end: ChunkEnd, generation: u32) -> Chunk {
        Chunk {
            len,
            end,
            generation,
        }
    }

    #[test]
    fn a_chunk_is_read_in_place_and_a_starved_port_is_pended_on_release() {
        let (mut end, mut side, status) = ends(2);
        let idle = chunk(3, ChunkEnd::Idle, 4);
        let mut fill = b"ab".iter().copied();
        let lent = end.pump(status, 9, |buf| {
            let byte = fill.next()?;
            buf[..3].fill(byte);
            Some(idle)
        });
        assert_eq!(lent, 2);
        // Pool empty: the bytes stay in FerroHAL's ring, the port starved.
        assert_eq!(end.pump(status, 9, |_| Some(idle)), 0);
        assert!(status.starved.load(Ordering::Acquire));
        let got = side.next().unwrap();
        assert_eq!((got.bytes(), got.chunk, got.at_us), (&b"aaa"[..], idle, 9));
        let before = KICKS.load(Ordering::Relaxed);
        drop(got);
        assert!(KICKS.load(Ordering::Relaxed) > before);
        assert!(!status.starved.load(Ordering::Acquire));
        // The released buffer is the port's again; the other is still lent.
        assert_eq!(end.pump(status, 10, |_| None), 0);
        assert_eq!(side.next().unwrap().bytes(), b"bbb");
        assert!(side.next().is_none());
        assert_eq!(status.lost.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn a_gap_chunk_reaches_the_consumer_at_its_place() {
        let (mut end, mut side, status) = ends(3);
        let order = [
            Ok(Some(chunk(2, ChunkEnd::Idle, 0))),
            Err(Error::Halted),
            Ok(None),
        ];
        let mut results = order.into_iter();
        let lent = end.pump(status, 0, |_| as_chunk(results.next()?, 1));
        // The halt ends the service: the stream stops there.
        assert_eq!(lent, 2);
        let ends: Vec<_> =
            core::iter::from_fn(|| side.next().map(|c| (c.chunk.end, c.chunk.generation)))
                .collect();
        assert_eq!(
            ends,
            [(ChunkEnd::Idle, 0), (ChunkEnd::Gap(Error::Halted), 1)]
        );
    }

    #[test]
    fn pool_buffers_the_port_cannot_use_are_refused() {
        let split = |pool: &mut dyn Iterator<Item = &'static mut [u8]>| {
            let queues: &'static mut RxQueues<4> = Box::leak(Box::new(RxQueues::new()));
            let status: &'static Status = Box::leak(Box::new(Status::new()));
            queues.split(status, pool, 16, kick).err()
        };
        // Two lengths, longer than half the ring, empty, more than fit.
        assert_eq!(split(&mut [leak(8), leak(4)].into_iter()), Some(4));
        assert_eq!(split(&mut [leak(17)].into_iter()), Some(17));
        assert_eq!(split(&mut [leak(0)].into_iter()), Some(0));
        assert_eq!(split(&mut (0..4).map(|_| leak(8))), Some(8));
        assert_eq!(split(&mut (0..3).map(|_| leak(8))), None);
    }
}
