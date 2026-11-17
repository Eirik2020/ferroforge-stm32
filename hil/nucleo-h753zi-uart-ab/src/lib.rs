//! PROTOTYPE: the one consumer both variants run. It speaks FerroHAL's
//! chunk vocabulary (`Chunk`, `ChunkEnd`, `try_send`, idle), which variant A
//! gets from the locked `Uart` and variant B from the port's `Handle`.

#![no_std]

use ferroforge_stm32::uart_queue::Handle;
use ferrohal_stm32::uart::{
    Chunk, ChunkEnd, Instance, RxDma, RxPin, SendError, TxDma, TxPin, Uart, Wiring,
};

/// Bytes per frame sent.
pub const FRAME: usize = 32;

/// What the consumer needs from a port.
pub trait Port {
    /// The next chunk and, where the port gives one, its arrival time.
    fn take(&mut self, buf: &mut [u8]) -> Option<(Chunk, Option<u32>)>;
    fn send(&mut self, frame: &[u8]) -> Result<(), SendError>;
    fn idle(&mut self) -> bool;
}

impl<U, Tx, Rx, TxD, RxD> Port for Uart<Wiring<U, Tx, Rx, TxD, RxD>>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    fn take(&mut self, buf: &mut [u8]) -> Option<(Chunk, Option<u32>)> {
        match self.read_chunk(buf) {
            Ok(chunk) => chunk.map(|c| (c, None)),
            Err(e) => Some((
                Chunk {
                    len: 0,
                    end: ChunkEnd::Gap(e),
                    generation: self.generation(),
                },
                None,
            )),
        }
    }
    fn send(&mut self, frame: &[u8]) -> Result<(), SendError> {
        self.try_send(frame)
    }
    fn idle(&mut self) -> bool {
        self.is_idle().unwrap_or(false)
    }
}

impl Port for Handle {
    fn take(&mut self, buf: &mut [u8]) -> Option<(Chunk, Option<u32>)> {
        self.read_chunk(buf).map(|(c, at)| (c, Some(at)))
    }
    fn send(&mut self, frame: &[u8]) -> Result<(), SendError> {
        self.try_send(frame)
    }
    fn idle(&mut self) -> bool {
        self.is_idle()
    }
}

/// One port's loopback: a counter wrapping at 256 goes out in frames, and
/// every byte read is checked against the next value.
#[derive(Clone, Copy, Default)]
pub struct Loop {
    /// Stop sending at this many bytes.
    pub limit: u32,
    /// Send but do not read: the slow consumer.
    pub hold: bool,
    pub sent: u32,
    pub read: u32,
    pub wrong: u32,
    /// Chunks that ended in a gap: loss the consumer was told about.
    pub gaps: u32,
    /// Chunks that ended where the line went idle (frame ends, D13).
    pub idles: u32,
    pub chunks: u32,
    pub wakes: u32,
    /// Times the consumer found the port idle with everything sent (D15).
    pub all_sent: u32,
    /// Longest the consumer's step took, in cycles; in A, all of it is
    /// under the port's lock.
    pub longest_step: u32,
    /// Oldest chunk the consumer took, in cycles since the port took it
    /// (B only: A has no arrival time).
    pub oldest: u32,
    pub expected: u8,
    /// After `limit` bytes are back, send `flood` more without reading,
    /// pause, then read: the slow consumer.
    pub flood: u32,
    pub flooding: bool,
    /// `read` and `gaps` when the flood started.
    pub before: (u32, u32),
}

impl Loop {
    /// Whether the consumer should now pause, outside any lock, and then
    /// call [`release`](Self::release).
    pub fn wants_pause(&self) -> bool {
        self.hold && self.sent >= self.limit
    }

    pub fn release(&mut self) {
        self.hold = false;
    }

    /// One wake: read what is there (unless holding), then send what fits.
    pub fn step<P: Port>(&mut self, port: &mut P, now: fn() -> u32) {
        self.wakes += 1;
        if !self.hold {
            let mut buf = [0u8; 64];
            for _ in 0..64 {
                let Some((chunk, at)) = port.take(&mut buf) else {
                    break;
                };
                self.chunks += 1;
                if let Some(at) = at {
                    self.oldest = self.oldest.max(now().wrapping_sub(at));
                }
                match chunk.end {
                    ChunkEnd::Idle => self.idles += 1,
                    ChunkEnd::Full => {}
                    ChunkEnd::Gap(_) => self.gaps += 1,
                }
                for &byte in &buf[..chunk.len] {
                    if byte != self.expected {
                        self.wrong += 1;
                    }
                    self.expected = byte.wrapping_add(1);
                }
                self.read += chunk.len as u32;
            }
        }
        if self.flood > 0 && !self.flooding && self.read >= self.limit {
            self.flooding = true;
            self.hold = true;
            self.before = (self.read, self.gaps);
            self.limit += self.flood;
        }
        for _ in 0..16 {
            // Frame by frame: the next goes out once the last is back, so
            // the line idles between frames as a protocol's would. The
            // flood sends without waiting.
            if self.sent >= self.limit {
                if port.idle() {
                    self.all_sent += 1;
                }
                break;
            }
            if !self.hold && self.sent > self.read {
                break;
            }
            let len = FRAME.min((self.limit - self.sent) as usize);
            let mut frame = [0u8; FRAME];
            for (i, byte) in frame[..len].iter_mut().enumerate() {
                *byte = (self.sent as usize + i) as u8;
            }
            match port.send(&frame[..len]) {
                Ok(()) => self.sent += len as u32,
                Err(_) => break,
            }
            if !self.hold {
                break;
            }
        }
    }
}
