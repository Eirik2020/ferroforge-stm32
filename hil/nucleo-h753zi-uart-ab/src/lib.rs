//! PROTOTYPE: the one consumer both variants run. It knows only
//! `embedded-io`'s non-blocking traits, so variant A hands it the locked
//! FerroHAL `Uart`, variant B the port's handles.

#![no_std]

use embedded_io::{Read, ReadReady, Write, WriteReady};

/// Bytes per write.
pub const FRAME: usize = 32;

/// One port's loopback: a counter wrapping at 256 goes out, and every byte
/// read is checked against the next value.
#[derive(Clone, Copy, Default)]
pub struct Loop {
    /// Stop sending at this many bytes.
    pub limit: u32,
    /// Send but do not read: the slow consumer.
    pub hold: bool,
    pub sent: u32,
    pub read: u32,
    pub wrong: u32,
    pub gaps: u32,
    pub wakes: u32,
    /// Longest the consumer's step took, in cycles; in A, all of it is
    /// under the port's lock.
    pub longest_step: u32,
    pub expected: u8,
}

impl Loop {
    /// One wake: read what is there (unless holding), then send what fits.
    pub fn step<P>(&mut self, port: &mut P)
    where
        P: Read + ReadReady + Write + WriteReady,
    {
        self.wakes += 1;
        if !self.hold {
            self.receive(port);
        }
        self.send(port);
    }

    fn receive<P: Read + ReadReady>(&mut self, port: &mut P) {
        let mut buf = [0u8; 64];
        for _ in 0..64 {
            match port.read_ready() {
                Ok(true) => {}
                Ok(false) => break,
                Err(_) => {
                    self.gaps += 1;
                    continue;
                }
            }
            match port.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    for &byte in &buf[..n] {
                        if byte != self.expected {
                            self.wrong += 1;
                        }
                        self.expected = byte.wrapping_add(1);
                    }
                    self.read += n as u32;
                }
                Err(_) => self.gaps += 1,
            }
        }
    }

    fn send<P: Write + WriteReady>(&mut self, port: &mut P) {
        for _ in 0..16 {
            if self.sent >= self.limit || !matches!(port.write_ready(), Ok(true)) {
                break;
            }
            let len = FRAME.min((self.limit - self.sent) as usize);
            let mut frame = [0u8; FRAME];
            for (i, byte) in frame[..len].iter_mut().enumerate() {
                *byte = (self.sent as usize + i) as u8;
            }
            match port.write(&frame[..len]) {
                Ok(0) | Err(_) => break,
                Ok(n) => self.sent += n as u32,
            }
        }
    }
}
