//! PROTOTYPE, variant B of the UART port comparison (port design record,
//! section 3). Not product code; not merged.
//!
//! The port's three members own the FerroHAL [`Uart`] together with the
//! port's ends of two `heapless` byte queues. The consumer owns the other
//! ends, as [`RxHandle`] and [`TxHandle`], and never names the UART, its
//! wiring, or a lock. Both handles implement `embedded-io`'s non-blocking
//! traits, the same ones FerroHAL's `Uart` implements.

use core::sync::atomic::{AtomicU32, Ordering};

use ferrohal_stm32::{
    Service,
    rcc::Rcc,
    uart::{
        Buffers, Config, Instance, RxDma, RxPin, SendError, TxDma, TxPin, Uart, UartRefused, Wiring,
    },
};
use heapless::spsc::{Consumer, Producer, Queue};

/// Bytes the port moves per step between FerroHAL and a queue.
const STEP: usize = 32;

/// What the port reports to the consumer outside the byte stream (D8).
#[derive(Default)]
pub struct Status {
    /// Bytes the port had to leave in FerroHAL's ring because the receive
    /// queue was full when it served the ring.
    deferred: AtomicU32,
    /// Gaps FerroHAL reported in the byte stream (overrun, line, DMA error).
    gaps: AtomicU32,
}

/// The memory both queues live in, provided by the firmware (`init` local)
/// and split once by [`new`].
pub struct Queues<const RX: usize, const TX: usize> {
    rx: Queue<u8, RX>,
    tx: Queue<u8, TX>,
    status: Status,
}

impl<const RX: usize, const TX: usize> Queues<RX, TX> {
    pub const fn new() -> Self {
        Self {
            rx: Queue::new(),
            tx: Queue::new(),
            status: Status {
                deferred: AtomicU32::new(0),
                gaps: AtomicU32::new(0),
            },
        }
    }
}

impl<const RX: usize, const TX: usize> Default for Queues<RX, TX> {
    fn default() -> Self {
        Self::new()
    }
}

/// A scheduler error the consumer learns from its next read (D8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortError {
    /// FerroHAL reported `gaps` breaks in the stream since the last read.
    Gap { gaps: u32 },
}

impl core::fmt::Display for PortError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Gap { gaps } => write!(f, "{gaps} gap(s) in the stream"),
        }
    }
}

impl core::error::Error for PortError {}

impl embedded_io::Error for PortError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// The consumer's receive end.
pub struct RxHandle {
    queue: Consumer<'static, u8>,
    status: &'static Status,
    seen_gaps: u32,
    /// Pends a port vector, so a port that deferred bytes refills the queue.
    kick: fn(),
}

/// The consumer's transmit end.
pub struct TxHandle {
    queue: Producer<'static, u8>,
    /// Pends a port vector, so the port moves the queue into FerroHAL.
    kick: fn(),
}

impl RxHandle {
    /// Bytes the port has had to leave in FerroHAL's ring since boot.
    pub fn deferred(&self) -> u32 {
        self.status.deferred.load(Ordering::Relaxed)
    }
}

impl embedded_io::ErrorType for RxHandle {
    type Error = PortError;
}

impl embedded_io::Read for RxHandle {
    /// Copies what the queue holds, without waiting. `Ok(0)` means nothing
    /// yet. A gap is reported once, before the bytes after it.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, PortError> {
        let gaps = self.status.gaps.load(Ordering::Acquire);
        if gaps != self.seen_gaps {
            let new = gaps.wrapping_sub(self.seen_gaps);
            self.seen_gaps = gaps;
            return Err(PortError::Gap { gaps: new });
        }
        let mut n = 0;
        while n < buf.len() {
            match self.queue.dequeue() {
                Some(byte) => {
                    buf[n] = byte;
                    n += 1;
                }
                None => break,
            }
        }
        if n > 0 && self.status.deferred.load(Ordering::Relaxed) > 0 {
            (self.kick)();
        }
        Ok(n)
    }
}

impl embedded_io::ReadReady for RxHandle {
    /// True too when a gap waits to be reported by `read`.
    fn read_ready(&mut self) -> Result<bool, PortError> {
        Ok(self.queue.ready() || self.status.gaps.load(Ordering::Acquire) != self.seen_gaps)
    }
}

impl embedded_io::ErrorType for TxHandle {
    type Error = PortError;
}

impl embedded_io::Write for TxHandle {
    /// Queues as much of `buf` as fits, without waiting, and wakes the port.
    fn write(&mut self, buf: &[u8]) -> Result<usize, PortError> {
        let mut n = 0;
        for &byte in buf {
            if self.queue.enqueue(byte).is_err() {
                break;
            }
            n += 1;
        }
        if n > 0 {
            (self.kick)();
        }
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), PortError> {
        (self.kick)();
        Ok(())
    }
}

impl embedded_io::WriteReady for TxHandle {
    fn write_ready(&mut self) -> Result<bool, PortError> {
        Ok(self.queue.ready())
    }
}

/// The port's own state: the UART and the port's queue ends.
pub struct Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>,
    rx: Producer<'static, u8>,
    tx: Consumer<'static, u8>,
    status: &'static Status,
    /// A transmit step taken from the queue but not yet accepted by FerroHAL.
    staged: [u8; STEP],
    staged_len: usize,
}

impl<U, Tx, Rx, TxD, RxD> Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    /// FerroHAL's UART, for checks such as `vectors().verify()`.
    pub fn uart(&self) -> &Uart<Wiring<U, Tx, Rx, TxD, RxD>> {
        &self.uart
    }

    /// Ring into queue. True when the consumer has something new.
    fn pump_rx(&mut self) -> bool {
        let mut woke = false;
        let mut buf = [0u8; STEP];
        // Bounded: one ring's worth of steps at most is served per call.
        for _ in 0..64 {
            let free = self.rx.capacity() - self.rx.len();
            if free == 0 {
                let left = self.uart.available().unwrap_or(0);
                if left > 0 {
                    self.status
                        .deferred
                        .fetch_add(u32::from(left), Ordering::Relaxed);
                }
                break;
            }
            match self.uart.try_read(&mut buf[..free.min(STEP)]) {
                Ok(0) => break,
                Ok(n) => {
                    for &byte in &buf[..n] {
                        // Room was measured above; only this end fills it.
                        let _ = self.rx.enqueue(byte);
                    }
                    woke = true;
                }
                Err(_) => {
                    self.status.gaps.fetch_add(1, Ordering::Release);
                    woke = true;
                }
            }
        }
        woke
    }

    /// Queue into FerroHAL's transmit buffer, one step at a time.
    fn pump_tx(&mut self) {
        for _ in 0..64 {
            if self.staged_len == 0 {
                while self.staged_len < STEP {
                    match self.tx.dequeue() {
                        Some(byte) => {
                            self.staged[self.staged_len] = byte;
                            self.staged_len += 1;
                        }
                        None => break,
                    }
                }
                if self.staged_len == 0 {
                    return;
                }
            }
            match self.uart.try_send(&self.staged[..self.staged_len]) {
                Ok(()) => self.staged_len = 0,
                Err(SendError::Full) => return,
                // Too large cannot happen with a buffer of STEP or more;
                // halted drops the step, as FerroHAL drops frames then.
                Err(_) => {
                    self.staged_len = 0;
                    return;
                }
            }
        }
    }
}

impl<U, Tx, Rx, TxD, RxD> Service for Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    /// True when the consumer should be woken.
    type Event = bool;

    fn on_interrupt(&mut self) -> bool {
        let events = self.uart.on_interrupt();
        self.pump_tx();
        self.pump_rx() || events.fault
    }
}

/// What [`new`] builds: the port's resource and the consumer's two handles.
pub struct Resources<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    /// Shared by the port's three members only.
    pub port: Port<U, Tx, Rx, TxD, RxD>,
    /// The consumer's, as `#[local]`.
    pub rx: RxHandle,
    /// The consumer's, as `#[local]`.
    pub tx: TxHandle,
}

/// Opens the UART and splits `queues`. `kick` pends any one of the port's
/// vectors (`|| rtic::pend(Interrupt::..)`): the firmware names it, not the
/// consumer.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn new<U, Tx, Rx, TxD, RxD, const RX: usize, const TX: usize>(
    rcc: &mut Rcc,
    wiring: Wiring<U, Tx, Rx, TxD, RxD>,
    config: &Config,
    tx_buf: &'static mut [u8],
    rx_buf: &'static mut [u8],
    queues: &'static mut Queues<RX, TX>,
    kick: fn(),
) -> Result<Resources<U, Tx, Rx, TxD, RxD>, UartRefused<Wiring<U, Tx, Rx, TxD, RxD>>>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    let uart = Uart::new(
        rcc,
        wiring,
        config,
        Buffers {
            tx: tx_buf,
            rx: rx_buf,
        },
    )?;
    let Queues { rx, tx, status } = queues;
    let status: &'static Status = status;
    let (rx_p, rx_c) = rx.split();
    let (tx_p, tx_c) = tx.split();
    Ok(Resources {
        port: Port {
            uart,
            rx: rx_p,
            tx: tx_c,
            status,
            staged: [0; STEP],
            staged_len: 0,
        },
        rx: RxHandle {
            queue: rx_c,
            status,
            seen_gaps: 0,
            kick,
        },
        tx: TxHandle { queue: tx_p, kick },
    })
}

/// The variant B group. Members share `port` and must sit at one priority,
/// so RTIC's lock among them costs nothing; the group cannot state that rule
/// (finding).
#[ferroforge::group]
pub mod uart_queue_port {
    pub use ferrohal_stm32::Service;

    /// The USART's own vector.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = bool>], spawn = [ready()])]
    pub fn q_on_uart(mut cx: q_on_uart::Context) {
        if cx.shared.port.lock(|port| port.on_interrupt()) {
            let _: Result<(), ()> = cx.spawn.ready();
        }
    }

    /// The receive stream's vector.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = bool>], spawn = [ready()])]
    pub fn q_on_rx_dma(mut cx: q_on_rx_dma::Context) {
        if cx.shared.port.lock(|port| port.on_interrupt()) {
            let _: Result<(), ()> = cx.spawn.ready();
        }
    }

    /// The transmit stream's vector, also pended by the handles.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = bool>], spawn = [ready()])]
    pub fn q_on_tx_dma(mut cx: q_on_tx_dma::Context) {
        if cx.shared.port.lock(|port| port.on_interrupt()) {
            let _: Result<(), ()> = cx.spawn.ready();
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_io::{Read, Write};
    use std::boxed::Box;

    fn kick() {}

    fn handles() -> (
        Producer<'static, u8>,
        RxHandle,
        TxHandle,
        Consumer<'static, u8>,
        &'static Status,
    ) {
        let q: &'static mut Queues<8, 8> = Box::leak(Box::new(Queues::new()));
        let Queues { rx, tx, status } = q;
        let status: &'static Status = status;
        let (rp, rc) = rx.split();
        let (tp, tc) = tx.split();
        (
            rp,
            RxHandle {
                queue: rc,
                status,
                seen_gaps: 0,
                kick,
            },
            TxHandle { queue: tp, kick },
            tc,
            status,
        )
    }

    #[test]
    fn bytes_pass_both_ways_and_a_full_queue_takes_what_fits() {
        let (mut rp, mut rx, mut tx, mut tc, _) = handles();
        for b in 1..=3 {
            rp.enqueue(b).unwrap();
        }
        let mut buf = [0; 8];
        assert_eq!(rx.read(&mut buf).unwrap(), 3);
        assert_eq!(&buf[..3], &[1, 2, 3]);
        assert_eq!(rx.read(&mut buf).unwrap(), 0);
        // heapless keeps one slot free: an 8-slot queue holds 7.
        assert_eq!(tx.write(&[9; 10]).unwrap(), 7);
        assert_eq!(tc.dequeue(), Some(9));
    }

    #[test]
    fn a_gap_is_reported_once_before_the_bytes_after_it() {
        let (mut rp, mut rx, _, _, status) = handles();
        rp.enqueue(5).unwrap();
        status.gaps.fetch_add(2, Ordering::Release);
        let mut buf = [0; 4];
        assert_eq!(rx.read(&mut buf), Err(PortError::Gap { gaps: 2 }));
        assert_eq!(rx.read(&mut buf).unwrap(), 1);
    }
}
