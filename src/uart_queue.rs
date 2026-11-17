//! PROTOTYPE, variant B of the UART port comparison (port design record,
//! section 3). Not product code; not merged.
//!
//! The port's three members own the FerroHAL [`Uart`] together with the
//! port's ends of two `heapless` queues: received chunks one way, frames to
//! send the other. The consumer owns the other ends in one [`Handle`] and
//! never names the UART, its wiring, its vectors or a lock. Every UART's
//! handle has the same type, so which consumer gets which UART can be chosen
//! at boot.
//!
//! The consumer sees FerroHAL's own vocabulary: [`Chunk`] (length, why it
//! ended, generation), `try_send` with [`SendError`], `is_idle`. On top it
//! gets what only the port can give: the time each chunk was taken from the
//! ring (D7), and a wake that reaches whichever task holds the handle.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use ferrohal_stm32::{
    Service,
    rcc::Rcc,
    uart::{
        Buffers, Chunk, ChunkEnd, Config, Error, Instance, RxDma, RxPin, SendError, TxDma, TxPin,
        Uart, UartRefused, Wiring,
    },
};
use heapless::spsc::{Consumer, Producer, Queue};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

/// The longest chunk or frame one queue slot holds.
pub const SLOT: usize = 64;

/// One received chunk, as the port took it from FerroHAL's ring.
#[derive(Clone, Copy)]
struct RxChunk {
    bytes: [u8; SLOT],
    chunk: Chunk,
    at: u32,
}

/// One frame waiting to be sent.
#[derive(Clone, Copy)]
struct TxFrame {
    bytes: [u8; SLOT],
    len: u8,
}

/// What the port tells the consumer outside the stream.
struct Status {
    /// The receive queue was full while the port served the ring: the
    /// consumer's next take pends the port, so the ring is served again.
    starved: AtomicBool,
    /// Frames FerroHAL refused other than for room (a halted transmitter).
    tx_faults: AtomicU32,
    /// Nothing queued in the port or in FerroHAL.
    idle: AtomicBool,
}

/// The memory the port and its handle share, provided by the firmware
/// (an `init` local) and split once by [`new`].
pub struct Queues<const RX: usize, const TX: usize> {
    rx: Queue<RxChunk, RX>,
    tx: Queue<TxFrame, TX>,
    status: Status,
    wake: Signal<()>,
}

impl<const RX: usize, const TX: usize> Queues<RX, TX> {
    pub const fn new() -> Self {
        Self {
            rx: Queue::new(),
            tx: Queue::new(),
            status: Status {
                starved: AtomicBool::new(false),
                tx_faults: AtomicU32::new(0),
                idle: AtomicBool::new(true),
            },
            wake: Signal::new(),
        }
    }
}

impl<const RX: usize, const TX: usize> Default for Queues<RX, TX> {
    fn default() -> Self {
        Self::new()
    }
}

/// The consumer's end of one UART port.
pub struct Handle {
    rx: Consumer<'static, RxChunk>,
    tx: Producer<'static, TxFrame>,
    status: &'static Status,
    wake: SignalReader<'static, ()>,
    kick: fn(),
}

impl Handle {
    /// Waits until the port has news: a chunk, the last frame sent, a fault.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// The next chunk, copied into `buf` (as FerroHAL's `read_chunk`), with
    /// the time the port took it. A chunk longer than `buf` is cut, and
    /// says so by ending `Full`.
    pub fn read_chunk(&mut self, buf: &mut [u8]) -> Option<(Chunk, u32)> {
        let slot = self.rx.dequeue()?;
        let len = slot.chunk.len.min(buf.len()).min(SLOT);
        buf[..len].copy_from_slice(&slot.bytes[..len]);
        let mut chunk = slot.chunk;
        if len < chunk.len {
            chunk.end = ChunkEnd::Full;
        }
        chunk.len = len;
        if self.status.starved.swap(false, Ordering::AcqRel) {
            (self.kick)();
        }
        Some((chunk, slot.at))
    }

    /// Queues `frame` whole, or not at all (as FerroHAL's `try_send`).
    pub fn try_send(&mut self, frame: &[u8]) -> Result<(), SendError> {
        if frame.is_empty() {
            return Ok(());
        }
        if frame.len() > SLOT {
            return Err(SendError::TooLarge);
        }
        let mut slot = TxFrame {
            bytes: [0; SLOT],
            len: frame.len() as u8,
        };
        slot.bytes[..frame.len()].copy_from_slice(frame);
        self.status.idle.store(false, Ordering::Release);
        self.tx.enqueue(slot).map_err(|_| SendError::Full)?;
        (self.kick)();
        Ok(())
    }

    /// Whether every frame queued so far has left the wire.
    pub fn is_idle(&self) -> bool {
        self.status.idle.load(Ordering::Acquire)
    }

    /// Frames the transmitter refused since boot: faults are not hidden.
    pub fn tx_faults(&self) -> u32 {
        self.status.tx_faults.load(Ordering::Relaxed)
    }
}

/// The port's own state: the UART and the port's queue ends.
pub struct Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>,
    rx: Producer<'static, RxChunk>,
    tx: Consumer<'static, TxFrame>,
    status: &'static Status,
    wake: SignalWriter<'static, ()>,
    now: fn() -> u32,
    /// PROTOTYPE measurement: the longest `on_interrupt`, in `now` ticks.
    longest: u32,
}

impl<U, Tx, Rx, TxD, RxD> Port<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
    Tx: TxPin<U>,
    Rx: RxPin<U>,
    TxD: TxDma<U>,
    RxD: RxDma<U>,
{
    /// FerroHAL's UART, for checks such as `health()`.
    pub fn uart(&self) -> &Uart<Wiring<U, Tx, Rx, TxD, RxD>> {
        &self.uart
    }

    /// PROTOTYPE measurement: the longest `on_interrupt`, in `now` ticks.
    /// At mixed priorities this is how long one port task can hold off
    /// another through the lock among them.
    pub fn longest(&self) -> u32 {
        self.longest
    }

    /// Ring into queue, whole chunks. True when the consumer has news.
    fn pump_rx(&mut self) -> bool {
        let mut news = false;
        // Bounded: more than a 256-byte ring's worth of 64-byte chunks and
        // the gaps between them.
        for _ in 0..16 {
            if !self.rx.ready() {
                self.status.starved.store(true, Ordering::Release);
                break;
            }
            let mut bytes = [0; SLOT];
            let chunk = match self.uart.read_chunk(&mut bytes) {
                Ok(Some(chunk)) => chunk,
                Ok(None) => break,
                // Only `Halted`: reported as a gap chunk, never dropped.
                Err(error) => Chunk {
                    len: 0,
                    end: ChunkEnd::Gap(error),
                    generation: self.uart.generation(),
                },
            };
            let slot = RxChunk {
                bytes,
                chunk,
                at: (self.now)(),
            };
            // Room was checked above; only this end fills the queue.
            let _ = self.rx.enqueue(slot);
            news = true;
            if chunk.end == ChunkEnd::Gap(Error::Halted) {
                break;
            }
        }
        news
    }

    /// Queue into FerroHAL, whole frames. True when the port just went idle.
    fn pump_tx(&mut self) -> bool {
        for _ in 0..16 {
            let Some(frame) = self.tx.peek() else { break };
            match self.uart.try_send(&frame.bytes[..usize::from(frame.len)]) {
                Ok(()) => {}
                Err(SendError::Full) => break,
                Err(_) => {
                    self.status.tx_faults.fetch_add(1, Ordering::Relaxed);
                }
            }
            self.tx.dequeue();
        }
        // `ready` on heapless 0.9's consumer means "holds an item".
        let idle = !self.tx.ready() && self.uart.queued() == 0;
        let was = self.status.idle.swap(idle, Ordering::AcqRel);
        idle && !was
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
    /// Nothing: the port wakes its consumer itself.
    type Event = ();

    fn on_interrupt(&mut self) {
        let start = (self.now)();
        let events = self.uart.on_interrupt();
        let went_idle = self.pump_tx();
        let news = self.pump_rx();
        if news || went_idle || events.fault {
            self.wake.write(());
        }
        self.longest = self.longest.max((self.now)().wrapping_sub(start));
    }
}

/// What [`new`] builds: the port's resource and the consumer's handle.
pub struct Resources<U, Tx, Rx, TxD, RxD>
where
    U: Instance,
{
    /// Shared by the port's three members only.
    pub port: Port<U, Tx, Rx, TxD, RxD>,
    /// The consumer's, as a `#[local]`, given to any task at boot.
    pub handle: Handle,
}

/// Opens the UART on `wiring` and splits `queues`. `kick` is the
/// selection's generated pend of the transmit stream's vector; `now` is the
/// firmware's clock, read once per chunk.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn new<U, Tx, Rx, TxD, RxD, const RX: usize, const TX: usize>(
    rcc: &mut Rcc,
    wiring: Wiring<U, Tx, Rx, TxD, RxD>,
    config: &Config,
    tx_buf: &'static mut [u8],
    rx_buf: &'static mut [u8],
    queues: &'static mut Queues<RX, TX>,
    kick: fn(),
    now: fn() -> u32,
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
    let Queues {
        rx,
        tx,
        status,
        wake,
    } = queues;
    let status: &'static Status = status;
    let wake: &'static Signal<()> = wake;
    let (rx_p, rx_c) = rx.split();
    let (tx_p, tx_c) = tx.split();
    let (writer, reader) = wake.split();
    Ok(Resources {
        port: Port {
            uart,
            rx: rx_p,
            tx: tx_c,
            status,
            wake: writer,
            now,
            longest: 0,
        },
        handle: Handle {
            rx: rx_c,
            tx: tx_p,
            status,
            wake: reader,
            kick,
        },
    })
}

/// The variant B group. Members share `port`; one priority for all three
/// makes RTIC's lock among them free, but the firmware decides (D14).
/// Nothing is spawned: the port wakes whichever task holds the handle,
/// which the firmware gives to any task at boot (D12). In `init`:
/// `uart_queue::new(.., queues, <selection>::kick, now)`.
#[ferroforge::group(
    slots = [instance: U, tx: Tx, rx: Rx, tx_dma: TxD, rx_dma: RxD],
    vectors = [q_on_uart = instance, q_on_rx_dma = rx_dma, q_on_tx_dma = tx_dma],
    shared = [port: Port<U, Tx, Rx, TxD, RxD>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [config: Config @ platform, now: fn() -> u32 @ platform],
    memory = [queues: Queues],
    handles = [handle: Handle],
    pends = [kick = q_on_tx_dma],
    constructor = new(rcc, wiring, config, tx_buf, rx_buf, queues, kick, now),
)]
pub mod uart_queue_port {
    #[allow(unused_imports)]
    use super::{Config, Handle, Port, Queues, new};
    pub use ferrohal_stm32::Service;

    /// The USART's own vector.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn q_on_uart(mut cx: q_on_uart::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }

    /// The receive stream's vector.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn q_on_rx_dma(mut cx: q_on_rx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }

    /// The transmit stream's vector, also pended by the handle.
    #[ferroforge::task(shared = [port], bounds = [port: Service<Event = ()>])]
    pub fn q_on_tx_dma(mut cx: q_on_tx_dma::Context) {
        cx.shared.port.lock(|port| port.on_interrupt());
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::AtomicUsize;
    use std::boxed::Box;

    static KICKS: AtomicUsize = AtomicUsize::new(0);
    fn kick() {
        KICKS.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn a_chunk_keeps_its_end_and_time_and_a_starved_port_is_kicked() {
        let q: &'static mut Queues<4, 4> = Box::leak(Box::new(Queues::new()));
        let Queues {
            rx,
            tx,
            status,
            wake,
        } = q;
        let status: &'static Status = status;
        let (mut rp, rc) = rx.split();
        let (tp, _tc) = tx.split();
        let (_w, r) = wake.split();
        let mut h = Handle {
            rx: rc,
            tx: tp,
            status,
            wake: r,
            kick,
        };
        let mut bytes = [0; SLOT];
        bytes[..3].copy_from_slice(b"abc");
        let chunk = Chunk {
            len: 3,
            end: ChunkEnd::Gap(Error::BufferOverrun),
            generation: 2,
        };
        assert!(
            rp.enqueue(RxChunk {
                bytes,
                chunk,
                at: 77
            })
            .is_ok()
        );
        status.starved.store(true, Ordering::Release);
        let before = KICKS.load(Ordering::Relaxed);
        let mut buf = [0; 2];
        let (got, at) = h.read_chunk(&mut buf).unwrap();
        assert_eq!(
            (got.len, got.end, got.generation, at),
            (2, ChunkEnd::Full, 2, 77)
        );
        assert_eq!(&buf, b"ab");
        assert!(KICKS.load(Ordering::Relaxed) > before);
        assert!(h.read_chunk(&mut buf).is_none());
    }

    #[test]
    fn a_frame_queues_whole_or_not_at_all() {
        let q: &'static mut Queues<4, 2> = Box::leak(Box::new(Queues::new()));
        let Queues {
            rx,
            tx,
            status,
            wake,
        } = q;
        let status: &'static Status = status;
        let (_rp, rc) = rx.split();
        let (tp, _tc) = tx.split();
        let (_w, r) = wake.split();
        let mut h = Handle {
            rx: rc,
            tx: tp,
            status,
            wake: r,
            kick,
        };
        assert_eq!(h.try_send(&[0; SLOT + 1]), Err(SendError::TooLarge));
        assert_eq!(h.try_send(b"x"), Ok(()));
        assert!(!h.is_idle());
        // heapless keeps one slot free: a 2-slot queue holds 1 frame.
        assert_eq!(h.try_send(b"y"), Err(SendError::Full));
    }
}
