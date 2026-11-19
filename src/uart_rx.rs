//! The receive-only UART port: the port's members own one FerroHAL [`Uart`] on
//! an [`RxWiring`], and the consumer owns an [`RxHandle`] that names no UART,
//! wiring, vector or lock.
//!
//! A link from a device that only sends (an RC receiver, ESC telemetry) has no
//! transmit pin or stream, so this port has two vectors, the USART's and the
//! receive stream's. Its receive side is the full port's (the crate's
//! `uart_receive` module): the port lends pool buffers filled
//! from FerroHAL's ring, each with FerroHAL's [`Chunk`] and the time the
//! service read, and dropping a [`Received`] returns one. A starved port is
//! pended through its receive stream's vector by the release. Every handle
//! has one type whatever its UART and pool, so `init` can give any handle to
//! any task. Nothing here panics. It carries no protocol.
//!
//! [`Chunk`]: ferrohal_stm32::uart::Chunk
//!
//! Requirements: PORT-001, PORT-002, PORT-101, PORT-102, PORT-103

use ferrohal_stm32::uart::{Instance, RxDma, RxPin, RxWiring, Uart, UartRxRefused};
use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

pub use crate::uart_receive::Received;
use crate::uart_receive::{RxEnd, RxQueues, RxSide, Status, as_chunk};

/// The memory the port and its handle share: the firmware's, given once to
/// [`uart_rx_port::new`] (an `init` local), which splits it. `RX` queue slots:
/// the pool may hold at most `RX - 1` buffers.
pub struct RxMemory<const RX: usize> {
    rx: RxQueues<RX>,
    status: Status,
    wake: Signal<()>,
}

impl<const RX: usize> RxMemory<RX> {
    pub const fn new() -> Self {
        Self {
            rx: RxQueues::new(),
            status: Status::new(),
            wake: Signal::new(),
        }
    }
}

impl<const RX: usize> Default for RxMemory<RX> {
    fn default() -> Self {
        Self::new()
    }
}

/// The consumer's end of one receive-only port, owned by one task as a
/// `#[local]`. Every receive-only port's handle has this type.
pub struct RxHandle {
    rx: RxSide,
    status: &'static Status,
    wake: SignalReader<'static, ()>,
}

impl RxHandle {
    /// Waits until the port has news: a chunk or a fault. Wakes coalesce, so
    /// drain [`next_chunk`](Self::next_chunk) after it.
    pub async fn wait(&mut self) {
        self.wake.wait().await;
    }

    /// The oldest chunk received, read in the port's buffer. Dropping it
    /// returns the buffer to the port.
    pub fn next_chunk(&mut self) -> Option<Received<'_>> {
        self.rx.next()
    }

    /// Buffers lost to a full queue: zero unless something is broken.
    pub fn lost(&self) -> u32 {
        self.status.lost.load(core::sync::atomic::Ordering::Relaxed)
    }
}

/// A receive-only port its members serve with the time they read in the
/// interrupt. [`RxPort`] is the one implementation.
pub trait ServeRx {
    /// Serves whatever is pending on either of the port's vectors, stamping
    /// what it hands the consumer with `at_us`.
    fn serve(&mut self, at_us: u64);
}

/// The port's own state, shared by its two members only.
pub struct RxPort<U, Rx, RxD>
where
    U: Instance,
{
    uart: Uart<RxWiring<U, Rx, RxD>>,
    rx: RxEnd,
    status: &'static Status,
    wake: SignalWriter<'static, ()>,
}

impl<U, Rx, RxD> RxPort<U, Rx, RxD>
where
    U: Instance,
    Rx: RxPin<U>,
    RxD: RxDma<U>,
{
    /// FerroHAL's UART, to read its health counters.
    pub fn uart(&self) -> &Uart<RxWiring<U, Rx, RxD>> {
        &self.uart
    }
}

impl<U, Rx, RxD> ServeRx for RxPort<U, Rx, RxD>
where
    U: Instance,
    Rx: RxPin<U>,
    RxD: RxDma<U>,
{
    fn serve(&mut self, at_us: u64) {
        let events = self.uart.on_interrupt();
        let uart = &mut self.uart;
        let chunks = self.rx.pump(self.status, at_us, |buf| {
            let result = uart.read_chunk(buf);
            as_chunk(result, uart.generation())
        });
        if chunks > 0 || events.fault {
            self.wake.write(());
        }
    }
}

/// What [`uart_rx_port::new`] builds: the port's resource and the consumer's
/// handle.
pub struct RxResources<U, Rx, RxD>
where
    U: Instance,
{
    /// Shared by the port's two members only, bound as the group's `port`.
    pub port: RxPort<U, Rx, RxD>,
    /// The consumer's, as a `#[local]` of whichever task `init` gives it to.
    pub handle: RxHandle,
}

/// Why [`uart_rx_port::new`] refused. Nothing is ever returned opened.
pub enum RxRefused<W> {
    /// FerroHAL refused the UART: its reason, with the wiring and ring.
    Open(UartRxRefused<W>),
    /// A pool buffer the port cannot use, found before the UART was opened:
    /// one empty, of another length than the first, longer than half the
    /// ring, or one more than the queues hold. `len` is its length. The
    /// memory stays split, which is harmless: a refused port stops the boot.
    Buffer {
        len: usize,
        wiring: W,
        ring: &'static mut [u8],
    },
}

/// The port's end and the handle, split from `memory`, the pool checked
/// against `longest` (half the ring). A buffer the port cannot use comes back
/// as `Err(len)`.
#[allow(clippy::type_complexity)]
fn split_memory<const RX: usize>(
    memory: &'static mut RxMemory<RX>,
    rx_pool: &mut dyn Iterator<Item = &'static mut [u8]>,
    longest: usize,
    kick: fn(),
) -> Result<(RxEnd, &'static Status, SignalWriter<'static, ()>, RxHandle), usize> {
    let RxMemory { rx, status, wake } = memory;
    let status: &'static Status = status;
    let (end, side) = rx.split(status, rx_pool, longest, kick)?;
    let (writer, reader) = wake.split();
    Ok((
        end,
        status,
        writer,
        RxHandle {
            rx: side,
            status,
            wake: reader,
        },
    ))
}

/// The receive-only UART port group. A firmware selects it once per UART,
/// binds the port's resource, and gives the handle to any task:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::uart_rx_port, shared = [port = uart5])]
/// mod uart5 {
///     #[task(binds = UART5, priority = 3)] fn on_rx_uart;
///     #[task(binds = DMA1_STREAM0, priority = 3)] fn on_rx_stream;
/// }
/// // in init:
/// let built = uart_rx_port::new(&mut rcc, wiring, &FRAME, ring, memory,
///     &mut pool, uart5::kick)?;
/// ```
///
/// Each member stamps its service with the firmware's monotonic, read in the
/// interrupt.
#[ferroforge::group(
    slots = [instance: U, rx: Rx, rx_dma: RxD],
    vectors = [on_rx_uart = instance, on_rx_stream = rx_dma],
    shared = [port: RxPort<U, Rx, RxD>],
    buffers = [#[dma] ring: [u8]],
    config = [
        config: Config @ platform,
        rx_pool: &mut dyn Iterator<Item = &'static mut [u8]> @ software,
    ],
    memory = [memory: RxMemory],
    handles = [handle: RxHandle],
    pends = [kick = on_rx_stream],
    constructor = new(rcc, wiring, config, ring, memory, rx_pool, kick),
)]
pub mod uart_rx_port {
    // Private: a `pub use` here would include a group (FerroForge's
    // `group`). The types are `ferroforge_stm32::uart_rx`'s.
    use super::{RxMemory, RxPort, RxRefused, RxResources, ServeRx, split_memory};
    use ferrohal_stm32::{
        rcc::Rcc,
        uart::{Config, Instance, RxDma, RxPin, RxWiring, Uart},
    };

    /// Checks the pool, then opens the UART receive-only on `wiring` and
    /// splits `memory`.
    ///
    /// - `ring` is FerroHAL's receive ring (FerroHAL's `Uart::new_rx` states
    ///   its limits).
    /// - `rx_pool` is the port's pool: buffers of one length, each at most
    ///   half `ring`.
    /// - `kick` is the selection's generated pend of the receive stream's
    ///   vector (the contract's `pends`), which a release uses when the port
    ///   was starved.
    ///
    /// The pool is checked before the UART is opened, so a refusal never
    /// carries an opened driver.
    #[allow(clippy::type_complexity)]
    pub fn new<U, Rx, RxD, const RX: usize>(
        rcc: &mut Rcc,
        wiring: RxWiring<U, Rx, RxD>,
        config: &Config,
        ring: &'static mut [u8],
        memory: &'static mut RxMemory<RX>,
        rx_pool: &mut dyn Iterator<Item = &'static mut [u8]>,
        kick: fn(),
    ) -> Result<RxResources<U, Rx, RxD>, RxRefused<RxWiring<U, Rx, RxD>>>
    where
        U: Instance,
        Rx: RxPin<U>,
        RxD: RxDma<U>,
    {
        let (end, status, writer, handle) =
            match split_memory(memory, rx_pool, ring.len() / 2, kick) {
                Ok(ends) => ends,
                Err(len) => return Err(RxRefused::Buffer { len, wiring, ring }),
            };
        let uart = Uart::new_rx(rcc, wiring, config, ring).map_err(RxRefused::Open)?;
        Ok(RxResources {
            port: RxPort {
                uart,
                rx: end,
                status,
                wake: writer,
            },
            handle,
        })
    }

    /// The USART's own vector: idle line, line errors.
    #[ferroforge::task(shared = [port], bounds = [port: ServeRx], monotonic = Mono)]
    pub fn on_rx_uart(mut cx: on_rx_uart::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        cx.shared.port.lock(|port| port.serve(at_us));
    }

    /// The receive stream's vector: the ring's half and end, DMA errors; also
    /// pended by the handle when a starved port gets a buffer back.
    #[ferroforge::task(shared = [port], bounds = [port: ServeRx], monotonic = Mono)]
    pub fn on_rx_stream(mut cx: on_rx_stream::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        cx.shared.port.lock(|port| port.serve(at_us));
    }
}

#[cfg(test)]
mod tests {
    // The constructor's full signature, spelt out, is the point of the test.
    #![allow(clippy::type_complexity)]

    use std::{string::ToString, vec::Vec};

    use crate::tests::groups;
    use std::{boxed::Box, vec};

    fn kick() {}

    fn leak(len: usize) -> &'static mut [u8] {
        Box::leak(vec![0u8; len].into_boxed_slice())
    }

    #[test]
    fn a_pool_the_port_cannot_use_is_refused_before_the_uart_opens() {
        let split = |pool: &mut dyn Iterator<Item = &'static mut [u8]>| {
            let memory: &'static mut super::RxMemory<4> =
                Box::leak(Box::new(super::RxMemory::new()));
            super::split_memory(memory, pool, 16, kick).err()
        };
        assert_eq!(split(&mut [leak(17)].into_iter()), Some(17));
        assert_eq!(split(&mut [leak(8), leak(4)].into_iter()), Some(4));
        assert_eq!(split(&mut (0..4).map(|_| leak(8))), Some(8));
        // A pool that fits gives a handle with nothing received yet.
        let memory: &'static mut super::RxMemory<4> = Box::leak(Box::new(super::RxMemory::new()));
        let Ok((_, _, _, mut handle)) =
            super::split_memory(memory, &mut (0..3).map(|_| leak(8)), 16, kick)
        else {
            panic!("the pool fits");
        };
        assert!(handle.next_chunk().is_none());
        assert_eq!(handle.lost(), 0);
    }

    #[test]
    fn uart_rx_port_states_its_contract() {
        let found = groups(include_str!("uart_rx.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in uart_rx.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "uart_rx_port");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["instance", "rx", "rx_dma"]);
        let vectors: Vec<_> = contract
            .vectors
            .iter()
            .map(|v| (v.member.to_string(), v.slot.to_string()))
            .collect();
        assert_eq!(
            vectors,
            [
                ("on_rx_uart".to_string(), "instance".to_string()),
                ("on_rx_stream".to_string(), "rx_dma".to_string()),
            ]
        );
        let names = |list: &[ferroforge_contracts::Typed]| -> Vec<std::string::String> {
            list.iter().map(|t| t.name.to_string()).collect()
        };
        assert_eq!(names(&contract.memory), ["memory"]);
        assert_eq!(names(&contract.handles), ["handle"]);
        let pends: Vec<_> = contract
            .pends
            .iter()
            .map(|p| (p.name.to_string(), p.member.to_string()))
            .collect();
        assert_eq!(pends, [("kick".to_string(), "on_rx_stream".to_string())]);
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["port"]);
        assert!(contract.local.is_empty());
        let constructor = contract.constructor.as_ref().unwrap();
        assert!(constructor.path.is_ident("new"));
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(
            arguments,
            [
                "rcc", "wiring", "config", "ring", "memory", "rx_pool", "kick"
            ]
        );
        assert!(contract.buffers[0].name == "ring" && contract.buffers[0].dma);
        for vector in &contract.vectors {
            let member = group
                .members
                .iter()
                .find(|m| m.name == vector.member)
                .expect("the vector's member is in the group");
            assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
        }
    }

    /// `uart_rx_port::new` at one wiring, with the result's fields the
    /// contract names: `port` (shared) and `handle`.
    macro_rules! uart_rx_port_builds {
        ($U:ty, $Rx:ty, $RxD:ty) => {{
            use crate::{
                uart_rx::{RxHandle, RxMemory, RxPort, RxRefused, RxResources},
                uart_rx_port,
            };
            use ferrohal_stm32::{
                rcc::Rcc,
                uart::{Config, RxWiring},
            };
            type W = RxWiring<$U, $Rx, $RxD>;
            let constructor: fn(
                &mut Rcc,
                W,
                &Config,
                &'static mut [u8],
                &'static mut RxMemory<8>,
                &mut dyn Iterator<Item = &'static mut [u8]>,
                fn(),
            ) -> Result<RxResources<$U, $Rx, $RxD>, RxRefused<W>> = uart_rx_port::new;
            let _ = constructor;
            let fields = |built: RxResources<$U, $Rx, $RxD>| {
                let port: RxPort<$U, $Rx, $RxD> = built.port;
                let handle: RxHandle = built.handle;
                (port, handle)
            };
            let _ = fields;
        }};
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn uart_rx_port_builds_on_an_f401_wiring() {
        use ferrohal_stm32::{
            chip::USART6,
            dma::Stream,
            gpio::{Alternate, Pin},
        };
        // USART6 RX on PA12, AF8 (DS10086 table 9), on DMA2 stream 1
        // channel 5 (RM0368 table 29).
        uart_rx_port_builds!(USART6, Pin<'A', 12, Alternate<8>>, Stream<2, 1>);
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn uart_rx_port_builds_on_an_h753_wiring() {
        use ferrohal_stm32::{
            chip::USART6,
            dma::Stream,
            gpio::{Alternate, Pin},
        };
        // USART6 RX on PC7, AF7 (DS12117 table 9), on any stream through
        // DMAMUX1 (RM0433 section 17.3.2).
        uart_rx_port_builds!(USART6, Pin<'C', 7, Alternate<7>>, Stream<1, 0>);
    }

    /// The receive-only UARTs of three flight controller boards, as their
    /// targets assign pins and streams: the port takes each. Pins from DS8626
    /// table 9; streams from RM0090 table 43 (DMA1, channel 4) and table 44
    /// (DMA2: USART1 channel 4, USART6 channel 5).
    #[cfg(feature = "stm32f405rg")]
    #[test]
    fn uart_rx_port_builds_on_f405_flight_controller_routes() {
        use ferrohal_stm32::{
            chip::{UART5, USART1, USART6},
            dma::Stream,
            gpio::{Alternate, Pin},
        };
        // USART1 RX on PA10, DMA2 stream 5.
        uart_rx_port_builds!(USART1, Pin<'A', 10, Alternate<7>>, Stream<2, 5>);
        // USART6 RX on PC7, DMA2 stream 2.
        uart_rx_port_builds!(USART6, Pin<'C', 7, Alternate<8>>, Stream<2, 2>);
        // UART5 RX on PD2, DMA1 stream 0.
        uart_rx_port_builds!(UART5, Pin<'D', 2, Alternate<8>>, Stream<1, 0>);
    }

    /// The receive-only UARTs of an H743 flight controller board, through
    /// DMAMUX1 (RM0433 section 17.3.2): USART1 and UART8.
    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn uart_rx_port_builds_on_h7_flight_controller_routes() {
        use ferrohal_stm32::{
            chip::{UART8, USART1},
            dma::Stream,
            gpio::{Alternate, Pin},
        };
        // USART1 RX on PA10 (AF7), DMA2 stream 4; UART8 RX on PE0 (AF8), DMA1
        // stream 2 (DS12117 table 9).
        uart_rx_port_builds!(USART1, Pin<'A', 10, Alternate<7>>, Stream<2, 4>);
        uart_rx_port_builds!(UART8, Pin<'E', 0, Alternate<8>>, Stream<1, 2>);
    }
}
