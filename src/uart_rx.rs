//! A receive-only UART port: one FerroHAL [`Uart`] on an [`RxWiring`], served
//! from its two vectors.
//!
//! The same port as [`uart_port`](crate::uart_port) without the transmit side:
//! no TX pin, no transmit stream, so one vector fewer. Each member calls
//! [`Service::on_interrupt`](ferrohal_stm32::Service::on_interrupt), as
//! `uart_port`'s do, and also stamps the wake with the firmware's monotonic,
//! read in the interrupt: a reader that judges how old its data is needs the
//! time the line delivered it, not the time the scheduler ran the reader.
//! Wakes coalesce, so when one is dropped because the reader was still queued,
//! the queued wake keeps the earlier stamp: data never looks newer than it is.
//! It carries no protocol: protocols are not hardware, so a decoder lives in
//! the application or a protocol crate, behind the port's `events` call.
//!
//! Requirements: PORT-001, PORT-002, PORT-101, PORT-102

/// The receive-only UART port group. A firmware selects it once per port:
///
/// The firmware's reader takes the events and the arrival time in
/// microseconds of the monotonic's epoch:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::uart_rx_port, shared = [uart = usart6],
///         spawn = [events = usart6_events])]
/// mod usart6 {
///     #[task(binds = USART6, priority = 2)] fn on_rx_uart;
///     #[task(binds = DMA2_STREAM1, priority = 2)] fn on_rx_stream;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, rx: Rx, rx_dma: RxD],
    vectors = [on_rx_uart = instance, on_rx_stream = rx_dma],
    shared = [uart: Uart<RxWiring<U, Rx, RxD>>],
    buffers = [#[dma] ring: [u8]],
    config = [config: Config @ platform],
    constructor = new(rcc, wiring, config, ring),
)]
pub mod uart_rx_port {
    use ferrohal_stm32::{
        rcc::Rcc,
        uart::{Config, Instance, RxDma, RxPin, RxWiring, Uart, UartRxRefused},
    };

    pub use ferrohal_stm32::{Service, uart::Events};

    /// What [`new`] builds: the port's one resource, the UART.
    pub struct Resources<U, Rx, RxD>
    where
        U: Instance,
    {
        /// The firmware's shared resource, bound as the group's `uart`.
        pub uart: Uart<RxWiring<U, Rx, RxD>>,
    }

    /// Opens the UART receive-only on `wiring` and starts reception into
    /// `ring`. A refusal is FerroHAL's, with the wiring and ring handed back.
    #[allow(clippy::type_complexity)]
    pub fn new<U, Rx, RxD>(
        rcc: &mut Rcc,
        wiring: RxWiring<U, Rx, RxD>,
        config: &Config,
        ring: &'static mut [u8],
    ) -> Result<Resources<U, Rx, RxD>, UartRxRefused<RxWiring<U, Rx, RxD>>>
    where
        U: Instance,
        Rx: RxPin<U>,
        RxD: RxDma<U>,
    {
        Uart::new_rx(rcc, wiring, config, ring).map(|uart| Resources { uart })
    }

    /// The USART's own vector: idle line, line errors.
    #[ferroforge::task(
        shared = [uart],
        bounds = [uart: Service<Event = Events>],
        spawn = [events(events: Events, at_us: u64)],
        monotonic = Mono,
    )]
    pub fn on_rx_uart(mut cx: on_rx_uart::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        let events = cx.shared.uart.lock(|uart| uart.on_interrupt());
        if events.any() {
            let _: Result<(), (Events, u64)> = cx.spawn.events(events, at_us);
        }
    }

    /// The receive stream's vector: the ring's half and end, DMA errors.
    #[ferroforge::task(
        shared = [uart],
        bounds = [uart: Service<Event = Events>],
        spawn = [events(events: Events, at_us: u64)],
        monotonic = Mono,
    )]
    pub fn on_rx_stream(mut cx: on_rx_stream::Context) {
        let at_us = Mono::now().duration_since_epoch().to_micros();
        let events = cx.shared.uart.lock(|uart| uart.on_interrupt());
        if events.any() {
            let _: Result<(), (Events, u64)> = cx.spawn.events(events, at_us);
        }
    }
}

#[cfg(test)]
mod tests {

    // The constructor's full signature, spelt out, is the point of the test.
    #![allow(clippy::type_complexity)]

    use std::{string::ToString, vec::Vec};

    use super::uart_rx_port;
    use crate::tests::groups;

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
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["uart"]);
        assert!(contract.local.is_empty());
        let constructor = contract.constructor.as_ref().unwrap();
        assert!(constructor.path.is_ident("new"));
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(arguments, ["rcc", "wiring", "config", "ring"]);
        assert!(contract.buffers[0].name == "ring" && contract.buffers[0].dma);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn uart_rx_port_builds_on_an_f401_wiring() {
        use ferrohal_stm32::{
            chip::USART6,
            dma::Stream,
            gpio::{Alternate, Pin},
            rcc::Rcc,
            uart::{Config, RxWiring, UartRxRefused},
        };
        // USART6 RX on PA12, AF8 (DS10086 table 9), on DMA2 stream 1
        // channel 5 (RM0368 table 29).
        type Rx = Pin<'A', 12, Alternate<8>>;
        type W = RxWiring<USART6, Rx, Stream<2, 1>>;
        type Built = uart_rx_port::Resources<USART6, Rx, Stream<2, 1>>;
        let constructor: fn(
            &mut Rcc,
            W,
            &Config,
            &'static mut [u8],
        ) -> Result<Built, UartRxRefused<W>> = uart_rx_port::new;
        let _ = constructor;
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn uart_rx_port_builds_on_an_h753_wiring() {
        use ferrohal_stm32::{
            chip::USART6,
            dma::Stream,
            gpio::{Alternate, Pin},
            rcc::Rcc,
            uart::{Config, RxWiring, UartRxRefused},
        };
        // USART6 RX on PC7, AF7 (DS12117 table 9), on any stream
        // through DMAMUX1 (RM0433 section 17.3).
        type Rx = Pin<'C', 7, Alternate<7>>;
        type W = RxWiring<USART6, Rx, Stream<1, 0>>;
        type Built = uart_rx_port::Resources<USART6, Rx, Stream<1, 0>>;
        let constructor: fn(
            &mut Rcc,
            W,
            &Config,
            &'static mut [u8],
        ) -> Result<Built, UartRxRefused<W>> = uart_rx_port::new;
        let _ = constructor;
    }
}
