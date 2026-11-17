//! A UART port: one FerroHAL [`Uart`] served from its three vectors.
//!
//! The UART has one owner, a shared resource; the members lock it, so each
//! vector's priority is the firmware's choice (FerroHAL ADR-011). Every
//! member calls the same [`Service::on_interrupt`], which serves whatever is
//! pending on any of the three, and hands what it found to the firmware's own
//! task through the `events` call.
//!
//! Wakes coalesce. While the firmware's task is still queued, a later
//! `events` call finds it full and is dropped: the task reads everything that
//! has arrived whenever it runs, so no byte is lost, only the record of which
//! event woke it.
//!
//! Requirements: PORT-001, PORT-002, PORT-003

/// The UART port group. A firmware selects it once per port:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::uart_port, shared = [uart = usart1],
///         spawn = [events = usart1_events])]
/// mod usart1 {
///     #[task(binds = USART1, priority = 2)] fn on_uart;
///     #[task(binds = DMA2_STREAM2, priority = 2)] fn on_rx_dma;
///     #[task(binds = DMA2_STREAM7, priority = 2)] fn on_tx_dma;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, tx: Tx, rx: Rx, tx_dma: TxD, rx_dma: RxD],
    vectors = [on_uart = instance, on_rx_dma = rx_dma, on_tx_dma = tx_dma],
    shared = [uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>],
    constructor = new,
)]
pub mod uart_port {
    use ferrohal_stm32::{
        rcc::Rcc,
        uart::{Buffers, Config, Instance, RxDma, RxPin, TxDma, TxPin, Uart, UartRefused, Wiring},
    };

    pub use ferrohal_stm32::{Service, uart::Events};

    /// What [`new`] builds: the port's one resource, the UART.
    pub struct Resources<U, Tx, Rx, TxD, RxD>
    where
        U: Instance,
    {
        /// The firmware's shared resource, bound as the group's `uart`.
        pub uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>,
    }

    /// Opens the UART on `wiring` and starts reception. A refusal is
    /// FerroHAL's, with the wiring and buffers handed back.
    #[allow(clippy::type_complexity)]
    pub fn new<U, Tx, Rx, TxD, RxD>(
        rcc: &mut Rcc,
        wiring: Wiring<U, Tx, Rx, TxD, RxD>,
        config: &Config,
        buffers: Buffers,
    ) -> Result<Resources<U, Tx, Rx, TxD, RxD>, UartRefused<Wiring<U, Tx, Rx, TxD, RxD>>>
    where
        U: Instance,
        Tx: TxPin<U>,
        Rx: RxPin<U>,
        TxD: TxDma<U>,
        RxD: RxDma<U>,
    {
        Uart::new(rcc, wiring, config, buffers).map(|uart| Resources { uart })
    }

    /// The USART's own vector: idle line, line errors.
    #[ferroforge::task(
        shared = [uart],
        bounds = [uart: Service<Event = Events>],
        spawn = [events(events: Events)],
    )]
    pub fn on_uart(mut cx: on_uart::Context) {
        let events = cx.shared.uart.lock(|uart| uart.on_interrupt());
        if events.any() {
            let _: Result<(), Events> = cx.spawn.events(events);
        }
    }

    /// The receive stream's vector: the ring's half and end, DMA errors.
    #[ferroforge::task(
        shared = [uart],
        bounds = [uart: Service<Event = Events>],
        spawn = [events(events: Events)],
    )]
    pub fn on_rx_dma(mut cx: on_rx_dma::Context) {
        let events = cx.shared.uart.lock(|uart| uart.on_interrupt());
        if events.any() {
            let _: Result<(), Events> = cx.spawn.events(events);
        }
    }

    /// The transmit stream's vector: a frame sent, the next one started.
    #[ferroforge::task(
        shared = [uart],
        bounds = [uart: Service<Event = Events>],
        spawn = [events(events: Events)],
    )]
    pub fn on_tx_dma(mut cx: on_tx_dma::Context) {
        let events = cx.shared.uart.lock(|uart| uart.on_interrupt());
        if events.any() {
            let _: Result<(), Events> = cx.spawn.events(events);
        }
    }
}
