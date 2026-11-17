//! An SPI port: one FerroHAL [`Spi`] master on two DMA streams and its
//! chip-select pin, served from the one vector the SPI raises.
//!
//! The port has one owner, a shared [`Device`](spi_port::Device): the SPI and
//! its chip select together. The firmware's own task starts each transfer
//! under the lock ([`Device::start`](spi_port::Device::start)), which selects
//! the device, and the group's member serves the receive stream's vector,
//! which ends the transfer: every byte sent clocks one in, so the receive
//! stream completes last, and the transmit stream raises no interrupt
//! (FerroHAL's `spi::stream_dma`). Serving it deselects the device at once,
//! inside the interrupt, so the select line is held no longer than the
//! transfer, whatever priority the firmware's task runs at. The member then
//! hands the outcome to the firmware's task through the `done` call, and that
//! task reads the bytes with [`Device::received`](spi_port::Device::received).
//!
//! What goes over the bus, a device's registers and framing, is the
//! firmware's: this port moves bytes and frames them with chip select, and
//! nothing else. A stuck transfer is the firmware's watchdog's to end, with
//! [`Device::abort`](spi_port::Device::abort) under the same lock.
//!
//! A transfer ends once, so `done` is not coalesced: the driver refuses a
//! second start while one is running, so at most one outcome is in flight.
//!
//! Requirements: PORT-001, PORT-002, PORT-201, PORT-202

/// The SPI port group. A firmware selects it once per device:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::spi_port, shared = [spi = spi2],
///         spawn = [done = spi2_done])]
/// mod spi2 {
///     #[task(binds = DMA1_STREAM3, priority = 3)] fn on_spi_rx_dma;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, sck: Sck, miso: Miso, mosi: Mosi, tx_dma: TxD, rx_dma: RxD, cs: Cs],
    vectors = [on_spi_rx_dma = rx_dma],
    shared = [spi: Device<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [config: Config @ board],
    constructor = new(rcc, wiring, cs, config, tx_buf, rx_buf),
)]
pub mod spi_port {
    use embedded_hal::digital::OutputPin;
    use ferrohal_stm32::{
        rcc::Rcc,
        spi::{
            Buffers, Config, Health, Instance, MisoPin, MosiPin, Parts, SckPin, Spi, SpiRefused,
            SpiRxDma, SpiTxDma, Vectors, Wiring,
        },
    };

    pub use ferrohal_stm32::{Service, spi::Error};

    /// What one service of the receive stream found: `None` when nothing
    /// ended, else the transfer's length or the reason it failed.
    pub type Event = Option<Result<usize, Error>>;

    /// One SPI device: FerroHAL's driver and the device's chip-select pin,
    /// active low. Selected for exactly as long as a transfer runs.
    pub struct Device<W: Parts, Cs> {
        spi: Spi<W>,
        cs: Cs,
    }

    impl<W: Parts, Cs: OutputPin> Device<W, Cs> {
        /// Pairs an opened driver with its chip select, deselected.
        pub fn new(spi: Spi<W>, mut cs: Cs) -> Self {
            let _ = cs.set_high();
            Self { spi, cs }
        }

        /// Selects the device and starts one full-duplex transfer of
        /// `tx.len()` bytes. A refusal leaves the device deselected.
        pub fn start(&mut self, tx: &[u8]) -> Result<(), Error> {
            if self.spi.is_busy() {
                return Err(Error::Busy);
            }
            let _ = self.cs.set_low();
            let started = self.spi.start(tx);
            if started.is_err() {
                let _ = self.cs.set_high();
            }
            started
        }

        /// The bytes the last completed transfer clocked in.
        pub fn received(&self) -> &[u8] {
            self.spi.received()
        }

        /// Whether a transfer is running.
        pub fn is_busy(&self) -> bool {
            self.spi.is_busy()
        }

        /// Stops a running transfer, if any, and deselects the device: the
        /// firmware watchdog's recovery.
        pub fn abort(&mut self) {
            self.spi.abort();
            let _ = self.cs.set_high();
        }

        /// The longest transfer the buffers allow.
        pub fn capacity(&self) -> usize {
            self.spi.capacity()
        }

        /// The driver's counts of transfers and faults.
        pub fn health(&self) -> Health {
            self.spi.health()
        }

        /// The vector the group's member must be bound to.
        pub fn vectors(&self) -> Vectors {
            self.spi.vectors()
        }

        /// The chip-select pin, to read, not to drive: the port drives it.
        pub fn chip_select(&self) -> &Cs {
            &self.cs
        }

        /// The driver and the chip select, back.
        pub fn free(self) -> (Spi<W>, Cs) {
            (self.spi, self.cs)
        }
    }

    impl<W: Parts, Cs: OutputPin> Service for Device<W, Cs> {
        type Event = Event;

        /// Serves the driver, and deselects the device when a transfer ends,
        /// whether it completed or failed.
        fn on_interrupt(&mut self) -> Event {
            let event = self.spi.on_interrupt();
            if event.is_some() {
                let _ = self.cs.set_high();
            }
            event
        }
    }

    /// What [`new`] builds: the port's one resource.
    pub struct Resources<U, Sck, Miso, Mosi, TxD, RxD, Cs>
    where
        U: Instance,
    {
        /// The firmware's shared resource, bound as the group's `spi`.
        pub spi: Device<Wiring<U, Sck, Miso, Mosi, TxD, RxD>, Cs>,
    }

    /// Opens the SPI master on `wiring`, with `cs` deselected: `tx_buf` holds
    /// what a transfer sends, `rx_buf` what it clocks in, and the shorter
    /// sets the longest transfer. A refusal is FerroHAL's, with the wiring and
    /// buffers handed back; the chip select is then left deselected.
    #[allow(clippy::type_complexity, clippy::too_many_arguments)]
    pub fn new<U, Sck, Miso, Mosi, TxD, RxD, Cs>(
        rcc: &mut Rcc,
        wiring: Wiring<U, Sck, Miso, Mosi, TxD, RxD>,
        mut cs: Cs,
        config: &Config,
        tx_buf: &'static mut [u8],
        rx_buf: &'static mut [u8],
    ) -> Result<Resources<U, Sck, Miso, Mosi, TxD, RxD, Cs>, SpiRefused<U, Sck, Miso, Mosi, TxD, RxD>>
    where
        U: Instance,
        Sck: SckPin<U>,
        Miso: MisoPin<U>,
        Mosi: MosiPin<U>,
        TxD: SpiTxDma<U>,
        RxD: SpiRxDma<U>,
        Cs: OutputPin,
    {
        let _ = cs.set_high();
        let buffers = Buffers {
            tx: tx_buf,
            rx: rx_buf,
        };
        Spi::new(rcc, wiring, config, buffers).map(|spi| Resources {
            spi: Device::new(spi, cs),
        })
    }

    /// The receive stream's vector: a transfer's end, or a DMA error. The
    /// device is deselected before the outcome is passed on.
    #[ferroforge::task(
        shared = [spi],
        bounds = [spi: Service<Event = Event>],
        spawn = [done(result: Result<usize, Error>)],
    )]
    pub fn on_spi_rx_dma(mut cx: on_spi_rx_dma::Context) {
        if let Some(result) = cx.shared.spi.lock(|spi| spi.on_interrupt()) {
            let _: Result<(), Result<usize, Error>> = cx.spawn.done(result);
        }
    }
}
