//! An ADC sequence: one FerroHAL [`AdcScan`](ferrohal_stm32::adc::AdcScan),
//! converting a fixed list of channels over a DMA stream, served from that
//! stream's vector.
//!
//! The scan has one owner, a shared [`Scan`](adc_scan::Scan). The firmware's
//! own task starts each sequence under the lock, at whatever rate its source
//! needs, and the group's member serves the stream's vector, which ends it.
//! The scan keeps the outcome, and the member wakes the firmware's task
//! through the `done` call; that task takes the outcome, and the samples in
//! channel order with it, with [`Scan::take`](adc_scan::Scan::take).
//!
//! What the samples mean, a divider's ratio or a sensor's scale, is the
//! firmware's: this port converts and nothing else. The scan refuses a start
//! while a sequence runs, and also while the last one's outcome waits to be
//! taken, so a task that starts sequences at a higher priority than the one
//! that takes them cannot overwrite samples not yet read, or end a sequence
//! whose `done` would find the last one still queued. At most one outcome is
//! in flight, so `done` is never coalesced.
//!
//! The channels are the board's configuration, named with FerroHAL's
//! `Channel::of(&pin)` for an analog pin, or `Channel::temperature()` and
//! `Channel::vrefint()` for the internal pair, before the converter opens.
//!
//! Requirements: PORT-001, PORT-002, PORT-401, PORT-402

/// The ADC sequence group. A firmware selects it once per converter:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::adc_scan, shared = [scan = adc1],
///         spawn = [done = adc1_done])]
/// mod adc1 {
///     #[task(binds = DMA2_STREAM0, priority = 2)] fn on_adc_dma;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: A, stream: S],
    vectors = [on_adc_dma = stream],
    shared = [scan: Scan<A, S>],
    buffers = [#[dma] samples: [u16]],
    config = [config: Config @ board, channels: [Channel<A>] @ board],
    constructor = new(rcc, instance, stream, config, channels, samples),
)]
pub mod adc_scan {
    use ferrohal_stm32::{
        adc::{Adc, AdcScan, Dma, Instance},
        rcc::Rcc,
        resource::Vectors,
    };

    pub use ferrohal_stm32::{
        Service,
        adc::{Channel, Config, ConfigError, Error},
    };

    /// What one service of the stream found: `None` when nothing ended, else
    /// whether the sequence completed.
    pub type Event = Option<Result<(), Error>>;

    /// One ADC sequence: FerroHAL's scan, and the last sequence's outcome
    /// until the firmware's task takes it.
    pub struct Scan<A, S> {
        scan: AdcScan<A, S>,
        outcome: Option<Result<(), Error>>,
    }

    impl<A: Instance, S> Scan<A, S> {
        /// Converts the sequence once; returns at once. Refused with
        /// `Error::Busy` while a sequence runs or the last one's outcome has
        /// not been taken.
        pub fn start(&mut self) -> Result<(), Error> {
            if self.outcome.is_some() {
                return Err(Error::Busy);
            }
            self.scan.start()
        }

        /// Takes the last sequence's outcome: its samples, in channel order,
        /// or why it failed. `None` when there is none to take. Taking it lets
        /// the next sequence start; the samples stay readable only until then.
        ///
        /// A `done` task that starts the next sequence itself takes again
        /// before it returns: a sequence that ends while that task still runs
        /// cannot wake it, and its outcome waits here, refusing further
        /// starts, until it is taken.
        pub fn take(&mut self) -> Option<Result<&[u16], Error>> {
            match self.outcome.take()? {
                Ok(()) => Some(Ok(self.scan.samples())),
                Err(error) => Some(Err(error)),
            }
        }

        /// Whether a sequence is running.
        pub const fn is_busy(&self) -> bool {
            self.scan.is_busy()
        }

        /// Stops a sequence that never finished, and drops an outcome not yet
        /// taken: the firmware watchdog's recovery.
        pub fn abort(&mut self) {
            self.scan.abort();
            self.outcome = None;
        }

        /// The largest sample: the full scale.
        pub const fn max_sample(&self) -> u16 {
            self.scan.max_sample()
        }

        /// The vector the group's member must be bound to.
        pub fn vectors(&self) -> Vectors {
            self.scan.vectors()
        }

        /// FerroHAL's scan, back.
        pub fn free(self) -> AdcScan<A, S> {
            self.scan
        }
    }

    impl<A: Instance, S> Service for Scan<A, S> {
        type Event = Event;

        /// Serves the scan, and keeps the outcome of a sequence that ends for
        /// [`take`](Scan::take).
        fn on_interrupt(&mut self) -> Event {
            let event = self.scan.on_interrupt();
            if event.is_some() {
                self.outcome = event;
            }
            event
        }
    }

    /// What [`new`] builds: the port's one resource.
    pub struct Resources<A, S> {
        /// The firmware's shared resource, bound as the group's `scan`.
        pub scan: Scan<A, S>,
    }

    /// Why [`new`] refused, with everything it took handed back: the ADC
    /// powered down.
    pub struct Refusal<A, S> {
        pub error: ConfigError,
        pub instance: A,
        pub stream: S,
        pub samples: &'static mut [u16],
    }

    /// Opens the ADC with `config` and readies a sequence of `channels`, in
    /// order, over `stream` into `samples`, which must hold at least one
    /// sample a channel. Nothing converts until the firmware starts it.
    pub fn new<A, S>(
        rcc: &mut Rcc,
        instance: A,
        stream: S,
        config: &Config,
        channels: &[Channel<A>],
        samples: &'static mut [u16],
    ) -> Result<Resources<A, S>, Refusal<A, S>>
    where
        A: Instance,
        S: Dma<A>,
    {
        let adc = match Adc::new(rcc, instance, config) {
            Ok(adc) => adc,
            Err(refused) => {
                let (error, instance) = refused.into_inner();
                return Err(Refusal {
                    error,
                    instance,
                    stream,
                    samples,
                });
            }
        };
        match adc.into_scan(stream, channels, samples) {
            Ok(scan) => Ok(Resources {
                scan: Scan {
                    scan,
                    outcome: None,
                },
            }),
            Err(refused) => {
                let (error, (adc, stream, samples)) = refused.into_inner();
                Err(Refusal {
                    error,
                    instance: adc.free(),
                    stream,
                    samples,
                })
            }
        }
    }

    /// The stream's vector: a sequence's end, or a DMA error. The scan keeps
    /// the outcome before the firmware's task is woken to take it.
    #[ferroforge::task(
        shared = [scan],
        bounds = [scan: Service<Event = Event>],
        spawn = [done()],
    )]
    pub fn on_adc_dma(mut cx: on_adc_dma::Context) {
        if cx.shared.scan.lock(|scan| scan.on_interrupt()).is_some() {
            // The scan refuses a start until the outcome is taken, so the last
            // `done` has run and this one cannot find it queued.
            let _: Result<(), ()> = cx.spawn.done();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use super::adc_scan;
    use crate::tests::groups;

    #[test]
    fn adc_scan_states_its_contract() {
        let found = groups(include_str!("adc.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in adc.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "adc_scan");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["instance", "stream"]);
        assert_eq!(contract.vectors[0].member, "on_adc_dma");
        assert_eq!(contract.vectors[0].slot, "stream");
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["scan"]);
        let buffers: Vec<_> = contract
            .buffers
            .iter()
            .map(|b| (b.name.to_string(), b.element.to_string(), b.dma))
            .collect();
        assert_eq!(buffers, [("samples".to_string(), "u16".to_string(), true)]);
        let config: Vec<_> = contract.config.iter().map(|c| c.name.to_string()).collect();
        assert_eq!(config, ["config", "channels"]);
        let constructor = contract.constructor.as_ref().unwrap();
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(
            arguments,
            ["rcc", "instance", "stream", "config", "channels", "samples"]
        );
        let member = &group.members[0];
        assert_eq!(member.name, "on_adc_dma");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn adc_scan_builds_on_an_f401_converter() {
        use ferrohal_stm32::{chip::ADC1, dma::Stream};
        // ADC1 on DMA2 stream 0, channel 0 (RM0368 table 29).
        type New = fn(
            &mut ferrohal_stm32::rcc::Rcc,
            ADC1,
            Stream<2, 0>,
            &adc_scan::Config,
            &[adc_scan::Channel<ADC1>],
            &'static mut [u16],
        ) -> Result<
            adc_scan::Resources<ADC1, Stream<2, 0>>,
            adc_scan::Refusal<ADC1, Stream<2, 0>>,
        >;
        let constructor: New = adc_scan::new;
        let _ = constructor;
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn adc_scan_builds_on_an_h753_converter() {
        use ferrohal_stm32::{chip::ADC3, dma::Stream};
        // ADC3 on DMA2 stream 0, through DMAMUX1 (RM0433 section 17.3).
        type New = fn(
            &mut ferrohal_stm32::rcc::Rcc,
            ADC3,
            Stream<2, 0>,
            &adc_scan::Config,
            &[adc_scan::Channel<ADC3>],
            &'static mut [u16],
        ) -> Result<
            adc_scan::Resources<ADC3, Stream<2, 0>>,
            adc_scan::Refusal<ADC3, Stream<2, 0>>,
        >;
        let constructor: New = adc_scan::new;
        let _ = constructor;
    }
}
