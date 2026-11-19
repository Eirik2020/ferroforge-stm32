//! The ADC port: an ADC sequence as an event port that carries its data
//! (D27 of the port design, `docs/design/event-ports.md`). One FerroHAL
//! [`AdcScan`](ferrohal_stm32::adc::AdcScan) converts a fixed list of
//! channels over a DMA stream; the port's member owns it as a local
//! resource, so the firmware never names or locks it.
//!
//! - The firmware starts a sequence with
//!   [`Kick::kick`](adc_event_port::Kick::kick), from any task: it counts a
//!   request and pends the member through the function FerroForge
//!   generates for the selection (`pends`), and the member starts the
//!   sequence. FerroForge's function only pends, so the request travels in
//!   the port's memory beside it.
//! - When the sequence ends, the member passes the firmware's task a
//!   [`Sequence`](adc_event_port::Sequence) by value: the samples in channel
//!   order (at most 16), when the sequence started by the firmware's
//!   monotonic, the outcome, and what was lost since the last one the task
//!   took.
//!
//! Nothing is lost silently (D8, D16):
//!
//! - A kick while a sequence runs starts nothing. It is counted, and the
//!   count (`late_kicks`) travels with the next sequence the task takes;
//!   several kicks before the member runs are each counted.
//! - A sequence that finds the task still queued or running is dropped and
//!   counted (`missed`), with those among them that failed (`missed_failed`).
//! - A start the driver refuses is passed on as a sequence with no samples
//!   and the refusal as its outcome.
//!
//! The time is when the sequence started, because the age of the samples
//! counts from it. What the samples mean, a divider's ratio or a sensor's
//! scale, is the firmware's: this port converts and nothing else.
//!
//! The channels are the board's configuration, named with FerroHAL's
//! `Channel::of(&pin)` for an analog pin, or `Channel::temperature()` and
//! `Channel::vrefint()` for the internal pair, before the converter opens.
//!
//! Requirements: PORT-001, PORT-002, PORT-402, PORT-403, PORT-404

/// The ADC event port. A firmware selects it once per converter:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::adc_event_port, local = [port = adc1],
///         spawn = [sequence = battery])]
/// mod adc1 {
///     #[task(binds = DMA2_STREAM0, priority = 3)] fn on_adc_dma;
/// }
/// // init: adc_event_port::new(.., memory, adc1::pend), and the `kick`
/// // handle to whichever task starts sequences.
/// ```
#[ferroforge::group(
    slots = [instance: A, stream: S],
    vectors = [on_adc_dma = stream],
    local = [port: AdcPort<AdcScan<A, S>>],
    buffers = [#[dma] samples: [u16]],
    config = [config: Config @ board, channels: [Channel<A>] @ board],
    memory = [memory: KickMemory],
    handles = [kick: Kick],
    pends = [pend = on_adc_dma],
    constructor = new(rcc, instance, stream, config, channels, samples, memory, pend),
)]
pub mod adc_event_port {
    use core::sync::atomic::{AtomicU32, Ordering};

    use ferrohal_stm32::{
        adc::{Adc, Dma, Instance},
        rcc::Rcc,
        resource::Vectors,
    };

    pub use ferrohal_stm32::adc::{AdcScan, Channel, Config, ConfigError, Error};

    /// The most channels a sequence holds (FerroHAL's `into_scan`).
    pub const MAX_CHANNELS: usize = 16;

    /// One sequence, as the firmware's task receives it.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct Sequence {
        samples: [u16; MAX_CHANNELS],
        len: u8,
        /// When the sequence started, in microseconds of the firmware's
        /// monotonic; for a refused start, when it was refused.
        pub at_us: u64,
        /// Whether it completed; the driver's error if not.
        pub outcome: Result<(), Error>,
        /// Sequences dropped since the last one the task took, because the
        /// task was still queued or running.
        pub missed: u32,
        /// How many of those [`missed`](Self::missed) had failed.
        pub missed_failed: u32,
        /// Kicks since the last sequence the task took that started nothing,
        /// because a sequence was running.
        pub late_kicks: u32,
    }

    impl Sequence {
        /// The samples in channel order; empty unless it completed.
        pub fn samples(&self) -> &[u16] {
            self.samples.get(..usize::from(self.len)).unwrap_or(&[])
        }
    }

    /// The port's static memory: the kicks not yet taken.
    pub struct KickMemory {
        requests: AtomicU32,
    }

    impl KickMemory {
        pub const fn new() -> Self {
            Self {
                requests: AtomicU32::new(0),
            }
        }
    }

    impl Default for KickMemory {
        fn default() -> Self {
            Self::new()
        }
    }

    /// What starts a sequence. `Copy`: any number of tasks may hold one.
    #[derive(Clone, Copy)]
    pub struct Kick {
        requests: &'static AtomicU32,
        pend: fn(),
    }

    impl Kick {
        /// Asks for one sequence and pends the member. Never waits.
        pub fn kick(&self) {
            self.requests.fetch_add(1, Ordering::Release);
            (self.pend)();
        }
    }

    /// What the port needs of the converter: FerroHAL's `AdcScan`, or a host
    /// test's stand-in. Each method is the `AdcScan` method of that name;
    /// `serve` is its `on_interrupt`.
    pub trait Converter {
        fn start(&mut self) -> Result<(), Error>;
        fn serve(&mut self) -> Option<Result<(), Error>>;
        fn samples(&self) -> &[u16];
        fn is_busy(&self) -> bool;
    }

    impl<A: Instance, S> Converter for AdcScan<A, S> {
        fn start(&mut self) -> Result<(), Error> {
            AdcScan::start(self)
        }
        fn serve(&mut self) -> Option<Result<(), Error>> {
            AdcScan::on_interrupt(self)
        }
        fn samples(&self) -> &[u16] {
            AdcScan::samples(self)
        }
        fn is_busy(&self) -> bool {
            AdcScan::is_busy(self)
        }
    }

    /// What the member needs of its local resource.
    pub trait SequencePort {
        /// One pass of the member, at `now_us` of the firmware's monotonic:
        /// the sequence to pass on, if any.
        fn serve(&mut self, now_us: u64) -> Option<Sequence>;
        /// Whether the sequence [`serve`](Self::serve) gave reached the
        /// task, and whether it had failed.
        fn delivered(&mut self, delivered: bool, failed: bool);
    }

    /// The member's local resource: the converter and the counts.
    pub struct AdcPort<C> {
        converter: C,
        requests: &'static AtomicU32,
        pend: fn(),
        started_us: u64,
        refused: Option<(Error, u64)>,
        missed: u32,
        missed_failed: u32,
        late_kicks: u32,
    }

    impl<C: Converter> AdcPort<C> {
        /// The port over `converter`, taking kicks from `memory`; `pend`
        /// brings the member back.
        pub fn new(converter: C, memory: &'static KickMemory, pend: fn()) -> Self {
            Self {
                converter,
                requests: &memory.requests,
                pend,
                started_us: 0,
                refused: None,
                missed: 0,
                missed_failed: 0,
                late_kicks: 0,
            }
        }

        /// Host tests: the converter, to drive the stand-in.
        #[cfg(test)]
        pub(crate) fn converter_mut(&mut self) -> &mut C {
            &mut self.converter
        }

        /// The kick handle for this port's memory.
        pub fn kick(&self) -> Kick {
            Kick {
                requests: self.requests,
                pend: self.pend,
            }
        }

        fn sequence(&self, at_us: u64, outcome: Result<(), Error>) -> Sequence {
            let mut sequence = Sequence {
                samples: [0; MAX_CHANNELS],
                len: 0,
                at_us,
                outcome,
                missed: self.missed,
                missed_failed: self.missed_failed,
                late_kicks: self.late_kicks,
            };
            if outcome.is_ok() {
                let samples = self.converter.samples();
                for (to, from) in sequence.samples.iter_mut().zip(samples) {
                    *to = *from;
                }
                sequence.len = samples.len().min(MAX_CHANNELS) as u8;
            }
            sequence
        }
    }

    impl<A: Instance, S> AdcPort<AdcScan<A, S>> {
        /// The largest sample, the full scale, for `init` to hand the task
        /// that reads the samples.
        pub const fn max_sample(&self) -> u16 {
            self.converter.max_sample()
        }

        /// The vector the member must be bound to: the stream's. `init` can
        /// keep it to check, once the scheduler runs, that it is enabled.
        pub fn vectors(&self) -> Vectors {
            self.converter.vectors()
        }
    }

    impl<C: Converter> SequencePort for AdcPort<C> {
        fn serve(&mut self, now_us: u64) -> Option<Sequence> {
            // The end first: a kick taken in the same pass starts the next
            // sequence over the samples this one leaves.
            let ended = self
                .converter
                .serve()
                .map(|outcome| self.sequence(self.started_us, outcome));
            let requests = self.requests.swap(0, Ordering::Acquire);
            if requests > 0 {
                if self.converter.is_busy() {
                    self.late_kicks = self.late_kicks.saturating_add(requests);
                } else {
                    self.late_kicks = self.late_kicks.saturating_add(requests - 1);
                    match self.converter.start() {
                        Ok(()) => {
                            self.started_us = now_us;
                        }
                        Err(error) => self.refused = Some((error, now_us)),
                    }
                }
            }
            if ended.is_some() {
                if self.refused.is_some() {
                    // One sequence a pass: the refusal goes out on the next.
                    (self.pend)();
                }
                return ended;
            }
            self.refused
                .take()
                .map(|(error, at_us)| self.sequence(at_us, Err(error)))
        }

        fn delivered(&mut self, delivered: bool, failed: bool) {
            if delivered {
                self.missed = 0;
                self.missed_failed = 0;
                self.late_kicks = 0;
            } else {
                self.missed = self.missed.saturating_add(1);
                self.missed_failed = self.missed_failed.saturating_add(u32::from(failed));
            }
        }
    }

    /// What [`new`] builds.
    pub struct Resources<A, S> {
        /// The firmware's local resource, bound as the group's `port`.
        pub port: AdcPort<AdcScan<A, S>>,
        /// For whichever task starts sequences.
        pub kick: Kick,
    }

    /// Why [`new`] refused, with everything it took handed back: the ADC
    /// powered down.
    pub struct Refusal<A, S> {
        /// Why: `ConfigError::Sequence` for a sequence that is empty, longer
        /// than [`MAX_CHANNELS`] or longer than `samples`, checked before
        /// anything opens; else the driver's own refusal.
        pub error: ConfigError,
        pub instance: A,
        pub stream: S,
        pub samples: &'static mut [u16],
        pub memory: &'static mut KickMemory,
    }

    /// Checks the sequence against `samples` before it opens anything, then
    /// opens the ADC with `config` and readies a sequence of `channels`, in
    /// order, over `stream` into `samples`. Nothing converts until a kick.
    #[allow(clippy::too_many_arguments)]
    pub fn new<A, S>(
        rcc: &mut Rcc,
        instance: A,
        stream: S,
        config: &Config,
        channels: &[Channel<A>],
        samples: &'static mut [u16],
        memory: &'static mut KickMemory,
        pend: fn(),
    ) -> Result<Resources<A, S>, Refusal<A, S>>
    where
        A: Instance,
        S: Dma<A>,
    {
        if channels.is_empty() || channels.len() > MAX_CHANNELS || channels.len() > samples.len() {
            return Err(Refusal {
                error: ConfigError::Sequence,
                instance,
                stream,
                samples,
                memory,
            });
        }
        let adc = match Adc::new(rcc, instance, config) {
            Ok(adc) => adc,
            Err(refused) => {
                let (error, instance) = refused.into_inner();
                return Err(Refusal {
                    error,
                    instance,
                    stream,
                    samples,
                    memory,
                });
            }
        };
        match adc.into_scan(stream, channels, samples) {
            Ok(scan) => {
                let port = AdcPort::new(scan, memory, pend);
                let kick = port.kick();
                Ok(Resources { port, kick })
            }
            Err(refused) => {
                let (error, (adc, stream, samples)) = refused.into_inner();
                Err(Refusal {
                    error,
                    instance: adc.free(),
                    stream,
                    samples,
                    memory,
                })
            }
        }
    }

    /// The stream's vector: a sequence's end, a DMA error, or a kick.
    #[ferroforge::task(
        local = [port],
        bounds = [port: SequencePort],
        spawn = [sequence(result: Sequence)],
        monotonic = Mono,
    )]
    pub fn on_adc_dma(cx: on_adc_dma::Context) {
        let now_us = Mono::now().duration_since_epoch().to_micros();
        if let Some(result) = cx.local.port.serve(now_us) {
            let failed = result.outcome.is_err();
            let delivered = cx.spawn.sequence(result).is_ok();
            cx.local.port.delivered(delivered, failed);
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::{boxed::Box, string::ToString, vec, vec::Vec};

    use super::adc_event_port::{AdcPort, Converter, Error, KickMemory, Sequence, SequencePort};
    use crate::tests::groups;

    #[test]
    fn adc_event_port_states_its_contract() {
        let found = groups(include_str!("adc_port.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in adc_port.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "adc_event_port");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["instance", "stream"]);
        assert_eq!(contract.vectors[0].member, "on_adc_dma");
        assert_eq!(contract.vectors[0].slot, "stream");
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["port"]);
        assert!(contract.shared.is_empty(), "the firmware never locks it");
        let config: Vec<_> = contract.config.iter().map(|c| c.name.to_string()).collect();
        assert_eq!(config, ["config", "channels"]);
        let memory: Vec<_> = contract.memory.iter().map(|m| m.name.to_string()).collect();
        assert_eq!(memory, ["memory"]);
        let handles: Vec<_> = contract
            .handles
            .iter()
            .map(|h| h.name.to_string())
            .collect();
        assert_eq!(handles, ["kick"]);
        let pends: Vec<_> = contract
            .pends
            .iter()
            .map(|p| (p.name.to_string(), p.member.to_string()))
            .collect();
        assert_eq!(pends, [("pend".to_string(), "on_adc_dma".to_string())]);
        let constructor = contract.constructor.as_ref().unwrap();
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(
            arguments,
            [
                "rcc", "instance", "stream", "config", "channels", "samples", "memory", "pend"
            ]
        );
        let member = &group.members[0];
        assert_eq!(member.name, "on_adc_dma");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    /// A stand-in converter: runs until the test ends it.
    #[derive(Default)]
    struct Fake {
        running: bool,
        done: bool,
        end: Option<Result<(), Error>>,
        refuse: Option<Error>,
        starts: u32,
        values: Vec<u16>,
    }

    impl Converter for Fake {
        fn start(&mut self) -> Result<(), Error> {
            if let Some(error) = self.refuse {
                return Err(error);
            }
            if self.running {
                return Err(Error::Busy);
            }
            self.running = true;
            self.done = false;
            self.starts += 1;
            Ok(())
        }
        fn serve(&mut self) -> Option<Result<(), Error>> {
            if !self.running {
                return None;
            }
            let end = self.end.take()?;
            self.running = false;
            self.done = end.is_ok();
            Some(end)
        }
        fn samples(&self) -> &[u16] {
            if self.done { &self.values } else { &[] }
        }
        fn is_busy(&self) -> bool {
            self.running
        }
    }

    fn pend() {}

    fn port() -> AdcPort<Fake> {
        let memory: &'static KickMemory = Box::leak(Box::new(KickMemory::new()));
        let fake = Fake {
            values: vec![10, 20, 30],
            ..Fake::default()
        };
        AdcPort::new(fake, memory, pend)
    }

    /// Ends the running sequence, serves it, and says whether the task took
    /// it.
    fn end(port: &mut AdcPort<Fake>, at: u64, outcome: Result<(), Error>, taken: bool) -> Sequence {
        converter(port).end = Some(outcome);
        let sequence = port.serve(at).expect("the end is passed on");
        port.delivered(taken, sequence.outcome.is_err());
        sequence
    }

    fn converter(port: &mut AdcPort<Fake>) -> &mut Fake {
        port.converter_mut()
    }

    #[test]
    fn a_kick_starts_one_sequence_and_its_end_carries_its_samples() {
        let mut port = port();
        port.kick().kick();
        assert_eq!(port.serve(100), None, "a start passes nothing on");
        assert_eq!(converter(&mut port).starts, 1);
        let first = end(&mut port, 150, Ok(()), true);
        assert_eq!(first.samples(), [10, 20, 30]);
        assert_eq!(first.at_us, 100, "the time is the start's");
        assert_eq!((first.missed, first.late_kicks), (0, 0));
    }

    #[test]
    fn a_kick_while_running_starts_nothing_and_is_counted() {
        let mut port = port();
        port.kick().kick();
        port.serve(0);
        // Two kicks before the member runs, while the sequence runs.
        port.kick().kick();
        port.kick().kick();
        assert_eq!(port.serve(1), None);
        assert_eq!(converter(&mut port).starts, 1);
        let ended = end(&mut port, 2, Ok(()), true);
        assert_eq!(ended.late_kicks, 2);
        port.kick().kick();
        port.serve(3);
        let next = end(&mut port, 4, Ok(()), true);
        assert_eq!(next.late_kicks, 0, "the count starts again once taken");
    }

    #[test]
    fn a_sequence_the_task_does_not_take_is_counted_on_the_next() {
        let mut port = port();
        for round in 0..3u64 {
            port.kick().kick();
            port.serve(round * 10);
            converter(&mut port).values = vec![round as u16];
            let outcome = if round == 1 {
                Err(Error::Overrun)
            } else {
                Ok(())
            };
            let sequence = end(&mut port, round * 10 + 5, outcome, round == 2);
            if round == 2 {
                // Each result carries its own samples: the window adc_scan
                // leaves, a start before the task read the last, is closed.
                assert_eq!(sequence.samples(), [2]);
                assert_eq!((sequence.missed, sequence.missed_failed), (2, 1));
            }
        }
        port.kick().kick();
        port.serve(40);
        let after = end(&mut port, 45, Ok(()), true);
        assert_eq!((after.missed, after.missed_failed), (0, 0));
    }

    #[test]
    fn a_refused_start_is_passed_on_as_its_outcome() {
        let mut port = port();
        converter(&mut port).refuse = Some(Error::Dma);
        port.kick().kick();
        let refused = port.serve(7).expect("the refusal is passed on");
        assert_eq!(refused.outcome, Err(Error::Dma));
        assert_eq!(refused.at_us, 7);
        assert!(refused.samples().is_empty());
    }

    #[test]
    fn a_refused_start_after_an_end_goes_out_on_the_next_pass() {
        let mut port = port();
        port.kick().kick();
        port.serve(0);
        port.kick().kick();
        converter(&mut port).end = Some(Ok(()));
        converter(&mut port).refuse = Some(Error::Dma);
        let ended = port.serve(5).expect("the end first");
        assert_eq!(ended.outcome, Ok(()));
        port.delivered(true, false);
        let refused = port.serve(6).expect("then the refusal");
        assert_eq!(refused.outcome, Err(Error::Dma));
        assert_eq!(port.serve(7), None);
    }

    #[test]
    fn a_pass_with_nothing_to_do_passes_nothing_on() {
        let mut port = port();
        assert_eq!(port.serve(0), None);
        assert_eq!(converter(&mut port).starts, 0);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn adc_event_port_builds_on_an_f401_converter() {
        use super::adc_event_port::{self, Channel, Config, Refusal, Resources};
        use ferrohal_stm32::{chip::ADC1, dma::Stream};
        type New = fn(
            &mut ferrohal_stm32::rcc::Rcc,
            ADC1,
            Stream<2, 0>,
            &Config,
            &[Channel<ADC1>],
            &'static mut [u16],
            &'static mut KickMemory,
            fn(),
        )
            -> Result<Resources<ADC1, Stream<2, 0>>, Refusal<ADC1, Stream<2, 0>>>;
        let constructor: New = adc_event_port::new;
        let _ = constructor;
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn adc_event_port_builds_on_an_h753_converter() {
        use super::adc_event_port::{self, Channel, Config, Refusal, Resources};
        use ferrohal_stm32::{chip::ADC3, dma::Stream};
        type New = fn(
            &mut ferrohal_stm32::rcc::Rcc,
            ADC3,
            Stream<2, 0>,
            &Config,
            &[Channel<ADC3>],
            &'static mut [u16],
            &'static mut KickMemory,
            fn(),
        )
            -> Result<Resources<ADC3, Stream<2, 0>>, Refusal<ADC3, Stream<2, 0>>>;
        let constructor: New = adc_event_port::new;
        let _ = constructor;
    }
}
