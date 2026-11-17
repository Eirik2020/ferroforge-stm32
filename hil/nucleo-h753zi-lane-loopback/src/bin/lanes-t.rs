//! PROTOTYPE, variant T: today's `compare_lanes` on the Nucleo-H753ZI. The
//! generator is the firmware's shared resource; the group's member serves it
//! and spawns `lanes_events`, the firmware's task, which calls `listen` under
//! the lock when the last lane has ended, and later copies the samples and
//! calls `speak`. The checks task loads and plays under the lock. Board and
//! links: `src/lib.rs`.
//!
//! `compare_lanes` is unchanged: the timing comes from [`Timed`], the
//! firmware's own wrapper around the generator, which stamps the start of
//! each service the member makes, as the frame port stamps its runs.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0`, `DMA2_STREAM1` | `lanes`' member, once each lane stream | 3 |
//! | `SPI2` | RTIC, dispatching `lanes_events` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::compare_lanes::{self, Error};
    use ferrohal_stm32::{Service, time::Hertz};
    use nucleo_h753zi_lane_loopback::{
        self as bench, Buffers, CLOCKS, Gen, LANE_BUFFER, LANES, LONG_RX, Outcome, Port,
        PortStats, Probe, RESPONDER_BUFFER, WITNESS_BUFFER, Received, Reply, Span, Took, cycles,
    };

    /// The generator with a stamp: when each service began, and when the
    /// one that saw the last lane end began.
    pub struct Timed {
        pub awg: Gen,
        pub ended_at: u32,
        pub runs: Span,
    }

    impl Service for Timed {
        type Event = Option<Result<u8, Error>>;
        fn on_interrupt(&mut self) -> Self::Event {
            let t = cycles();
            let event = self.awg.on_interrupt();
            if matches!(event, Some(Ok(_))) && !self.awg.is_busy() {
                self.ended_at = t;
            }
            self.runs.add(cycles().wrapping_sub(t));
            event
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Phase {
        Idle,
        Sending,
        Listening,
        Done,
    }

    /// The exchange as the firmware's task runs it, and what it read.
    pub struct Log {
        pub phase: Phase,
        pub receive: Option<(Hertz, usize)>,
        pub took: Took,
        pub outcome: Outcome,
        pub count: usize,
        pub rx: [[u16; LONG_RX]; LANES],
        pub stats: PortStats,
    }

    impl Log {
        fn finish(&mut self, outcome: Outcome) {
            self.outcome = outcome;
            self.phase = Phase::Done;
        }
    }

    impl Received for Log {
        fn lane(&self, lane: usize) -> &[u16] {
            match (self.outcome, self.rx.get(lane)) {
                (Outcome::Received, Some(rx)) => &rx[..self.count.min(LONG_RX)],
                _ => &[],
            }
        }
    }

    /// The checks' view of variant T: both locks.
    pub struct T<'a, G: rtic::Mutex<T = Timed>, L: rtic::Mutex<T = Log>> {
        lanes: &'a mut G,
        log: &'a mut L,
    }

    impl<G: rtic::Mutex<T = Timed>, L: rtic::Mutex<T = Log>> Port for T<'_, G, L> {
        fn lanes(&self) -> usize {
            LANES
        }
        fn period(&mut self) -> u32 {
            self.lanes.lock(|g| g.awg.period())
        }
        fn sample_rate(&mut self) -> Hertz {
            self.lanes.lock(|g| g.awg.sample_rate())
        }
        fn submit(&mut self, frames: &[&[u32]], receive: Option<(Hertz, usize)>) -> bool {
            let lanes = &mut *self.lanes;
            let log = &mut *self.log;
            log.lock(|log| {
                lanes.lock(|g| {
                    let t0 = cycles();
                    log.took = Took { submitted: t0, ..Took::default() };
                    log.receive = receive;
                    let mut ok = true;
                    for (lane, frame) in frames.iter().enumerate() {
                        if !frame.is_empty() && g.awg.load(lane, frame).is_err() {
                            ok = false;
                            break;
                        }
                    }
                    ok = ok && g.awg.play().is_ok();
                    if !ok {
                        g.awg.abort();
                        let _ = g.awg.speak();
                        log.phase = Phase::Idle;
                        return false;
                    }
                    let played = cycles();
                    log.took.played = played;
                    log.phase = Phase::Sending;
                    log.stats.start.add(played.wrapping_sub(t0));
                    log.stats.hold.add(cycles().wrapping_sub(t0));
                    true
                })
            })
        }
        fn poll(&mut self, read: &mut dyn FnMut(Outcome, &Took, &dyn Received)) -> bool {
            self.log.lock(|log| {
                if log.phase != Phase::Done {
                    return false;
                }
                log.took.seen = cycles();
                let took = log.took;
                read(log.outcome, &took, &*log);
                log.phase = Phase::Idle;
                true
            })
        }
        fn cancel(&mut self) {
            let lanes = &mut *self.lanes;
            self.log.lock(|log| {
                lanes.lock(|g| {
                    if matches!(log.phase, Phase::Sending | Phase::Listening) {
                        g.awg.abort();
                        let _ = g.awg.speak();
                        log.finish(Outcome::Aborted);
                    }
                })
            });
        }
        fn stats(&mut self) -> PortStats {
            let runs = self.lanes.lock(|g| g.runs);
            let mut s = self.log.lock(|log| log.stats);
            s.run = runs;
            s
        }
        fn clear_stats(&mut self) {
            self.lanes.lock(|g| g.runs = Span::default());
            self.log.lock(|log| log.stats = PortStats::default());
        }
    }

    #[shared]
    struct Shared {
        lanes: Timed,
        log: Log,
    }

    #[local]
    struct Local {
        reply: Reply,
        probe: Probe,
    }

    #[init(local = [
        lane0: [u32; LANE_BUFFER] = [0; LANE_BUFFER],
        lane1: [u32; LANE_BUFFER] = [0; LANE_BUFFER],
        read0: [u16; LONG_RX] = [0; LONG_RX],
        read1: [u16; LONG_RX] = [0; LONG_RX],
        responder: [u32; RESPONDER_BUFFER] = [0; RESPONDER_BUFFER],
        witness: [u16; WITNESS_BUFFER] = [0; WITNESS_BUFFER],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let l = cx.local;
        let buffers = Buffers {
            lanes: [l.lane0, l.lane1],
            samples: [l.read0, l.read1],
            responder: l.responder,
            witness: l.witness,
        };
        let (generator, reply, probe) = bench::setup(cx.device, buffers);
        let lanes = compare_lanes::new(Timed { awg: generator, ended_at: 0, runs: Span::default() });
        defmt::info!("variant T on the Nucleo-H753ZI, {} lane(s), clocks: {}", LANES, CLOCKS);
        checks::spawn().ok();
        let log = Log {
            phase: Phase::Idle,
            receive: None,
            took: Took::default(),
            outcome: Outcome::Failed,
            count: 0,
            rx: [[0; LONG_RX]; LANES],
            stats: PortStats::default(),
        };
        (Shared { lanes: lanes.generator, log }, Local { reply, probe })
    }

    #[group(from = ferroforge_stm32::compare_lanes, shared = [generator = lanes],
            spawn = [events = lanes_events])]
    mod lanes {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn on_lane_dma;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn on_lane_dma;
    }

    /// The firmware's task: on the last lane's end, listen; on the receive's
    /// end, copy what each lane read and speak. Holds the generator's lock
    /// for each step, as today's firmware does.
    #[task(priority = 2, shared = [lanes, log])]
    async fn lanes_events(cx: lanes_events::Context, event: Result<u8, Error>) {
        let lanes_events::SharedResources { mut lanes, mut log, .. } = cx.shared;
        log.lock(|log| {
            lanes.lock(|g| {
                let t0 = cycles();
                match event {
                    Err(_) => {
                        g.awg.abort();
                        let _ = g.awg.speak();
                        log.finish(Outcome::Failed);
                    }
                    Ok(_) if !g.awg.is_busy() => match log.phase {
                        Phase::Sending => {
                            log.took.frame_end = g.ended_at;
                            match log.receive {
                                Some((rate, count)) => {
                                    let before = cycles();
                                    match g.awg.listen(rate, count) {
                                        Ok(_) => {
                                            let after = cycles();
                                            log.stats.turn.add(after.wrapping_sub(g.ended_at));
                                            log.stats.listen.add(after.wrapping_sub(before));
                                            log.took.listening = after;
                                            log.count = count;
                                            log.phase = Phase::Listening;
                                        }
                                        Err(_) => {
                                            g.awg.abort();
                                            let _ = g.awg.speak();
                                            log.finish(Outcome::Failed);
                                        }
                                    }
                                }
                                None => log.finish(Outcome::Sent),
                            }
                        }
                        Phase::Listening => {
                            log.took.received = g.ended_at;
                            let count = log.count;
                            for (lane, rx) in log.rx.iter_mut().enumerate() {
                                let read = g.awg.samples(lane);
                                if let (Some(to), Some(from)) = (rx.get_mut(..count), read.get(..count)) {
                                    to.copy_from_slice(from);
                                }
                            }
                            match g.awg.speak() {
                                Ok(()) => log.finish(Outcome::Received),
                                Err(_) => log.finish(Outcome::Failed),
                            }
                        }
                        _ => {}
                    },
                    Ok(_) => {}
                }
                log.stats.hold.add(cycles().wrapping_sub(t0));
            })
        });
    }

    #[task(priority = 1, shared = [lanes, log], local = [reply, probe])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut lanes, mut log, .. } = cx.shared;
        let mut t = T { lanes: &mut lanes, log: &mut log };
        let failed = bench::run(&bench::board("T"), &mut t, cx.local.reply, cx.local.probe, cycles);
        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        cortex_m::asm::delay(80_000_000);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }
}
