//! Variant T, FerroHAL's lanes under RTIC with no port, on the STM32F4 Discovery. The
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
//! | the lane streams of the build's layout | a hardware task each, serving the generator | 3 |
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
    use ferrohal_stm32::{Service, time::Hertz, timer::Error};
    use stm32f4_discovery_lane_port::{
        self as bench, Buffers, CLOCKS, Gen, LANE_BUFFER, LANES, LONG_RX, Outcome, Port,
        PortStats, Probe, RESPONDER_BUFFER, WITNESS_BUFFER, Received, Reply, Span, Took, cycles,
    };

    /// The generator with a stamp: when each service began, and when the
    /// one that saw the last lane end began.
    pub struct Timed {
        pub awg: Gen,
        /// Each lane's frame memory, while not lent to it.
        pub words: [Option<&'static mut [u32]>; 4],
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
                    // The listen is computed before the frame,
                    // outside the turn-around.
                    if let Some((rate, count)) = receive {
                        let fresh = match g.awg.prepared() {
                            Some((r, c, _)) => r != rate || c != count,
                            None => true,
                        };
                        if fresh && g.awg.prepare_listen(rate, count).is_err() {
                            ok = false;
                        }
                    }
                    for (lane, frame) in frames.iter().enumerate() {
                        if !ok {
                            break;
                        }
                        if !frame.is_empty()
                            && !g.words.get_mut(lane).is_some_and(|words| {
                                bench::load_words(&mut g.awg, lane, words, frame)
                            })
                        {
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
        lane2: [u32; LANE_BUFFER] = [0; LANE_BUFFER],
        lane3: [u32; LANE_BUFFER] = [0; LANE_BUFFER],
        read0: [u16; LONG_RX] = [0; LONG_RX],
        read1: [u16; LONG_RX] = [0; LONG_RX],
        read2: [u16; LONG_RX] = [0; LONG_RX],
        read3: [u16; LONG_RX] = [0; LONG_RX],
        witness: [u16; WITNESS_BUFFER] = [0; WITNESS_BUFFER],
        responder: [u32; RESPONDER_BUFFER] = [0; RESPONDER_BUFFER],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let l = cx.local;
        let buffers = Buffers {
            witness: l.witness,
            responder: l.responder,
        };
        let (generator, reply, probe) = bench::setup(cx.device, buffers);
        // A lane keeps no memory (FerroHAL ADR-014): each lane reads into
        // the memory lent here, every exchange, and plays frames from its
        // block of `words`.
        let mut generator = generator;
        for (lane, read) in [l.read0, l.read1, l.read2, l.read3].into_iter().enumerate() {
            if generator.lend_samples(lane, read).is_err() && lane < LANES {
                defmt::panic!("lending lane {}'s samples", lane);
            }
        }
        let words: [Option<&'static mut [u32]>; 4] =
            [Some(&mut l.lane0[..]), Some(&mut l.lane1[..]), Some(&mut l.lane2[..]), Some(&mut l.lane3[..])];
        let lanes = Timed { awg: generator, words, ended_at: 0, runs: Span::default() };
        defmt::info!("variant T on the STM32F4 Discovery, {} lane(s), clocks: {}", LANES, CLOCKS);
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
        (Shared { lanes, log }, Local { reply, probe })
    }

    /// Serves the generator from a lane stream's vector and passes a
    /// nonempty event to the firmware's task: what a task group would do,
    /// written out, so this image measures FerroHAL's lanes under RTIC with
    /// no port in between. A vector no lane of the layout uses never runs.
    fn serve(lanes: &mut impl rtic::Mutex<T = Timed>) {
        if let Some(event) = lanes.lock(|g| g.on_interrupt()) {
            lanes_events::spawn(event).ok();
        }
    }

    #[task(binds = DMA2_STREAM2, priority = 3, shared = [lanes])]
    fn on_dma2_stream2(mut cx: on_dma2_stream2::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA2_STREAM6, priority = 3, shared = [lanes])]
    fn on_dma2_stream6(mut cx: on_dma2_stream6::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA2_STREAM3, priority = 3, shared = [lanes])]
    fn on_dma2_stream3(mut cx: on_dma2_stream3::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA2_STREAM4, priority = 3, shared = [lanes])]
    fn on_dma2_stream4(mut cx: on_dma2_stream4::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA2_STREAM7, priority = 3, shared = [lanes])]
    fn on_dma2_stream7(mut cx: on_dma2_stream7::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA2_STREAM5, priority = 3, shared = [lanes])]
    fn on_dma2_stream5(mut cx: on_dma2_stream5::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA2_STREAM1, priority = 3, shared = [lanes])]
    fn on_dma2_stream1(mut cx: on_dma2_stream1::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA1_STREAM1, priority = 3, shared = [lanes])]
    fn on_dma1_stream1(mut cx: on_dma1_stream1::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA1_STREAM6, priority = 3, shared = [lanes])]
    fn on_dma1_stream6(mut cx: on_dma1_stream6::Context) {
        serve(&mut cx.shared.lanes);
    }

    #[task(binds = DMA1_STREAM2, priority = 3, shared = [lanes])]
    fn on_dma1_stream2(mut cx: on_dma1_stream2::Context) {
        serve(&mut cx.shared.lanes);
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
                                    let _ = rate;
                                    match g.awg.listen() {
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
        cortex_m::asm::delay(33_600_000);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }
}
