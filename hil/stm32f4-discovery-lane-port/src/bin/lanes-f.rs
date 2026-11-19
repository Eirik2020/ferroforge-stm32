//! Image F: `lane_frame_port` on the STM32F4 Discovery. The port's
//! member owns FerroHAL's generator and switches to listening in the
//! interrupt that sees the last lane end; the checks task holds the handle
//! and names no generator or lock. Board and links: `src/lib.rs`.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | the lane streams of the build's layout | `lanes`' member, once each lane stream | 3 |
//! | `TIM7` | `lanes`' start member, which the handle pends | 3 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |
//!
//! A stream the build has no lane on never raises its vector.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::lanes_frame::{self, FrameHandle, FrameMemory, FramePort, Outcome as FrameOutcome};
    use ferrohal_stm32::time::Hertz;
    use stm32f4_discovery_lane_port::{
        self as bench, Buffers, CLOCKS, Gen, LANES, LONG, LONG_RX, Outcome, Port,
        PortStats, Probe, RESPONDER_BUFFER, WITNESS_BUFFER, Received, Reply, Took, cycles,
    };

    /// A returned exchange's samples, for the checks.
    pub struct Got<'r, 'h>(&'r lanes_frame::Returned<'h>);

    impl Received for Got<'_, '_> {
        fn lane(&self, lane: usize) -> &[u16] {
            self.0.received(lane)
        }
    }

    /// The checks' view of variant F: the handle, and the port's lock for
    /// its statistics only.
    pub struct F<'a, M: rtic::Mutex<T = FramePort<Gen>>> {
        handle: &'a mut FrameHandle,
        port: &'a mut M,
    }

    impl<M: rtic::Mutex<T = FramePort<Gen>>> Port for F<'_, M> {
        fn lanes(&self) -> usize {
            self.handle.lanes()
        }
        fn period(&mut self) -> u32 {
            self.port.lock(|p| p.generator().period())
        }
        fn sample_rate(&mut self) -> Hertz {
            self.port.lock(|p| p.generator().sample_rate())
        }
        fn submit(&mut self, frames: &[&[u32]], receive: Option<(Hertz, usize)>) -> bool {
            let Ok(mut x) = self.handle.exchange() else {
                return false;
            };
            for (lane, frame) in frames.iter().enumerate() {
                if frame.is_empty() {
                    continue;
                }
                match x.frame(lane, frame.len()) {
                    Some(slot) => slot.copy_from_slice(frame),
                    None => return false,
                }
            }
            if let Some((rate, count)) = receive
                && x.receive(rate, count).is_err()
            {
                return false;
            }
            x.submit().is_ok()
        }
        fn poll(&mut self, read: &mut dyn FnMut(Outcome, &Took, &dyn Received)) -> bool {
            let Some(r) = self.handle.take() else {
                return false;
            };
            let t = r.times();
            let took = Took {
                submitted: t.submitted,
                played: t.played,
                frame_end: t.frame_end,
                listening: t.listening,
                received: t.received,
                seen: cycles(),
            };
            let outcome = match r.outcome() {
                FrameOutcome::Sent => Outcome::Sent,
                FrameOutcome::Received => Outcome::Received,
                FrameOutcome::Aborted => Outcome::Aborted,
                FrameOutcome::Failed(_) => Outcome::Failed,
            };
            read(outcome, &took, &Got(&r));
            true
        }
        fn cancel(&mut self) {
            self.handle.cancel();
        }
        fn stats(&mut self) -> PortStats {
            let s = self.port.lock(|p| p.stats());
            PortStats {
                turn: s.turn,
                listen: s.listen,
                run: s.run,
                hold: Default::default(),
                start: s.start,
                sent: s.sent,
                end: s.end,
                submit: self.handle.submit_stats(),
            }
        }
        fn clear_stats(&mut self) {
            self.port.lock(|p| p.clear_stats());
            self.handle.clear_submit_stats();
        }
        fn show_parts(&mut self, per_us: u32) {
            let (parts, turn) = self.port.lock(|p| (p.stats().parts, p.stats().turn_parts));
            bench::show_start_parts(&parts, LANES, per_us);
            bench::show_turn_parts(&turn, per_us);
        }
    }

    #[shared]
    struct Shared {
        port: FramePort<Gen>,
    }

    #[local]
    struct Local {
        handle: FrameHandle,
        reply: Reply,
        probe: Probe,
    }

    #[init(local = [
        witness: [u16; WITNESS_BUFFER] = [0; WITNESS_BUFFER],
        responder: [u32; RESPONDER_BUFFER] = [0; RESPONDER_BUFFER],
        memory: FrameMemory = FrameMemory::new(),
        // Each lane's share holds the long frame and its closing 0.
        frames: [u32; LANES * (LONG + 1)] = [0; LANES * (LONG + 1)],
        received: [u16; LANES * LONG_RX] = [0; LANES * LONG_RX],
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
        // `lanes::kick` is generated from the group's `pends`: the vector is
        // the member's first binding below.
        let mut built = lanes_frame::new_frame(generator, l.memory, l.frames, l.received, lanes::kick, cycles);
        let Some(handle) = built.handles.take() else {
            defmt::panic!("the handle")
        };
        defmt::info!("variant F on the STM32F4 Discovery, {} lane(s), clocks: {}", LANES, CLOCKS);
        checks::spawn().ok();
        (Shared { port: built.port }, Local { handle, reply, probe })
    }

    #[group(from = ferroforge_stm32::lanes_frame::lane_frame_port, shared = [port = port])]
    mod lanes {
        // The start vector the handle pends: TIM7's, which nothing in this
        // bench raises (its update interrupt stays off).
        #[task(binds = TIM7, priority = 3)]
        fn f_on_start;
        #[task(binds = DMA2_STREAM2, priority = 3)]
        fn f_on_lane_dma;
        #[task(binds = DMA2_STREAM6, priority = 3)]
        fn f_on_lane_dma;
        #[task(binds = DMA2_STREAM3, priority = 3)]
        fn f_on_lane_dma;
        #[task(binds = DMA2_STREAM4, priority = 3)]
        fn f_on_lane_dma;
        // The three-port layout's lane 3.
        #[task(binds = DMA2_STREAM7, priority = 3)]
        fn f_on_lane_dma;
        // The readers armed ahead (`reader-ahead`), or the paced reader.
        #[task(binds = DMA2_STREAM5, priority = 3)]
        fn f_on_lane_dma;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn f_on_lane_dma;
        // The paced-reader layout's lanes 2, 3 and 0.
        #[task(binds = DMA1_STREAM1, priority = 3)]
        fn f_on_lane_dma;
        #[task(binds = DMA1_STREAM6, priority = 3)]
        fn f_on_lane_dma;
        #[task(binds = DMA1_STREAM2, priority = 3)]
        fn f_on_lane_dma;
    }

    #[task(priority = 1, shared = [port], local = [handle, reply, probe])]
    async fn checks(cx: checks::Context) {
        let mut port = cx.shared.port;
        let mut f = F { handle: cx.local.handle, port: &mut port };
        let failed = bench::run(&bench::board("F"), &mut f, cx.local.reply, cx.local.probe, cycles);
        let lost = f.port.lock(|p| p.stats().lost) + f.handle.full();
        let s = f.port.lock(|p| p.stats());
        defmt::info!("copies: by the port {} out, {} in; shares not given back: {}", s.copied_out, s.copied_in, s.stuck);
        let failed = failed + bench::verdict("every lent share given back", s.stuck == 0);
        let failed = failed + bench::verdict("no exchange lost by the port's queues", lost == 0);
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
