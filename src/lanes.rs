//! Compare lanes: one FerroHAL [`Awg`](ferrohal_stm32::timer::Awg), the
//! waveform generator, whose lanes each feed a timer channel's compare
//! register from a DMA stream, all started together, one sample a timer
//! period. Every lane's stream raises its own vector, and the group's one
//! member serves each of them.
//!
//! The generator has one owner, a shared resource. The firmware's own task
//! loads each lane's samples and plays them under the lock, and the member,
//! declared once per lane's stream vector, serves whichever lane ended. A
//! waveform's outcome reaches the firmware's task once, through the `done`
//! call, when the last lane has ended or one has failed: the generator stops
//! every lane on a failure, since its waveforms belong together. It refuses
//! to play again while one plays, so `done` is not coalesced.
//!
//! What the samples mean is the firmware's: a sample is a duty in timer
//! counts held for one period, and a protocol encodes its frames as samples
//! above this port. The same holds for what a two-way lane reads back
//! (`Awg::listen`): the port moves samples and nothing else.
//!
//! Which timers, channels, pins and streams make up the generator is the
//! board's, so the firmware assembles it with FerroHAL in its `init`, from
//! the lane wirings the chip accepts (`LaneWiring::assemble`) and one timer
//! (`Awg::single`) or two (`Awg::linked`, `Awg::independent`), and hands it to
//! [`new`](compare_lanes::new). Its type is the group's `generator` slot.
//!
//! Requirements: PORT-001, PORT-002, PORT-501, PORT-502

/// The compare lanes group. A firmware selects it once per generator and
/// declares its member once per lane, each bound to that lane's stream:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::compare_lanes, shared = [generator = lanes],
///         spawn = [done = lanes_done])]
/// mod lanes {
///     #[task(binds = DMA1_STREAM7, priority = 4)] fn on_lane_dma;
///     #[task(binds = DMA1_STREAM2, priority = 4)] fn on_lane_dma;
/// }
/// ```
#[ferroforge::group(
    slots = [generator: G],
    vectors = [on_lane_dma = [generator]],
    shared = [generator: G],
    config = [active_low: bool @ software],
    constructor = new(generator, active_low),
)]
pub mod compare_lanes {
    use ferrohal_stm32::timer::{ChannelTimer, Lanes};

    pub use ferrohal_stm32::{
        Service,
        timer::{Awg, Error},
    };

    /// What one service of a lane's stream found: `None` when no lane ended,
    /// else the lanes that ended (bit `n` for lane `n`) or why the generator
    /// stopped. FerroHAL's `Awg` event.
    pub type Event = Option<Result<u8, Error>>;

    /// A waveform generator as the member needs it: served through
    /// [`Service`], and able to say whether any lane still plays. Every
    /// FerroHAL [`Awg`] is one.
    pub trait Generator: Service<Event = Event> {
        /// Whether a waveform is loaded or still playing on any lane.
        fn is_busy(&self) -> bool;
    }

    impl<A, B, LA, LB> Generator for Awg<A, B, LA, LB>
    where
        A: ChannelTimer,
        B: ChannelTimer,
        LA: Lanes<A>,
        LB: Lanes<B>,
    {
        fn is_busy(&self) -> bool {
            Awg::is_busy(self)
        }
    }

    /// What [`new`] builds: the port's one resource.
    pub struct Resources<G> {
        /// The firmware's shared resource, bound as the group's `generator`.
        pub generator: G,
    }

    /// Readies `generator` for its first waveform: every output active low
    /// when `active_low` (the line idles high and a pulse pulls it low), else
    /// active high, then every output on at its inactive level, with nothing
    /// playing. No pin pulses until the firmware plays a waveform. Refuses
    /// nothing: whatever the generator's assembly could refuse, FerroHAL
    /// already did.
    pub fn new<A, B, LA, LB>(
        mut generator: Awg<A, B, LA, LB>,
        active_low: bool,
    ) -> Resources<Awg<A, B, LA, LB>>
    where
        A: ChannelTimer,
        B: ChannelTimer,
        LA: Lanes<A>,
        LB: Lanes<B>,
    {
        generator.abort();
        generator.set_active_low(active_low);
        generator.set_outputs(true);
        Resources { generator }
    }

    /// A lane's stream vector: serves every lane, and once no lane plays,
    /// hands the waveform's outcome to the firmware's task.
    #[ferroforge::task(
        shared = [generator],
        bounds = [generator: Generator],
        spawn = [done(result: Result<(), Error>)],
    )]
    pub fn on_lane_dma(mut cx: on_lane_dma::Context) {
        let outcome = cx.shared.generator.lock(|generator| {
            match generator.on_interrupt() {
                // Another lane still plays: its own vector ends the waveform.
                Some(Ok(_)) if generator.is_busy() => None,
                Some(Ok(_)) => Some(Ok(())),
                Some(Err(error)) => Some(Err(error)),
                None => None,
            }
        });
        if let Some(result) = outcome {
            let _: Result<(), Result<(), Error>> = cx.spawn.done(result);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use super::compare_lanes;
    use crate::tests::groups;

    #[test]
    fn compare_lanes_states_its_contract() {
        let found = groups(include_str!("lanes.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in lanes.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "compare_lanes");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["generator"]);
        let [vector] = contract.vectors.as_slice() else {
            panic!("one vector role");
        };
        assert_eq!(vector.member, "on_lane_dma");
        // One member, declared once per lane's stream vector.
        assert!(vector.is_repeated());
        let slots: Vec<_> = vector.slots().iter().map(|s| s.to_string()).collect();
        assert_eq!(slots, ["generator"]);
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["generator"]);
        assert!(contract.buffers.is_empty());
        let config: Vec<_> = contract.config.iter().map(|c| c.name.to_string()).collect();
        assert_eq!(config, ["active_low"]);
        let constructor = contract.constructor.as_ref().unwrap();
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(arguments, ["generator", "active_low"]);
        let member = &group.members[0];
        assert_eq!(member.name, "on_lane_dma");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn compare_lanes_builds_on_f401_generators() {
        use ferrohal_stm32::{
            chip::{TIM2, TIM3},
            dma::Stream,
            gpio::{Alternate, Pin},
            timer::{Awg, CompareLane, LaneWiring},
        };
        // TIM3 CH3 on PB0 (AF2, DS10086 table 9) over DMA1 stream 7, and
        // TIM2 CH3 on PB10 (AF1) over DMA1 stream 1 (RM0368 table 27).
        type Lane3 =
            CompareLane<TIM3, 3, LaneWiring<TIM3, 3, Pin<'B', 0, Alternate<2>>, Stream<1, 7>>>;
        type Lane23 =
            CompareLane<TIM2, 3, LaneWiring<TIM2, 3, Pin<'B', 10, Alternate<1>>, Stream<1, 1>>>;
        // One timer, and two linked (TIM3's trigger output starts TIM2).
        type One = Awg<TIM3, TIM3, (Lane3,), ()>;
        type Two = Awg<TIM3, TIM2, (Lane3,), (Lane23,)>;
        let one: fn(One, bool) -> compare_lanes::Resources<One> = compare_lanes::new;
        let two: fn(Two, bool) -> compare_lanes::Resources<Two> = compare_lanes::new;
        let _ = (one, two);
        fn generator<G: compare_lanes::Generator>() {}
        generator::<One>();
        generator::<Two>();
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn compare_lanes_builds_on_an_h753_generator() {
        use ferrohal_stm32::{
            chip::TIM3,
            dma::Stream,
            gpio::{Alternate, Pin},
            timer::{Awg, CompareLane, LaneWiring},
        };
        // TIM3 CH3 on PB0 and CH4 on PB1 (AF2, DS12117 table 10) over DMA2
        // streams 0 and 1, through DMAMUX1 (RM0433 section 17.3).
        type Lane3 =
            CompareLane<TIM3, 3, LaneWiring<TIM3, 3, Pin<'B', 0, Alternate<2>>, Stream<2, 0>>>;
        type Lane4 =
            CompareLane<TIM3, 4, LaneWiring<TIM3, 4, Pin<'B', 1, Alternate<2>>, Stream<2, 1>>>;
        type One = Awg<TIM3, TIM3, (Lane3, Lane4), ()>;
        let one: fn(One, bool) -> compare_lanes::Resources<One> = compare_lanes::new;
        let _ = one;
        fn generator<G: compare_lanes::Generator>() {}
        generator::<One>();
    }
}
