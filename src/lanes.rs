//! Compare lanes: FerroHAL's waveform generator,
//! [`Awg`](ferrohal_stm32::timer::Awg), bound to its lanes' vectors. Each lane
//! feeds a timer channel's compare register from its own DMA stream, and each
//! stream raises its own vector; the group's one member serves the generator
//! from every one of them.
//!
//! The group schedules FerroHAL and adds nothing to it. The firmware builds
//! the generator in `init` with FerroHAL's own calls (lane wirings, one timer
//! or two, polarity, outputs) and drives it through FerroHAL's own API under
//! the lock (`load`, `play`, `listen`, `samples`). The member calls
//! `Service::on_interrupt` and passes a nonempty event to the firmware's task
//! through the `events` call, unchanged: which lanes ended, or why the
//! generator stopped. Wakes coalesce, as every port's do: a call that finds
//! the task queued is dropped, and the task reads the generator's own state.
//!
//! Requirements: PORT-001, PORT-002, PORT-501

/// The compare lanes group. A firmware selects it once per generator and
/// declares its member once per lane, each bound to that lane's stream:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::compare_lanes, shared = [generator = lanes],
///         spawn = [events = lanes_events])]
/// mod lanes {
///     #[task(binds = DMA1_STREAM7, priority = 4)] fn on_lane_dma;
///     #[task(binds = DMA1_STREAM2, priority = 4)] fn on_lane_dma;
/// }
/// ```
#[ferroforge::group(
    slots = [generator: G],
    vectors = [on_lane_dma = [generator]],
    shared = [generator: G],
    constructor = new(generator),
)]
pub mod compare_lanes {
    pub use ferrohal_stm32::{
        Service,
        timer::{Awg, Error},
    };

    /// What one service of a lane's stream found: FerroHAL's `Awg` event,
    /// `None` when no lane ended, else the lanes that ended (bit `n` for lane
    /// `n`) or why the generator stopped.
    pub type Event = Option<Result<u8, Error>>;

    /// What [`new`] builds: the port's one resource.
    pub struct Resources<G> {
        /// The firmware's shared resource, bound as the group's `generator`.
        pub generator: G,
    }

    /// Hands the generator the firmware built back as the group's resource.
    pub fn new<G: Service<Event = Event>>(generator: G) -> Resources<G> {
        Resources { generator }
    }

    /// A lane's stream vector: serves the generator and passes what it found
    /// to the firmware's task.
    #[ferroforge::task(
        shared = [generator],
        bounds = [generator: Service<Event = Event>],
        spawn = [events(event: Result<u8, Error>)],
    )]
    pub fn on_lane_dma(mut cx: on_lane_dma::Context) {
        if let Some(event) = cx
            .shared
            .generator
            .lock(|generator| generator.on_interrupt())
        {
            let _: Result<(), Result<u8, Error>> = cx.spawn.events(event);
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
        assert!(contract.config.is_empty());
        let constructor = contract.constructor.as_ref().unwrap();
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(arguments, ["generator"]);
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
        let one: fn(One) -> compare_lanes::Resources<One> = compare_lanes::new;
        let two: fn(Two) -> compare_lanes::Resources<Two> = compare_lanes::new;
        let _ = (one, two);
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
        let one: fn(One) -> compare_lanes::Resources<One> = compare_lanes::new;
        let _ = one;
    }
}
