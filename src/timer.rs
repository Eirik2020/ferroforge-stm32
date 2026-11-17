//! A periodic tick: one FerroHAL [`Tick`] served from its update vector.
//!
//! The timer has one owner, a shared resource, so a firmware can pause or
//! resume it from its own tasks; the member locks it to serve the vector. Each
//! served update hands the firmware's own task a sequence number counting every
//! update since the port was opened. A tick the task had not yet taken when the
//! next arrived is dropped, as every outgoing call coalesces, and shows as a
//! gap in the sequence: an overrun the firmware can see and count. It carries
//! no protocol; what a tick is for is the firmware's.
//!
//! Requirements: PORT-001, PORT-002, PORT-111

/// The tick port group. A firmware selects it once per timer:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::tick, shared = [timer = loop_timer],
///         spawn = [tick = control])]
/// mod loop_tick {
///     #[task(binds = TIM4, priority = 3)] fn on_tick;
/// }
/// ```
#[ferroforge::group(
    slots = [timer: T],
    vectors = [on_tick = timer],
    shared = [timer: Tick<T>],
    constructor = new,
)]
pub mod tick {
    use ferrohal_stm32::{
        rcc::Rcc,
        resource::Refused,
        time::Hertz,
        timer::{ConfigError, Instance, Tick},
    };

    pub use ferrohal_stm32::Service;

    /// What [`new`] builds: the port's one resource, the running timer.
    pub struct Resources<T: Instance> {
        /// The firmware's shared resource, bound as the group's `timer`.
        pub timer: Tick<T>,
    }

    /// Starts `timer` overflowing at `rate`. A refusal is FerroHAL's, with
    /// the timer handed back: the rate is checked before the timer is touched.
    pub fn new<T: Instance>(
        rcc: &mut Rcc,
        timer: T,
        rate: Hertz,
    ) -> Result<Resources<T>, Refused<ConfigError, T>> {
        Tick::new(rcc, timer, rate).map(|timer| Resources { timer })
    }

    /// The timer's update vector. Serves it and, when its own update was
    /// pending, hands the firmware the update's sequence number.
    #[ferroforge::task(
        shared = [timer],
        bounds = [timer: Service<Event = bool>],
        local = [sequence: u32 = 0],
        spawn = [tick(sequence: u32)],
    )]
    pub fn on_tick(mut cx: on_tick::Context) {
        if cx.shared.timer.lock(|timer| timer.on_interrupt()) {
            *cx.local.sequence = cx.local.sequence.wrapping_add(1);
            let _: Result<(), u32> = cx.spawn.tick(*cx.local.sequence);
        }
    }
}

#[cfg(test)]
mod tests {

    use std::{string::ToString, vec::Vec};

    use super::tick;
    use crate::tests::groups;

    #[test]
    fn tick_states_its_contract() {
        let found = groups(include_str!("timer.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in timer.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "tick");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["timer"]);
        let vectors: Vec<_> = contract
            .vectors
            .iter()
            .map(|v| (v.member.to_string(), v.slot.to_string()))
            .collect();
        assert_eq!(vectors, [("on_tick".to_string(), "timer".to_string())]);
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["timer"]);
        assert!(contract.local.is_empty());
        assert!(contract.constructor.as_ref().unwrap().is_ident("new"));
        let member = group
            .members
            .iter()
            .find(|m| m.name == "on_tick")
            .expect("on_tick is in the group");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    /// Every chip here has TIM3 and TIM4 (RM0368 section 13, RM0433 section
    /// 39), and the constructor takes either.
    #[test]
    #[allow(clippy::type_complexity)]
    fn tick_builds_on_this_chips_timers() {
        use ferrohal_stm32::{
            chip::{TIM3, TIM4},
            rcc::Rcc,
            resource::Refused,
            time::Hertz,
            timer::ConfigError,
        };
        let three: fn(
            &mut Rcc,
            TIM3,
            Hertz,
        ) -> Result<tick::Resources<TIM3>, Refused<ConfigError, TIM3>> = tick::new;
        let four: fn(
            &mut Rcc,
            TIM4,
            Hertz,
        ) -> Result<tick::Resources<TIM4>, Refused<ConfigError, TIM4>> = tick::new;
        let _ = (three, four);
    }
}
