//! PROTOTYPE (event port K): a timer tick, one FerroHAL
//! [`Tick`](ferrohal_stm32::timer::Tick), served from the timer's update
//! vector. The same four steps as `exti_line`'s member over another source.
//!
//! The member reads the firmware's monotonic first, asks the timer whether it
//! fired (FerroHAL's `Service` for `Tick`, `false` on a shared vector raised
//! by another timer), and passes the tick's time to the firmware's task
//! through the `tick` call. A tick that finds the firmware's task still
//! queued or running is not passed on; the member counts it, and the next
//! tick passed on carries the count of those dropped since the one before
//! (`missed`).
//!
//! What the member cannot count: a tick the timer raised while its previous
//! one was still pending. The update flag is one bit, so a member held off
//! for more than a period loses ticks without a trace. The same holds for an
//! EXTI line's pending bit.
//!
//! The rate is the firmware's: it builds the `Tick` in `init` with FerroHAL's
//! `Tick::new` and hands it to [`new`](timer_tick::new). One selection serves
//! one timer on its update vector; timers sharing a vector (TIM1's update and
//! TIM10 on the F401, TIM8's and TIM13 on the H753) need a group that serves
//! both, which this one is not (RM0368 table 38, RM0433 table 147).

/// The tick group. A firmware selects it once per timer:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::timer_tick, local = [tick = tim4],
///         spawn = [tick = control_step])]
/// mod control {
///     #[task(binds = TIM4, priority = 3)] fn on_tick;
/// }
/// ```
#[ferroforge::group(
    slots = [timer: T],
    vectors = [on_tick = timer],
    local = [tick: Tick<T>],
    constructor = new(timer),
)]
pub mod timer_tick {
    pub use ferrohal_stm32::{
        Service,
        timer::{Instance, Tick},
    };

    /// What [`new`] builds: the member's timer.
    pub struct Resources<T: Instance> {
        /// The firmware's local resource, bound as the group's `tick`.
        pub tick: Tick<T>,
    }

    /// Takes a timer `init` has set ticking with FerroHAL's `Tick::new`, and
    /// gives it to the member. The rate stays as `init` set it.
    pub fn new<T: Instance>(timer: Tick<T>) -> Resources<T> {
        Resources { tick: timer }
    }

    /// The count a member keeps after one tick it tried to pass on: none
    /// left once the tick reached the task, one more if it did not.
    pub const fn settle(missed: u32, delivered: bool) -> u32 {
        if delivered {
            0
        } else {
            missed.saturating_add(1)
        }
    }

    /// The timer's update vector: a tick. A pass with the flag clear
    /// (another timer on a shared vector) does nothing.
    #[ferroforge::task(
        local = [tick, missed: u32 = 0],
        bounds = [tick: Service<Event = bool>],
        spawn = [tick(at_us: u64, missed: u32)],
        monotonic = Mono,
    )]
    pub fn on_tick(cx: on_tick::Context) {
        let entered = crate::probe::cycles();
        let at_us = Mono::now().duration_since_epoch().to_micros();
        if !cx.local.tick.on_interrupt() {
            return;
        }
        let missed = *cx.local.missed;
        let delivered = cx.spawn.tick(at_us, missed).is_ok();
        *cx.local.missed = settle(missed, delivered);
        crate::probe::TICK.served(entered);
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use super::timer_tick;
    use crate::tests::groups;

    #[test]
    fn timer_tick_states_its_contract() {
        let found = groups(include_str!("tick.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in tick.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "timer_tick");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["timer"]);
        assert_eq!(contract.vectors[0].member, "on_tick");
        assert_eq!(contract.vectors[0].slot, "timer");
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["tick"]);
        assert!(contract.shared.is_empty() && contract.buffers.is_empty());
        assert!(contract.config.is_empty(), "the rate is the firmware's");
        let constructor = contract.constructor.as_ref().unwrap();
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(arguments, ["timer"]);
        let member = &group.members[0];
        assert_eq!(member.name, "on_tick");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    #[test]
    fn every_tick_is_delivered_or_counted() {
        // Five ticks, the task taking the first, missing three, then the
        // fifth: the fifth carries three, and the count starts again.
        let delivered = [true, false, false, false, true];
        let mut missed = 0;
        let mut carried = Vec::new();
        for delivered in delivered {
            if delivered {
                carried.push(missed);
            }
            missed = timer_tick::settle(missed, delivered);
        }
        assert_eq!(carried, [0, 3]);
        assert_eq!(missed, 0);
        assert_eq!(timer_tick::settle(u32::MAX, false), u32::MAX);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn timer_tick_builds_on_an_f401_timer() {
        use ferrohal_stm32::{chip::TIM4, timer::Tick};
        // TIM4 on its own update vector, TIM4 (RM0368 table 38).
        let constructor: fn(Tick<TIM4>) -> timer_tick::Resources<TIM4> = timer_tick::new;
        let _ = constructor;
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn timer_tick_builds_on_an_h753_timer() {
        use ferrohal_stm32::{chip::TIM4, timer::Tick};
        // TIM4 on its own update vector, TIM4 (RM0433 table 147).
        let constructor: fn(Tick<TIM4>) -> timer_tick::Resources<TIM4> = timer_tick::new;
        let _ = constructor;
    }
}
