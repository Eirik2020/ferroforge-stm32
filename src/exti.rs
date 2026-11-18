//! An external-interrupt line: one FerroHAL EXTI line, routed to a pin,
//! served from its vector.
//!
//! The member checks and clears the line's pending flag, reads the firmware's
//! monotonic in the interrupt, and passes the edge's time to the firmware's
//! task through the `edge` call. The time is read before anything else, so it
//! is the edge's, not the scheduler's: a task that times a sample by the edge
//! that announced it gets the edge's time at any priority of its own.
//!
//! An edge that finds the firmware's task still queued is not passed on; the
//! member counts it, and the next edge passed on carries the count of those
//! dropped since the one before (`missed`). The line has no other state, and
//! no owner: FerroHAL's [`ExtiHandle`](ferrohal_stm32::exti::ExtiHandle)
//! touches only its own line's bits, so it is the member's local resource.
//!
//! Lines 0 to 4 each have their own vector; 5 to 9 share one, and 10 to 15
//! another (RM0368 table 38, RM0433 table 147). One selection serves one line
//! on its vector, so two lines on a shared vector need a group that serves
//! both, which this one is not.
//!
//! The pin's mode and pull, and the edge, are the firmware's: it routes the
//! line in `init` with FerroHAL's `Line::route` and hands the routed line to
//! [`new`](exti_line::new).
//!
//! Requirements: PORT-001, PORT-002, PORT-301, PORT-302

/// The EXTI line group. A firmware selects it once per line:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::exti_line, local = [line = button],
///         spawn = [edge = button_edge])]
/// mod button {
///     #[task(binds = EXTI0, priority = 3)] fn on_exti_line;
/// }
/// ```
#[ferroforge::group(
    slots = [line: L],
    vectors = [on_exti_line = line],
    local = [line: ExtiHandle],
    constructor = new(line),
)]
pub mod exti_line {
    use ferrohal_stm32::gpio::{Port, PortInstance};

    pub use ferrohal_stm32::exti::{Exti, ExtiHandle};

    /// A routed EXTI line: what [`new`] takes.
    pub trait Routed {
        /// The handle an interrupt handler uses.
        fn handle(&self) -> ExtiHandle;
    }

    impl<const P: char, const N: u8> Routed for Exti<P, N>
    where
        Port<P>: PortInstance,
    {
        fn handle(&self) -> ExtiHandle {
            Exti::handle(self)
        }
    }

    /// What [`new`] builds: the member's handle on the line.
    pub struct Resources {
        /// The firmware's local resource, bound as the group's `line`.
        pub line: ExtiHandle,
    }

    /// Takes a line `init` has routed to its pin, with its edge set and
    /// unmasked (FerroHAL's `Line::route` does all three), and gives the
    /// member its handle. The routing stays as `init` left it.
    pub fn new<L: Routed>(line: L) -> Resources {
        Resources {
            line: line.handle(),
        }
    }

    /// The line's vector: an edge. A pass with the line not pending (another
    /// line on a shared vector) does nothing.
    #[ferroforge::task(
        local = [line: ExtiHandle, missed: u32 = 0],
        spawn = [edge(at_us: u64, missed: u32)],
        monotonic = Mono,
    )]
    pub fn on_exti_line(cx: on_exti_line::Context) {
        // PROTOTYPE (event ports): the bench's marks; nothing without
        // `prototype-timing`.
        let entered = crate::probe::cycles();
        let at_us = Mono::now().duration_since_epoch().to_micros();
        let line = *cx.local.line;
        if !line.is_pending() {
            return;
        }
        line.clear();
        let missed = *cx.local.missed;
        match cx.spawn.edge(at_us, missed) {
            Ok(()) => *cx.local.missed = 0,
            Err(_) => *cx.local.missed = missed.saturating_add(1),
        }
        crate::probe::EXTI.served(entered);
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use super::exti_line;
    use crate::tests::groups;

    #[test]
    fn exti_line_states_its_contract() {
        let found = groups(include_str!("exti.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in exti.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "exti_line");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["line"]);
        assert_eq!(contract.vectors[0].member, "on_exti_line");
        assert_eq!(contract.vectors[0].slot, "line");
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["line"]);
        assert!(contract.shared.is_empty() && contract.buffers.is_empty());
        let constructor = contract.constructor.as_ref().unwrap();
        assert!(constructor.path.is_ident("new"));
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(arguments, ["line"]);
        let member = &group.members[0];
        assert_eq!(member.name, "on_exti_line");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn exti_line_builds_on_an_f401_line() {
        use ferrohal_stm32::exti::Exti;
        // PB4 on line 4, its own vector EXTI4 (RM0368 table 38).
        let constructor: fn(Exti<'B', 4>) -> exti_line::Resources = exti_line::new;
        let _ = constructor;
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn exti_line_builds_on_an_h753_line() {
        use ferrohal_stm32::exti::Exti;
        // PC13 on line 13, the shared vector EXTI15_10 (RM0433 table 147).
        let constructor: fn(Exti<'C', 13>) -> exti_line::Resources = exti_line::new;
        let _ = constructor;
    }
}
