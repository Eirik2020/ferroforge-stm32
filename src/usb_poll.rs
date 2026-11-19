//! PROTOTYPE, variant P (polled in the vector): not product code. A USB
//! device port whose member polls the firmware's stack from the vector, as
//! RTIC examples commonly do. Behind this crate's `usb` feature.
//!
//! The firmware builds its `UsbDevice` and classes on the bus [`new`]
//! (usb_polled::new) hands back, puts them in one `#[shared]` resource and
//! implements [`UsbStack`](usb_polled::UsbStack) for it: one call that
//! polls the device with its classes. The member locks that resource, polls
//! it, and passes a wake to the firmware's task through the `events` call
//! when the stack reports class activity. The task locks the same resource
//! to read and write its classes. The group never names a class.
//!
//! Nothing is masked and nothing waits: the vector serves the core each
//! time it fires, so the bus runs at the vector's priority, and the task's
//! lock holds the vector off while it reads or writes.
//!
//! Requirements: PORT-001, PORT-002, PORT-501

/// The polled USB device group. A firmware selects it once:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::usb_polled, shared = [stack = usb],
///         spawn = [events = usb_events])]
/// mod usb {
///     #[task(binds = OTG_FS, priority = 2)] fn on_usb_polled;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, dm: Dm, dp: Dp],
    vectors = [on_usb_polled = instance],
    shared = [stack: S],
    buffers = [ep_memory: [u32]],
    constructor = new(rcc, wiring, ep_memory),
)]
pub mod usb_polled {
    use ferrohal_stm32::{
        rcc::Rcc,
        usb::{UsbFs, UsbRefused, usb_device::bus::UsbBusAllocator},
    };

    pub use ferrohal_stm32::{
        chip::{USB_OTG_FS as U, UsbDm as Dm, UsbDp as Dp},
        usb::{UsbBus, Wiring},
    };

    /// The firmware's device and classes, polled together: what the member
    /// serves the vector with.
    pub trait UsbStack {
        /// Polls the device with every class once; true when a class may
        /// have data to read or room to write (`UsbDevice::poll`'s result).
        fn poll(&mut self) -> bool;
    }

    /// Opens the port on `wiring` and builds its bus, for a vector that
    /// polls the stack itself. A refusal is FerroHAL's, with the wiring
    /// handed back.
    pub fn new(
        rcc: &mut Rcc,
        wiring: Wiring,
        ep_memory: &'static mut [u32],
    ) -> Result<UsbBusAllocator<UsbBus>, UsbRefused> {
        Ok(UsbBus::new(UsbFs::new(rcc, wiring)?, ep_memory))
    }

    /// The port's vector: polls the stack, and wakes the firmware's task on
    /// class activity.
    #[ferroforge::task(
        shared = [stack],
        bounds = [stack: UsbStack],
        spawn = [events()],
    )]
    pub fn on_usb_polled(mut cx: on_usb_polled::Context) {
        if cx.shared.stack.lock(|stack| stack.poll()) {
            let _: Result<(), ()> = cx.spawn.events();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use super::usb_polled;
    use crate::tests::groups;

    #[test]
    fn usb_polled_states_its_contract() {
        let found = groups(include_str!("usb_poll.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in usb_poll.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "usb_polled");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["instance", "dm", "dp"]);
        assert_eq!(contract.vectors[0].member, "on_usb_polled");
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["stack"]);
        let [member] = group.members.as_slice() else {
            panic!("one member, found {}", group.members.len());
        };
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    #[test]
    fn usb_polled_builds_on_the_chips_wiring() {
        let wiring = |usb: usb_polled::U, dm: usb_polled::Dm, dp: usb_polled::Dp| {
            usb_polled::Wiring { usb, dm, dp }
        };
        let _ = wiring;
        let _ = usb_polled::new;
    }
}
