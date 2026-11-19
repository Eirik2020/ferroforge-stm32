//! PROTOTYPE, variant D (deferred): not product code. A USB device port: FerroHAL's full-speed OTG port as a `usb-device` bus,
//! its vector served by the group and the device stack run by the
//! firmware's own task. Behind this crate's `usb` feature.
//!
//! The port is hardware only. Enumeration, the descriptors and every class
//! (CDC serial, HID, DFU) are protocol: the firmware builds its
//! `UsbDevice` and classes on the bus [`new`](usb_device::new) hands back,
//! and polls them from its own task. The group never names a class.
//!
//! The member serves the vector with FerroHAL's
//! [`UsbVector`](ferrohal_stm32::usb::UsbVector): it masks whatever the core
//! has pending, so the vector does not fire again before the stack has
//! served it, and passes what waits to the firmware's task through the
//! `events` call. That task's `UsbDevice::poll` serves it and unmasks it.
//! The vector is the port's only state, so it is the member's local
//! resource; the bus and the stack are the firmware's.
//!
//! Wakes coalesce, and none is lost. A call that finds the firmware's task
//! still queued or running is dropped, but what it reported stays pending
//! in the core: the start-of-frame interrupt, every millisecond while the
//! bus runs, repeats the report until a poll has served it. A task that
//! polls until [`Events::any`](usb_device::Events::any) is false on a fresh
//! read sees no such repeat at all.
//!
//! Only the OTG_FS port: the one the chip's glue wires (`USB_OTG_FS`, PA11
//! and PA12 on the parts here; RM0368 table 9 and DS10086 table 9 for the
//! F401, DS8626 table 9 for the F405/F407, DS12117 table 10 for the H753).
//!
//! Requirements: PORT-001, PORT-002, PORT-501

/// The USB device group. A firmware selects it once:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::usb_device, local = [vector = usb_vector],
///         spawn = [events = usb_events])]
/// mod usb {
///     #[task(binds = OTG_FS, priority = 2)] fn on_usb_fs;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, dm: Dm, dp: Dp],
    vectors = [on_usb_fs = instance],
    local = [vector: UsbVector],
    buffers = [ep_memory: [u32]],
    constructor = new(rcc, wiring, ep_memory),
)]
pub mod usb_device {
    use ferrohal_stm32::{
        rcc::Rcc,
        usb::{UsbFs, UsbRefused, usb_device::bus::UsbBusAllocator},
    };

    pub use ferrohal_stm32::{
        Service,
        chip::{USB_OTG_FS as U, UsbDm as Dm, UsbDp as Dp},
        usb::{Events, UsbBus, UsbVector, Wiring},
    };

    /// What [`new`] builds: the member's resource, and the bus for the
    /// firmware's own stack.
    pub struct Resources {
        /// The firmware's local resource, bound as the group's `vector`.
        pub vector: UsbVector,
        /// The bus, for the firmware's `UsbDeviceBuilder` and classes. Not a
        /// group resource: the firmware keeps it for good (`usb-device`
        /// needs it `'static`), in an `init` local or a static.
        pub bus: UsbBusAllocator<UsbBus>,
    }

    /// Opens the port on `wiring` and builds its bus, deferring the vector
    /// to the firmware's task. `ep_memory` holds the OUT endpoints' packets,
    /// a word per four bytes of each endpoint's largest packet: its length
    /// is the firmware's, set by the classes it builds. A refusal is
    /// FerroHAL's, with the wiring handed back: no 48 MHz USB clock, or an
    /// AHB clock too slow for the core.
    pub fn new(
        rcc: &mut Rcc,
        wiring: Wiring,
        ep_memory: &'static mut [u32],
    ) -> Result<Resources, UsbRefused> {
        let (bus, vector) = UsbFs::new(rcc, wiring)?.deferred(ep_memory);
        Ok(Resources { vector, bus })
    }

    /// The port's vector: a reset, a suspend or a resume, a packet received
    /// or sent, or the start of frame that repeats what still waits.
    #[ferroforge::task(
        local = [vector: UsbVector],
        spawn = [events(events: Events)],
    )]
    pub fn on_usb_fs(cx: on_usb_fs::Context) {
        let events = cx.local.vector.on_interrupt();
        if events.any() {
            let _: Result<(), Events> = cx.spawn.events(events);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use super::usb_device;
    use crate::tests::groups;

    #[test]
    fn usb_device_states_its_contract() {
        let found = groups(include_str!("usb.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in usb.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "usb_device");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["instance", "dm", "dp"]);
        assert_eq!(contract.vectors.len(), 1);
        assert_eq!(contract.vectors[0].member, "on_usb_fs");
        assert_eq!(contract.vectors[0].slot, "instance");
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["vector"]);
        assert!(contract.shared.is_empty() && contract.config.is_empty());
        let buffers: Vec<_> = contract
            .buffers
            .iter()
            .map(|b| (b.name.to_string(), b.element.to_string(), b.dma))
            .collect();
        assert_eq!(
            buffers,
            [("ep_memory".to_string(), "u32".to_string(), false)]
        );
        let constructor = contract.constructor.as_ref().unwrap();
        assert!(constructor.path.is_ident("new"));
        let arguments: Vec<_> = constructor
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(arguments, ["rcc", "wiring", "ep_memory"]);
        let [member] = group.members.as_slice() else {
            panic!("one member, found {}", group.members.len());
        };
        assert_eq!(member.name, "on_usb_fs");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }

    /// The constructor takes the chip's own wiring: the OTG_FS token and its
    /// two data pins, whose types the slots name.
    #[test]
    fn usb_device_builds_on_the_chips_wiring() {
        use ferrohal_stm32::rcc::Rcc;
        let constructor: fn(
            &mut Rcc,
            usb_device::Wiring,
            &'static mut [u32],
        )
            -> Result<usb_device::Resources, ferrohal_stm32::usb::UsbRefused> = usb_device::new;
        let _ = constructor;
        let wiring = |usb: usb_device::U, dm: usb_device::Dm, dp: usb_device::Dp| {
            usb_device::Wiring { usb, dm, dp }
        };
        let _ = wiring;
    }
}
