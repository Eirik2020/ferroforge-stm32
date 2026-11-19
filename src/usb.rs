//! The USB device port: FerroHAL's full-speed OTG port with its vector
//! deferred to a task of the firmware's. Behind this crate's `usb` feature.
//! Design: `docs/design/usb-port.md`.
//!
//! The firmware owns the device and its classes: `usb-device` builds them
//! on the bus [`new`](usb_device::new) returns, and enumeration and the
//! classes are protocol (D6). The port carries wakes, not packets or bytes.
//! The member serves the vector, masking what the core has pending, and
//! writes an `rtic-sync` signal the firmware's task awaits through its
//! [`UsbWaiter`](usb_device::UsbWaiter). The signal holds until the task
//! reads it, so no wake is lost, and the bus is built without the
//! start-of-frame heartbeat: a quiet configured bus raises the vector not at
//! all.
//!
//! FerroHAL's rule of use (USB-002): the vector's service and every bus call
//! never preempt one another. The firmware keeps the vector service, the
//! device and its classes in one shared resource, its [`UsbStack`], and the
//! member reaches the service only through that resource's lock. A task
//! holding the lock to poll or to read or write a class then cannot be cut
//! into by the vector, and nothing reaches the bus without the lock.
//!
//! Requirements: PORT-001, PORT-501, PORT-502
//!
//! [`UsbStack`]: usb_device::UsbStack

/// The USB device group. A firmware selects it once, with its stack shared
/// between the member and its own task:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::usb_device, shared = [stack = usb], local = [wake = usb_wake])]
/// mod usb {
///     #[task(binds = OTG_FS, priority = 3)] fn on_usb;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, dm: Dm, dp: Dp],
    vectors = [on_usb = instance],
    shared = [stack: S],
    local = [wake: UsbWake],
    buffers = [ep_memory: [u32]],
    memory = [memory: Memory],
    handles = [waiter: UsbWaiter, vector: UsbVector, bus: UsbBusAllocator<UsbBus>],
    constructor = new(rcc, wiring, ep_memory, memory),
)]
pub mod usb_device {
    use ferrohal_stm32::{
        rcc::Rcc,
        usb::{UsbFs, UsbRefused},
    };
    use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

    pub use ferrohal_stm32::{
        Service,
        chip::{USB_OTG_FS as U, UsbDm as Dm, UsbDp as Dp},
        usb::{Events, UsbBus, UsbVector, Wiring, usb_device::bus::UsbBusAllocator},
    };

    /// The member's end of the wake.
    pub type UsbWake = SignalWriter<'static, ()>;

    /// The port's memory: the wake between the member and the firmware's
    /// task.
    pub struct Memory {
        wake: Signal<()>,
    }

    impl Memory {
        /// Empty memory, for a `static` or `init(local = [..])`.
        pub const fn new() -> Self {
            Self {
                wake: Signal::new(),
            }
        }
    }

    impl Default for Memory {
        fn default() -> Self {
            Self::new()
        }
    }

    /// The firmware's shared stack: its device and classes, and the
    /// [`UsbVector`] [`new`] returned, all behind one lock. The member
    /// reaches the vector service only through it.
    pub trait UsbStack {
        /// The vector service, for the member.
        fn vector(&mut self) -> &mut UsbVector;
    }

    /// The firmware's task's end of the wake.
    pub struct UsbWaiter {
        wake: SignalReader<'static, ()>,
    }

    impl UsbWaiter {
        /// Returns once the vector has found something waiting for a poll
        /// since the last return; at once if it already has. Wakes
        /// coalesce: one return covers every vector entry before it.
        pub async fn wait(&mut self) {
            self.wake.wait().await;
        }
    }

    /// What [`new`] builds.
    pub struct Resources {
        /// The member's local resource, bound as the group's `wake`.
        pub wake: UsbWake,
        /// The firmware task's end of the wake.
        pub waiter: UsbWaiter,
        /// The vector service, for the firmware's stack.
        pub vector: UsbVector,
        /// The bus, for the firmware's `UsbDeviceBuilder` and classes.
        pub bus: UsbBusAllocator<UsbBus>,
    }

    /// Opens the port on `wiring`, builds its bus without the heartbeat,
    /// and splits the wake in `memory` into its two ends. A refusal is
    /// FerroHAL's, with the wiring handed back.
    pub fn new(
        rcc: &mut Rcc,
        wiring: Wiring,
        ep_memory: &'static mut [u32],
        memory: &'static mut Memory,
    ) -> Result<Resources, UsbRefused> {
        let (bus, vector) = UsbFs::new(rcc, wiring)?.deferred_without_heartbeat(ep_memory);
        let (wake, reader) = memory.wake.split();
        Ok(Resources {
            wake,
            waiter: UsbWaiter { wake: reader },
            vector,
            bus,
        })
    }

    /// The port's vector: takes the stack's lock, masks what waits and wakes
    /// the firmware's task.
    #[ferroforge::task(shared = [stack], bounds = [stack: UsbStack], local = [wake: UsbWake])]
    pub fn on_usb(mut cx: on_usb::Context) {
        if cx
            .shared
            .stack
            .lock(|stack| stack.vector().on_interrupt())
            .any()
        {
            cx.local.wake.write(());
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

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
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["wake"]);
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["stack"]);
        let handles: Vec<_> = contract
            .handles
            .iter()
            .map(|r| r.name.to_string())
            .collect();
        assert_eq!(handles, ["waiter", "vector", "bus"]);
        assert_eq!(contract.vectors[0].member, "on_usb");
        let [member] = group.members.as_slice() else {
            panic!("one member, found {}", group.members.len());
        };
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }
}
