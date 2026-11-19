//! PROTOTYPE, variant L2 (stacked): not product code. Variant L with the
//! vector service inside the firmware's own shared stack, beside the device
//! and its classes, behind the one-method
//! [`UsbStack`](usb_stacked::UsbStack) trait. Nothing reaches the device,
//! its classes or the vector service without that one lock, so the rule L
//! writes in its doc is in the types here. Behind this crate's `usb` feature.
//!
//! The member masks what the core has pending, as in D, and writes an
//! `rtic-sync` signal, which holds until the firmware's task reads it. That
//! task owns the device and its classes and awaits the signal through the
//! [`UsbWaiter`](usb_stacked::UsbWaiter) handle, polling once per wake.
//! The task holds the lock for its whole poll and echo pass: the lock
//! raises the priority to the vector's, not above, so higher-priority
//! vectors (timer lanes) still run. Built with synopsys-usb-otg's
//! `no-fifo-critical-section`, the bench tests whether this lock alone
//! covers the F407's transmit FIFO stall. Because no wake is lost, the
//! start-of-frame heartbeat is masked
//! ([`UsbFs::deferred_without_heartbeat`](ferrohal_stm32::usb::UsbFs::deferred_without_heartbeat)):
//! a quiet configured bus raises the vector not at all.
//!
//! As in D, the firmware's task names FerroHAL's bus type and owns the
//! device: `usb-device` builds the device and its classes on the bus, and
//! enumeration and the classes are protocol (D6). The port carries wakes,
//! not packets or bytes.
//!
//! Requirements: PORT-001, PORT-002, PORT-501

/// The stacked USB device group. A firmware selects it once:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::usb_stacked, shared = [stack = usb], local = [wake = usb_wake])]
/// mod usb {
///     #[task(binds = OTG_FS, priority = 3)] fn on_usb_stacked;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, dm: Dm, dp: Dp],
    vectors = [on_usb_stacked = instance],
    shared = [stack: S],
    local = [wake: UsbWake],
    buffers = [ep_memory: [u32]],
    constructor = new(rcc, wiring, ep_memory),
)]
pub mod usb_stacked {
    use core::sync::atomic::{AtomicU32, Ordering};

    use ferrohal_stm32::{
        rcc::Rcc,
        usb::{UsbFs, UsbRefused, usb_device::bus::UsbBusAllocator},
    };
    use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

    pub use ferrohal_stm32::{
        Service,
        chip::{USB_OTG_FS as U, UsbDm as Dm, UsbDp as Dp},
        usb::{Events, UsbBus, UsbVector, Wiring},
    };

    /// PROTOTYPE bench counter: how often the member ran.
    pub static VECTOR_ENTRIES: AtomicU32 = AtomicU32::new(0);

    /// The member's end of the wake.
    pub type UsbWake = SignalWriter<'static, ()>;

    /// The wake. One per chip, as the port is: `new` takes the chip's one
    /// OTG_FS token, and a port inside `usb-device`'s allocator is never
    /// freed, so the signal is split once.
    static WAKE: Signal<()> = Signal::new();

    /// The firmware's shared stack: its device and classes, and the
    /// vector service [`new`] returned, all behind one lock.
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
        /// since the last return; at once if it already has.
        pub async fn wait(&mut self) {
            self.wake.wait().await;
        }
    }

    /// What [`new`] builds.
    pub struct Resources {
        /// The vector service, for the firmware's stack.
        pub vector: UsbVector,
        /// The member's local resource, bound as the group's `wake`.
        pub wake: UsbWake,
        /// The firmware task's end of the wake.
        pub waiter: UsbWaiter,
        /// The bus, for the firmware's `UsbDeviceBuilder` and classes.
        pub bus: UsbBusAllocator<UsbBus>,
    }

    /// Opens the port on `wiring`, builds its bus with the heartbeat masked,
    /// and splits the wake into its two ends. A refusal is
    /// FerroHAL's, as for `usb_device::new`.
    pub fn new(
        rcc: &mut Rcc,
        wiring: Wiring,
        ep_memory: &'static mut [u32],
    ) -> Result<Resources, UsbRefused> {
        let (bus, vector) = UsbFs::new(rcc, wiring)?.deferred_without_heartbeat(ep_memory);
        let (wake, reader) = WAKE.split();
        Ok(Resources {
            vector,
            wake,
            waiter: UsbWaiter { wake: reader },
            bus,
        })
    }

    /// The port's vector: masks what waits and wakes the firmware's task.
    #[ferroforge::task(shared = [stack], bounds = [stack: UsbStack], local = [wake: UsbWake])]
    pub fn on_usb_stacked(mut cx: on_usb_stacked::Context) {
        VECTOR_ENTRIES.fetch_add(1, Ordering::Relaxed);
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
    fn usb_stacked_states_its_contract() {
        let found = groups(include_str!("usb_stack.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in usb_stack.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "usb_stacked");
        let contract = &group.arguments.contract;
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["wake"]);
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["stack"]);
        assert_eq!(contract.vectors[0].member, "on_usb_stacked");
        let [member] = group.members.as_slice() else {
            panic!("one member, found {}", group.members.len());
        };
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }
}
