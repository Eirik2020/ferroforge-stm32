//! PROTOTYPE, variant L (locked): not product code. Variant S with the
//! vector service shared between the member and the firmware's task, so
//! the task's lock on it keeps the member out of an IN packet write. Behind this crate's `usb` feature.
//!
//! The member masks what the core has pending, as in D, and writes an
//! `rtic-sync` signal, which holds until the firmware's task reads it. That
//! task owns the device and its classes and awaits the signal through the
//! [`UsbWaiter`](usb_locked::UsbWaiter) handle, polling once per wake.
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

/// The locked USB device group. A firmware selects it once:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::usb_locked, shared = [vector = usb_vector], local = [wake = usb_wake])]
/// mod usb {
///     #[task(binds = OTG_FS, priority = 3)] fn on_usb_locked;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, dm: Dm, dp: Dp],
    vectors = [on_usb_locked = instance],
    shared = [vector: UsbVector],
    local = [wake: UsbWake],
    buffers = [ep_memory: [u32]],
    constructor = new(rcc, wiring, ep_memory),
)]
pub mod usb_locked {
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
        /// The member's shared resource, bound as the group's `vector`; the
        /// firmware's task locks it for each poll and echo pass.
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
    #[ferroforge::task(shared = [vector: UsbVector], local = [wake: UsbWake])]
    pub fn on_usb_locked(mut cx: on_usb_locked::Context) {
        VECTOR_ENTRIES.fetch_add(1, Ordering::Relaxed);
        if cx.shared.vector.lock(|vector| vector.on_interrupt()).any() {
            cx.local.wake.write(());
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{string::ToString, vec::Vec};

    use crate::tests::groups;

    #[test]
    fn usb_locked_states_its_contract() {
        let found = groups(include_str!("usb_lock.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in usb_lock.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "usb_locked");
        let contract = &group.arguments.contract;
        let local: Vec<_> = contract.local.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(local, ["wake"]);
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["vector"]);
        assert_eq!(contract.vectors[0].member, "on_usb_locked");
        let [member] = group.members.as_slice() else {
            panic!("one member, found {}", group.members.len());
        };
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }
}
