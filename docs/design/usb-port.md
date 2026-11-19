# The USB device port

`usb_device` (`src/usb.rs`) serves FerroHAL's full-speed OTG port from a task
of the firmware's, with the port's vector deferred to it. Behind the crate's
`usb` feature. Requirements: `src/requirements/usb.yaml` (PORT-501, PORT-502).

## Shape

- **The firmware owns the device.** `usb-device` builds the device and its
  classes on the bus `new` returns; enumeration and the classes are protocol
  (D6), so the port carries wakes, not packets or bytes.
- **The stack.** The firmware puts the vector service `new` returns, the device
  and its classes in one shared resource, and implements `UsbStack` on it: one
  method handing the member the vector service. The member reaches the service
  only through that resource's lock, so the rule below is in the types.
- **The member**, `on_usb`, on the OTG_FS vector, takes the lock, masks what
  the core has pending (FerroHAL's `Service::on_interrupt`), and writes the
  wake if anything waits for a poll.
- **The wake** is `rtic-sync`'s `Signal`, in the port's `Memory`, split by
  `new` into the member's `UsbWake` and the firmware's `UsbWaiter`. It holds
  until the task reads it and wakes coalesce, so no wake is lost, and the bus
  is built without the start-of-frame heartbeat a droppable wake would need.
  A configured, quiet bus raises the vector not at all.
- **The task** awaits `UsbWaiter::wait`, then polls and serves its classes
  under the lock until a round moves nothing. FerroHAL's `poll` unmasks what
  it served.

## The rule of use

The OTG cores' transmit FIFO erratum (ES0182 2.14.1 and its twins) corrupts
the FIFO when an OTG_FS register access cuts into a write to it, and the
deferred vector's service is such an access. FerroHAL states the rule of use
(USB-002): the vector's service and every bus call never preempt one another.
Broken on the F407, the IN endpoint stalls for good. FerroHAL's fork could
instead write each IN packet inside a critical section, but that held every
interrupt off for about 10 us on the F407 and 9.3 us on the H753, longer than
the timer lanes' margins. The lock holds off only vectors at or below the USB
vector's priority, so a lane above it is untouched (D35).

## What it needs from FerroHAL

| Port requirement | FerroHAL behaviour | FerroHAL requirement |
| --- | --- | --- |
| PORT-501, PORT-502 | a deferred vector, kept from preempting the bus, carries CDC serial | USB-002 |

## The lock's longest hold

The task holds the lock for one round: a poll, then one class read or write.
On the STM32F4 Discovery at 168 MHz, under FerroHAL's 64 KiB host echo at
about 510 KiB/s, the longest round held it 10,859 and 12,283 cycles in two
runs (65 and 73 us, 2026-11-19). Holding it for a whole pass of up to four
rounds held it up to 38,100 cycles (230 us) at the same throughput. The hold
is the firmware's choice, not the port's: it bounds how long a vector at or
below the USB vector's priority can wait.

## Contract

The contract states the slots (the OTG_FS token and its two data pins), the
vector, the shared `stack` (bounded on `UsbStack`), the local `wake`, the OUT
endpoints' buffer, `memory = [memory: Memory]`, and three handles: the
`waiter` for the firmware's task, and the `vector` and `bus` the firmware
builds its stack from. One port per chip: `new` takes the chip's one OTG_FS
token.

## Betaflight targets (D29)

Checked against three flight-controller board manifests (2026-11-19): two
F405 boards and one H743 board all put USB on PA11 (DM) and PA12 (DP), the
port on the OTG_FS vector. That is the STM32F4 Discovery bench's wiring on the
F405 die. The H743's PA11 and PA12 are its full-speed core, which FerroHAL
serves through the same `UsbFs`, so the port builds for it unchanged.
