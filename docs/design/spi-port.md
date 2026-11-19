# The SPI port

`spi_port` (`src/spi.rs`) shares one FerroHAL `Spi` master, on two DMA streams,
between one or more devices, each behind its own chip select. Requirements:
`src/requirements/spi.yaml` (PORT-201 to PORT-209).

## Shape

The consumer owns its memory and lends it to the port, as a shared SPI bus is
built elsewhere (Linux's `spi_message`, Zephyr's RTIO): the queues hold
references to buffers, never bytes.

- **Buffers.** At boot the firmware gives each device buffers of its own
  lengths, `&'static mut [u8]` up to the bus's capacity (the shorter of
  FerroHAL's two DMA buffers). The device's `SpiHandle` keeps the free ones.
- **A message** is one chip-select assertion. `SpiHandle::message(priority,
  len)` takes the shortest free buffer that holds `len` bytes; the consumer
  appends parts in place: `write` (sent, what comes back is not kept), `read`
  (filler sent, kept), `transfer` (sent and kept). `Message::submit` lends the
  buffer to the port and pends its vector.
- **The port** is one member, `on_spi_rx_dma`, on the receive stream's vector,
  which every handle also pends (the contract's `pends = [kick = ..]`). Each
  run ends a finished transfer, releases its chip select inside the
  interrupt, copies what came in over the lent buffer, and gives the buffer
  back through the device's return queue with a wake; it serves cancels; and,
  on a free bus, it starts the next message, switching FerroHAL's settings
  when the device changes.
- **Back.** `SpiHandle::take` gives a `Taken`, read in place by kept part;
  dropping it frees the buffer. `SpiHandle::wait` awaits the wake; wakes
  coalesce.
- **Order.** One device's messages run in the order submitted. Across
  devices, the front message with the highest priority goes first, the
  earliest submitted among equals, by a port-wide submission counter, so the
  order does not depend on the clock's resolution.
- **Refusal (D8).** No free buffer long enough: `Outstanding`, or `TooLong`
  when none of the device's buffers is that long. A malformed message
  (`Parts`, `Empty`, overflowing its buffer) frees its buffer. Nothing lent is
  touched, and one device with every buffer out never refuses another's
  message.
- **Cancel.** `SpiHandle::cancel` marks the submission counter; the port
  aborts the device's running message and drops its queued ones submitted
  before the mark, each back as `Aborted` with the line high. Messages
  submitted after the cancel run.
- **Boot binding (D12).** `Handles::take(i)` gives device `i`'s handle, one
  type for every device and every port size; `init` moves each into the task
  it chooses. One implementation serves one device or many (D21, D22).

No queue turns interrupts off: each device's two queues have one producer and
one consumer (`heapless::spsc`). The wake is `rtic-sync`'s `Signal`, whose
critical section is a few instructions, not measured. Times (`waited`, `took`)
come from the `now` the firmware passes: its monotonic's ticks, or a cycle
counter where it wants finer stamps.

## What it needs from FerroHAL

| Port requirement | FerroHAL behaviour | FerroHAL requirement |
| --- | --- | --- |
| PORT-201 | a DMA transfer starts at once and reports its end; a start that cannot run is refused | SPI-002, SPI-003 |
| PORT-206 | an abort ends a running transfer and leaves the driver usable | SPI-006 |
| PORT-208 | every clock mode carries bytes; the reported SCK is the wire's | SPI-004, SPI-005 |
| PORT-208 | a settings change between transfers applies to the next transfer, refused while one runs | none yet: DEBT S1 |

## Contract

The contract states the slots (SPI, SCK, MISO, MOSI, both streams, and the
chip selects as one tuple slot), the vector, the shared `port`, FerroHAL's two
DMA buffers, the config (`configs` per device from the board, the lent buffers
from software, `now` from the platform), `memory = [memory: Memory]`,
`handles = [handles: Handles]` and the pend. What it cannot state is in DEBT
S3: the number of devices, and each device's buffers, are checked by the
constructor at boot.

## Betaflight targets (D29)

Checked against three flight-controller board manifests (2026-11-19):

- Two F405 boards put their IMU on SPI1, PA5/PA6/PA7 on AF5, chip select a
  GPIO, RX on DMA2 stream 0 channel 3 and TX on DMA2 stream 3 channel 3. That
  is the STM32F4 Discovery bench's wiring, pin for pin and stream for stream.
- One H743 board puts its IMU on SPI1, PA5/PA6/PD7 on AF5, RX on DMA1 stream 4
  and TX on DMA1 stream 5 through DMAMUX. That is the Nucleo-H753ZI bench's.
- Both F405 boards run their flash on a second SPI with no DMA stream free.
  This port needs both streams, so such a bus is not a port: the firmware
  drives FerroHAL's polled `BlockingSpi` from a low-priority task (D10).

## Properties of the pattern

These follow from lending memory, and were measured in the prototype
(port design record, section 3a); they are the design, not debt:

- The consumer states a message's length before writing it, and a message
  cannot outgrow the buffer it took.
- A `Taken` borrows the handle: no new message on that device while a result
  is held.
- Priority orders devices, not one device's backlog: an urgent message waits
  behind what its own device submitted before it.
- A transfer that never ends is bounded by its consumer: a timeout, then
  `cancel`.
