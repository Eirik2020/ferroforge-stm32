# The UART port

`uart_port` (`src/uart.rs`) is the full-duplex UART port. Its shape follows
decisions D12 to D23, D29, D34 and D15's line-idle signal in the port design
record (project files, `ferroforge-stm32-port-design/port-design-record.md`),
which keeps why each was decided. This chapter states what binds the code. The requirements are `src/requirements/uart.yaml`.

## Shape

- **The port owns the driver, the consumer owns a handle** (D20). The three
  members share one `Port`, which holds FerroHAL's `Uart` and the port's ends
  of four queues. The consumer's `Handle` names no UART, wiring, vector or
  lock, so a consumer at any priority never delays the port's interrupts.
- **One handle type** (D12). Every port's handle is a `Handle`, whatever its
  UART and buffers. `init` decides which task gets which port. Nothing is
  spawned: the port wakes whoever holds the handle through a signal in it.
- **Lent buffers** (D23). The firmware gives all memory at boot.
  - Transmit: the handle keeps transmit buffers of the lengths the firmware
    chooses. `frame(len)` takes the shortest free one that fits. The consumer
    writes the frame in place and submits it. The port passes it to FerroHAL's
    `try_send`, the one copy left. The buffer comes back through `sent()`,
    in submission order, with the frame's `Outcome` (D15).
  - Receive: the port keeps a pool of buffers of one length. In each service
    it has `read_chunk` copy the ring into a free buffer, the one copy left,
    and lends it to the consumer with FerroHAL's `Chunk` and the time.
    Dropping the `Received` returns the buffer.
  - The queues carry references, so an entry's size does not depend on the
    data's length.
- **Loss** (D19). With no pool buffer free, the bytes stay in FerroHAL's ring
  and the port marks itself starved. The next release pends the port. Bytes
  are lost only when the ring laps. FerroHAL then reports a gap, which reaches
  the consumer as a chunk ending in `Gap` at its place in the stream.
- **Time** (D17, D26). Each member reads the firmware's monotonic in its
  interrupt. Every chunk the service lends carries that time.
- **Refusals** (D8). `frame` refuses when no free buffer is long enough
  (`Outstanding`) or none could ever be (`TooLong`). The constructor checks
  every lent buffer before it opens the UART, and refuses with the wiring and
  buffers handed back. It never refuses with an opened driver: with two UARTs
  that cost about 11 KB of `init` code in the second prototype.
- Nothing panics, and nothing is lost silently: a buffer a queue cannot take
  is counted (`Handle::lost`). Protocols stay out (D6).

## Limits the constructor checks

The contract's `memory` entry says nothing about lengths or counts. The
constructor checks them at boot instead:

- Pool buffers have one length, at most half the receive ring. A longer one
  would let a continuous stream lap the ring before a chunk is taken
  (FerroHAL's `read_chunk` waits for the buffer's length or a mark).
- At most `RX - 1` pool buffers and `TX - 1` transmit buffers, the queues'
  capacity.
- Each transmit buffer is at most FerroHAL's transmit buffer's length.

A consumer needs as many transmit buffers as frames it wants in flight: a
buffer stays out until its frame has finished and its outcome is taken.

## How "sent" is known

FerroHAL gives each transmit frame's outcome back once, in queue order
(`Uart::take_outcome`, FerroHAL's UARTDMA-013). In each service the port takes
every outcome there is and gives the oldest frame in flight back with it:
`Sent`, `Failed` (a DMA error), `Dropped` (FerroHAL dropped its queue) or
`Lost`, for an outcome FerroHAL overwrote before it was taken (UARTDMA-015),
which the port's taking every outcome each service rules out unless something
is broken (`Port::lost_outcomes`). The in-flight depth is FerroHAL's
`TX_FRAMES` (UARTDMA-014).

"Sent" is FerroHAL's DMA completion, which comes before the USART's
transmission complete. It stays there, and the line's quiet is its own
signal (D15): the port turns on
FerroHAL's TC interrupt (`set_tx_idle_interrupt`, UART-023), and
`Handle::drained` is true, with a wake, once FerroHAL reports the line quiet
and no frame is lent or in flight. Submitting a frame clears it at once.
Measured on the Discovery at 115 200 baud for 64-byte frames: the outcome
reached the consumer about 22 000 to 23 200 cycles (about 1.5 characters at
168 MHz) before the last stop bit, and the quiet 5 200 to 6 600 cycles after
it, with the port's and the wake's latency included in both.

## Flight controller routes (D29)

The full-duplex UARTs of three flight controller boards type-check against the
constructor on the host (`tests::uart_port_builds_on_f405_flight_controller_routes`
and `..._h7_...`): USART1, USART2, USART3 and UART4 on the F405 with the DMA1
and DMA2 streams their targets assign (RM0090 tables 43 and 44), and USART3 and
USART6 on the H743 through DMAMUX1. The Discovery bench runs USART3 and UART5
on the pins and streams such targets assign on die 0x413.

Those boards also route receive-only UARTs, with no transmit stream. This port
needs both streams, so those UARTs are `uart_rx_port`'s.

## Boards

The STM32F4 Discovery proves the port: one board proves a port (D34).
