# Port groups and their contract

A port is a FerroForge task group with a contract. The contract is what a
firmware, or a tool writing one, needs to select the group without knowing which
group it is (FerroSmith design record v3, sections 6.1 and 8.2). Its syntax and
checks are FerroForge's (`ferroforge-contracts`); this chapter is how this crate
uses them.

## The contract

```rust,ignore
#[ferroforge::group(
    slots = [instance: U, tx: Tx, rx: Rx, tx_dma: TxD, rx_dma: RxD],
    vectors = [on_uart = instance, on_rx_dma = rx_dma, on_tx_dma = tx_dma],
    shared = [uart: Uart<Wiring<U, Tx, Rx, TxD, RxD>>],
    buffers = [#[dma] tx_buf: [u8], #[dma] rx_buf: [u8]],
    config = [config: Config @ platform],
    constructor = new(rcc, wiring, config, tx_buf, rx_buf),
)]
pub mod uart_port { .. }
```

- **Slots** are FerroHAL's wiring roles, each with the constructor's type
  parameter that carries it. A tool fills each with the type the board and chip
  give that role (`USART1`, `Pin<'A', 9, Alternate<7>>`, `Stream<2, 7>`), so the
  generated code is a wiring alias and nothing else port-specific.
- **Vectors** say which member serves which slot's interrupt. The vector's name
  comes from FerroHAL's data for the slot's type; the member's `binds` is that
  name, and its priority is the firmware's.
- **Resources** are every resource the members leave to the firmware, with its
  type over the slots' parameters. The firmware declares each as a `#[shared]` or
  `#[local]` field and binds it in the selection.
- **Buffers** are the memory the constructor takes, each a slice of an integer
  type. The firmware sets each one's length, because the device or the traffic
  fixes it, not the port; a tool writes it as a zeroed array in
  `init(local = [..])`. A buffer a DMA stream reads or writes is `#[dma]`.
- **Config** fields are the constructor's other inputs, each with the input
  layer that supplies it: a UART's frame from the platform, an SPI bus's clock
  and mode from the board's device.
- **Constructor** builds the resources. It is generic over the slots, and its
  arguments are listed in order: `rcc`, `wiring` (FerroHAL's wiring of the
  slots), a slot FerroHAL's wiring does not hold (the SPI port's chip select),
  the buffers and the config fields. It returns a struct with one public field
  per resource, of the same name, or the driver's refusal.

## Rules for a port group

- One owner per driver, shared under the scheduler's lock, as FerroHAL's drivers
  are built. A member that serves a vector calls the driver's
  `Service::on_interrupt` and does nothing else that takes time.
- Members are bounded on FerroHAL's traits (`Service<Event = Events>`), not on a
  concrete driver type, so one definition serves every instance and chip.
- What a service found goes to the firmware's own task through an outgoing call
  (`events`) the firmware binds. Wakes coalesce: a call that finds the task
  queued is dropped, and the task drains everything when it runs.
- The constructor and the members build for the host (G2), so a tool can check a
  selection, and host tests can instantiate a constructor with a real wiring.
- A receive-only port is its own group, not an optional half of the full one:
  the contract has no optional members yet, and a receive-only UART has one
  vector fewer. Its members are its own, because each also stamps the wake.
- A receive member reads the firmware's monotonic in the interrupt and passes
  the time with the events, so a reader can judge how old its data is. A wake
  dropped because the reader was still queued leaves the queued wake's earlier
  stamp, so data never looks newer than it is.
- A port may frame its transfers with hardware FerroHAL's driver does not
  hold, such as the SPI port's chip select. The port drives it inside the
  vector that ends a transfer, so the line is held no longer than the
  transfer at any priority of the firmware's task.
- Member names are unique in this crate: FerroForge exports each task's
  callback at the crate's root, so two `on_rx_dma` would collide. A member is
  named for its port, `on_spi_rx_dma`.
- A port carries no protocol. Protocols are not hardware: a decoder or encoder
  lives in the application or a protocol crate, as a task behind the port's
  `events` call, never in a port group.

## Receive constraints

Two constraints the receive members hold, and that bind any protocol a firmware
puts behind them:

- A frame ends on an idle line and nowhere else. In circular reception a full
  buffer means the ring wrapped, not that a frame ended, so a decoder above the
  port treats a chunk ended by a full buffer or a line error as suspect whatever
  its bytes.
- The USART's error flags are the receive path's business, not diagnostics: an
  uncleared overrun stops the peripheral requesting DMA at all, so ignoring one
  wedges reception rather than degrading it. The members serve every vector
  through `Service::on_interrupt` for this reason; skipping a vector is not an
  optimisation.

## Not in the contract yet

Optional members behind a feature, from the FerroSmith descriptor, wait for a
port that has one; until then a variant is its own group, as the receive-only
UART is.
