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
    constructor = new,
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
- **Constructor** builds the resources. It is generic over the slots, takes the
  wiring and whatever else it needs (the RCC, the frame, the buffers), and
  returns a struct with one public field per resource, of the same name, or the
  driver's refusal.

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

## Not in the contract yet

Decided content of the descriptor that this first cut leaves out, to settle on
the next group rather than guess on this one: buffers (sizes as Rust expressions,
emitted verbatim), config fields and the layer that supplies each, and optional
members behind a feature. Today a firmware writes the buffers in
`init(local = [..])` and passes the frame to the constructor.
