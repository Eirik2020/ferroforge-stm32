# Debt

Owed work on what ships. An item leaves when it is done; hardware items when a
bench proves them.

- **D2 (deps).** FerroForge is pinned by git revision to a branch carrying the
  group contract, and FerroHAL by git revision. Both move to crates.io versions
  when FerroForge 0.5 and FerroHAL are published.
- **D4 (contract).** Optional members are not in the contract yet
  (`docs/design/port-contract.md`).
- **S1 (code, FerroHAL).** `spi_port` switches each device's settings through
  FerroHAL's `Spi::set_config` (PORT-208), and FerroHAL states no requirement
  for that behaviour: a settings change between transfers applying to the
  next transfer, refused while one runs. Fixed by that FerroHAL requirement
  with its evidence, then a `library:` entry on PORT-208; the Nucleo-H753ZI
  entry on PORT-208 then goes. *Spawned:* the car rule check of the SPI port,
  2026-11-19.
- **S2 (perf, FerroHAL).** The port copies each message twice: FerroHAL's
  `start` copies the lent buffer into its own DMA buffer, and the port copies
  what came in back over it (about 1.1 cycles a byte each way on the F401,
  prototype 2), and FerroHAL's `start` costs about 1,000 cycles on the F401
  before a byte moves, not profiled. Fixed by DMA on lent memory in FerroHAL,
  or a profiled `start`. *Spawned:* SPI prototype 2 results, 2026-11-17.
- **S3 (contract).** The contract cannot state the number of devices, nor
  each device's buffers and their lengths: the constructor checks them at
  boot. Fixed by a contract entry for a repeated unit (a device: its chip
  select, settings, handle and memory). *Spawned:* SPI prototype 2 results,
  2026-11-17.
- **S4 (mem).** The port's `Memory` is not all zero bytes (`rtic-sync`'s
  `Signal` starts as a non-zero variant), so a firmware's `init(local = [..])`
  places it in `.data`, costing its size in flash as well. Fixed by a wake
  whose empty state is zero bytes. *Spawned:* SPI prototype 2 results,
  2026-11-17.
- **S5 (perf, FerroHAL).** A receive stream that completes early holds the
  port's vector for about 1.2 ms. FerroHAL's `Spi::on_interrupt`, on the
  receive stream's completion, waits for the SPI to finish its frame. It
  bounds that wait at 32 SCK periods counted as polling passes, each counted
  as at least one cycle (SPI-008's floor). When the stream stops before the
  frame ends (its EN cleared under the driver), the wait runs to the bound
  and ends `Timeout`. On the F401 at 328 kHz the bound is 8,192 passes, and
  the port's run took 103,005 cycles (1.23 ms), about 12.6 cycles a pass.
  Measured twice on the SPI variant bench, 2026-11-19. A DMA transfer error
  does not reach the wait: FerroHAL fails at once. Fixed in FerroHAL: a
  bound in real time, or an end of frame that does not wait inside the
  interrupt. *Spawned:* the SPI port variant bench, 2026-11-19.
- **U1 (code, FerroHAL).** `uart_port` infers which transmit frames finished
  from FerroHAL's `queued()` and the deltas of `tx_health()`, and restates
  FerroHAL's private queue depth of 8 (`IN_FLIGHT`). FerroHAL states no
  requirement for those counters, so PORT-009 has no `library:` entry for them,
  and a call that finishes a sent and a failed frame together reports both
  failed (`Port::unsure`, zero in every run). Fixed by a per-frame completion
  from FerroHAL with its depth exported (the review's recommendation), then a
  `library:` entry on PORT-009. *Spawned:* UART prototype 2 results, finding 1
  (2026-11-17); the car rule check of the UART port, 2026-11-19.
- **U2 (code, FerroHAL).** The UART port has no line-idle completion. A frame's
  `Sent` is FerroHAL's DMA completion; on the Discovery bench it reached the
  consumer about 1.6 characters before the line went quiet
  (`docs/design/uart-port.md`). The owner decided on 2026-11-19 that "sent"
  stays there and D15 is met by a separate completion on the USART's
  transmission complete, for a consumer that turns a line around (RS-485, half
  duplex). Fixed by FerroHAL's TC completion, then the port passing it to the
  handle. *Spawned:* UART prototype 2 results, finding 4 (2026-11-17); the
  owner's answer, 2026-11-19.
