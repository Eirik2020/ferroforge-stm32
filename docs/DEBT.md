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
- **D6 (lanes, bench).** Every lane port margin is one lane's: each bench links
  one lane to the responder, and the other lanes read only their idle level.
  A second linked lane on each bench would measure the rest.
- **D7 (lanes, bench).** The STM32F4 Discovery's three-port layout has no pin
  witness: TIM1 and TIM8 are both lane timers there, and on the F407 only they
  have requests on DMA2, the only controller that reaches GPIO (RM0090 table
  44, figure 33). What the line does between exchanges is witnessed only on
  the H753.
- **D8 (lanes).** On the H753 a lane's line shows a pulse of about 210 ns at the
  release (FerroHAL's DEBT); a reply starting inside it would lose its first
  edge. The F407 shows none. It leaves when FerroHAL's item does.
