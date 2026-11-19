# Debt

Owed work on what ships. An item leaves when it is done; hardware items when a
bench proves them.

- **D2 (deps).** FerroForge is pinned by git revision to a branch carrying the
  group contract, and FerroHAL by git revision. Both move to crates.io versions
  when FerroForge 0.5 and FerroHAL are published.
- **D4 (contract).** Optional members are not in the contract yet
  (`docs/design/port-contract.md`).
- `code` `api` **Medium: the UART port infers each frame's outcome.** `uart_port`
  reads which transmit frames finished from FerroHAL's `queued()` and the deltas
  of `tx_health()`, and restates FerroHAL's private queue depth of 8
  (`IN_FLIGHT`). FerroHAL states no requirement for those counters, so PORT-009
  has no `library:` entry for them, and a call that finishes a sent and a failed
  frame together reports both failed (`Port::unsure`, zero in every run). Fix: a
  FerroHAL requirement for the counters, or a per-frame completion from FerroHAL,
  and its depth exported. *Spawned:* UART prototype 2 results, findings 1 and
  the DEBT table (2026-11-17); the car rule (2026-11-19).
- `code` **Low: the UART port has no line-idle completion.** A frame's `Sent` is
  FerroHAL's DMA completion; on the Discovery bench it reached the consumer about
  1.6 characters before the line went quiet (`docs/design/uart-port.md`). The
  owner decided on 2026-11-19 that "sent" stays there and D15 is met by a
  separate line-idle completion on the USART's transmission complete, for a
  consumer that turns a line around (RS-485, half duplex). Fix: FerroHAL's TC
  completion, then the port passes it to the handle. *Spawned:* UART prototype 2
  results, finding 4 (2026-11-17); the owner's answer (2026-11-19).
