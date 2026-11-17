# Debt

Owed work on what ships. An item leaves when it is done; hardware items when a
bench proves them.

- **D1 (bench).** `uart_port` on the Nucleo-F401RE has no recorded run yet
  (PORT-003).
- **D2 (deps).** FerroForge is pinned by git revision to a branch carrying the
  group contract, and FerroHAL by git revision. Both move to crates.io versions
  when FerroForge 0.5 and FerroHAL are published.
- **D3 (usart_v4).** No bench on an H7: `uart_port` is proven on `usart_v2`
  only. The H753 host test shows it builds there.
- **D4 (contract).** Buffers, config fields and optional members are not in the
  contract yet (`docs/design/port-contract.md`).
