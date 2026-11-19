# Debt

Owed work on what ships. An item leaves when it is done; hardware items when a
bench proves them.

- **D2 (deps).** FerroForge is pinned by git revision to a branch carrying the
  group contract, and FerroHAL by git revision. Both move to crates.io versions
  when FerroForge 0.5 and FerroHAL are published.
- **D4 (contract).** Optional members are not in the contract yet
  (`docs/design/port-contract.md`).
- **D5 (contract).** The ADC port cannot state that a channel's pin is analog:
  its channels are a `[Channel<A>]` config value, which FerroHAL's
  `Channel::of(&pin)` builds from a pin `init` has made analog, so a tool
  reading the contract sees neither the pin nor its mode and cannot catch a pin
  conflict. Fixed by a typed analog-pin slot in the contract. *Spawned:* the
  event and ADC port prototype results, limitation E3 (2026-11-18), carried
  into `adc_event_port`.
