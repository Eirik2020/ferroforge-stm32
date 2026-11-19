# Debt

Owed work on what ships. An item leaves when it is done; hardware items when a
bench proves them.

- **D2 (deps).** FerroForge and FerroHAL are pinned by git revision, each a
  commit on its own repository. Both move to crates.io versions when FerroForge
  0.5 and FerroHAL are published.
- **D4 (contract).** Optional members are not in the contract yet
  (`docs/design/port-contract.md`).
- **D5 (lanes, bench).** Every lane port margin is one lane's: each bench links
  one lane to the responder, and the other lanes read only their idle level.
  A second linked lane on each bench would measure the rest.
- **D6 (lanes, bench).** The STM32F4 Discovery's three-port layout has no pin
  witness: TIM1 and TIM8 are both lane timers there, and on the F407 only they
  have requests on DMA2, the only controller that reaches GPIO (RM0090 table
  44, figure 33). What the line does between exchanges is witnessed only on
  the H753.
- **D7 (lanes).** On the H753 a lane's line shows a pulse of about 210 ns at the
  release (FerroHAL's DEBT); a reply starting inside it would lose its first
  edge. The F407 shows none. It leaves when FerroHAL's item does.
