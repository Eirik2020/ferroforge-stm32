# ferroforge-stm32 - agent entry

FerroForge task groups for STM32 hardware ports, on FerroHAL. Groups: `uart_port`,
`uart_rx_port`. Benches: `hil/nucleo-f401re-uart-loopback` and
`hil/nucleo-f401re-rx-ports` (Nucleo-F401RE), `hil/nucleo-h753zi-uart-loopback`
(Nucleo-H753ZI), all HIL tier.

## Authority

- This file governs this tree; only the owner outranks it. Then
  `requirements/goals.md` (G1-G5), then `docs/`. A document that diverges from
  either is wrong: fix it. A conflict between this file and the goals needs the
  owner's decision.
- Governance mirrors FerroHAL's (`Eirik2020/ferrohal-stm32`): reqtool, one
  `shall` per requirement, `checks/run.sh`, CI. Where this file says nothing,
  FerroHAL's AGENTS.md rules are the model, not the authority.

## Read when

| task | docs |
|---|---|
| add or change a group, its contract | `docs/design/port-contract.md`; `requirements/goals.md` G1, G2 |
| add or move requirements | `src/requirements/<group>.yaml`; `reqtool check <file>` before changing a file |
| a hardware fact (pin, AF, DMA route, clock) | the ST document it comes from, cited where it is used (G3) |
| run or extend the bench | `requirements/benches.md`; the bench crate's module doc |
| what is owed | `docs/DEBT.md` |

## Commands

```
checks/run.sh                  # fmt, clippy + rustdoc per chip, host tests, bench images, reqtool validate
hil/run-f401.sh                # Nucleo-F401RE: the uart_port loopback (F401_PROBE picks the probe)
hil/run-f401-rx.sh             # Nucleo-F401RE: uart_rx_port twice, same links
hil/run-h753.sh                # Nucleo-H753ZI: the same loopback (H753_PROBE)
reqtool validate | check       # requirements and evidence (reqtool at the revision CI pins)
```

## Rules

- G5: `#![forbid(unsafe_code)]`. A chip's unsafe belongs to FerroHAL; anything that
  needs it is a FerroHAL change.
- A group names FerroHAL's chip-neutral types only (`uart::Uart`, `Service`), never
  an IP-version module or a chip feature.
- Every group states its contract (G2); the contract's syntax and what the macro
  checks are FerroForge's (`ferroforge-contracts`).
- Hardware facts cite ST documents (RM, DS, ES; the owner's copies are in the
  project's `docs/`, downloads in the ignored `local/`). Never infer one from
  another HAL.
- The rig: flash only with the owner's or coordinator's grant for that board.
  Never claim a pass from a build; record bench evidence with `reqtool record`.
- reqtool hashes a procedure's section with its trailing blank line, so appending a
  section to `requirements/benches.md` re-owes the old last one: add new sections
  above the last.
- Dependencies are pinned exactly (FerroHAL and FerroForge by git revision until
  they are on crates.io). A bump re-owes the files that import it.
- One commit per item, detailed body. Nothing documented only in a commit, PR or
  chat. No progress narration in docs.
