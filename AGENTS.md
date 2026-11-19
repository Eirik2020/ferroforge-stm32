# ferroforge-stm32 - agent entry

FerroForge task groups for STM32 hardware ports, on FerroHAL. Groups: `uart_port`,
`uart_rx_port`, `spi_port`, `exti_line`, `timer_tick`, `adc_event_port`, `usb_device` (feature `usb`), `lane_frame_port`. Benches:
`hil/nucleo-f401re-uart-loopback`, `hil/nucleo-f401re-rx-ports`, `hil/nucleo-f401re-spi-loopback`,
`hil/nucleo-f401re-exti-line` (Nucleo-F401RE), `hil/stm32f4-discovery-spi-device`,
`hil/stm32f4-discovery-usb-device`, `hil/stm32f4-discovery-timer-tick` and
`hil/stm32f4-discovery-adc-port` (STM32F4 Discovery), `hil/nucleo-h753zi-uart-loopback`
and `hil/nucleo-h753zi-spi-loopback` (Nucleo-H753ZI),
`hil/stm32f4-discovery-lane-port` (STM32F4 Discovery, the lane port's proving board) and
`hil/nucleo-h753zi-lane-port` (the H753's lane figures, which verify no requirement),
all HIL tier. `hil/common/` holds checks several bench crates include.

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
| the SPI port | `docs/design/spi-port.md` |
| the USB port | `docs/design/usb-port.md` |
| add or move requirements | `src/requirements/<group>.yaml`; `reqtool check <file>` before changing a file |
| a hardware fact (pin, AF, DMA route, clock) | the ST document it comes from, cited where it is used (G3) |
| run or extend the bench | `requirements/benches.md`; the bench crate's module doc |
| what is owed | `docs/DEBT.md` |

## Commands

```
checks/run.sh                  # fmt, clippy + rustdoc per chip, host tests, bench images, reqtool validate
hil/run-f401.sh                # Nucleo-F401RE: the uart_port loopback (F401_PROBE picks the probe)
hil/run-f401-rx.sh             # Nucleo-F401RE: uart_rx_port twice, same links
hil/run-discovery-spi.sh       # STM32F4 Discovery: spi_port on the board's LIS3DSH, nothing fitted
hil/run-discovery-usb.sh       # STM32F4 Discovery: usb_device on CN5 (FERROHAL_DIR for the host script)
hil/run-f401-spi.sh            # Nucleo-F401RE: spi_port over FerroHAL's link S1
hil/run-f401-exti.sh           # Nucleo-F401RE: exti_line over FerroHAL's link J2
hil/run-discovery-tick.sh      # STM32F4 Discovery: timer_tick on TIM4, no link (DISCOVERY_PROBE)
hil/run-discovery-adc.sh       # STM32F4 Discovery: adc_event_port over link A1 (PA2 to PA3)
hil/run-h753.sh                # Nucleo-H753ZI: the same loopback (H753_PROBE)
hil/run-h753-spi.sh            # Nucleo-H753ZI: spi_port over FerroHAL's link S1
hil/run-discovery-lane-port.sh <layout> [f] [t]  # Discovery: lane_frame_port (F) and lanes under RTIC (T); layout reader-ahead, own-streams or paced-reader
hil/run-h753-lane-port.sh <layout> [f] [t]       # Nucleo-H753ZI: the same timing, reader-ahead or own-streams
reqtool validate | check       # requirements and evidence (reqtool at the revision CI pins)
```

Before every push, run what CI would fail the change on:
- Docs only (Markdown outside `src/` and `requirements/`): no local run, since no
  `checks/run.sh` step reads such files; push and let CI confirm.
- Anything else (code, requirements, evidence, procedures, benches, manifests): the full
  `checks/run.sh` with a reqtool built at CI's `REQTOOL_REV` (`.github/workflows/ci.yml`) on
  the PATH; run.sh skips `reqtool validate` when it finds none.

A local pass is not "fixed": only green repo CI on the pushed head is. When repo CI fails,
read the failing jobs' own logs first (`gh run view <id> --log-failed`, or the job log) and fix
every failure they show before pushing again; never guess from a local run.

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
