# The lane frame port

Accepted by the owner on 2026-11-19 (decisions D28 and D30 to D34 of the port design
record). The FerroHAL lanes it runs on are FerroHAL's `docs/design/lanes.md` (ADR-014).
The group is `lanes_frame::lane_frame_port`; its requirements are PORT-601 to PORT-609
(`src/requirements/lanes.yaml`).

## What the port is

A frame port (D28). The consumer hands it one frame of samples per lane and, optionally,
asks for a receive of so many samples at such a rate. The port plays the frames from one
start, switches the lanes to listening in the interrupt that sees the frames end, and
returns the exchange with its outcome, what was read, and the times. A sample is one
timer period's duty on send and one reading of the pin's port on receive. The port never
sees protocol bits (D6): a protocol encodes and decodes in the firmware, behind the
handle.

It must work with the pins, timers and DMA streams a board gives its motor outputs
(D29). A board's resources choose one of three reader forms per port (D31); the port's
code is the same for all three.

## How it is built

- The port's member owns FerroHAL's generator; the consumer holds a `FrameHandle` and
  never names or locks the driver (as on the SPI port, D22).
- One exchange runs at a time; `exchange()` while one is out is refused (D8).
- `submit` checks and closes every frame in the consumer's context (`Frame::new`), and
  refuses the exchange whole if any frame is refused. The member only starts streams.
- The exchange lives in the port's memory (`FrameMemory`). Submit and return pass a
  reference to it through one-slot queues, so its 300 bytes are never copied (D33).
- Two members. `f_on_lane_dma` serves every lane's stream vector (`vectors = [f_on_lane_dma
  = [generator]]`); `f_on_start` serves one vector the firmware chooses from those
  nothing else raises, and the handle pends it (`pends = [kick = f_on_start]`): FerroForge
  pends only a member that serves one vector.
- A receive returns the reader's share of lent memory: every lane on one timer and port
  gets the same port words.
- Any fault, refused load or cancel stops the generator, leaves every output at its
  inactive level and comes back as the exchange's outcome (D16). Nothing panics.
- Priorities are the firmware's (D14). The figures below say how long the members run.

## Which form a board gets

On the F4 only DMA2 reaches GPIO (RM0090 figure 33 note 1), and on the F405 and F407 only
TIM1 and TIM8 have requests on DMA2 (RM0090 table 44).

| form | when | bench layout |
|---|---|---|
| readers armed ahead | lanes on TIM1 or TIM8, a DMA2 stream with a spare TIM1 or TIM8 request free | `reader-ahead` |
| the lanes' own streams | lanes on TIM1 or TIM8, no spare DMA2 stream | `own-streams` |
| a paced reader | lanes on any other timer (sending on DMA1), TIM1 or TIM8 and one DMA2 stream free | `paced-reader` |

A board with none of these cannot receive. On the H7 every stream reaches GPIO and
DMAMUX routes any request, so a reader can be armed ahead whenever a stream is free.

## Evidence

The port is proven on the STM32F4 Discovery alone (D34): its FerroHAL needs are FerroHAL
requirements (`library: ferrohal-stm32:AWG-0xx` entries), and each reader form is a
layout of the one bench, `hil/stm32f4-discovery-lane-port`. A Discovery run with a
layout borrowed from a board's motor outputs is HIL evidence for the port on die 0x413,
not evidence for that board. The Nucleo-H753ZI bench, `hil/nucleo-h753zi-lane-port`,
verifies no requirement: it measures the H753's figures below, which the contract states
per chip.

## Figures

Measured 2026-11-19 on the product build (FerroHAL c723b1c), four lanes on the Discovery
and two on the H753, 600 kHz, the reply's start swept in steps of 1.67 us. The margin is
from the latest reply start read whole every time to 30 us, with no profiling feature on.
F is the frame port; T is FerroHAL's lanes under RTIC with no port, the firmware's task
switching. The switch is FerroHAL's listen whole (`profile` alone, without
`profile-start`); cycles are DWT counts, means over 1000 exchanges (the switch over the
sweep's 37,000).

STM32F4 Discovery (168 MHz):

| layout | margin F / T | round 2 prototype F / T | switch F / T | start whole (F) |
|---|---|---|---|---|
| reader-ahead | 10841 / 10841 ns | 10.8 / 9.2 us | 686 / 639 cycles | 5563 cycles |
| own-streams | 5843 / 4177 ns | 5.8 / 4.2 us | 1661 / 1611 cycles | 4474 cycles |
| paced-reader | 10841 / 9175 ns | 10.8 / 9.2 us | 711 / 715 cycles | 4920 cycles |

Nucleo-H753ZI (400 MHz):

| layout | margin F / T | round 2 prototype F / T | switch F / T | start whole (F) |
|---|---|---|---|---|
| reader-ahead | 14183 / 12518 ns | 14.2 / 12.5 us | 1513 / 1440 cycles | 12854 cycles |
| own-streams | 10853 / 9188 ns | 9.2 / 7.5 us | 2958 / 2860 cycles | 11280 cycles |

Every margin is at the prototype's sweep step or past it. The start without the repeated
checks and the queue's copy (D33): 5563 cycles with readers armed ahead on the Discovery,
against 8195 on the round 2 prototype's same layout, and 12854 on the H753 against 19106.

## Not in this port

- Repeat, stream and endless receive (D32): their own group beside this one, not built.
- Input capture as a receive (D31): left out.
