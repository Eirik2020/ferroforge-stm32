# Bench Procedures

What each requirement's bench entry runs. The benches are under `hil/`, one crate
per board; each is a `ferroforge::app!` firmware, and doubles as the golden
reference a generated firmware must match.

## Receive ports on the Nucleo-F401RE

`hil/run-f401-rx.sh`, with FerroHAL's F401 links J1 (PA9 to PA10) and J2 (PA11 to
PA12) fitted, as `hil/nucleo-f401re-rx-ports/src/main.rs` lists them. The
firmware selects `uart_rx_port` on USART6 and on USART1, every vector bound,
with SysTick as its monotonic at 1 kHz, and drives PA9 and PA11 as GPIO, timed
by the cycle counter with interrupts masked a frame at a time. It sends 1024
bytes, a counter wrapping at 256, in frames of 1 to 32 bytes, into USART6 at
115 200 baud 8N1 and then into USART1 at 100 000 baud 8E2. Pass: every check
prints `PASS`, none prints `FAIL`, and the script exits zero. The checks: every
member's vector is enabled [PORT-101]; on each port every byte came back once
and in order with no gap reported [PORT-101]; each port woke its reader on idle
lines [PORT-101]; every wake's stamp lay within the run, no earlier than the
wake before it and no later than its reader ran [PORT-102]; and USART6's wakes
were all stamped no later than USART1's first [PORT-102].

## SPI port on the Nucleo-F401RE

`hil/run-f401-spi.sh`, with FerroHAL's F401 link S1 (PA7 to PA6) fitted, as
`hil/nucleo-f401re-spi-loopback/src/main.rs` lists it. The firmware selects
`spi_port` on SPI1 at its slowest clock, 328 kHz, with PA4 as chip select and
nothing on it, and runs 256 transfers one at a time, of 1 to 64 bytes, a counter
carrying on from one to the next, then aborts a 64-byte transfer mid-frame and
runs one more. Pass: every check prints `PASS`, none prints `FAIL`, and the
script exits zero. The checks: the member's vector is enabled [PORT-201]; the
device starts deselected [PORT-202]; every transfer's end reached the firmware's
task, and completed with its length and the bytes sent [PORT-201]; chip select
read low while each 64-byte transfer ran [PORT-202], and high when the
firmware's task first ran for every transfer [PORT-202]; the abort left chip
select high and the bus idle [PORT-202] and passed no outcome on [PORT-201];
and the transfer after it completed [PORT-201].

## SPI port on the Nucleo-H753ZI

`hil/run-h753-spi.sh`, with FerroHAL's H753 link S1 (PD7 to PA6) fitted, as
`hil/nucleo-h753zi-spi-loopback/src/main.rs` lists it. The same firmware and
checks as on the Nucleo-F401RE, on `spi_v4` and DMAMUX: SPI1 at 391 kHz on DMA1
streams 5 and 4, PD14 as chip select, the data cache off. Pass: every check
prints `PASS`, none prints `FAIL`, and the script exits zero [PORT-201,
PORT-202].

## EXTI line on the Nucleo-F401RE

`hil/run-f401-exti.sh`, with FerroHAL's F401 link J2 (PA11 to PA12) fitted, as
`hil/nucleo-f401re-exti-line/src/main.rs` lists it. The firmware selects
`exti_line` on line 12, PA12's rising edge, with SysTick at 1 kHz as the
monotonic, and drives PA11 as a GPIO: 100 edges one at a time, each awaited;
one edge while it holds the firmware's task off for 5 ms; three edges while it
holds the task off, then one more. Pass: every check prints `PASS`, none prints
`FAIL`, and the script exits zero. The checks: the member's vector is enabled
[PORT-301]; every one of the 100 edges reached the firmware's task, none missed
[PORT-301], each stamped within a tick of being driven and no later than its
task ran [PORT-302]; the held edge's stamp is within a tick of its driving and
at least 4 ms before its task ran [PORT-302]; of the three held edges one
reached the task, and the edge after carried a count of 2 missed [PORT-301]; and
the line is left clear [PORT-301].

## EXTI line on the Nucleo-H753ZI

`hil/run-h753-exti.sh`, with FerroHAL's H753 link Z1 (PD5 to PD6) fitted, as
`hil/nucleo-h753zi-exti-line/src/main.rs` lists it. The same firmware and checks
as on the Nucleo-F401RE, on line 6, PD6's rising edge, served from the shared
`EXTI9_5` vector. Pass: every check prints `PASS`, none prints `FAIL`, and the
script exits zero [PORT-301, PORT-302].

## ADC sequence on the Nucleo-F401RE

`hil/run-f401-adc.sh`, with FerroHAL's F401 link S1 (PA7 to PA6) fitted, as
`hil/nucleo-f401re-adc-scan/src/main.rs` lists it. The firmware selects
`adc_scan` on ADC1 and DMA2 stream 0, its channels PA6, VREFINT and the
temperature sensor named before the converter opens, and runs 200 sequences one
at a time, driving PA7 low and high in turn. Pass: every check prints `PASS`,
none prints `FAIL`, and the script exits zero. The checks: the member's vector
is enabled [PORT-401]; every sequence's end reached the firmware's task, and
every one completed [PORT-401]; PA6's sample sat within 3% of the rail driven in
every sequence [PORT-401]; and VREFINT gave VDDA between 3.0 and 3.6 V and the
sensor a die temperature between 0 and 85 degrees in every sequence, so each
sample landed in its channel's place [PORT-401].

## ADC sequence on the Nucleo-H753ZI

`hil/run-h753-adc.sh`, with FerroHAL's H753 link Z10 (PF7 to PF6) fitted, as
`hil/nucleo-h753zi-adc-scan/src/main.rs` lists it. The same firmware and checks
as on the Nucleo-F401RE, on `adc_v4` and DMAMUX: ADC3, which carries the
internal pair, on DMA2 stream 0, reading PF6. Pass: every check prints `PASS`,
none prints `FAIL`, and the script exits zero [PORT-401].

## UART port on the STM32F4 Discovery

`hil/run-discovery-uart.sh`, with FerroHAL's Discovery links D1 (PC10 to PC11)
and D2 (PC12 to PD2) fitted, as `hil/stm32f4discovery-uart-loopback/src/main.rs`
lists them. The firmware, on die 0x413 at 168 MHz with the CS43L22 held in
reset, selects `uart_port` for USART3 (DMA1 streams 3 and 1) and UART5 (DMA1
streams 7 and 0), every vector bound, with SysTick at 1 kHz as the monotonic, and gives the two handles to two consumer
tasks, which task gets which UART chosen at boot from a run-time value. Each
consumer (`src/consumer.rs`) sends 16 384 bytes, a counter wrapping at 256, in
32-byte frames written in its lent buffers, one at a time, and reads every
chunk in place. USART3 then sends 2048 bytes while its consumer holds every
receive buffer for 300 ms, and 256 more after; UART5 sends eight frames of 4
bytes, eight of 64 and one of 280. Pass: every check prints `PASS`, none prints
`FAIL`, and the script exits zero. The checks: every member's vector is enabled
[PORT-003]; on each port every byte came back once and in order, before the
flood on USART3 [PORT-003, PORT-008]; the loss while held reached the consumer
as a gap, with the stream in order on both sides and the generation advanced
[PORT-006]; the bytes the pool held were read first and the 256 after the gap
read too [PORT-007]; frame ends arrived as idle chunks [PORT-005]; no chunk
changed generation before the first gap [PORT-005]; every chunk's stamp came
from the monotonic, in order and no later than its read [PORT-005]; every frame
came back sent, in order, with every byte, and no buffer was lost [PORT-009];
the 280-byte frame came back whole with one frame end [PORT-008]; with every
buffer out a frame was refused as `Outstanding` and every byte was still sent
[PORT-010]; and each consumer drove the UART bound to it at boot [PORT-004].
The run also prints how long before the last stop bit each 64-byte frame's
outcome arrived; that figure is measured, not judged.
