# Bench Procedures

What each requirement's bench entry runs. The benches are under `hil/`, one crate
per board; each is a `ferroforge::app!` firmware, and doubles as the golden
reference a generated firmware must match.

## UART port on the Nucleo-F401RE

`hil/run-f401.sh`, with FerroHAL's F401 links J1 (PA9 to PA10) and J2 (PA11 to
PA12) fitted, as `hil/nucleo-f401re-uart-loopback/src/main.rs` lists them. The
firmware selects `uart_port` for USART1 and USART6, with every vector bound, and
sends 4096 bytes, a counter wrapping at 256, round each loop in frames of 1 to 64
bytes. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero. The checks: every member's vector is enabled; both ports queued every
byte; on each port every byte came back once and in order with no gap reported;
the reader was woken by the group, at least once on an idle line; and the
transmit vector reported frames sent [PORT-003].

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

## UART port on the Nucleo-H753ZI

`hil/run-h753.sh`, with FerroHAL's H753 links Z2 (PC6 to PC7) and Z3 (PB6 to
PB7) fitted, as `hil/nucleo-h753zi-uart-loopback/src/main.rs` lists them. The
same firmware and checks as on the Nucleo-F401RE, on `usart_v4` and DMAMUX:
USART1 on DMA2 streams 5 and 4, USART6 on DMA1 streams 6 and 0, the data cache
off. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero [PORT-003].
