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

## UART port on the Nucleo-H753ZI

`hil/run-h753.sh`, with FerroHAL's H753 links Z2 (PC6 to PC7) and Z3 (PB6 to
PB7) fitted, as `hil/nucleo-h753zi-uart-loopback/src/main.rs` lists them. The
same firmware and checks as on the Nucleo-F401RE, on `usart_v4` and DMAMUX:
USART1 on DMA2 streams 5 and 4, USART6 on DMA1 streams 6 and 0, the data cache
off. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero [PORT-003].
