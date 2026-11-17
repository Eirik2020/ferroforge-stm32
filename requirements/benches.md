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

## Receive ports and ticks on the Nucleo-F401RE

`hil/run-f401-rx.sh`, with FerroHAL's F401 links J1 (PA9 to PA10) and J2 (PA11 to
PA12) fitted, as `hil/nucleo-f401re-rx-ports/src/main.rs` lists them. The
firmware selects `uart_rx_port` on USART6 and on USART1 and `tick` on TIM4
(1 kHz) and TIM3 (250 Hz), every vector bound, and drives PA9 and PA11 as GPIO,
timed by the cycle counter with interrupts masked a frame at a time. It counts
ticks for 200 ms, then pauses both timers; then sends 1024 bytes, a counter
wrapping at 256, in frames of 1 to 32 bytes, into USART6 at 115 200 baud 8N1 and
then into USART1 at 100 000 baud 8E2. Pass: every check prints `PASS`, none
prints `FAIL`, and the script exits zero. The checks: every member's vector is
enabled [PORT-101, PORT-111]; TIM4 ticked 199 to 201 times and TIM3 49 to 51,
each numbered without a gap [PORT-111]; on each port every byte came back once
and in order with no gap reported [PORT-101]; and each port woke its reader on
idle lines [PORT-101].

## UART port on the Nucleo-H753ZI

`hil/run-h753.sh`, with FerroHAL's H753 links Z2 (PC6 to PC7) and Z3 (PB6 to
PB7) fitted, as `hil/nucleo-h753zi-uart-loopback/src/main.rs` lists them. The
same firmware and checks as on the Nucleo-F401RE, on `usart_v4` and DMAMUX:
USART1 on DMA2 streams 5 and 4, USART6 on DMA1 streams 6 and 0, the data cache
off. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero [PORT-003].
