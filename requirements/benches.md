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
firmware selects `sbus_rx` on USART1, `uart_rx_port` on USART6 and `tick` on TIM4
(1 kHz) and TIM3 (250 Hz), every vector bound, and drives PA9 and PA11 as GPIO,
timed by the cycle counter with interrupts masked a frame at a time. It counts
ticks for 200 ms, then pauses both timers; sends 1024 bytes, a counter wrapping
at 256, into USART6 at 115 200 baud 8N1 in frames of 1 to 32 bytes; then sends
100 SBUS frames with known channels into USART1 at 100 000 baud 8E2, not
inverted, with one frame whose last byte is wrong after the fiftieth. Pass: every
check prints `PASS`, none prints `FAIL`, and the script exits zero. The checks:
every member's vector is enabled [PORT-101, PORT-111]; TIM4 ticked 199 to 201
times and TIM3 49 to 51, each numbered without a gap [PORT-111]; every telemetry
byte came back once and in order with no gap reported, and the reader was woken
on idle lines [PORT-101]; the decoder counted 100 good frames and 1 bad chunk
[PORT-121, PORT-122, PORT-123]; and the firmware's task was handed 100 updates, the last
one equal to the last frame sent [PORT-122].

## UART port on the Nucleo-H753ZI

`hil/run-h753.sh`, with FerroHAL's H753 links Z2 (PC6 to PC7) and Z3 (PB6 to
PB7) fitted, as `hil/nucleo-h753zi-uart-loopback/src/main.rs` lists them. The
same firmware and checks as on the Nucleo-F401RE, on `usart_v4` and DMAMUX:
USART1 on DMA2 streams 5 and 4, USART6 on DMA1 streams 6 and 0, the data cache
off. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero [PORT-003].
