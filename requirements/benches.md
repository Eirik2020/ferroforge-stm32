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

## Timer tick on the STM32F4 Discovery

`hil/run-discovery-tick.sh`, with no link, as
`hil/stm32f4-discovery-timer-tick/src/main.rs` lists it. The firmware selects
`timer_tick` on TIM4 at 1 kHz, with SysTick at 1 kHz as the monotonic, on the
F407 at 168 MHz from the ST-LINK's 8 MHz clock, and lets it run: a quiet window
of 1000 ticks with an empty task; the task stalling once for 5.5 periods; the
task held off for 5 ms from just after a tick. Pass: every check prints `PASS`,
none prints `FAIL`, and the script exits zero. The checks: the member's vector
is enabled [PORT-501]; every tick of the quiet window reached the task, none
missed, as many as the monotonic's elapsed periods [PORT-501]; every tick was
stamped a period after the one taken before it, counting the missed ones,
within a monotonic tick, and no later than its task ran [PORT-502]; after the
stall the next tick carried 5 missed, and the stamps' span matched the periods
taken and counted [PORT-501, PORT-502]; the tick queued while the task was held
kept a stamp within a period and a monotonic tick of the hold's start, at least
3 ms before its task ran [PORT-502]; and the tick after it carried the 4 that
found the task queued [PORT-501].

## Timer tick on the Nucleo-H753ZI

`hil/run-h753-tick.sh`, with no link, as
`hil/nucleo-h753zi-timer-tick/src/main.rs` lists it. The same firmware and
checks as on the STM32F4 Discovery, on `timer_v3` at 400 MHz, TIM4 counting at
200 MHz. It runs because the port rests on `Tick::on_interrupt` clearing its
update flag and answering `false` for another timer's pass, which no FerroHAL
requirement covers yet, and `timer_v3` is the other timer IP version. Pass:
every check prints `PASS`, none prints `FAIL`, and the script exits zero
[PORT-501, PORT-502].

## ADC port on the STM32F4 Discovery

`hil/run-discovery-adc.sh`, with link A1 (PA2 to PA3) fitted, as
`hil/stm32f4-discovery-adc-port/src/main.rs` lists it. The firmware selects
`adc_event_port` on ADC1 and DMA2 stream 0, its channels PA3, VREFINT and the
temperature sensor named before the converter opens, with SysTick at 1 kHz as
the monotonic. It kicks 200 sequences one at a time, driving PA2 low and high in
turn; then lets two sequences end, low then high, while it holds the firmware's
task off; then kicks twice 20 us apart. Pass: every check prints `PASS`, none
prints `FAIL`, and the script exits zero. The checks: the member's vector is
enabled [PORT-402, PORT-403]; every kick started one sequence whose end reached
the task, completed, with nothing missed or late [PORT-402, PORT-403,
PORT-404]; in every sequence PA3's sample sat within 3% of the rail driven and
VREFINT gave VDDA between 2.7 and 3.3 V and the sensor a die temperature
between 0 and 85 degrees, so each sample landed in its channel's place
[PORT-403]; every sequence was stamped within a monotonic tick of its kick and
no later than its task ran [PORT-403]; of the two held sequences the task took
the first with its own low sample [PORT-403], and the next sequence carried 1
missed [PORT-404]; and the two quick kicks ended one sequence, which carried 1
late kick [PORT-402].

## UART port on the Nucleo-H753ZI

`hil/run-h753.sh`, with FerroHAL's H753 links Z2 (PC6 to PC7) and Z3 (PB6 to
PB7) fitted, as `hil/nucleo-h753zi-uart-loopback/src/main.rs` lists them. The
same firmware and checks as on the Nucleo-F401RE, on `usart_v4` and DMAMUX:
USART1 on DMA2 streams 5 and 4, USART6 on DMA1 streams 6 and 0, the data cache
off. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero [PORT-003].
