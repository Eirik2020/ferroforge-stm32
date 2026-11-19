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

## SPI port on the STM32F4 Discovery

`hil/run-discovery-spi.sh`, with nothing fitted: the board's LIS3DSH
accelerometer is the device, on SPI1 with its chip select on PE3, as
`hil/stm32f4-discovery-spi-device/src/main.rs` lists it (UM1472 table 7). The
firmware selects `spi_port` on SPI1 at 168 MHz with three devices: device 0 the
LIS3DSH, mode 3 at 5.25 MHz, four 16-byte buffers; devices 1 and 2 free I/Os,
PE7 (mode 0 at 656 kHz, four 64-byte buffers) and PE8 (mode 0 at 1.3125 MHz,
three 16-byte buffers), with nothing on them. A probe on device 0 reads
WHO_AM_I and must give 0x3F; a probe on devices 1 and 2 writes four bytes and
keeps none. The shared checks are `hil/common/spi_port.rs`'s. Pass: every check
prints `PASS`, none prints `FAIL`, and the script exits zero. The checks: the
member's vector is enabled [PORT-201]; the handles are one type, taken by a
run-time choice [PORT-207]; every device starts deselected [PORT-202]; 100
reads of WHO_AM_I give 0x3F [PORT-201, PORT-209]; 100 three-byte patterns
written to OFF_X to OFF_Z in one message read back in another [PORT-209]; a
write, a read, a second write and a read queued at once, at priorities against
that order, read back the two writes in order [PORT-203]; 100 alternations
with device 1 each read 0x3F, with a settings switch at every change of device
[PORT-208]; 1000 probes come back done with 0x3F [PORT-201]; while a 64-byte
message runs on device 1 only its line is low, and all are high once it is
back [PORT-202]; device 1's buffers all out, the next message is refused
`Outstanding` and every lent one comes back [PORT-205]; with a low and a high
on device 1 and two medium probes on device 0, they end low, high, medium,
medium [PORT-204]; a cancel on device 1 aborts its running and two queued
messages with the line high, leaves device 0's probe alone, and the probe
submitted straight after the cancel runs [PORT-206]; with device 1's buffers
all out, device 0's probe is accepted and comes back [PORT-205]; the checker
on device 0 and a task at priority 2 on device 2 each run 500 probes at once,
all back done with their data [PORT-201]; and the port lost no message and
FerroHAL counted no fault [PORT-201].

## SPI port on the Nucleo-F401RE

`hil/run-f401-spi.sh`, with FerroHAL's F401 link S1 (PA7 to PA6) fitted, as
`hil/nucleo-f401re-spi-loopback/src/main.rs` lists it. The firmware selects
`spi_port` on SPI1 with four devices, each with its own settings (mode 0 at
328 kHz, mode 3 at 1.3125 MHz, mode 1 at 656 kHz, mode 2 at 328 kHz), chip
selects on PA4, PC8, PC5 and PC4 with nothing on them, and runs
`hil/common/spi_port.rs`'s checks round the loop, with messages of 4 to 256
bytes. One board proves a port (D34), the STM32F4 Discovery, so this run is no
requirement's entry; it is the loopback bench the port's timings are compared
on. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero.

## SPI port on the Nucleo-H753ZI

`hil/run-h753-spi.sh`, with FerroHAL's H753 link S1 (PD7 to PA6) fitted, as
`hil/nucleo-h753zi-spi-loopback/src/main.rs` lists it. The same firmware and
checks as on the Nucleo-F401RE, on `spi_v4` and DMAMUX: SPI1 on DMA1 streams 5
and 4, chip selects on PD14, PD15, PE9 and PE11, the data cache off. Pass:
every check prints `PASS`, none prints `FAIL`, and the script exits zero. It
is an entry of PORT-208 only, whose FerroHAL need, a settings change between
transfers, has no FerroHAL requirement to map yet (`docs/DEBT.md` S1), and
whose path differs on `spi_v4`, which disables the SPI after each transfer
[PORT-208].

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
is enabled [PORT-701]; every tick of the quiet window reached the task, none
missed, as many as the monotonic's elapsed periods [PORT-701]; every tick was
stamped a period after the one taken before it, counting the missed ones,
within a monotonic tick, and no later than its task ran [PORT-702]; after the
stall the next tick carried 5 missed, and the stamps' span matched the periods
taken and counted [PORT-701, PORT-702]; the tick queued while the task was held
kept a stamp within a period and a monotonic tick of the hold's start, at least
3 ms before its task ran [PORT-702]; and the tick after it carried the 4 that
found the task queued [PORT-701].

## Timer tick on the Nucleo-H753ZI

`hil/run-h753-tick.sh`, with no link, as
`hil/nucleo-h753zi-timer-tick/src/main.rs` lists it. The same firmware and
checks as on the STM32F4 Discovery, on `timer_v3` at 400 MHz, TIM4 counting at
200 MHz. It runs because the port rests on `Tick::on_interrupt` clearing its
update flag and answering `false` for another timer's pass, which no FerroHAL
requirement covers yet, and `timer_v3` is the other timer IP version. Pass:
every check prints `PASS`, none prints `FAIL`, and the script exits zero
[PORT-701, PORT-702].

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

## Lane frame port on the STM32F4 Discovery, readers armed ahead

`hil/run-discovery-lane-port.sh reader-ahead f`, with the link PB0 (P1-22) to PC9 (P2-46) through 1 kOhm, as `hil/stm32f4-discovery-lane-port/src/layout_three_ports.rs` lists it: four lanes on TIM1 and TIM8 over ports A, B and C, each port read by a reader armed ahead on a spare TIM1 or TIM8 request on DMA2; the responder is TIM3 CH3 on PB0. The port plays each frame in place [PORT-603] and listens in the run that sees the frames end [PORT-604]; the reply 30 us after the frames end is read whole through the readers armed ahead [PORT-607]. The checks, `hil/lane-common/loopback.rs`, image F (`lanes-f`, the lane frame port; image T, FerroHAL's lanes under RTIC with no port, runs the same checks for the switch's figures and verifies nothing here): 1000 exchanges whose reply starts 30 us after the frames end, each read whole by the linked lane, every other lane reading its idle level, and every exchange coming back; then the reply start swept earlier one step (1.67 us at 600 kHz) at a time, which reports the margin; 1000 send-only exchanges, each sent once, and a frame seen played exactly once [PORT-603]; a 500-sample frame that plays once and ends, and a 1024-sample receive on every lane that comes back whole; a cancel mid-frame and mid-receive, each coming back aborted with the line idle and the next exchange read whole [PORT-606]; every lent share given back, and no exchange lost by the port's queues. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits zero.

## Lane frame port on the STM32F4 Discovery, lanes' own streams

`hil/run-discovery-lane-port.sh own-streams f`, the same link and lanes as with readers armed ahead, with no reader registered: each port is read on its first lane's own stream, started at the switch [PORT-608]. The checks, `hil/lane-common/loopback.rs`, image F (`lanes-f`, the lane frame port; image T, FerroHAL's lanes under RTIC with no port, runs the same checks for the switch's figures and verifies nothing here): 1000 exchanges whose reply starts 30 us after the frames end, each read whole by the linked lane, every other lane reading its idle level, and every exchange coming back; then the reply start swept earlier one step (1.67 us at 600 kHz) at a time, which reports the margin; 1000 send-only exchanges, each sent once, and a frame seen played exactly once [PORT-603]; a 500-sample frame that plays once and ends, and a 1024-sample receive on every lane that comes back whole; a cancel mid-frame and mid-receive, each coming back aborted with the line idle and the next exchange read whole [PORT-606]; every lent share given back, and no exchange lost by the port's queues. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits zero.

## Lane frame port on the STM32F4 Discovery, a paced reader

`hil/run-discovery-lane-port.sh paced-reader f`, with the link PA1 (P1-11) to PB1 (P1-21) through 1 kOhm, as `hil/stm32f4-discovery-lane-port/src/layout_paced.rs` lists it: four lanes on TIM3 and TIM2 over port B, sending on DMA1, which cannot reach GPIO, and port B read by one reader paced by TIM8 on DMA2 stream 1; the responder is TIM5 CH2 on PA1, started with TIM3 [PORT-609]. The checks, `hil/lane-common/loopback.rs`, image F (`lanes-f`, the lane frame port; image T, FerroHAL's lanes under RTIC with no port, runs the same checks for the switch's figures and verifies nothing here): 1000 exchanges whose reply starts 30 us after the frames end, each read whole by the linked lane, every other lane reading its idle level, and every exchange coming back; then the reply start swept earlier one step (1.67 us at 600 kHz) at a time, which reports the margin; 1000 send-only exchanges, each sent once, and a frame seen played exactly once [PORT-603]; a 500-sample frame that plays once and ends, and a 1024-sample receive on every lane that comes back whole; a cancel mid-frame and mid-receive, each coming back aborted with the line idle and the next exchange read whole [PORT-606]; every lent share given back, and no exchange lost by the port's queues. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits zero.

## Lane frame port timing on the Nucleo-H753ZI

`hil/run-h753-lane-port.sh reader-ahead` and `hil/run-h753-lane-port.sh own-streams`, with the link PC8 to PE11 through 1 kOhm, as `hil/nucleo-h753zi-lane-port/src/lib.rs` lists it. This run verifies no requirement: the port is proven on the STM32F4 Discovery (D34). It measures the H753's figures the port's contract states, its longest runs and margins (`docs/design/lane-port.md`), with the same checks. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits zero.
## USB port on the STM32F4 Discovery

`hil/run-discovery-usb.sh`, with CN5 cabled to the host, nothing fitted.
FerroHAL's `hil/usb_cdc_host.py` (at the FerroHAL revision this crate pins, in
`FERROHAL_DIR`) runs beside the firmware and checks every byte it gets back.
The firmware, `hil/stm32f4-discovery-usb-device/src/main.rs`, selects
`usb_device` at 168 MHz with the vector at priority 3 and its CDC echo task at
priority 2; its shared stack holds the vector service, the device and one
`usbd-serial` class, and the task takes the stack's lock once per poll and
echo round. Pass: every check prints `PASS`, none prints `FAIL`, the host
prints no `FAIL`, and the script exits zero. The checks: the host configured
the device [PORT-501]; the host's echo session, 1 byte to 64 KiB, finished
with every byte echoed but the end marker [PORT-501, PORT-502]; and a
configured, quiet second woke the task not at all [PORT-501]. The log gives
the session's throughput and the longest the task held the lock, in CPU
cycles.

## UART port on the Nucleo-H753ZI

`hil/run-h753.sh`, with FerroHAL's H753 links Z2 (PC6 to PC7) and Z3 (PB6 to
PB7) fitted, as `hil/nucleo-h753zi-uart-loopback/src/main.rs` lists them. The
same firmware and checks as on the Nucleo-F401RE, on `usart_v4` and DMAMUX:
USART1 on DMA2 streams 5 and 4, USART6 on DMA1 streams 6 and 0, the data cache
off. Pass: every check prints `PASS`, none prints `FAIL`, and the script exits
zero [PORT-003].
