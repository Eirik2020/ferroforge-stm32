# ferroforge-stm32

FerroForge task groups for STM32 hardware ports, built on
[FerroHAL](https://github.com/Eirik2020/ferrohal-stm32).

Each port is a `#[ferroforge::group]` that states its contract: the hardware
slots it is built from, which member serves which slot's vector, the resources
it leaves to the firmware, and its constructor. A firmware selects a port with
one `#[group(from = ..)]` inside `ferroforge::app!` and calls the constructor
from `init`; a tool can read the contract from source and write that firmware.

| Group | What it is | Proven on |
| --- | --- | --- |
| `uart_port` | One FerroHAL `Uart`, served from its three vectors | Nucleo-F401RE, Nucleo-H753ZI |
| `uart_rx_port` | A receive-only `Uart`, served from its two vectors, each wake stamped | Nucleo-F401RE |
| `spi_port` | One FerroHAL `Spi` master on DMA and its chip select, served from one vector | Nucleo-F401RE, Nucleo-H753ZI |
| `exti_line` | One EXTI line, each edge stamped in its vector | Nucleo-F401RE, Nucleo-H753ZI |
| `adc_scan` | One FerroHAL `AdcScan`, a sequence of channels over DMA, served from one vector | - |

The golden reference is `hil/nucleo-f401re-uart-loopback`, a hand-written
`ferroforge::app!`. Agents start at [AGENTS.md](AGENTS.md); the design is in
[docs/design/port-contract.md](docs/design/port-contract.md).

Licensed under either of Apache-2.0 or MIT, at your option.
