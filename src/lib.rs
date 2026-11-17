//! FerroForge task groups for STM32 hardware ports, built on FerroHAL.
//!
//! Each port is a `#[ferroforge::group]` with a contract: the hardware slots it
//! is built from, which member serves which slot's vector, the resources it
//! leaves to the firmware, and the constructor that builds them. A firmware,
//! hand-written or generated, selects a port with one `#[group(from = ..)]`
//! inside `ferroforge::app!`, and calls its constructor from `init`.
//!
//! The constructors are generic over FerroHAL's wiring types, so a firmware's
//! only port-specific code is its wiring alias and the constructor call. Every
//! group also builds for the host, so a host tool can check what it reads.

#![no_std]
#![forbid(unsafe_code)]

mod adc;
mod exti;
mod spi;
pub mod spi_queue;
mod uart;
pub mod uart_queue;
mod uart_rx;

pub use adc::adc_scan;
pub use exti::exti_line;
pub use spi::spi_port;
pub use spi_queue::{spi_multi_port, spi_single_port};
pub use uart::uart_port;
pub use uart_queue::uart_queue_port;
pub use uart_rx::uart_rx_port;

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests;
