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
pub mod lanes_frame;
mod spi;
mod uart;
mod uart_rx;
#[cfg(feature = "usb")]
mod usb;

pub use adc::adc_scan;
pub use exti::exti_line;
pub use lanes_frame::lane_frame_port;
pub use spi::spi_port;
pub use uart::uart_port;
pub use uart_rx::uart_rx_port;
#[cfg(feature = "usb")]
pub use usb::usb_device;

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests;
