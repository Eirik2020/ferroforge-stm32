//! The SPI port's checks for this bench, from `hil/common/spi_port.rs`.

#![no_std]

#[path = "../../common/spi_port.rs"]
mod checks;

pub use checks::*;
