//! PROTOTYPE bench, variant P: `ferroforge_stm32::usb_polled`. The member polls the
//! firmware's CDC device from the vector; the firmware's task locks it to echo.
//! On the Nucleo-H753ZI's CN13; `hil/run-h753-usb.sh` runs FerroHAL's `usb_cdc_host.py`
//! alongside.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `OTG_FS` | the group's member | 3 |
//! | `SPI2` | RTIC, dispatching `usb_events` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |

#![no_std]
#![no_main]

#[allow(dead_code)]
#[path = "../board.rs"]
mod board;
#[path = "../../../usb-common/common.rs"]
mod common;

// The app itself, the same on every board.
include!("../../../usb-common/polled.rs");
