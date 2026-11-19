//! PROTOTYPE bench, variant L2: `ferroforge_stm32::usb_stacked`. As L, with the vector
//! service inside the firmware's shared stack beside the device and class,
//! and the bus built without the FIFO critical section (`no-fifo-cs`).
//! On the Nucleo-H753ZI's CN13; `hil/run-h753-usb.sh` runs FerroHAL's `usb_cdc_host.py`
//! alongside.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `OTG_FS` | the group's member | 3 |
//! | `SPI2` | RTIC, dispatching `usb_task` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |

#![no_std]
#![no_main]

#[allow(dead_code)]
#[path = "../board.rs"]
mod board;
#[path = "../../../usb-common/common.rs"]
mod common;

// The app itself, the same on every board.
include!("../../../usb-common/stacked.rs");
