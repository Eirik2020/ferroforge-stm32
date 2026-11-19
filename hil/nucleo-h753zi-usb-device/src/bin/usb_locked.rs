//! PROTOTYPE bench, variant L: `ferroforge_stm32::usb_locked`. As S, with the task's
//! poll and echo under a lock it shares with the member, and the bus built
//! without the FIFO critical section (feature `no-fifo-cs`).
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
include!("../../../usb-common/locked.rs");
