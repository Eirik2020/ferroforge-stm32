//! PROTOTYPE bench, variant D: `ferroforge_stm32::usb_device`. The member masks and reports; the
//! firmware's task owns the CDC device and polls it.
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
include!("../../../usb-common/deferred.rs");
