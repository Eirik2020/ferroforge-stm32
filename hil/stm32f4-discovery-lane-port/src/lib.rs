//! The lane port bench on the STM32F4 Discovery (MB997, STM32F407VGT6, die
//! 0x413): the board both images share, in one of three layouts named by
//! their shape. The checks are `../lane-common/loopback.rs`.
//!
//! - `reader-ahead` and `own-streams`: four lanes on TIM1 and TIM8 over
//!   ports A, B and C (`layout_three_ports.rs`), each port read by a reader
//!   armed ahead on a spare lane-timer request, or on its first lane's own
//!   stream.
//! - `paced-reader`: four lanes on TIM3 and TIM2 over port B, sending on
//!   DMA1, the port read by a reader TIM8 paces on DMA2 (`layout_paced.rs`).
//!
//! Only DMA2's peripheral port reaches the bus matrix and so GPIO (RM0090
//! figure 33 and its note), and on the F407 only TIM1 and TIM8 have requests
//! on DMA2 (RM0090 table 44); each layout's module cites its own routes.
//! A run here is HIL evidence for the port on die 0x413, whatever board's
//! pins and streams a layout borrows its shape from.
//!
//! ES0182 2.2.19 (DMA2 repeating AHB transfers while serving APB2) corrupts
//! DMA2 writes to GPIO only; "transfers from the GPIOs register are not
//! impacted". Every GPIO transfer here is a read.

#![no_std]

#[path = "../../lane-common/loopback.rs"]
mod loopback;
pub use loopback::*;

use ferrohal_stm32::{
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
        Sysclk,
    },
    time::Hertz,
};

/// 168 MHz, as FerroHAL's Discovery bench: the ST-LINK's 8 MHz MCO on PH0
/// in bypass (UM1472 section 7.11.1), / 8 * 336 = 336 MHz VCO, / 2 =
/// 168 MHz; APB2 at 84 MHz and APB1 at 42 MHz, so TIM1 and TIM8 count at
/// 168 MHz and TIM3 at 84 MHz (RM0090 figure 16).
pub const CLOCK_CONFIG: Config = Config {
    hse: Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Bypass,
    }),
    pll: Some(Pll {
        source: PllSource::Hse,
        prediv: 8,
        mul: 336,
        divp: PllP::Div2,
        divq: Some(7),
    }),
    sys: Sysclk::PllP,
    ahb_pre: AhbPrescaler::Div1,
    apb1_pre: ApbPrescaler::Div4,
    apb2_pre: ApbPrescaler::Div2,
    supply: Supply::V2_7To3_6,
};

/// Checked while compiling, as FerroHAL's benches check theirs.
pub const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
    Ok(clocks) => clocks,
    Err(_) => panic!("CLOCK_CONFIG breaks the F407's limits"),
};

/// Core cycles a microsecond.
pub const PER_US: u32 = 168;
/// The send rate: DShot600's bit rate, 280 counts of TIM1's 168 MHz.
pub const RATE: Hertz = Hertz::khz(600);

pub fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

#[cfg(not(any(
    feature = "reader-ahead",
    feature = "own-streams",
    feature = "paced-reader"
)))]
compile_error!("pick one layout: reader-ahead, own-streams or paced-reader");
// `reader-ahead` is the default: a layout chosen beside it wins, so a build
// with the defaults and one more layout is that layout.
#[cfg(all(feature = "own-streams", feature = "paced-reader"))]
compile_error!("pick one layout only: own-streams or paced-reader");

/// Lanes on two timers and three ports, read ahead or on their own streams.
#[cfg(all(
    any(feature = "reader-ahead", feature = "own-streams"),
    not(feature = "paced-reader")
))]
#[path = "layout_three_ports.rs"]
mod layout;
/// Lanes on TIM3 and TIM2, sending on DMA1, read by a reader TIM8 paces.
#[cfg(feature = "paced-reader")]
#[path = "layout_paced.rs"]
mod layout;
pub use layout::*;
