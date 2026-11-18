//! PROTOTYPE: the event port and ADC port benches on the STM32F4 Discovery
//! (MB997, STM32F407VGT6): the board every image shares. The checks shared
//! with the H753 are `../event-adc-common/common.rs`.
//!
//! Two jumpers on the P1 header, each between pins UM1472 Rev 9 table 7
//! lists as free I/O, side by side so a cap fits:
//!
//! | Link | From | To | Use |
//! | --- | --- | --- | --- |
//! | E1 | P1-20 (PC4) | P1-19 (PC5) | a GPIO output driving EXTI line 5 (E) |
//! | A1 | P1-14 (PA2) | P1-13 (PA3) | a GPIO output driving ADC1 channel 3 (A, P) |
//!
//! PC5's line 5 shares the `EXTI9_5` vector with lines 6 to 9 (RM0090 table
//! 62, position 23). PA3 is ADC123_IN3 (DS8626 table 7); the sequence is
//! PA3, then VREFINT, then the temperature sensor, on DMA2 stream 0 channel
//! 0 (RM0090 table 44). The tick (K) is TIM4 and the higher-priority load
//! TIM5, each on its own vector (table 62, positions 30 and 50); no image
//! here uses another timer. Both are on APB1, whose timers count at 84 MHz
//! under this clock tree (RM0090 section 6.2, figure 16).
//!
//! The clock is FerroHAL's Discovery bench's: the ST-LINK's 8 MHz MCO on PH0
//! in bypass (UM1472 section 7.11.1), 168 MHz core.

#![no_std]

#[path = "../../event-adc-common/common.rs"]
mod common;
pub use common::*;

use ferrohal_stm32::{
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
        Sysclk,
    },
    time::Hertz,
};

/// 168 MHz, the die's ceiling, as FerroHAL's Discovery bench: 8 MHz / 8 *
/// 336 = 336 MHz VCO, / 2 = 168 MHz; APB1 at 42 MHz and APB2 at 84 MHz, so
/// their timers count at 84 and 168 MHz (RM0090 figure 16).
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

pub const BOARD: &str = "STM32F4 Discovery";
pub const SYSCLK_HZ: u32 = 168_000_000;
/// Core cycles a microsecond, and a millisecond.
pub const PER_US: u32 = 168;
pub const MS: u32 = 168_000;
/// The APB1 timers' clock, which TIM4 and TIM5 count.
pub const TIMER_HZ: u32 = 84_000_000;

/// The tick's rate for this image.
pub const TICK_HZ: u32 = tick_hz(option_env!("TICK_HZ"));
pub const TICK: Hertz = Hertz(TICK_HZ);
/// Above 10 kHz an image measures the highest rate only, quiet, and the
/// higher-priority task runs slowly as the guard; at 10 kHz and below it
/// runs at 1.25 kHz as the load.
pub const SWEEP: bool = TICK_HZ > 10_000;
pub const HIGH: Hertz = if SWEEP { Hertz(100) } else { Hertz(1_250) };
/// The tick's period in core cycles.
pub const PERIOD: u32 = SYSCLK_HZ / TICK_HZ;

const _: () = assert!(
    ferrohal_stm32::timer::Tick::<ferrohal_stm32::chip::TIM4>::check(&CLOCKS, TICK).is_ok(),
    "TIM4 cannot tick at TICK_HZ exactly"
);
const _: () = assert!(
    ferrohal_stm32::timer::Tick::<ferrohal_stm32::chip::TIM5>::check(&CLOCKS, HIGH).is_ok(),
    "TIM5 cannot tick at the load's rate"
);

/// Channels in P's sequence: the driven pin, VREFINT, temperature, and
/// with `ch16` the pin thirteen more times.
#[cfg(not(feature = "ch16"))]
pub const CHANNELS: usize = 3;
#[cfg(feature = "ch16")]
pub const CHANNELS: usize = 16;
