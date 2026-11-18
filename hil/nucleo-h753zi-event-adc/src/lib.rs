//! PROTOTYPE: the event port and ADC port benches on the Nucleo-H753ZI
//! (MB1364): the board every image shares. The checks shared with the
//! Discovery are `../event-adc-common/common.rs`.
//!
//! The links are FerroHAL's H753 bench's own, on the Zio headers (UM2407
//! tables 18 to 21), as the existing `exti_line` and `adc_scan` benches use
//! them:
//!
//! | Link | From | To | Use |
//! | --- | --- | --- | --- |
//! | Z1 | CN9-6 (PD5) | CN9-4 (PD6) | a GPIO output driving EXTI line 6 (E) |
//! | Z10 | CN9-26 (PF7) | CN10-11 (PF6) | a GPIO output driving ADC3 channel 8 (A, P) |
//!
//! PD6's line 6 shares the `EXTI9_5` vector (RM0433 table 147). PF6 is
//! ADC3's channel 8 (DS12117 table 9); ADC3 carries the internal pair, so
//! the sequence is PF6, VREFINT, then the temperature sensor (RM0433
//! section 25.4.11), on DMA2 stream 0 through DMAMUX1 (RM0433 section 17.3).
//! The tick (K) is TIM4 and the higher-priority load TIM5, each on its own
//! vector (RM0433 table 147); no image here uses another timer. Both count
//! at 200 MHz under this clock tree (APB1 at 100 MHz, doubled for its
//! timers; DS12117 table 23). Everything runs from AXI SRAM, which DMA2
//! reaches (`memory.x`).

#![no_std]

#[path = "../../event-adc-common/common.rs"]
mod common;
pub use common::*;

use ferrohal_stm32::{
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
    },
    time::Hertz,
};

/// 400 MHz from HSI, as FerroHAL's H753 bench runs: 64 MHz / 4 = 16 MHz
/// reference, * 50 = 800 MHz VCO, / 2 = 400 MHz core; AXI and AHB at
/// 200 MHz, every APB at 100 MHz (DS12117 table 23, VOS1).
pub const CLOCK_CONFIG: Config = Config {
    hse: None,
    pll1: Some(Pll {
        source: PllSource::Hsi,
        prediv: 4,
        mul: 50,
        divp: Some(2),
        divq: Some(8),
        divr: None,
    }),
    sys: Sysclk::Pll1P,
    cpu_pre: AhbPrescaler::Div1,
    ahb_pre: AhbPrescaler::Div2,
    apb1_pre: ApbPrescaler::Div2,
    apb2_pre: ApbPrescaler::Div2,
    apb3_pre: ApbPrescaler::Div2,
    apb4_pre: ApbPrescaler::Div2,
    supply: Supply::Ldo,
    voltage_scale: VoltageScale::Scale1,
};

/// Checked while compiling, as FerroHAL's benches check theirs.
pub const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
    Ok(clocks) => clocks,
    Err(_) => panic!("CLOCK_CONFIG breaks the H753's limits"),
};

pub const BOARD: &str = "Nucleo-H753ZI";
pub const SYSCLK_HZ: u32 = 400_000_000;
/// Core cycles a microsecond, and a millisecond.
pub const PER_US: u32 = 400;
pub const MS: u32 = 400_000;
/// The APB1 timers' clock, which TIM4 and TIM5 count.
pub const TIMER_HZ: u32 = 200_000_000;

/// VDDA as the internal-pair check accepts it, in millivolts: the 3.3 V
/// supply less and more about 10 %, the bench's own tolerance.
pub const VDDA_MV: core::ops::RangeInclusive<u32> = 3_000..=3_600;

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
