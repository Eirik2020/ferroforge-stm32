//! The STM32F4 Discovery's (MB997, STM32F407VGT6) part of the USB prototype
//! bench: clocks, the CN5 wiring, and the checks both variants run.
//!
//! CN5 is the F407's own micro-AB connector, on OTG_FS's PA11 (DM) and PA12
//! (DP), alternate function 10 (UM1472 section 6.10; DS8626 table 9).
//! PD4 is the CS43L22's reset, held low so the audio DAC stays powered down
//! (UM1472 section 6.13), as FerroHAL's Discovery bench does.

use ferrohal_stm32::{
    chip::Peripherals,
    gpio::Speed,
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Rcc,
        Supply, Sysclk,
    },
    time::Hertz,
    usb::Wiring,
};

#[path = "../../usb-common/checks.rs"]
mod checks;
pub use checks::{Stats, run_checks};

pub const NAME: &str = "STM32F4 Discovery CN5";
pub const SERIAL_DEFERRED: &str = "f407-usb-deferred";
pub const SERIAL_POLLED: &str = "f407-usb-polled";
pub const SERIAL_SIGNALLED: &str = "f407-usb-signalled";
pub const SERIAL_LOCKED: &str = "f407-usb-locked";

/// 168 MHz from the ST-LINK's 8 MHz MCO in bypass (UM1472 section 6.11.1):
/// 8 MHz / 8 * 336 = 336 MHz VCO, / 2 = 168 MHz core, / 7 = 48 MHz for
/// OTG_FS (RM0090 section 6.3.2).
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

pub const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
    Ok(clocks) => clocks,
    Err(_) => panic!("CLOCK_CONFIG breaks the F407's limits"),
};

pub const SYSCLK_HZ: u32 = 168_000_000;

/// The clocks, and CN5's wiring with both data pins at very high speed.
pub fn usb_wiring(p: Peripherals) -> (Rcc, Wiring) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
    let mut d = p.GPIOD.split(&mut rcc);
    let mut codec_reset = d.p4.into_push_pull_output(&mut d.ctrl);
    codec_reset.set_low();
    let mut a = p.GPIOA.split(&mut rcc);
    let mut dm = a.p11.into_alternate::<10>(&mut a.ctrl);
    dm.set_speed(&mut a.ctrl, Speed::VeryHigh);
    let mut dp = a.p12.into_alternate::<10>(&mut a.ctrl);
    dp.set_speed(&mut a.ctrl, Speed::VeryHigh);
    let wiring = Wiring {
        usb: p.USB_OTG_FS,
        dm,
        dp,
    };
    (rcc, wiring)
}
