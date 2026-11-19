//! The Nucleo-H753ZI's part of the USB prototype bench: clocks, the CN13
//! wiring, and the checks both variants run.
//!
//! CN13 is the user USB connector, on OTG_FS's PA11 (DM) and PA12 (DP),
//! alternate function 10 (UM2407 section 6.11; DS12117 table 10). Its 48 MHz
//! kernel clock is HSI48, which FerroHAL's `UsbFs::new` turns on.

use ferrohal_stm32::{
    chip::Peripherals,
    gpio::Speed,
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Rcc, Supply, Sysclk,
        VoltageScale,
    },
    usb::Wiring,
};

#[path = "../../usb-common/checks.rs"]
mod checks;
pub use checks::{Stats, run_checks};

pub const NAME: &str = "Nucleo-H753ZI CN13";
pub const SERIAL_DEFERRED: &str = "h753-usb-deferred";
pub const SERIAL_POLLED: &str = "h753-usb-polled";
pub const SERIAL_SIGNALLED: &str = "h753-usb-signalled";
pub const SERIAL_LOCKED: &str = "h753-usb-locked";

/// 400 MHz from HSI, as the crate's other H753 benches run: 64 MHz / 4 =
/// 16 MHz reference, * 50 = 800 MHz VCO, / 2 = 400 MHz core; AXI and AHB at
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

pub const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
    Ok(clocks) => clocks,
    Err(_) => panic!("CLOCK_CONFIG breaks the H753's limits"),
};

pub const SYSCLK_HZ: u32 = 400_000_000;

/// The clocks, and CN13's wiring with both data pins at very high speed.
pub fn usb_wiring(p: Peripherals) -> (Rcc, Wiring) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
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
