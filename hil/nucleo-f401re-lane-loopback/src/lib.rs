//! PROTOTYPE: the timer lane loopback on the Nucleo-F401RE (MB1136): the
//! board both variants share. The checks are `../lane-common/loopback.rs`.
//!
//! The lanes are on TIM1, the only timer whose requests DMA2 serves (RM0368
//! table 29), because only DMA2's peripheral port reaches GPIO (RM0368
//! section 9.2) and a two-way lane reads its port through its stream. One
//! timer, so there is no second timer for an independent pin sampler: the
//! on-wire figures on this board are inferred.
//!
//! | Lane | Timer channel | Pin (AF) | DMA2 stream, channel | Built with |
//! | --- | --- | --- | --- | --- |
//! | 0 | TIM1 CH4 | PA11 (AF1), CN10-14, linked | 4, 6 | every lane count |
//! | 1 | TIM1 CH2 | PA9 (AF1), CN10-21 | 2, 6 | `lanes2`, `lanes4` |
//! | 2 | TIM1 CH1N | PB13 (AF1), CN10-30 | 1, 6 | `lanes4` |
//! | 3 | TIM1 CH3N | PB1 (AF1), CN10-24 | 6, 6 | `lanes4` |
//!
//! The responder: TIM3 CH3 on PC8 (AF2, CN10-2), DMA1 stream 7, channel 5
//! (RM0368 table 28), started by TIM1's trigger output through TIM3's ITR0
//! (RM0368 table 54). Pins and alternate functions are DS10086 table 9; the
//! connector positions UM1724 table 29.
//!
//! The link: PC8 to PA11 through 1 kOhm, fitted by the owner. PA11 is also
//! FerroHAL's link J2 to PA12, which stays an input here. PA9 is J1 to PA10,
//! which also stays an input. PA8 (TIM1 CH1) is the Pico's SCL and is not
//! touched.
//!
//! Lanes 2 and 3 are complementary outputs: TIM1's plain channel pins are
//! all on the rig's other links. By FerroHAL's code (`Pwm::set_output`) a
//! complementary lane stops driving its pin after its first receive; with
//! no pin sampler this board cannot show it, and the four-lane run measures
//! timing only.
//!
//! Every lane pin has its pull-up, so a released line idles high.
//! ES0299 2.2.11 (DMA2 corrupting data serving AHB and APB2 at once) cannot
//! strike: every lane sends (to TIM1, on APB2) or every lane reads (GPIO, on
//! AHB1), never both at once.

#![no_std]

#[path = "../../lane-common/loopback.rs"]
mod loopback;
pub use loopback::*;

#[cfg(feature = "lanes4")]
use ferrohal_stm32::timer::ComplementaryLaneWiring;
use ferrohal_stm32::{
    chip::{TIM1, TIM3},
    device::Peripherals,
    dma::Stream,
    gpio::{Alternate, Pin, Pull, Speed},
    rcc::{AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllP, PllSource, Supply, Sysclk},
    time::Hertz,
    timer::{Awg, CompareLane, LaneWiring, Pwm},
};

/// 84 MHz from HSI, the F401's ceiling (DS10086 section 3.13): 16 MHz / 16 *
/// 336 = 336 MHz VCO, / 4 = 84 MHz; APB1 at 42 MHz, its timers and APB2's at
/// 84 MHz (RM0368 figure 12).
pub const CLOCK_CONFIG: Config = Config {
    hse: None,
    pll: Some(Pll {
        source: PllSource::Hsi,
        prediv: 16,
        mul: 336,
        divp: PllP::Div4,
        divq: Some(7),
    }),
    sys: Sysclk::PllP,
    ahb_pre: AhbPrescaler::Div1,
    apb1_pre: ApbPrescaler::Div2,
    apb2_pre: ApbPrescaler::Div1,
    supply: Supply::V2_7To3_6,
};

/// Checked while compiling, as FerroHAL's benches check theirs.
pub const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
    Ok(clocks) => clocks,
    Err(_) => panic!("CLOCK_CONFIG breaks the F401's limits"),
};

/// Core cycles a microsecond.
pub const PER_US: u32 = 84;
/// The send rate: DShot600's bit rate, 140 counts of 84 MHz.
pub const RATE: Hertz = Hertz::khz(600);

pub type L0 = CompareLane<TIM1, 4, LaneWiring<TIM1, 4, Pin<'A', 11, Alternate<1>>, Stream<2, 4>>>;
pub type L1 = CompareLane<TIM1, 2, LaneWiring<TIM1, 2, Pin<'A', 9, Alternate<1>>, Stream<2, 2>>>;
#[cfg(feature = "lanes4")]
pub type L2 = CompareLane<
    TIM1,
    1,
    ComplementaryLaneWiring<TIM1, 1, Pin<'B', 13, Alternate<1>>, Stream<2, 1>>,
>;
#[cfg(feature = "lanes4")]
pub type L3 =
    CompareLane<TIM1, 3, ComplementaryLaneWiring<TIM1, 3, Pin<'B', 1, Alternate<1>>, Stream<2, 6>>>;

#[cfg(feature = "lanes1")]
pub type Gen = Awg<TIM1, TIM1, (L0,), ()>;
#[cfg(all(feature = "lanes2", not(feature = "lanes1")))]
pub type Gen = Awg<TIM1, TIM1, (L0, L1), ()>;
#[cfg(all(feature = "lanes4", not(feature = "lanes1"), not(feature = "lanes2")))]
pub type Gen = Awg<TIM1, TIM1, (L0, L1, L2, L3), ()>;

#[cfg(feature = "lanes1")]
pub const LANES: usize = 1;
#[cfg(all(feature = "lanes2", not(feature = "lanes1")))]
pub const LANES: usize = 2;
#[cfg(all(feature = "lanes4", not(feature = "lanes1"), not(feature = "lanes2")))]
pub const LANES: usize = 4;

/// Each lane's pin on the port its stream reads.
pub const PINS: [u8; 4] = [11, 9, 13, 1];

pub fn board(variant: &'static str) -> Board {
    Board {
        name: "Nucleo-F401RE",
        variant,
        per_us: PER_US,
        pins: &PINS,
        witnessed: &[],
        complementary: 0b1100,
    }
}

pub fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

type ResponderLane =
    CompareLane<TIM3, 3, LaneWiring<TIM3, 3, Pin<'C', 8, Alternate<2>>, Stream<1, 7>>>;

/// The bench's responder: TIM3 CH3 on PC8, started with TIM1.
pub struct Reply {
    pwm: Pwm<TIM3>,
    lane: ResponderLane,
}

impl Responder for Reply {
    fn arm(&mut self, wave: &[u32]) -> bool {
        self.pwm.stop();
        if self.lane.arm(&self.pwm, wave).is_err() {
            return false;
        }
        // Stopped at the reload value with CH3's request open: TIM1's next
        // start starts it.
        self.pwm.stage(1 << 2);
        true
    }

    fn finish(&mut self) -> bool {
        let start = cycles();
        loop {
            match self.lane.on_interrupt() {
                Some(Ok(())) => {
                    self.pwm.stop();
                    return true;
                }
                Some(Err(_)) => break,
                None if cycles().wrapping_sub(start) > 2_000 * PER_US => break,
                None => {}
            }
        }
        self.abort();
        false
    }

    fn abort(&mut self) {
        self.lane.abort(&self.pwm);
        self.pwm.stop();
    }
}

/// The buffers `init` lends the board: each lane's DMA buffer and samples,
/// and the responder's.
pub struct Buffers {
    pub lanes: [&'static mut [u32]; 4],
    pub samples: [&'static mut [u16]; 4],
    pub responder: &'static mut [u32],
}

/// Clocks, pins, the lanes' generator (active low, outputs on) and the
/// responder. Panics on a refusal: a bench's setup either holds or the run
/// is void.
pub fn setup(p: Peripherals, buffers: Buffers) -> (Gen, Reply) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
    let mut a = p.GPIOA.split(&mut rcc);
    #[cfg(feature = "lanes4")]
    let mut b = p.GPIOB.split(&mut rcc);
    let mut c = p.GPIOC.split(&mut rcc);
    let dma1 = p.DMA1.split(&mut rcc);
    let dma2 = p.DMA2.split(&mut rcc);
    let Buffers {
        lanes: [buf0, buf1, buf2, buf3],
        samples: [s0, s1, s2, s3],
        responder,
    } = buffers;
    let _ = (&buf1, &buf2, &buf3, &s1, &s2, &s3);

    let (Ok(tim1), Ok(tim3)) = (
        Pwm::new(&mut rcc, p.TIM1, RATE),
        Pwm::new(&mut rcc, p.TIM3, RATE),
    ) else {
        defmt::panic!("TIM1 or TIM3 at the send rate")
    };
    // TIM1's start starts TIM3 (ITR0), for the responder.
    tim1.lead();
    tim3.follow(&tim1);

    let mut pa11 = a.p11.into_alternate::<1>(&mut a.ctrl);
    pa11.set_speed(&mut a.ctrl, Speed::High);
    pa11.set_pull(&mut a.ctrl, Pull::Up);
    let Ok(l0) =
        LaneWiring::<_, 4, _, _>::new(pa11, dma2.s4).assemble_two_way::<'A'>(&tim1, buf0, s0)
    else {
        defmt::panic!("lane 0")
    };
    #[cfg(not(feature = "lanes1"))]
    let l1 = {
        let mut pa9 = a.p9.into_alternate::<1>(&mut a.ctrl);
        pa9.set_speed(&mut a.ctrl, Speed::High);
        pa9.set_pull(&mut a.ctrl, Pull::Up);
        let Ok(l1) =
            LaneWiring::<_, 2, _, _>::new(pa9, dma2.s2).assemble_two_way::<'A'>(&tim1, buf1, s1)
        else {
            defmt::panic!("lane 1")
        };
        l1
    };
    #[cfg(feature = "lanes4")]
    let (l2, l3) = {
        let mut pb13 = b.p13.into_alternate::<1>(&mut b.ctrl);
        pb13.set_speed(&mut b.ctrl, Speed::High);
        pb13.set_pull(&mut b.ctrl, Pull::Up);
        let mut pb1 = b.p1.into_alternate::<1>(&mut b.ctrl);
        pb1.set_speed(&mut b.ctrl, Speed::High);
        pb1.set_pull(&mut b.ctrl, Pull::Up);
        let (Ok(l2), Ok(l3)) = (
            ComplementaryLaneWiring::<_, 1, _, _>::new(pb13, dma2.s1)
                .assemble_two_way::<'B'>(&tim1, buf2, s2),
            ComplementaryLaneWiring::<_, 3, _, _>::new(pb1, dma2.s6)
                .assemble_two_way::<'B'>(&tim1, buf3, s3),
        ) else {
            defmt::panic!("lanes 2 and 3")
        };
        (l2, l3)
    };
    #[cfg(feature = "lanes1")]
    let mut generator = Awg::single(tim1, (l0,));
    #[cfg(all(feature = "lanes2", not(feature = "lanes1")))]
    let mut generator = Awg::single(tim1, (l0, l1));
    #[cfg(all(feature = "lanes4", not(feature = "lanes1"), not(feature = "lanes2")))]
    let mut generator = Awg::single(tim1, (l0, l1, l2, l3));
    generator.set_active_low(true);
    generator.set_outputs(true);

    let mut pc8 = c.p8.into_alternate::<2>(&mut c.ctrl);
    pc8.set_speed(&mut c.ctrl, Speed::High);
    let Ok(lane) = LaneWiring::<_, 3, _, _>::new(pc8, dma1.s7).assemble(&tim3, responder) else {
        defmt::panic!("the responder lane")
    };
    tim3.set_active_low::<3>(true);
    tim3.set_output::<3>(true);
    (generator, Reply { pwm: tim3, lane })
}
