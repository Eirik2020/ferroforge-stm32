//! PROTOTYPE: the timer lane loopback on the STM32F4 Discovery (MB997,
//! STM32F407VGT6): the board both variants share. The checks are
//! `../lane-common/loopback.rs`.
//!
//! The lanes are on TIM1, whose channel requests DMA2 serves (RM0090 table
//! 44), because only DMA2's peripheral port reaches the bus matrix and so
//! GPIO (RM0090 figure 33 and its note) and a two-way lane reads its port
//! through its stream.
//!
//! | Lane | Timer channel | Pin (AF) | Header | DMA2 stream, channel | Built with |
//! | --- | --- | --- | --- | --- | --- |
//! | 0 | TIM1 CH2 | PE11 (AF1), linked | P1-29 | 2, 6 | every lane count |
//! | 1 | TIM1 CH3N | PE12 (AF1) | P1-30 | 6, 6 | `lanes2`, `lanes4` |
//! | 2 | TIM1 CH1 | PE9 (AF1) | P1-27 | 3, 6 | `lanes4` |
//! | 3 | TIM1 CH4 | PE14 (AF1) | P1-32 | 4, 6 | `lanes4` |
//!
//! The responder: TIM3 CH3 on PC8 (AF2, P2-45), DMA1 stream 7 channel 5
//! (RM0090 table 43), started by TIM1's trigger output through TIM3's ITR0
//! (RM0090 table 98). The witness: TIM8's update request has DMA2 stream 1
//! channel 7 (table 44) copy port E's input register into memory, eight
//! times a period, so it sees every lane and the reply on lane 0's pin, as
//! FerroHAL's run D on this board reads PE8. Alternate functions are DS8626
//! table 9; header positions UM1472 Rev 9 table 7, which lists every pin
//! here as free I/O.
//!
//! The link: PC8 to PE11 through 1 kOhm, fitted by the owner. Lane 1 is a
//! complementary output on purpose: by FerroHAL's code (`Pwm::set_output`)
//! a complementary lane stops driving its pin after its first receive, and
//! the witness shows whether it does. Every lane pin has its pull-up.
//!
//! ES0182 2.2.19 (DMA2 repeating AHB transfers while serving APB2) corrupts
//! DMA2 writes to GPIO only; "transfers from the GPIOs register are not
//! impacted". Every GPIO transfer here is a read: the lanes' receives and
//! the witness.

#![no_std]

#[path = "../../lane-common/loopback.rs"]
mod loopback;
pub use loopback::*;

#[cfg(not(feature = "lanes1"))]
use ferrohal_stm32::timer::ComplementaryLaneWiring;
use ferrohal_stm32::{
    chip::{TIM1, TIM3, TIM8},
    device::Peripherals,
    dma::Stream,
    gpio::{Alternate, Pin, Pull, Speed},
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Hse, HseMode, Pll, PllP, PllSource, Supply,
        Sysclk,
    },
    time::Hertz,
    timer::{Awg, CompareLane, LaneWiring, Pwm, Sampler},
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

pub type L0 = CompareLane<TIM1, 2, LaneWiring<TIM1, 2, Pin<'E', 11, Alternate<1>>, Stream<2, 2>>>;
#[cfg(not(feature = "lanes1"))]
pub type L1 = CompareLane<
    TIM1,
    3,
    ComplementaryLaneWiring<TIM1, 3, Pin<'E', 12, Alternate<1>>, Stream<2, 6>>,
>;
#[cfg(feature = "lanes4")]
pub type L2 = CompareLane<TIM1, 1, LaneWiring<TIM1, 1, Pin<'E', 9, Alternate<1>>, Stream<2, 3>>>;
#[cfg(feature = "lanes4")]
pub type L3 = CompareLane<TIM1, 4, LaneWiring<TIM1, 4, Pin<'E', 14, Alternate<1>>, Stream<2, 4>>>;

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

/// Each lane's pin on port E, which every lane and the witness read.
pub const PINS: [u8; 4] = [11, 12, 9, 14];
const WITNESSED: [Option<u8>; 4] = [Some(11), Some(12), Some(9), Some(14)];

pub fn board(variant: &'static str) -> Board {
    Board {
        name: "STM32F4 Discovery",
        variant,
        per_us: PER_US,
        pins: &PINS,
        witnessed: &WITNESSED,
        complementary: 0b10,
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

/// The independent pin sampler on port E.
pub struct Probe {
    sampler: Sampler<TIM8, Stream<2, 1>>,
}

impl Witness for Probe {
    fn rate(&self) -> Hertz {
        self.sampler.rate()
    }

    fn start(&mut self, count: usize) -> bool {
        self.sampler.start(count).is_ok()
    }

    fn wait(&mut self) -> Option<&[u16]> {
        let start = cycles();
        loop {
            match self.sampler.on_interrupt() {
                Some(Ok(_)) => return Some(self.sampler.samples()),
                Some(Err(_)) => return None,
                None if cycles().wrapping_sub(start) > 20_000 * PER_US => {
                    self.sampler.abort();
                    return None;
                }
                None => {}
            }
        }
    }
}

/// The buffers `init` lends the board.
pub struct Buffers {
    pub lanes: [&'static mut [u32]; 4],
    pub samples: [&'static mut [u16]; 4],
    pub responder: &'static mut [u32],
    pub witness: &'static mut [u16],
}

/// Clocks, pins, the lanes' generator (active low, outputs on), the
/// responder and the witness. Panics on a refusal: a bench's setup either
/// holds or the run is void.
pub fn setup(p: Peripherals, buffers: Buffers) -> (Gen, Reply, Probe) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
    let mut e = p.GPIOE.split(&mut rcc);
    let mut c = p.GPIOC.split(&mut rcc);
    let dma1 = p.DMA1.split(&mut rcc);
    let dma2 = p.DMA2.split(&mut rcc);
    let Buffers {
        lanes: [buf0, buf1, buf2, buf3],
        samples: [s0, s1, s2, s3],
        responder,
        witness,
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

    let mut pe11 = e.p11.into_alternate::<1>(&mut e.ctrl);
    pe11.set_speed(&mut e.ctrl, Speed::High);
    pe11.set_pull(&mut e.ctrl, Pull::Up);
    let Ok(l0) =
        LaneWiring::<_, 2, _, _>::new(pe11, dma2.s2).assemble_two_way::<'E'>(&tim1, buf0, s0)
    else {
        defmt::panic!("lane 0")
    };
    #[cfg(not(feature = "lanes1"))]
    let l1 = {
        let mut pe12 = e.p12.into_alternate::<1>(&mut e.ctrl);
        pe12.set_speed(&mut e.ctrl, Speed::High);
        pe12.set_pull(&mut e.ctrl, Pull::Up);
        let Ok(l1) = ComplementaryLaneWiring::<_, 3, _, _>::new(pe12, dma2.s6)
            .assemble_two_way::<'E'>(&tim1, buf1, s1)
        else {
            defmt::panic!("lane 1")
        };
        l1
    };
    #[cfg(feature = "lanes4")]
    let (l2, l3) = {
        let mut pe9 = e.p9.into_alternate::<1>(&mut e.ctrl);
        pe9.set_speed(&mut e.ctrl, Speed::High);
        pe9.set_pull(&mut e.ctrl, Pull::Up);
        let mut pe14 = e.p14.into_alternate::<1>(&mut e.ctrl);
        pe14.set_speed(&mut e.ctrl, Speed::High);
        pe14.set_pull(&mut e.ctrl, Pull::Up);
        let (Ok(l2), Ok(l3)) = (
            LaneWiring::<_, 1, _, _>::new(pe9, dma2.s3).assemble_two_way::<'E'>(&tim1, buf2, s2),
            LaneWiring::<_, 4, _, _>::new(pe14, dma2.s4).assemble_two_way::<'E'>(&tim1, buf3, s3),
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

    let rate = Hertz(RATE.0 * WITNESS_PER_PERIOD);
    let Ok(sampler) = Sampler::new::<'E'>(&mut rcc, p.TIM8, dma2.s1, rate, witness) else {
        defmt::panic!("the TIM8 witness")
    };
    (generator, Reply { pwm: tim3, lane }, Probe { sampler })
}
