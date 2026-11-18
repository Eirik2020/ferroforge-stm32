//! PROTOTYPE: the timer lane loopback on the Nucleo-H753ZI (MB1364): the
//! board both variants share. The checks are `../lane-common/loopback.rs`.
//!
//! The lanes are on TIM1, because FerroHAL's H753 glue has no route for
//! TIM3 (the lane benches' timer) to start another timer, and TIM1 starts
//! TIM3 (RM0433 table 341, ITR0). DMAMUX1 lets any stream serve any request
//! (RM0433 section 17.3).
//!
//! | Lane | Timer channel | Pin (AF) | DMA2 stream, DMAMUX1 request |
//! | --- | --- | --- | --- |
//! | 0 | TIM1 CH2 | PE11 (AF1), CN10-6, linked | 0, 12 |
//! | 1 | TIM1 CH3N | PE12 (AF1), CN10-26 | 1, 13 |
//!
//! The responder: TIM3 CH3 on PC8 (AF2, CN8-2), DMA1 stream 1, request 25,
//! started by TIM1's trigger output through TIM3's ITR0. The witness: TIM8's
//! update request has DMA1 stream 0 copy port E's input register into
//! memory, eight times a period, so it sees both lanes and the reply on
//! lane 0's pin. Pins and alternate functions are DS12117 table 10; the
//! connector positions UM2407 table 18; the requests RM0433 table 121.
//!
//! The link: PC8 to PE11 through 1 kOhm, fitted by the owner. Lane 1 is a
//! complementary output on purpose: by FerroHAL's code (`Pwm::set_output`)
//! a complementary lane stops driving its pin after its first receive, and
//! the witness shows whether it does. Every lane pin has its pull-up.

#![no_std]

#[path = "../../lane-common/loopback.rs"]
mod loopback;
pub use loopback::*;

use ferrohal_stm32::{
    chip::{TIM1, TIM3, TIM8},
    device::Peripherals,
    dma::Stream,
    gpio::{Alternate, Pin, Pull, Speed},
    rcc::{
        AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
    },
    time::Hertz,
    timer::{Awg, CompareLane, ComplementaryLaneWiring, LaneWiring, Pwm, Sampler},
};

/// 400 MHz from HSI, as FerroHAL's H753 bench runs: 64 MHz / 4 = 16 MHz
/// reference, * 50 = 800 MHz VCO, / 2 = 400 MHz core; AXI and AHB at 200 MHz,
/// every APB at 100 MHz, so the timers count at 200 MHz (DS12117 table 23,
/// VOS1).
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

/// Core cycles a microsecond.
pub const PER_US: u32 = 400;
/// The send rate: DShot600's bit rate, 333 counts of 200 MHz.
pub const RATE: Hertz = Hertz::khz(600);

pub type L0 = CompareLane<TIM1, 2, LaneWiring<TIM1, 2, Pin<'E', 11, Alternate<1>>, Stream<2, 0>>>;
pub type L1 = CompareLane<
    TIM1,
    3,
    ComplementaryLaneWiring<TIM1, 3, Pin<'E', 12, Alternate<1>>, Stream<2, 1>>,
>;
pub type Gen = Awg<TIM1, TIM1, (L0, L1), ()>;
pub const LANES: usize = 2;

/// Each lane's pin on port E, which both lanes and the witness read.
pub const PINS: [u8; 2] = [11, 12];
const WITNESSED: [Option<u8>; 2] = [Some(11), Some(12)];

pub fn board(variant: &'static str) -> Board {
    Board {
        name: "Nucleo-H753ZI",
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
    CompareLane<TIM3, 3, LaneWiring<TIM3, 3, Pin<'C', 8, Alternate<2>>, Stream<1, 1>>>;

/// The bench's responder: TIM3 CH3 on PC8, started with TIM1.
pub struct Reply {
    pwm: Pwm<TIM3>,
    lane: ResponderLane,
}

impl Responder for Reply {
    fn period(&self) -> u32 {
        self.pwm.period()
    }

    fn arm(&mut self, wave: &[u32]) -> bool {
        self.pwm.stop();
        if self.lane.arm(&self.pwm, wave).is_err() {
            return false;
        }
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
    sampler: Sampler<TIM8, Stream<1, 0>>,
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
    pub lanes: [&'static mut [u32]; 2],
    pub samples: [&'static mut [u16]; 2],
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
        lanes: [buf0, buf1],
        samples: [s0, s1],
        responder,
        witness,
    } = buffers;

    let (Ok(tim1), Ok(tim3)) = (
        Pwm::new(&mut rcc, p.TIM1, RATE),
        Pwm::new(&mut rcc, p.TIM3, RATE),
    ) else {
        defmt::panic!("TIM1 or TIM3 at the send rate")
    };
    tim1.lead();
    tim3.follow(&tim1);

    let mut pe11 = e.p11.into_alternate::<1>(&mut e.ctrl);
    pe11.set_speed(&mut e.ctrl, Speed::High);
    pe11.set_pull(&mut e.ctrl, Pull::Up);
    let mut pe12 = e.p12.into_alternate::<1>(&mut e.ctrl);
    pe12.set_speed(&mut e.ctrl, Speed::High);
    pe12.set_pull(&mut e.ctrl, Pull::Up);
    let (Ok(l0), Ok(l1)) = (
        LaneWiring::<_, 2, _, _>::new(pe11, dma2.s0).assemble_two_way::<'E'>(&tim1, buf0, s0),
        ComplementaryLaneWiring::<_, 3, _, _>::new(pe12, dma2.s1)
            .assemble_two_way::<'E'>(&tim1, buf1, s1),
    ) else {
        defmt::panic!("the two lanes")
    };
    let mut generator = Awg::single(tim1, (l0, l1));
    generator.set_active_low(true);
    generator.set_outputs(true);

    let mut pc8 = c.p8.into_alternate::<2>(&mut c.ctrl);
    pc8.set_speed(&mut c.ctrl, Speed::High);
    let Ok(lane) = LaneWiring::<_, 3, _, _>::new(pc8, dma1.s1).assemble(&tim3, responder) else {
        defmt::panic!("the responder lane")
    };
    tim3.set_active_low::<3>(true);
    tim3.set_output::<3>(true);

    let rate = Hertz(RATE.0 * WITNESS_PER_PERIOD);
    let Ok(sampler) = Sampler::new::<'E'>(&mut rcc, p.TIM8, dma1.s0, rate, witness) else {
        defmt::panic!("the TIM8 witness")
    };
    (generator, Reply { pwm: tim3, lane }, Probe { sampler })
}
