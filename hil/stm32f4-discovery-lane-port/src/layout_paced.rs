//! The `paced-reader` layout: four lanes on two APB1 timers, whose requests
//! only DMA1 serves, and DMA1 cannot reach GPIO (RM0090 figure 33 and its
//! note). So no lane reads its own pin: one reader on DMA2 stream 1, paced by
//! TIM8's update (RM0090 table 44, channel 7), reads port B for every lane.
//!
//! | Lane | Timer channel | Pin (AF) | Header | DMA1 stream, channel |
//! | --- | --- | --- | --- | --- |
//! | 0 | TIM3 CH4 | PB1 (AF2), linked | P1-21 | 2, 5 |
//! | 1 | TIM3 CH3 | PB0 (AF2) | P1-22 | 7, 5 |
//! | 2 | TIM2 CH3 | PB10 (AF1) | P1-34 | 1, 3 |
//! | 3 | TIM2 CH4 | PB11 (AF1) | P1-35 | 6, 3 |
//!
//! Pins and functions: DS8626 table 9; headers: UM1472 table 7. PB10 is also
//! the MEMS microphone's clock input on this board (UM1472 table 7), which
//! the chip drives in normal use. Streams: RM0090 table 43. TIM2 starts on
//! TIM3's trigger output, ITR2 (RM0090 table 98), through `Awg::linked`.
//!
//! The responder: TIM5 CH2 on PA1 (AF2, P1-11), DMA1 stream 4 channel 6
//! (table 43), started by TIM3's trigger output through TIM5's ITR1 (table
//! 98), so it starts on the same edge as both lane timers. The link: PA1 to
//! PB1 through 1 kOhm. PB0 is also linked to PC9, the three-port layout's
//! link; PC9 is an unused input here.
//!
//! The pacer, TIM8, runs at the capture rate from `setup` on, its update
//! interrupt and request off, so the switch opens one DIER bit.
//!
//! Lane 1's stream 7 vector is left unbound: only the last lane (3) raises
//! its end, and TIM3's lanes end before TIM2's, which TIM3 starts. No pin
//! witness: TIM1 and TIM8 are the only timers with requests on DMA2, and TIM8
//! paces the reader.

use super::*;
use ferrohal_stm32::{
    chip::{TIM2, TIM3, TIM5, TIM8},
    device::Peripherals,
    dma::Stream,
    gpio::{Alternate, Pin, Pull, Speed},
    timer::{Awg, CompareLane, LaneWiring, Pwm},
};

pub type L0 = CompareLane<TIM3, 4, LaneWiring<TIM3, 4, Pin<'B', 1, Alternate<2>>, Stream<1, 2>>>;
pub type L1 = CompareLane<TIM3, 3, LaneWiring<TIM3, 3, Pin<'B', 0, Alternate<2>>, Stream<1, 7>>>;
pub type L2 = CompareLane<TIM2, 3, LaneWiring<TIM2, 3, Pin<'B', 10, Alternate<1>>, Stream<1, 1>>>;
pub type L3 = CompareLane<TIM2, 4, LaneWiring<TIM2, 4, Pin<'B', 11, Alternate<1>>, Stream<1, 6>>>;

pub type Gen = Awg<TIM3, TIM2, (L0, L1), (L2, L3)>;
pub const LANES: usize = 4;

/// Each lane's pin, all on port B.
pub const PINS: [u8; 4] = [1, 0, 10, 11];
const WITNESSED: [Option<u8>; 4] = [None; 4];

/// The capture rate: eight samples a send period, 35 counts of TIM8's
/// 168 MHz.
pub const CAPTURE: Hertz = Hertz::khz(4800);

pub fn board(variant: &'static str) -> Board {
    Board {
        name: "STM32F4 Discovery, paced-reader layout (TIM3 and TIM2, port B)",
        variant,
        per_us: PER_US,
        pins: &PINS,
        witnessed: &WITNESSED,
        complementary: 0,
        linked: 0,
    }
}

type ResponderLane =
    CompareLane<TIM5, 2, LaneWiring<TIM5, 2, Pin<'A', 1, Alternate<2>>, Stream<1, 4>>>;

/// The bench's responder: TIM5 CH2 on PA1, started with TIM3.
pub struct Reply {
    pwm: Pwm<TIM5>,
    lane: ResponderLane,
    memory: Option<&'static mut [u32]>,
    /// Kept running at the capture rate; the generator holds its request.
    _pacer: Pwm<TIM8>,
}

impl Responder for Reply {
    fn period(&self) -> u32 {
        self.pwm.period()
    }

    fn arm(&mut self, wave: &[u32]) -> bool {
        self.pwm.stop();
        if !crate::loopback::arm_wave(&mut self.lane, &self.pwm, &mut self.memory, wave) {
            return false;
        }
        // Stopped at the reload value with CH2's request open: TIM3's next
        // start starts it.
        self.pwm.stage(1 << 1);
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

/// No witness on this layout.
pub type Probe = NoWitness;

/// The buffers `init` lends the board.
pub struct Buffers {
    pub responder: &'static mut [u32],
    pub witness: &'static mut [u16],
}

/// Clocks, pins, the lanes' generator (active low, outputs on), the paced
/// reader and the responder. Panics on a refusal.
pub fn setup(p: Peripherals, buffers: Buffers) -> (Gen, Reply, Probe) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
    let mut a = p.GPIOA.split(&mut rcc);
    let mut b = p.GPIOB.split(&mut rcc);
    let dma1 = p.DMA1.split(&mut rcc);
    let dma2 = p.DMA2.split(&mut rcc);
    let Buffers {
        responder,
        witness: _,
    } = buffers;

    let (Ok(tim3), Ok(tim2), Ok(tim5), Ok(tim8)) = (
        Pwm::new(&mut rcc, p.TIM3, RATE),
        Pwm::new(&mut rcc, p.TIM2, RATE),
        Pwm::new(&mut rcc, p.TIM5, RATE),
        Pwm::new(&mut rcc, p.TIM8, CAPTURE),
    ) else {
        defmt::panic!("TIM3, TIM2, TIM5 or TIM8")
    };
    // TIM3's start starts TIM5 (ITR1), for the responder; `Awg::linked`
    // makes it start TIM2 as well.
    tim3.lead();
    tim5.follow(&tim3);

    let mut pb1 = b.p1.into_alternate::<2>(&mut b.ctrl);
    pb1.set_speed(&mut b.ctrl, Speed::High);
    pb1.set_pull(&mut b.ctrl, Pull::Up);
    let mut pb0 = b.p0.into_alternate::<2>(&mut b.ctrl);
    pb0.set_speed(&mut b.ctrl, Speed::High);
    pb0.set_pull(&mut b.ctrl, Pull::Up);
    let mut pb10 = b.p10.into_alternate::<1>(&mut b.ctrl);
    pb10.set_speed(&mut b.ctrl, Speed::High);
    pb10.set_pull(&mut b.ctrl, Pull::Up);
    let mut pb11 = b.p11.into_alternate::<1>(&mut b.ctrl);
    pb11.set_speed(&mut b.ctrl, Speed::High);
    pb11.set_pull(&mut b.ctrl, Pull::Up);

    let Ok(l0) = LaneWiring::<_, 4, _, _>::new(pb1, dma1.s2).assemble_paced::<'B'>(&tim3) else {
        defmt::panic!("lane 0")
    };
    let Ok(l1) = LaneWiring::<_, 3, _, _>::new(pb0, dma1.s7).assemble_paced::<'B'>(&tim3) else {
        defmt::panic!("lane 1")
    };
    let Ok(l2) = LaneWiring::<_, 3, _, _>::new(pb10, dma1.s1).assemble_paced::<'B'>(&tim2) else {
        defmt::panic!("lane 2")
    };
    let Ok(l3) = LaneWiring::<_, 4, _, _>::new(pb11, dma1.s6).assemble_paced::<'B'>(&tim2) else {
        defmt::panic!("lane 3")
    };
    let Ok(mut generator) = Awg::linked(tim3, tim2, (l0, l1), (l2, l3)) else {
        defmt::panic!("the two-timer generator")
    };
    generator.set_active_low(true);
    generator.set_outputs(true);
    if generator
        .read_ahead_paced::<_, _, 'B'>(&tim8, dma2.s1)
        .is_err()
    {
        defmt::panic!("the paced reader")
    }
    tim8.start();

    let mut pa1 = a.p1.into_alternate::<2>(&mut a.ctrl);
    pa1.set_speed(&mut a.ctrl, Speed::High);
    let Ok(lane) = LaneWiring::<_, 2, _, _>::new(pa1, dma1.s4).assemble(&tim5) else {
        defmt::panic!("the responder lane")
    };
    tim5.set_active_low::<2>(true);
    tim5.set_output::<2>(true);

    (
        generator,
        Reply {
            pwm: tim5,
            lane,
            memory: Some(responder),
            _pacer: tim8,
        },
        NoWitness,
    )
}
