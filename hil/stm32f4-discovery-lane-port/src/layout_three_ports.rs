//! The `reader-ahead` and `own-streams` layouts: four lanes on two timers and
//! three ports, with one complementary output.
//!
//! | Lane | Timer channel | Pin (AF) | Header | DMA2 stream, channel |
//! | --- | --- | --- | --- | --- |
//! | 0 | TIM1 CH1 | PA8 (AF1) | P2-43 | 3, 6 |
//! | 1 | TIM1 CH3N | PB15 (AF1) | P1-39 | 6, 6 |
//! | 2 | TIM8 CH3 | PC8 (AF3) | P2-45 | 2, 0 |
//! | 3 | TIM8 CH4 | PC9 (AF3), linked | P2-46 | 7, 7 |
//!
//! Pins and functions: DS8626 table 9; headers: UM1472 table 7, every pin
//! free I/O. Streams: RM0090 table 44. TIM1_CH1 takes stream 3, not 1,
//! because stream 1 is TIM8_UP's only one. TIM8 starts on TIM1's trigger
//! output, ITR0 (RM0090 table 94), through `Awg::linked`.
//!
//! The responder: TIM3 CH3 on PB0 (AF2, P1-22), DMA1 stream 7 channel 5,
//! started by TIM1's trigger output through TIM3's ITR0 (RM0090 table 98).
//! The link: PB0 to PC9 through 1 kOhm.
//!
//! No witness: TIM1 and TIM8 are both lane timers, and table 44 gives timer
//! requests on DMA2, the only controller that reaches GPIO, to those two
//! alone.

use super::*;
use ferrohal_stm32::{
    chip::{TIM1, TIM3, TIM8},
    device::Peripherals,
    dma::Stream,
    gpio::{Alternate, Pin, Pull, Speed},
    timer::{Awg, CompareLane, ComplementaryLaneWiring, LaneWiring, Pwm},
};

pub type L0 = CompareLane<TIM1, 1, LaneWiring<TIM1, 1, Pin<'A', 8, Alternate<1>>, Stream<2, 3>>>;
pub type L1 = CompareLane<
    TIM1,
    3,
    ComplementaryLaneWiring<TIM1, 3, Pin<'B', 15, Alternate<1>>, Stream<2, 6>>,
>;
pub type L2 = CompareLane<TIM8, 3, LaneWiring<TIM8, 3, Pin<'C', 8, Alternate<3>>, Stream<2, 2>>>;
pub type L3 = CompareLane<TIM8, 4, LaneWiring<TIM8, 4, Pin<'C', 9, Alternate<3>>, Stream<2, 7>>>;

pub type Gen = Awg<TIM1, TIM8, (L0, L1), (L2, L3)>;
pub const LANES: usize = 4;

/// Each lane's pin, on its own port (A, B, C, C).
pub const PINS: [u8; 4] = [8, 15, 8, 9];
const WITNESSED: [Option<u8>; 4] = [None; 4];

pub fn board(variant: &'static str) -> Board {
    Board {
        name: "STM32F4 Discovery, three-port layout (TIM1 and TIM8)",
        variant,
        per_us: PER_US,
        pins: &PINS,
        witnessed: &WITNESSED,
        complementary: 0b10,
        linked: 3,
    }
}

type ResponderLane =
    CompareLane<TIM3, 3, LaneWiring<TIM3, 3, Pin<'B', 0, Alternate<2>>, Stream<1, 7>>>;

/// The bench's responder: TIM3 CH3 on PB0, started with TIM1.
pub struct Reply {
    pwm: Pwm<TIM3>,
    lane: ResponderLane,
    memory: Option<&'static mut [u32]>,
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

/// No witness on this layout; see the module doc.
pub type Probe = NoWitness;

/// The buffers `init` lends the board.
pub struct Buffers {
    pub responder: &'static mut [u32],
    pub witness: &'static mut [u16],
}

/// Clocks, pins, the lanes' generator (active low, outputs on) and the
/// responder. Panics on a refusal: a bench's setup either holds or the run
/// is void.
pub fn setup(p: Peripherals, buffers: Buffers) -> (Gen, Reply, Probe) {
    let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
        Ok(rcc) => rcc,
        Err(refused) => defmt::panic!("clocks: {}", refused.error()),
    };
    let mut a = p.GPIOA.split(&mut rcc);
    let mut b = p.GPIOB.split(&mut rcc);
    let mut c = p.GPIOC.split(&mut rcc);
    let dma1 = p.DMA1.split(&mut rcc);
    let dma2 = p.DMA2.split(&mut rcc);
    let Buffers {
        responder,
        witness: _,
    } = buffers;

    let (Ok(tim1), Ok(tim8), Ok(tim3)) = (
        Pwm::new(&mut rcc, p.TIM1, RATE),
        Pwm::new(&mut rcc, p.TIM8, RATE),
        Pwm::new(&mut rcc, p.TIM3, RATE),
    ) else {
        defmt::panic!("TIM1, TIM8 or TIM3 at the send rate")
    };
    // TIM1's start starts TIM3 (ITR0), for the responder; `Awg::linked`
    // makes it start TIM8 as well.
    tim1.lead();
    tim3.follow(&tim1);

    let mut pa8 = a.p8.into_alternate::<1>(&mut a.ctrl);
    pa8.set_speed(&mut a.ctrl, Speed::High);
    pa8.set_pull(&mut a.ctrl, Pull::Up);
    let mut pb15 = b.p15.into_alternate::<1>(&mut b.ctrl);
    pb15.set_speed(&mut b.ctrl, Speed::High);
    pb15.set_pull(&mut b.ctrl, Pull::Up);
    let mut pc8 = c.p8.into_alternate::<3>(&mut c.ctrl);
    pc8.set_speed(&mut c.ctrl, Speed::High);
    pc8.set_pull(&mut c.ctrl, Pull::Up);
    let mut pc9 = c.p9.into_alternate::<3>(&mut c.ctrl);
    pc9.set_speed(&mut c.ctrl, Speed::High);
    pc9.set_pull(&mut c.ctrl, Pull::Up);

    let Ok(l0) = LaneWiring::<_, 1, _, _>::new(pa8, dma2.s3).assemble_two_way::<'A'>(&tim1) else {
        defmt::panic!("lane 0")
    };
    let Ok(l1) =
        ComplementaryLaneWiring::<_, 3, _, _>::new(pb15, dma2.s6).assemble_two_way::<'B'>(&tim1)
    else {
        defmt::panic!("lane 1")
    };
    let Ok(l2) = LaneWiring::<_, 3, _, _>::new(pc8, dma2.s2).assemble_two_way::<'C'>(&tim8) else {
        defmt::panic!("lane 2")
    };
    let Ok(l3) = LaneWiring::<_, 4, _, _>::new(pc9, dma2.s7).assemble_two_way::<'C'>(&tim8) else {
        defmt::panic!("lane 3")
    };
    let Ok(mut generator) = Awg::linked(tim1, tim8, (l0, l1), (l2, l3)) else {
        defmt::panic!("the two-timer generator")
    };
    generator.set_active_low(true);
    generator.set_outputs(true);
    // `reader-ahead`: a reader per timer and port on the spare DMA2 streams, armed
    // while the frame plays (RM0090 table 44): TIM1_UP on stream 5 for port
    // A, TIM1_CH4 on stream 4 for port B (no lane uses CH4), TIM8_UP on
    // stream 1 for port C.
    // Mixed: ports A and B ahead, port C (the linked lane 3) on its lane's
    // stream.
    #[cfg(all(feature = "reader-ahead", not(feature = "own-streams")))]
    {
        let (Ok(()), Ok(()), Ok(())) = (
            generator.read_ahead_first_update::<_, 'A'>(dma2.s5),
            generator.read_ahead_first_compare::<4, _, 'B'>(dma2.s4),
            generator.read_ahead_second_update::<_, 'C'>(dma2.s1),
        ) else {
            defmt::panic!("the readers armed ahead")
        };
    }

    let mut pb0 = b.p0.into_alternate::<2>(&mut b.ctrl);
    pb0.set_speed(&mut b.ctrl, Speed::High);
    let Ok(lane) = LaneWiring::<_, 3, _, _>::new(pb0, dma1.s7).assemble(&tim3) else {
        defmt::panic!("the responder lane")
    };
    tim3.set_active_low::<3>(true);
    tim3.set_output::<3>(true);

    (
        generator,
        Reply {
            pwm: tim3,
            lane,
            memory: Some(responder),
        },
        NoWitness,
    )
}
