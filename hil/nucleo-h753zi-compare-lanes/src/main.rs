//! The compare lanes group's checks: a hand-written `ferroforge::app!` on the
//! Nucleo-H753ZI (MB1364) that selects `ferroforge_stm32::compare_lanes` once,
//! over two lanes of one timer, declares its member once per lane's stream
//! vector, and reads every lane's waveform back with a pin sampler.
//!
//! No links: a pin's input register reads its level while a timer drives it
//! (RM0433 section 11.3.12), so the sampler records each output's own
//! waveform. Nothing is wired to the pins.
//!
//! | Lane | Timer channel | Pin (AF) | Stream |
//! | --- | --- | --- | --- |
//! | 0 | TIM3 CH3 | PB0 (AF2) | DMA2 stream 0 |
//! | 1 | TIM3 CH4 | PB1 (AF2) | DMA2 stream 1 |
//!
//! Pins and alternate functions are DS12117 table 10; each stream takes its
//! request through DMAMUX1 (RM0433 section 17.3, table 121). TIM3 counts from
//! the APB1 timer clock, 200 MHz (RM0433 section 8.5.7), at 600 kHz: 333
//! counts a period. TIM8's update request has DMA1 stream 0 copy port B's
//! input register into memory, about 12 times a period; every DMA1 and DMA2
//! stream reaches GPIO on the H7 (RM0433 figure 1). The data cache stays off,
//! so the buffers need no non-cacheable region.
//!
//! The generator is built active high, so every line idles low and a pulse
//! drives it high.
//!
//! | Vector | Serves | Priority |
//! | --- | --- | --- |
//! | `DMA2_STREAM0`, `DMA2_STREAM1` | `lanes`' member, once each | 3 |
//! | `SPI2` | RTIC, dispatching `lanes_done` | 2 |
//! | `SPI3` | RTIC, dispatching `checks` | 1 |
//!
//! Each check prints `PASS` or `FAIL` with the requirements it verifies in
//! brackets; the run passes when every check does.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

ferroforge::app! {
    device = ferrohal_stm32::device,
    peripherals = true,
    dispatchers = [SPI2, SPI3],

    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug::{self, EXIT_FAILURE, EXIT_SUCCESS};
    use ferroforge_stm32::compare_lanes::{self, Error};
    use ferrohal_stm32::{
        chip::{TIM3, TIM8},
        dma::Stream,
        gpio::{Alternate, Pin, Speed},
        rcc::{
            AhbPrescaler, ApbPrescaler, Clocks, Config, Pll, PllSource, Supply, Sysclk, VoltageScale,
        },
        time::Hertz,
        timer::{Awg, CompareLane, LaneWiring, Pwm, Sampler},
    };

    /// 400 MHz from HSI, as FerroHAL's H753 bench runs: 64 MHz / 4 = 16 MHz
    /// reference, * 50 = 800 MHz VCO, / 2 = 400 MHz core; AXI and AHB at
    /// 200 MHz, every APB at 100 MHz (DS12117 table 23, VOS1).
    const CLOCK_CONFIG: Config = Config {
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
    const CLOCKS: Clocks = match CLOCK_CONFIG.validate() {
        Ok(clocks) => clocks,
        Err(_) => panic!("CLOCK_CONFIG breaks the H753's limits"),
    };

    /// CPU cycles in a millisecond, for the busy waits.
    const MS: u32 = 400_000;

    type Lane<const C: u8, const N: u8, const S: u8> =
        CompareLane<TIM3, C, LaneWiring<TIM3, C, Pin<'B', N, Alternate<2>>, Stream<2, S>>>;
    type Lanes = Awg<TIM3, TIM3, (Lane<3, 0, 0>, Lane<4, 1, 1>), ()>;
    type PortSampler = Sampler<TIM8, Stream<1, 0>>;

    /// Each lane's pin on port B, in lane order.
    const PINS: [u8; 2] = [0, 1];
    const LANES: usize = 2;
    /// The bit rate, and samples a bit.
    const RATE_KHZ: u32 = 600;
    const PER_BIT: u32 = 12;
    /// Bits a waveform, most significant first; a 1 is a pulse of 12/16 of
    /// the period, a 0 one of 6/16.
    const BITS: usize = 16;
    /// Waveforms played one at a time, and how often one is sampled.
    const WAVEFORMS: u32 = 400;
    const SAMPLE_EVERY: u32 = 10;

    /// What the firmware's task has seen.
    #[derive(Clone, Copy, Default)]
    pub struct Log {
        /// Waveforms whose outcome reached the task, and those that completed.
        pub ended: u32,
        pub completed: u32,
        /// Outcomes that found a lane still playing.
        pub early: u32,
    }

    #[shared]
    struct Shared {
        lanes: Lanes,
        log: Log,
    }

    #[local]
    struct Local {
        sampler: PortSampler,
    }

    #[init(local = [
        lane0: [u32; 20] = [0; 20],
        lane1: [u32; 20] = [0; 20],
        samples: [u16; 512] = [0; 512],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = cx.device;
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let mut rcc = match p.RCC.freeze(&CLOCK_CONFIG) {
            Ok(rcc) => rcc,
            Err(refused) => defmt::panic!("clocks: {}", refused.error()),
        };
        let mut b = p.GPIOB.split(&mut rcc);
        let dma1 = p.DMA1.split(&mut rcc);
        let dma2 = p.DMA2.split(&mut rcc);
        let mut pb0 = b.p0.into_alternate::<2>(&mut b.ctrl);
        pb0.set_speed(&mut b.ctrl, Speed::High);
        let mut pb1 = b.p1.into_alternate::<2>(&mut b.ctrl);
        pb1.set_speed(&mut b.ctrl, Speed::High);

        let Ok(tim3) = Pwm::new(&mut rcc, p.TIM3, Hertz::khz(RATE_KHZ)) else {
            defmt::panic!("TIM3 at the bit rate")
        };
        let (Ok(lane0), Ok(lane1)) = (
            LaneWiring::<_, 3, _, _>::new(pb0, dma2.s0).assemble(&tim3, cx.local.lane0),
            LaneWiring::<_, 4, _, _>::new(pb1, dma2.s1).assemble(&tim3, cx.local.lane1),
        ) else {
            defmt::panic!("the two lanes")
        };
        let generator = Awg::single(tim3, (lane0, lane1));
        let lanes = compare_lanes::new(generator, false);
        let rate = Hertz(RATE_KHZ * 1000 * PER_BIT);
        let Ok(sampler) = Sampler::new::<'B'>(&mut rcc, p.TIM8, dma1.s0, rate, cx.local.samples)
        else {
            defmt::panic!("the TIM8 sampler")
        };

        defmt::info!("compare_lanes on the Nucleo-H753ZI, clocks: {}", CLOCKS);
        checks::spawn().ok();
        (
            Shared { lanes: lanes.generator, log: Log::default() },
            Local { sampler },
        )
    }

    #[group(from = ferroforge_stm32::compare_lanes, shared = [generator = lanes],
            spawn = [done = lanes_done])]
    mod lanes {
        #[task(binds = DMA2_STREAM0, priority = 3)]
        fn on_lane_dma;
        #[task(binds = DMA2_STREAM1, priority = 3)]
        fn on_lane_dma;
    }

    /// The firmware's task: takes each waveform's outcome.
    #[task(priority = 2, shared = [lanes, log])]
    async fn lanes_done(cx: lanes_done::Context, result: Result<(), Error>) {
        let lanes_done::SharedResources { mut lanes, mut log, .. } = cx.shared;
        let busy = lanes.lock(|lanes| lanes.is_busy());
        log.lock(|log| {
            log.ended += 1;
            log.completed += u32::from(result.is_ok());
            log.early += u32::from(busy);
        });
    }

    /// Plays the waveforms and judges what the group made of them.
    #[task(priority = 1, shared = [lanes, log], local = [sampler])]
    async fn checks(cx: checks::Context) {
        let checks::SharedResources { mut lanes, mut log, .. } = cx.shared;
        let sampler = cx.local.sampler;
        let mut failed = 0u32;

        failed += verdict(
            "every lane's vector is enabled [PORT-501]",
            lanes.lock(|lanes| lanes.vectors().verify()).is_ok(),
        );

        // Before anything plays, every line sits at its inactive level: low,
        // active high. 256 samples, about 36 us.
        let idle = match sample(sampler, 256, || {}) {
            Some(samples) => samples.iter().all(|s| PINS.iter().all(|pin| s & (1 << pin) == 0)),
            None => false,
        };
        failed += verdict(
            "the constructor left every output on at its inactive level, low [PORT-502]",
            idle,
        );

        let period = lanes.lock(|lanes| lanes.period());
        let count = (PER_BIT as usize) * (BITS + 16);
        let (mut refused, mut sampled, mut wrong) = (0u32, 0u32, 0u32);
        for index in 0..WAVEFORMS {
            let patterns = patterns(index);
            let before = log.lock(|log| log.ended);
            let loaded = lanes.lock(|lanes| {
                (0..LANES).all(|lane| lanes.load(lane, &duties(patterns[lane], period)).is_ok())
            });
            if !loaded {
                refused += 1;
                lanes.lock(|lanes| lanes.abort());
                continue;
            }
            let mut played = true;
            if index % SAMPLE_EVERY == 0 {
                sampled += 1;
                let samples = sample(sampler, count, || {
                    played = lanes.lock(|lanes| lanes.play()).is_ok();
                });
                let right = samples.is_some_and(|samples| {
                    PINS.iter()
                        .zip(patterns)
                        .all(|(pin, pattern)| decode(samples, *pin) == Some(pattern))
                });
                wrong += u32::from(!right);
            } else {
                played = lanes.lock(|lanes| lanes.play()).is_ok();
            }
            if !played {
                refused += 1;
                lanes.lock(|lanes| lanes.abort());
                continue;
            }
            await_ended(&mut log, before + 1);
        }
        // Any outcome after the last would be a second one for a waveform.
        wait(MS);
        let seen = log.lock(|log| *log);
        defmt::info!(
            "{} waveforms: {} ended, {} completed, {} with a lane still playing, {} refused; {} sampled, {} read back wrong",
            WAVEFORMS, seen.ended, seen.completed, seen.early, refused, sampled, wrong
        );
        failed += verdict(
            "each waveform's outcome reached the firmware's task once, completed, with no lane playing [PORT-501]",
            refused == 0 && seen.ended == WAVEFORMS && seen.completed == WAVEFORMS && seen.early == 0,
        );
        failed += verdict(
            "every sampled waveform carried each lane's own 16 bits, active high, on its pin [PORT-501]",
            wrong == 0 && sampled > 0,
        );
        let idle = match sample(sampler, 256, || {}) {
            Some(samples) => samples.iter().all(|s| PINS.iter().all(|pin| s & (1 << pin) == 0)),
            None => false,
        };
        failed += verdict(
            "after the last waveform every line is back at its inactive level [PORT-502]",
            idle,
        );

        if failed == 0 {
            defmt::info!("all checks passed");
        } else {
            defmt::error!("{} checks failed", failed);
        }
        // The host drains RTT while the target runs; the semihosting exit ends
        // that, so give it the last lines first (about 200 ms).
        wait(200 * MS);
        if failed == 0 {
            debug::exit(EXIT_SUCCESS);
        } else {
            debug::exit(EXIT_FAILURE);
        }
    }

    /// Two different patterns for waveform `index`, one a lane, each with
    /// both bit values in it.
    fn patterns(index: u32) -> [u16; LANES] {
        let base = (index as u16).wrapping_mul(0x9e37);
        [0xa5c3 ^ base, 0x5b29 ^ base.rotate_left(8)]
    }

    /// A lane's samples for `pattern`, most significant bit first.
    fn duties(pattern: u16, period: u32) -> [u32; BITS] {
        core::array::from_fn(|i| {
            let one = pattern & (0x8000 >> i) != 0;
            period * if one { 12 } else { 6 } / 16
        })
    }

    /// Takes `count` samples of port B, calling `during` once the sampler
    /// runs; `None` if it did not finish within 10 ms.
    fn sample<'a>(sampler: &'a mut PortSampler, count: usize, during: impl FnOnce()) -> Option<&'a [u16]> {
        sampler.start(count).ok()?;
        during();
        let start = DWT::cycle_count();
        loop {
            match sampler.on_interrupt() {
                Some(Ok(_)) => return Some(sampler.samples()),
                Some(Err(_)) => return None,
                None if DWT::cycle_count().wrapping_sub(start) > 10 * MS => {
                    sampler.abort();
                    return None;
                }
                None => {}
            }
        }
    }

    /// The 16 bits on `pin`, active high: each pulse's start one period after
    /// the last (within two samples), a 1 longer than 9/16 of a period and a
    /// 0 shorter, and exactly 16 pulses. `None` for anything else.
    fn decode(samples: &[u16], pin: u8) -> Option<u16> {
        let active = |s: &u16| s & (1 << pin) != 0;
        let (mut bits, mut found) = (0u16, 0usize);
        let (mut start, mut last_start) = (None::<usize>, None::<usize>);
        for (i, s) in samples.iter().enumerate() {
            match (active(s), start) {
                (true, None) => {
                    if let Some(last) = last_start {
                        if (i - last).abs_diff(PER_BIT as usize) > 2 {
                            return None;
                        }
                    }
                    start = Some(i);
                    last_start = Some(i);
                }
                (false, Some(begun)) => {
                    let width = (i - begun) as u32;
                    if found == BITS {
                        return None;
                    }
                    bits = (bits << 1) | u16::from(width * 16 > PER_BIT * 9);
                    found += 1;
                    start = None;
                }
                _ => {}
            }
        }
        (found == BITS && start.is_none()).then_some(bits)
    }

    /// Spins until the task has taken `ended` outcomes, for at most 10 ms.
    fn await_ended(log: &mut impl rtic::Mutex<T = Log>, ended: u32) {
        for _ in 0..100 {
            if log.lock(|log| log.ended) >= ended {
                return;
            }
            wait(MS / 10);
        }
    }

    /// Spins until `cycles` core cycles have passed. Interrupts still run.
    fn wait(cycles: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < cycles {}
    }

    /// Prints the check's verdict; 1 for a failure, to count them.
    fn verdict(name: &str, pass: bool) -> u32 {
        if pass {
            defmt::info!("PASS {}", name);
            0
        } else {
            defmt::error!("FAIL {}", name);
            1
        }
    }
}
