//! An SBUS receiver: the receive-only UART port with a decoder behind it.
//!
//! SBUS is a receive-only UART line with a fixed frame: 100 000 baud, eight
//! data bits, even parity and two stop bits, sent inverted. A frame is 25
//! bytes - `0x0F`, sixteen 11-bit channels packed least significant bit first
//! into 22 bytes, a flags byte, then `0x00` - and frames are separated by a
//! pause, so the line going idle is a frame's end. That is the framing receivers
//! send and open flight stacks decode; Futaba publishes no specification.
//!
//! The group nests [`uart_rx_port`](crate::uart_rx_port): its two members
//! serve the UART, and their `events` call is wired inside this group to
//! [`decode`](sbus_rx::decode()), which reads every whole chunk and hands the latest
//! good frame of each wake to the firmware as [`Channels`], counting the rest. A firmware selecting it binds the
//! UART and where channels go, and nothing else.
//!
//! Whether the line can be un-inverted on chip is the UART IP's: `usart_v4`
//! (H7) can, `usart_v2` (F4) cannot and needs an external inverter in front of
//! the RX pin. [`config`] asks for what the board has, and FerroHAL refuses an
//! inversion the IP lacks.
//!
//! Requirements: PORT-001, PORT-002, PORT-121, PORT-122, PORT-123

use ferrohal_stm32::{
    time::Hertz,
    uart::{self, Chunk, Parity, StopBits},
};

/// The bytes in one frame.
pub const FRAME_LEN: usize = 25;
/// A frame's first byte.
pub const HEADER: u8 = 0x0F;
/// A frame's last byte.
pub const FOOTER: u8 = 0x00;

/// SBUS's line: 100 000 baud, 8E2. `inverted` is true when the RX pin sees
/// the receiver's own, inverted, levels and the UART must undo them; false
/// when an external inverter already has.
pub const fn config(inverted: bool) -> uart::Config {
    uart::Config::new(Hertz(100_000))
        .with_parity(Parity::Even)
        .with_stop_bits(StopBits::Two)
        .with_inverted_rx(inverted)
}

/// One decoded frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Frame {
    /// The sixteen proportional channels, in SBUS's own 11-bit units.
    pub values: [u16; 16],
    /// The two digital channels, 17 and 18 (flags bits 0 and 1).
    pub digital: [bool; 2],
    /// The receiver missed this frame from the transmitter (flags bit 2).
    pub lost: bool,
    /// The receiver has lost the transmitter and is sending its failsafe
    /// (flags bit 3).
    pub failsafe: bool,
}

/// Decodes one frame, or `None` if `bytes` is not one: the wrong length, or
/// the wrong first or last byte.
pub fn decode_frame(bytes: &[u8]) -> Option<Frame> {
    let bytes: &[u8; FRAME_LEN] = bytes.try_into().ok()?;
    if bytes[0] != HEADER || bytes[FRAME_LEN - 1] != FOOTER {
        return None;
    }
    let mut values = [0u16; 16];
    let mut bits = 0u32;
    let mut held = 0u32;
    let mut channel = 0usize;
    for byte in &bytes[1..23] {
        bits |= u32::from(*byte) << held;
        held += 8;
        while held >= 11 && channel < 16 {
            values[channel] = (bits & 0x07FF) as u16;
            bits >>= 11;
            held -= 11;
            channel += 1;
        }
    }
    let flags = bytes[23];
    Some(Frame {
        values,
        digital: [flags & 0x01 != 0, flags & 0x02 != 0],
        lost: flags & 0x04 != 0,
        failsafe: flags & 0x08 != 0,
    })
}

/// Encodes one frame: the inverse of [`decode_frame`], for a bench's or a
/// test's stand-in receiver. Channel values are masked to 11 bits.
pub fn encode_frame(frame: &Frame) -> [u8; FRAME_LEN] {
    let mut bytes = [0u8; FRAME_LEN];
    bytes[0] = HEADER;
    let mut bits = 0u32;
    let mut held = 0u32;
    let mut index = 1;
    for value in &frame.values {
        bits |= u32::from(*value & 0x07FF) << held;
        held += 11;
        while held >= 8 {
            bytes[index] = bits as u8;
            index += 1;
            bits >>= 8;
            held -= 8;
        }
    }
    bytes[23] = u8::from(frame.digital[0])
        | u8::from(frame.digital[1]) << 1
        | u8::from(frame.lost) << 2
        | u8::from(frame.failsafe) << 3;
    bytes[FRAME_LEN - 1] = FOOTER;
    bytes
}

/// What the decoder hands the firmware: the latest good frame, and running
/// counts of what it has read.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Channels {
    pub frame: Frame,
    /// Frames decoded since the port was opened.
    pub good: u32,
    /// Chunks that were not a frame: the wrong length or framing, or struck
    /// by a line error, a ring overrun or a DMA error.
    pub bad: u32,
}

/// What [`decode`](sbus_rx::decode()) reads chunks from: a FerroHAL [`Uart`].
/// A trait so the member is bounded on what it does, as `uart_port`'s
/// members are, rather than on one wiring.
///
/// [`Uart`]: ferrohal_stm32::uart::Uart
pub trait ChunkSource {
    /// The next whole chunk, as [`Uart::read_chunk`] gives it.
    ///
    /// [`Uart::read_chunk`]: ferrohal_stm32::uart::Uart::read_chunk
    fn next_chunk(&mut self, buf: &mut [u8]) -> Result<Option<Chunk>, uart::Error>;
}

impl<W: uart::Parts> ChunkSource for uart::Uart<W> {
    fn next_chunk(&mut self, buf: &mut [u8]) -> Result<Option<Chunk>, uart::Error> {
        self.read_chunk(buf)
    }
}

/// The most chunks one wake reads: a 25-byte frame per chunk, so more than a
/// 256-byte ring holds. What is left waits for the next wake.
const CHUNKS_PER_WAKE: usize = 16;

/// Sorts one chunk: a good frame, or one more bad chunk.
fn classify(buf: &[u8], chunk: &Chunk) -> Option<Frame> {
    match chunk.end {
        uart::ChunkEnd::Idle => decode_frame(&buf[..chunk.len]),
        uart::ChunkEnd::Full | uart::ChunkEnd::Gap(_) => None,
    }
}

/// The SBUS group, for a firmware to select once per receiver:
///
/// ```rust,ignore
/// #[group(from = ferroforge_stm32::sbus_rx, shared = [uart = rc_uart],
///         spawn = [channels = rc_input])]
/// mod rc {
///     #[task(binds = USART1, priority = 3)] fn on_uart;
///     #[task(binds = DMA2_STREAM2, priority = 3)] fn on_rx_dma;
///     #[task(priority = 2)] async fn decode;
/// }
/// ```
#[ferroforge::group(
    slots = [instance: U, rx: Rx, rx_dma: RxD],
    vectors = [on_uart = instance, on_rx_dma = rx_dma],
    shared = [uart: Uart<RxWiring<U, Rx, RxD>>],
    constructor = new,
    spawn = [events = decode],
)]
pub mod sbus_rx {
    use ferrohal_stm32::{
        rcc::Rcc,
        uart::{Instance, RxDma, RxPin, RxWiring, Uart, UartRxRefused},
    };

    use super::{CHUNKS_PER_WAKE, ChunkSource, FRAME_LEN, classify, config};
    pub use super::{Channels, Frame};
    pub use crate::uart_rx_port::{Events, Resources, on_rx_dma, on_uart};

    /// Opens the UART receive-only for SBUS on `wiring`; see
    /// [`config`] for `inverted`. A refusal is FerroHAL's, with
    /// the wiring and ring handed back.
    #[allow(clippy::type_complexity)]
    pub fn new<U, Rx, RxD>(
        rcc: &mut Rcc,
        wiring: RxWiring<U, Rx, RxD>,
        inverted: bool,
        ring: &'static mut [u8],
    ) -> Result<Resources<U, Rx, RxD>, UartRxRefused<RxWiring<U, Rx, RxD>>>
    where
        U: Instance,
        Rx: RxPin<U>,
        RxD: RxDma<U>,
    {
        Uart::new_rx(rcc, wiring, &config(inverted), ring).map(|uart| Resources { uart })
    }

    /// Reads every whole chunk the UART has and decodes each one: a good
    /// frame goes to the firmware as [`Channels`], anything else is counted.
    #[ferroforge::task(
        shared = [uart],
        bounds = [uart: ChunkSource],
        local = [good: u32 = 0, bad: u32 = 0],
        spawn = [channels(update: Channels)],
    )]
    pub async fn decode(mut cx: decode::Context, _events: Events) {
        // Room for a frame and more, so a frame that ran long ends `Full`
        // here rather than passing as a short one.
        let mut buf = [0u8; FRAME_LEN + 7];
        let mut latest = None;
        for _ in 0..CHUNKS_PER_WAKE {
            match cx.shared.uart.lock(|uart| uart.next_chunk(&mut buf)) {
                Ok(Some(chunk)) => match classify(&buf, &chunk) {
                    Some(frame) => {
                        *cx.local.good = cx.local.good.wrapping_add(1);
                        latest = Some(frame);
                    }
                    None => *cx.local.bad = cx.local.bad.wrapping_add(1),
                },
                Ok(None) => break,
                // Halted: nothing more comes until the UART is restarted.
                Err(_) => {
                    *cx.local.bad = cx.local.bad.wrapping_add(1);
                    break;
                }
            }
        }
        if let Some(frame) = latest {
            let update = Channels {
                frame,
                good: *cx.local.good,
                bad: *cx.local.bad,
            };
            let _: Result<(), Channels> = cx.spawn.channels(update);
        }
    }
}

#[cfg(test)]
mod tests {

    // The constructor's full signature, spelt out, is the point of the test.
    #![allow(clippy::type_complexity)]

    use std::{string::ToString, vec::Vec};

    use super::*;
    use crate::tests::groups;

    #[test]
    fn sbus_rx_states_its_contract() {
        let found = groups(include_str!("sbus.rs"));
        let [group] = found.as_slice() else {
            panic!("one group in sbus.rs, found {}", found.len());
        };
        assert_eq!(group.path.last().unwrap(), "sbus_rx");
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(roles, ["instance", "rx", "rx_dma"]);
        let vectors: Vec<_> = contract
            .vectors
            .iter()
            .map(|v| (v.member.to_string(), v.slot.to_string()))
            .collect();
        assert_eq!(
            vectors,
            [
                ("on_uart".to_string(), "instance".to_string()),
                ("on_rx_dma".to_string(), "rx_dma".to_string()),
            ]
        );
        let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
        assert_eq!(shared, ["uart"]);
        assert!(contract.constructor.as_ref().unwrap().is_ident("new"));
    }

    #[test]
    fn a_frame_round_trips() {
        let mut frame = Frame {
            digital: [true, false],
            failsafe: true,
            ..Frame::default()
        };
        for (index, value) in frame.values.iter_mut().enumerate() {
            *value = 172 + 107 * index as u16;
        }
        let bytes = encode_frame(&frame);
        assert_eq!(bytes[0], HEADER);
        assert_eq!(bytes[FRAME_LEN - 1], FOOTER);
        assert_eq!(decode_frame(&bytes), Some(frame));
    }

    /// Bit order, worked by hand: channel 1 fills byte 1 and the low three
    /// bits of byte 2, least significant bit first, and channel 2 starts at
    /// byte 2's bit 3.
    #[test]
    fn channels_are_packed_least_significant_bit_first() {
        let mut bytes = [0u8; FRAME_LEN];
        bytes[0] = HEADER;
        bytes[1] = 0xFF;
        bytes[2] = 0x07 | 0x08;
        bytes[23] = 0x04;
        let frame = decode_frame(&bytes).expect("a frame");
        assert_eq!(frame.values[0], 0x7FF);
        assert_eq!(frame.values[1], 1);
        assert!(frame.values[2..].iter().all(|&value| value == 0));
        assert!(frame.lost && !frame.failsafe);
        assert_eq!(frame.digital, [false, false]);
    }

    #[test]
    fn anything_but_a_frame_is_refused() {
        let good = encode_frame(&Frame::default());
        assert!(decode_frame(&good).is_some());
        assert!(decode_frame(&good[..FRAME_LEN - 1]).is_none());
        let mut longer = [0u8; FRAME_LEN + 1];
        longer[..FRAME_LEN].copy_from_slice(&good);
        assert!(decode_frame(&longer).is_none());
        let mut header = good;
        header[0] = 0x0E;
        assert!(decode_frame(&header).is_none());
        let mut footer = good;
        footer[FRAME_LEN - 1] = 0x04;
        assert!(decode_frame(&footer).is_none());
    }

    /// Only a chunk the line ended is a frame: one cut by a full buffer or
    /// struck by a gap is counted bad, whatever its bytes.
    #[test]
    fn only_a_chunk_ended_by_an_idle_line_is_a_frame() {
        let good = encode_frame(&Frame::default());
        let chunk = |end| Chunk {
            len: FRAME_LEN,
            end,
            generation: 0,
        };
        assert!(classify(&good, &chunk(uart::ChunkEnd::Idle)).is_some());
        assert!(classify(&good, &chunk(uart::ChunkEnd::Full)).is_none());
        let gap = uart::ChunkEnd::Gap(uart::Error::Parity);
        assert!(classify(&good, &chunk(gap)).is_none());
    }

    #[test]
    fn the_line_is_sbus_framing() {
        let line = config(true);
        assert_eq!(line.baudrate, Hertz(100_000));
        assert_eq!(line.parity, Parity::Even);
        assert_eq!(line.stop_bits, StopBits::Two);
        assert!(line.invert_rx);
        assert!(!config(false).invert_rx);
    }

    #[cfg(feature = "stm32f401re")]
    #[test]
    fn sbus_rx_builds_on_an_f401_wiring() {
        use ferrohal_stm32::{
            chip::USART1,
            dma::Stream,
            gpio::{Alternate, Pin},
            rcc::Rcc,
            uart::{RxWiring, UartRxRefused},
        };
        // USART1 RX on PA10, AF7 (DS10086 table 9), on DMA2 stream 2
        // channel 4 (RM0368 table 28).
        type Rx = Pin<'A', 10, Alternate<7>>;
        type W = RxWiring<USART1, Rx, Stream<2, 2>>;
        type Built = crate::uart_rx_port::Resources<USART1, Rx, Stream<2, 2>>;
        let constructor: fn(
            &mut Rcc,
            W,
            bool,
            &'static mut [u8],
        ) -> Result<Built, UartRxRefused<W>> = sbus_rx::new;
        let _ = constructor;
    }

    #[cfg(feature = "stm32h753zi")]
    #[test]
    fn sbus_rx_builds_on_an_h753_wiring() {
        use ferrohal_stm32::{
            chip::USART1,
            dma::Stream,
            gpio::{Alternate, Pin},
            rcc::Rcc,
            uart::{RxWiring, UartRxRefused},
        };
        // USART1 RX on PB7, AF7 (DS12117 tables 9 to 11), on any stream
        // through DMAMUX1 (RM0433 section 17.3).
        type Rx = Pin<'B', 7, Alternate<7>>;
        type W = RxWiring<USART1, Rx, Stream<2, 4>>;
        type Built = crate::uart_rx_port::Resources<USART1, Rx, Stream<2, 4>>;
        let constructor: fn(
            &mut Rcc,
            W,
            bool,
            &'static mut [u8],
        ) -> Result<Built, UartRxRefused<W>> = sbus_rx::new;
        let _ = constructor;
    }
}
