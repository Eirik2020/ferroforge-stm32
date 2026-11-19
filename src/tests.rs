//! Host tests: each group's contract, read from this crate's source the way
//! a host tool reads it, and each constructor instantiated with a real
//! chip's wiring, built for the host.
//!
//! Requirements: PORT-001, PORT-002

// The constructor's full signature, spelt out, is the point of the test.
#![allow(clippy::type_complexity)]

use std::{string::ToString, vec::Vec};

use crate::{spi_port, uart_port};

/// Every group in `source`, by name, as `ferroforge_contracts` reads it.
pub(crate) fn groups(source: &str) -> Vec<ferroforge_contracts::GroupDeclaration> {
    let file: syn::File = syn::parse_str(source).expect("the source parses");
    ferroforge_contracts::groups(&file).expect("every contract is well formed")
}

#[test]
fn uart_port_states_its_contract() {
    let found = groups(include_str!("uart.rs"));
    let [group] = found.as_slice() else {
        panic!("one group in uart.rs, found {}", found.len());
    };
    assert_eq!(group.path.last().unwrap(), "uart_port");
    let contract = &group.arguments.contract;
    let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
    assert_eq!(roles, ["instance", "tx", "rx", "tx_dma", "rx_dma"]);
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
            ("on_tx_dma".to_string(), "tx_dma".to_string()),
        ]
    );
    let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
    assert_eq!(shared, ["uart"]);
    assert!(contract.local.is_empty());
    let constructor = contract.constructor.as_ref().unwrap();
    assert!(constructor.path.is_ident("new"));
    let arguments: Vec<_> = constructor
        .arguments
        .iter()
        .map(|a| a.to_string())
        .collect();
    assert_eq!(arguments, ["rcc", "wiring", "config", "tx_buf", "rx_buf"]);
    let buffers: Vec<_> = contract
        .buffers
        .iter()
        .map(|b| (b.name.to_string(), b.dma))
        .collect();
    assert_eq!(
        buffers,
        [("tx_buf".to_string(), true), ("rx_buf".to_string(), true)]
    );
    assert_eq!(
        contract.config[0].layer,
        ferroforge_contracts::Layer::Platform
    );
    // Every vector's member is a hardware task defined in the group.
    for vector in &contract.vectors {
        let member = group
            .members
            .iter()
            .find(|m| m.name == vector.member)
            .expect("the vector's member is in the group");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }
}

#[cfg(feature = "stm32f401re")]
#[test]
fn uart_port_builds_on_an_f401_wiring() {
    use ferrohal_stm32::{
        chip::USART1,
        dma::Stream,
        gpio::{Alternate, Pin},
        rcc::Rcc,
        uart::{Config, UartRefused, Wiring},
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // USART1 on PA9/PA10, DMA2 streams 7 and 2 (RM0368 table 29).
    type W = Wiring<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    type Built =
        uart_port::Resources<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    let constructor: fn(
        &mut Rcc,
        W,
        &Config,
        &'static mut [u8],
        &'static mut [u8],
    ) -> Result<Built, UartRefused<W>> = uart_port::new;
    let _ = constructor;
}

#[cfg(feature = "stm32h753zi")]
#[test]
fn uart_port_builds_on_an_h753_wiring() {
    use ferrohal_stm32::{
        chip::USART1,
        dma::Stream,
        gpio::{Alternate, Pin},
        rcc::Rcc,
        uart::{Config, UartRefused, Wiring},
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // FerroHAL's H753 bench USART1 (its Z3): DMA2 streams 5 and 4 through
    // DMAMUX.
    type W = Wiring<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    type Built = uart_port::Resources<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    let constructor: fn(
        &mut Rcc,
        W,
        &Config,
        &'static mut [u8],
        &'static mut [u8],
    ) -> Result<Built, UartRefused<W>> = uart_port::new;
    let _ = constructor;
}

#[test]
fn spi_port_states_its_contract() {
    let found = groups(include_str!("spi.rs"));
    let [group] = found.as_slice() else {
        panic!("one group in spi.rs, found {}", found.len());
    };
    assert_eq!(group.path.last().unwrap(), "spi_port");
    let contract = &group.arguments.contract;
    let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
    assert_eq!(
        roles,
        ["instance", "sck", "miso", "mosi", "tx_dma", "rx_dma", "cs"]
    );
    let vectors: Vec<_> = contract
        .vectors
        .iter()
        .map(|v| (v.member.to_string(), v.slot.to_string()))
        .collect();
    assert_eq!(
        vectors,
        [("on_spi_rx_dma".to_string(), "rx_dma".to_string())]
    );
    let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
    assert_eq!(shared, ["port"]);
    assert!(contract.local.is_empty());
    let buffers: Vec<_> = contract
        .buffers
        .iter()
        .map(|b| (b.name.to_string(), b.element.to_string(), b.dma))
        .collect();
    assert_eq!(
        buffers,
        [
            ("tx_buf".to_string(), "u8".to_string(), true),
            ("rx_buf".to_string(), "u8".to_string(), true),
        ]
    );
    let config: Vec<_> = contract
        .config
        .iter()
        .map(|c| (c.name.to_string(), c.layer))
        .collect();
    assert_eq!(
        config,
        [
            ("configs".to_string(), ferroforge_contracts::Layer::Board),
            ("lent".to_string(), ferroforge_contracts::Layer::Software),
            ("now".to_string(), ferroforge_contracts::Layer::Platform),
        ]
    );
    let names = |list: &[ferroforge_contracts::Typed]| -> Vec<std::string::String> {
        list.iter().map(|t| t.name.to_string()).collect()
    };
    assert_eq!(names(&contract.memory), ["memory"]);
    assert_eq!(names(&contract.handles), ["handles"]);
    let pends: Vec<_> = contract
        .pends
        .iter()
        .map(|p| (p.name.to_string(), p.member.to_string()))
        .collect();
    assert_eq!(pends, [("kick".to_string(), "on_spi_rx_dma".to_string())]);
    let constructor = contract.constructor.as_ref().unwrap();
    assert!(constructor.path.is_ident("new"));
    let arguments: Vec<_> = constructor
        .arguments
        .iter()
        .map(|a| a.to_string())
        .collect();
    assert_eq!(
        arguments,
        [
            "rcc", "wiring", "cs", "configs", "tx_buf", "rx_buf", "memory", "lent", "kick", "now"
        ]
    );
    for vector in &contract.vectors {
        let member = group
            .members
            .iter()
            .find(|m| m.name == vector.member)
            .expect("the vector's member is in the group");
        assert_eq!(member.kind, ferroforge_contracts::TaskKind::Hardware);
    }
}

/// The constructor's signature over one wiring, with two chip selects and
/// memory for two devices of four slots.
macro_rules! spi_port_constructor {
    ($u:ty, $sck:ty, $miso:ty, $mosi:ty, $txd:ty, $rxd:ty, $cs:ty) => {{
        use ferrohal_stm32::{
            rcc::Rcc,
            spi::{Config, Wiring},
        };
        type W = Wiring<$u, $sck, $miso, $mosi, $txd, $rxd>;
        let constructor: fn(
            &mut Rcc,
            W,
            $cs,
            &[Config],
            &'static mut [u8],
            &'static mut [u8],
            &'static mut spi_port::Memory<4, 2>,
            &mut dyn Iterator<Item = (usize, &'static mut [u8])>,
            fn(),
            fn() -> u32,
        ) -> Result<
            spi_port::Resources<W, $cs>,
            spi_port::PortRefused<$u, $sck, $miso, $mosi, $txd, $rxd, $cs>,
        > = spi_port::new;
        let _ = constructor;
    }};
}

#[cfg(feature = "stm32f401re")]
#[test]
fn spi_port_builds_on_an_f401_wiring() {
    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin},
    };
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    // SPI1 on PA5/PA6/PA7, DMA2 streams 3 (TX) and 0 (RX) (RM0368 table 29).
    spi_port_constructor!(
        SPI1,
        Af5<'A', 5>,
        Af5<'A', 6>,
        Af5<'A', 7>,
        Stream<2, 3>,
        Stream<2, 0>,
        (Pin<'A', 4, Output>, Pin<'C', 8, Output>)
    );
}

#[cfg(feature = "stm32h753zi")]
#[test]
fn spi_port_builds_on_an_h753_wiring() {
    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin},
    };
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    // SPI1 on PA5/PA6/PD7 (DS12117 table 9), DMA1 streams 5 and 4 through
    // DMAMUX (RM0433 section 17.3).
    spi_port_constructor!(
        SPI1,
        Af5<'A', 5>,
        Af5<'A', 6>,
        Af5<'D', 7>,
        Stream<1, 5>,
        Stream<1, 4>,
        (Pin<'D', 14, Output>, Pin<'D', 15, Output>)
    );
}
