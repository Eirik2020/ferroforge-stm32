//! Host tests: each group's contract, read from this crate's source the way
//! a host tool reads it, and each constructor instantiated with a real
//! chip's wiring, built for the host.
//!
//! Requirements: PORT-001, PORT-002

// The constructor's full signature, spelt out, is the point of the test.
#![allow(clippy::type_complexity)]

use std::{string::ToString, vec::Vec};

use crate::uart_port;

/// Every group in `source`, by name, as `ferroforge_contracts` reads it.
fn groups(source: &str) -> Vec<ferroforge_contracts::GroupDeclaration> {
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
    assert!(contract.constructor.as_ref().unwrap().is_ident("new"));
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
        uart::{Buffers, Config, UartRefused, Wiring},
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // The golden app's USART1 (RM0368 table 28: DMA2 streams 7 and 2).
    type W = Wiring<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    type Built =
        uart_port::Resources<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    let constructor: fn(&mut Rcc, W, &Config, Buffers) -> Result<Built, UartRefused<W>> =
        uart_port::new;
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
        uart::{Buffers, Config, UartRefused, Wiring},
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // FerroHAL's H753 bench USART1 (its Z3): DMA2 streams 5 and 4 through
    // DMAMUX.
    type W = Wiring<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    type Built = uart_port::Resources<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    let constructor: fn(&mut Rcc, W, &Config, Buffers) -> Result<Built, UartRefused<W>> =
        uart_port::new;
    let _ = constructor;
}
