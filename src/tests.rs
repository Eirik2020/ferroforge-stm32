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
    assert_eq!(shared, ["spi"]);
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
    assert_eq!(contract.config[0].name, "config");
    assert_eq!(contract.config[0].layer, ferroforge_contracts::Layer::Board);
    let constructor = contract.constructor.as_ref().unwrap();
    assert!(constructor.path.is_ident("new"));
    let arguments: Vec<_> = constructor
        .arguments
        .iter()
        .map(|a| a.to_string())
        .collect();
    assert_eq!(
        arguments,
        ["rcc", "wiring", "cs", "config", "tx_buf", "rx_buf"]
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

#[cfg(feature = "stm32f401re")]
#[test]
fn spi_port_builds_on_an_f401_wiring() {
    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin},
        rcc::Rcc,
        spi::{Config, SpiRefused, Wiring},
    };
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    // SPI1 on PA5/PA6/PA7, DMA2 streams 3 (TX) and 0 (RX) (RM0368 table 29).
    type W = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'A', 7>, Stream<2, 3>, Stream<2, 0>>;
    type Cs = Pin<'A', 4, Output>;
    type Built = spi_port::Resources<
        SPI1,
        Af5<'A', 5>,
        Af5<'A', 6>,
        Af5<'A', 7>,
        Stream<2, 3>,
        Stream<2, 0>,
        Cs,
    >;
    let constructor: fn(
        &mut Rcc,
        W,
        Cs,
        &Config,
        &'static mut [u8],
        &'static mut [u8],
    ) -> Result<
        Built,
        SpiRefused<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'A', 7>, Stream<2, 3>, Stream<2, 0>>,
    > = spi_port::new;
    let _ = constructor;
}

#[cfg(feature = "stm32h753zi")]
#[test]
fn spi_port_builds_on_an_h753_wiring() {
    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin},
        rcc::Rcc,
        spi::{Config, SpiRefused, Wiring},
    };
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    // SPI1 on PA5/PA6/PD7 (DS12117 table 9), DMA1 streams 5 and 4 through
    // DMAMUX (RM0433 section 17.3).
    type W = Wiring<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'D', 7>, Stream<1, 5>, Stream<1, 4>>;
    type Cs = Pin<'D', 14, Output>;
    type Built = spi_port::Resources<
        SPI1,
        Af5<'A', 5>,
        Af5<'A', 6>,
        Af5<'D', 7>,
        Stream<1, 5>,
        Stream<1, 4>,
        Cs,
    >;
    let constructor: fn(
        &mut Rcc,
        W,
        Cs,
        &Config,
        &'static mut [u8],
        &'static mut [u8],
    ) -> Result<
        Built,
        SpiRefused<SPI1, Af5<'A', 5>, Af5<'A', 6>, Af5<'D', 7>, Stream<1, 5>, Stream<1, 4>>,
    > = spi_port::new;
    let _ = constructor;
}

#[test]
fn uart_queue_port_states_its_contract() {
    let found = groups(include_str!("uart_queue.rs"));
    let [group] = found.as_slice() else {
        panic!("one group in uart_queue.rs, found {}", found.len());
    };
    assert_eq!(group.path.last().unwrap(), "uart_queue_port");
    let contract = &group.arguments.contract;
    let names = |list: &[ferroforge_contracts::Typed]| -> Vec<std::string::String> {
        list.iter().map(|t| t.name.to_string()).collect()
    };
    assert_eq!(names(&contract.memory), ["queues"]);
    assert_eq!(names(&contract.handles), ["handle"]);
    let pends: Vec<_> = contract
        .pends
        .iter()
        .map(|p| (p.name.to_string(), p.member.to_string()))
        .collect();
    assert_eq!(pends, [("kick".to_string(), "q_on_tx_dma".to_string())]);
    let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
    assert_eq!(shared, ["port"]);
    let arguments: Vec<_> = contract
        .constructor
        .as_ref()
        .unwrap()
        .arguments
        .iter()
        .map(|a| a.to_string())
        .collect();
    assert_eq!(
        arguments,
        [
            "rcc", "wiring", "config", "tx_buf", "rx_buf", "queues", "kick", "now"
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

/// `uart_queue::new` at one wiring, with the result's fields the contract
/// names: `port` (shared) and `handle`.
macro_rules! uart_queue_builds {
    ($W:ty, $U:ty, $Tx:ty, $Rx:ty, $TxD:ty, $RxD:ty) => {{
        use crate::uart_queue::{self, Handle, Port, Queues, Resources};
        use ferrohal_stm32::{
            rcc::Rcc,
            uart::{Config, UartRefused},
        };
        let constructor: fn(
            &mut Rcc,
            $W,
            &Config,
            &'static mut [u8],
            &'static mut [u8],
            &'static mut Queues<8, 8>,
            fn(),
            fn() -> u32,
        ) -> Result<Resources<$U, $Tx, $Rx, $TxD, $RxD>, UartRefused<$W>> = uart_queue::new;
        let _ = constructor;
        let fields = |built: Resources<$U, $Tx, $Rx, $TxD, $RxD>| -> (Port<$U, $Tx, $Rx, $TxD, $RxD>, Handle) {
            (built.port, built.handle)
        };
        let _ = fields;
    }};
}

#[cfg(feature = "stm32f401re")]
#[test]
fn uart_queue_port_builds_on_an_f401_wiring() {
    use ferrohal_stm32::{
        chip::USART1,
        dma::Stream,
        gpio::{Alternate, Pin},
        uart::Wiring,
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // USART1 on PA9/PA10, DMA2 streams 7 and 2 (RM0368 table 29).
    type W = Wiring<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    uart_queue_builds!(W, USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>);
}

#[cfg(feature = "stm32h753zi")]
#[test]
fn uart_queue_port_builds_on_an_h753_wiring() {
    use ferrohal_stm32::{
        chip::USART1,
        dma::Stream,
        gpio::{Alternate, Pin},
        uart::Wiring,
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // FerroHAL's H753 bench USART1 (its Z3): DMA2 streams 5 and 4 through
    // DMAMUX.
    type W = Wiring<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    uart_queue_builds!(W, USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>);
}

#[test]
fn uart_lent_port_states_its_contract() {
    let found = groups(include_str!("uart_lent.rs"));
    let [group] = found.as_slice() else {
        panic!("one group in uart_lent.rs, found {}", found.len());
    };
    assert_eq!(group.path.last().unwrap(), "uart_lent_port");
    let contract = &group.arguments.contract;
    let names = |list: &[ferroforge_contracts::Typed]| -> Vec<std::string::String> {
        list.iter().map(|t| t.name.to_string()).collect()
    };
    assert_eq!(names(&contract.memory), ["memory"]);
    assert_eq!(names(&contract.handles), ["handle"]);
    let pends: Vec<_> = contract
        .pends
        .iter()
        .map(|p| (p.name.to_string(), p.member.to_string()))
        .collect();
    assert_eq!(pends, [("kick".to_string(), "l_on_tx_dma".to_string())]);
    let shared: Vec<_> = contract.shared.iter().map(|r| r.name.to_string()).collect();
    assert_eq!(shared, ["port"]);
    let arguments: Vec<_> = contract
        .constructor
        .as_ref()
        .unwrap()
        .arguments
        .iter()
        .map(|a| a.to_string())
        .collect();
    assert_eq!(
        arguments,
        [
            "rcc", "wiring", "config", "tx_buf", "rx_buf", "memory", "rx_pool", "tx_lent", "kick",
            "now"
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

/// `uart_lent::new_lent` at one wiring, with the result's fields the
/// contract names: `port` (shared) and `handle`.
macro_rules! uart_lent_builds {
    ($W:ty, $U:ty, $Tx:ty, $Rx:ty, $TxD:ty, $RxD:ty) => {{
        use crate::uart_lent::{
            self, LentUartHandle, LentUartMemory, LentUartPort, LentUartRefused, LentUartResources,
        };
        use ferrohal_stm32::{rcc::Rcc, uart::Config};
        let constructor: fn(
            &mut Rcc,
            $W,
            &Config,
            &'static mut [u8],
            &'static mut [u8],
            &'static mut LentUartMemory<8, 8>,
            &mut dyn Iterator<Item = &'static mut [u8]>,
            &mut dyn Iterator<Item = &'static mut [u8]>,
            fn(),
            fn() -> u32,
        ) -> Result<
            LentUartResources<$U, $Tx, $Rx, $TxD, $RxD>,
            LentUartRefused<$W, $U, $Tx, $Rx, $TxD, $RxD>,
        > = uart_lent::new_lent;
        let _ = constructor;
        let fields = |built: LentUartResources<$U, $Tx, $Rx, $TxD, $RxD>| {
            let port: LentUartPort<$U, $Tx, $Rx, $TxD, $RxD> = built.port;
            let handle: LentUartHandle = built.handle;
            (port, handle)
        };
        let _ = fields;
    }};
}

#[cfg(feature = "stm32f401re")]
#[test]
fn uart_lent_port_builds_on_an_f401_wiring() {
    use ferrohal_stm32::{
        chip::USART1,
        dma::Stream,
        gpio::{Alternate, Pin},
        uart::Wiring,
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // USART1 on PA9/PA10, DMA2 streams 7 and 2 (RM0368 table 29).
    type W = Wiring<USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>>;
    uart_lent_builds!(W, USART1, Af7<'A', 9>, Af7<'A', 10>, Stream<2, 7>, Stream<2, 2>);
}

#[cfg(feature = "stm32h753zi")]
#[test]
fn uart_lent_port_builds_on_an_h753_wiring() {
    use ferrohal_stm32::{
        chip::USART1,
        dma::Stream,
        gpio::{Alternate, Pin},
        uart::Wiring,
    };
    type Af7<const P: char, const N: u8> = Pin<P, N, Alternate<7>>;
    // FerroHAL's H753 bench USART1 (its Z3): DMA2 streams 5 and 4 through
    // DMAMUX.
    type W = Wiring<USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>>;
    uart_lent_builds!(W, USART1, Af7<'B', 6>, Af7<'B', 7>, Stream<2, 5>, Stream<2, 4>);
}

#[test]
fn spi_single_and_multi_ports_state_their_contracts() {
    let found = groups(include_str!("spi_queue.rs"));
    let [single, multi] = found.as_slice() else {
        panic!("two groups in spi_queue.rs, found {}", found.len());
    };
    assert_eq!(single.path.last().unwrap(), "spi_single_port");
    assert_eq!(multi.path.last().unwrap(), "spi_multi_port");
    let names = |list: &[ferroforge_contracts::Typed]| -> Vec<std::string::String> {
        list.iter().map(|t| t.name.to_string()).collect()
    };
    for (group, member, handles, config) in [
        (single, "s_on_spi_rx_dma", "handle", "config"),
        (multi, "m_on_spi_rx_dma", "handles", "configs"),
    ] {
        let contract = &group.arguments.contract;
        let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
        assert_eq!(
            roles,
            ["instance", "sck", "miso", "mosi", "tx_dma", "rx_dma", "cs"]
        );
        assert_eq!(names(&contract.memory), ["memory"]);
        assert_eq!(names(&contract.handles), [handles]);
        let pends: Vec<_> = contract
            .pends
            .iter()
            .map(|p| (p.name.to_string(), p.member.to_string()))
            .collect();
        assert_eq!(pends, [("kick".to_string(), member.to_string())]);
        let arguments: Vec<_> = contract
            .constructor
            .as_ref()
            .unwrap()
            .arguments
            .iter()
            .map(|a| a.to_string())
            .collect();
        assert_eq!(
            arguments,
            [
                "rcc", "wiring", "cs", config, "tx_buf", "rx_buf", "memory", "kick", "now"
            ]
        );
        let [vector] = contract.vectors.as_slice() else {
            panic!("one vector");
        };
        assert_eq!(vector.member, member);
        assert_eq!(vector.slot, "rx_dma");
    }
}

#[test]
fn spi_lent_port_states_its_contract() {
    let found = groups(include_str!("spi_lent.rs"));
    let [lent] = found.as_slice() else {
        panic!("one group in spi_lent.rs, found {}", found.len());
    };
    assert_eq!(lent.path.last().unwrap(), "spi_lent_port");
    let contract = &lent.arguments.contract;
    let roles: Vec<_> = contract.slots.iter().map(|s| s.role.to_string()).collect();
    assert_eq!(
        roles,
        ["instance", "sck", "miso", "mosi", "tx_dma", "rx_dma", "cs"]
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
    assert_eq!(pends, [("kick".to_string(), "r_on_spi_rx_dma".to_string())]);
    let arguments: Vec<_> = contract
        .constructor
        .as_ref()
        .unwrap()
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
    let [vector] = contract.vectors.as_slice() else {
        panic!("one vector");
    };
    assert_eq!(vector.member, "r_on_spi_rx_dma");
    assert_eq!(vector.slot, "rx_dma");
}

/// `spi_queue::new_single` and `new_multi` at one wiring, with the result's
/// fields the contracts name.
macro_rules! spi_queue_builds {
    ($U:ty, $Sck:ty, $Miso:ty, $Mosi:ty, $TxD:ty, $RxD:ty, $Cs:ty) => {{
        use crate::spi_queue::{
            self, Handles, MultiMemory, MultiRefused, MultiResources, Port, SingleMemory,
            SingleResources, SpiHandle,
        };
        use ferrohal_stm32::{
            rcc::Rcc,
            spi::{Config, SpiRefused, Wiring},
        };
        type W = Wiring<$U, $Sck, $Miso, $Mosi, $TxD, $RxD>;
        let single: fn(
            &mut Rcc,
            W,
            $Cs,
            &Config,
            &'static mut [u8],
            &'static mut [u8],
            &'static mut SingleMemory<4>,
            fn(),
            fn() -> u32,
        ) -> Result<
            SingleResources<W, $Cs>,
            SpiRefused<$U, $Sck, $Miso, $Mosi, $TxD, $RxD>,
        > = spi_queue::new_single;
        let _ = single;
        let fields = |built: SingleResources<W, $Cs>| -> (Port<W, ($Cs,)>, SpiHandle) {
            (built.port, built.handle)
        };
        let _ = fields;
        type Four = ($Cs, $Cs, $Cs, $Cs);
        let multi: fn(
            &mut Rcc,
            W,
            Four,
            &[Config],
            &'static mut [u8],
            &'static mut [u8],
            &'static mut MultiMemory<8, 8, 4>,
            fn(),
            fn() -> u32,
        ) -> Result<
            MultiResources<W, Four>,
            MultiRefused<$U, $Sck, $Miso, $Mosi, $TxD, $RxD, Four>,
        > = spi_queue::new_multi;
        let _ = multi;
        let fields = |built: MultiResources<W, Four>| -> (Port<W, Four>, Handles) {
            (built.port, built.handles)
        };
        let _ = fields;
        use crate::spi_lent::{
            self, LentHandles, LentMemory, LentPort, LentRefused, LentResources,
        };
        let lent: fn(
            &mut Rcc,
            W,
            Four,
            &[Config],
            &'static mut [u8],
            &'static mut [u8],
            &'static mut LentMemory<9, 4>,
            &mut dyn Iterator<Item = (usize, &'static mut [u8])>,
            fn(),
            fn() -> u32,
        ) -> Result<
            LentResources<W, Four>,
            LentRefused<$U, $Sck, $Miso, $Mosi, $TxD, $RxD, Four>,
        > = spi_lent::new_lent;
        let _ = lent;
        let fields = |built: LentResources<W, Four>| -> (LentPort<W, Four>, LentHandles) {
            (built.port, built.handles)
        };
        let _ = fields;
    }};
}

#[cfg(feature = "stm32f401re")]
#[test]
fn spi_single_and_multi_ports_build_on_an_f401_wiring() {
    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin},
    };
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    // SPI1 on PA5/PA6/PA7, DMA2 streams 3 and 0 (RM0368 table 29).
    spi_queue_builds!(
        SPI1,
        Af5<'A', 5>,
        Af5<'A', 6>,
        Af5<'A', 7>,
        Stream<2, 3>,
        Stream<2, 0>,
        Pin<'A', 4, Output>
    );
}

#[cfg(feature = "stm32h753zi")]
#[test]
fn spi_single_and_multi_ports_build_on_an_h753_wiring() {
    use ferrohal_stm32::{
        chip::SPI1,
        dma::Stream,
        gpio::{Alternate, Output, Pin},
    };
    type Af5<const P: char, const N: u8> = Pin<P, N, Alternate<5>>;
    // FerroHAL's H753 bench SPI1 (its S1): DMA1 streams 5 and 4 through
    // DMAMUX.
    spi_queue_builds!(
        SPI1,
        Af5<'A', 5>,
        Af5<'A', 6>,
        Af5<'D', 7>,
        Stream<1, 5>,
        Stream<1, 4>,
        Pin<'D', 14, Output>
    );
}
