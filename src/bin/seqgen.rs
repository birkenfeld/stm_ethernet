#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use defmt::info;

use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::{
    prelude::*,
    gpio::GpioExt,
    pac::{Peripherals, CorePeripherals, SYST},
    rcc::RccExt,
};

use core::cell::Cell;

use smoltcp::time::Instant;
use smoltcp::wire::EthernetAddress;
use smoltcp::iface::{SocketSet, Config, Interface, SocketStorage};
use smoltcp::socket::udp::{Socket as UdpSocket, PacketBuffer as UdpPacketBuffer};
use smoltcp::socket::dhcpv4::{Socket as DhcpSocket, Event as DhcpEvent};
use smoltcp::storage::PacketMetadata;

use stm32_eth::{dma::{RxRingEntry, TxRingEntry}, Parts, PartsIn, EthPins};

static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(180.MHz()).hclk(180.MHz()).freeze();

    setup_systick(&mut cp.SYST);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    let pins = EthPins {
        ref_clk: gpioa.pa1,
        crs: gpioa.pa7,
        tx_en: gpiog.pg11,
        tx_d0: gpiog.pg13,
        tx_d1: gpiob.pb13,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };
    let mdio = gpioa.pa2.into_alternate();
    let mdc = gpioc.pc1.into_alternate();

    let mut rx_ring: [RxRingEntry; 16] = Default::default();
    let mut tx_ring: [TxRingEntry; 4] = Default::default();
    let Parts { mut dma, .. } = stm32_eth::new_with_mii(
        PartsIn { dma: p.ETHERNET_DMA, mac: p.ETHERNET_MAC, mmc: p.ETHERNET_MMC, ptp: p.ETHERNET_PTP },
        &mut rx_ring[..],
        &mut tx_ring[..],
        clocks,
        pins,
        mdio,
        mdc,
    ).unwrap();
    // dma.enable_interrupt();

    let serial = read_serno();
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);
    let mut config = Config::new();
    config.hardware_addr = Some(ethernet_addr.into());
    let mut iface = Interface::new(config, &mut &mut dma);

    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*4];

    let udp_socket = UdpSocket::new(
        UdpPacketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpPacketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [SocketStorage::EMPTY, SocketStorage::EMPTY];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    let dhcp_socket = DhcpSocket::new();

    let udp_handle = sockets.add(udp_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    let mut target = None;
    let mut seq_number = 0u32;

    info!("------------------------------------------------------------------------");

    // give the remote partner time to realize the link is up,
    // so we don't run into the 10sec DHCP discover interval
    while interrupt::free(|cs| ETH_TIME.borrow(cs).get()) < 2000 { }

    loop {
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        iface.poll(time, &mut &mut dma, &mut sockets);

        let event = sockets.get_mut::<DhcpSocket>(dhcp_handle).poll();
        if let Some(DhcpEvent::Configured(config)) = event {
            let addr = config.address;
            iface.update_ip_addrs(|addrs| addrs.push(addr.into()).unwrap());

            if let Some(router) = config.router {
                iface.routes_mut().add_default_ipv4_route(router).unwrap();
            } else {
                iface.routes_mut().remove_default_ipv4_route();
            }

            sockets.get_mut::<UdpSocket>(udp_handle).bind((addr.address(), 50000)).unwrap();
            break;
        }
    }

    loop {
        // process packets
        {
            let socket = sockets.get_mut::<UdpSocket>(udp_handle);
            while let Ok((msg, ep)) = socket.recv() {
                if msg == b"start" {
                    info!("got start message");
                    target = Some(ep);
                    seq_number = 0;
                } else if msg == b"stop" {
                    info!("got stop message");
                    target = None;
                }
            }
            if let Some(tgt) = target {
                while let Ok(buf) = socket.send(1400, tgt) {
                    buf[0] = (seq_number >> 24) as u8;
                    buf[1] = (seq_number >> 16) as u8;
                    buf[2] = (seq_number >> 8) as u8;
                    buf[3] = seq_number as u8;
                    seq_number += 1;
                }
            }
        }
        // handle ethernet
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        iface.poll(time, &mut &mut dma, &mut sockets);
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(22_500 - 1);
    syst.enable_counter();
    syst.enable_interrupt();
}

fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

#[exception]
fn SysTick() {
    interrupt::free(|cs| {
        let time = ETH_TIME.borrow(cs);
        time.set(time.get().wrapping_add(1));
    });
}
