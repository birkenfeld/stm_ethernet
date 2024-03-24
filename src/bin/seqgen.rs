#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal::{
    self as hal,
    prelude::*,
    gpio::GpioExt,
    rcc::RccExt,
};
use systick_monotonic::Systick;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpCidr, IpAddress};
use smoltcp::iface::{SocketSet, SocketHandle, Config, Interface, SocketStorage};
use smoltcp::socket::udp::{Socket as UdpSocket, PacketBuffer as UdpPacketBuffer, UdpMetadata};
use smoltcp::socket::dhcpv4::{Socket as DhcpSocket, Event as DhcpEvent};
use smoltcp::storage::PacketMetadata;

use stm32_eth::{dma::{RxRingEntry, TxRingEntry}, Parts, PartsIn, EthPins};

use stm_ethernet::{Leds, Net, read_serno};

const TIME_GRANULARITY: u32 = 1000;  // 1000 Hz granularity
const PORT: u16 = 50000;

#[rtic::app(
    device = crate::hal::pac,
    dispatchers = [USART1, UART4],
)]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Time = Systick<TIME_GRANULARITY>;

    #[shared]
    struct Shared {
        net: Net<TIME_GRANULARITY>,
        use_dhcp: bool,
    }

    #[local]
    struct Local {
        leds: Leds,
        udp_handle: SocketHandle,
    }

    #[init(local = [
        rx_ring: [RxRingEntry; 16] = [RxRingEntry::RX_INIT; 16],
        tx_ring: [TxRingEntry; 4] = [TxRingEntry::INIT; 4],
        udp_rx_meta_buffer: [PacketMetadata<UdpMetadata>; 4] = [PacketMetadata::EMPTY; 4],
        udp_tx_meta_buffer: [PacketMetadata<UdpMetadata>; 4] = [PacketMetadata::EMPTY; 4],
        udp_rx_data_buffer: [u8; 1500*4] = [0; 1500*4],
        udp_tx_data_buffer: [u8; 1500*4] = [0; 1500*4],
        sockets_storage: [SocketStorage<'static>; 2] = [SocketStorage::EMPTY, SocketStorage::EMPTY],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p = cx.device;
        let cp = cx.core;

        let rcc = p.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(180.MHz()).hclk(180.MHz()).freeze();

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

        // DHCP if user button pressed
        let dhcp_btn = gpioc.pc13.into_pull_down_input();
        let use_dhcp = dhcp_btn.is_high();

        let led_green = gpiob.pb0.into_push_pull_output();
        let led_blue = gpiob.pb7.into_push_pull_output();
        let led_red = gpiob.pb14.into_push_pull_output();
        let mut leds = Leds::new(led_red.into(), led_green.into(), led_blue.into());
        // red LED: indicate "booting"
        leds.set(true, false, false);

        let Parts { mut dma, .. } = stm32_eth::new_with_mii(
            PartsIn { dma: p.ETHERNET_DMA, mac: p.ETHERNET_MAC, mmc: p.ETHERNET_MMC, ptp: p.ETHERNET_PTP },
            &mut cx.local.rx_ring[..],
            &mut cx.local.tx_ring[..],
            clocks,
            pins,
            mdio,
            mdc,
        ).unwrap();

        let serial = read_serno();
        let ethernet_addr = EthernetAddress([
            0x46, 0x52, 0x4d,  // F R M
            (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
        ]);
        let config = Config::new(ethernet_addr.into());
        let mut iface = Interface::new(config, &mut &mut dma, Instant::ZERO);
        // select the default Mesytec IP if static configuration
        if !use_dhcp {
            iface.update_ip_addrs(|addrs| {
                addrs.push(IpCidr::new(IpAddress::v4(192, 168, 168, 121), 24)).unwrap();
            });
        }

        let udp_socket = UdpSocket::new(
            UdpPacketBuffer::new(&mut cx.local.udp_rx_meta_buffer[..], &mut cx.local.udp_rx_data_buffer[..]),
            UdpPacketBuffer::new(&mut cx.local.udp_tx_meta_buffer[..], &mut cx.local.udp_tx_data_buffer[..])
        );
        let mut sockets = SocketSet::new(&mut cx.local.sockets_storage[..]);

        let udp_handle = sockets.add(udp_socket);

        // use systick monotonic clock for now
        let mono = Systick::new(cp.SYST, clocks.sysclk().to_Hz());

        info!("------------------------------------------------------------------------");
        if use_dhcp {
            dhcp::spawn().unwrap();
        } else {
            main::spawn().unwrap();
        }
        let net = Net { sockets, iface, dma, ntp_time: None };
        (Shared { net, use_dhcp }, Local { leds, udp_handle }, init::Monotonics(mono))
    }

    #[task(shared = [net])]
    fn dhcp(mut cx: dhcp::Context) {
        // give the remote partner time to realize the link is up,
        // so we don't run into the 10sec DHCP discover interval
        while monotonics::now().ticks() < 2000 { }

        cx.shared.net.lock(|net| {
            info!("Starting DHCP");
            let dhcp_socket = DhcpSocket::new();
            let dhcp_handle = net.sockets.add(dhcp_socket);

            loop {
                net.poll(monotonics::now());

                let event = net.sockets.get_mut::<DhcpSocket>(dhcp_handle).poll();
                if let Some(DhcpEvent::Configured(config)) = event {
                    let addr = config.address;
                    net.iface.update_ip_addrs(|addrs| addrs.push(addr.into()).unwrap());

                    if let Some(router) = config.router {
                        net.iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        net.iface.routes_mut().remove_default_ipv4_route();
                    }

                    break;
                }
            }

        });
        main::spawn().unwrap();
    }

    #[task(local = [leds, udp_handle], shared = [net, &use_dhcp])]
    fn main(cx: main::Context) {
        let main::LocalResources { leds, udp_handle } = cx.local;
        let main::SharedResources { mut net, use_dhcp } = cx.shared;
        let mut target = None;
        let mut seq_number = 0u32;

        net.lock(|net| {
            let ip_addr = net.iface.ipv4_addr().unwrap();
            info!("IP setup done ({}), binding to {}:{}",
                  if *use_dhcp { "dhcp" } else { "static" }, ip_addr, PORT);
            net.sockets.get_mut::<UdpSocket>(*udp_handle).bind((ip_addr, PORT)).unwrap();
            leds.set(false, true, false);

            loop {
                // process packets
                {
                    let socket = net.sockets.get_mut::<UdpSocket>(*udp_handle);
                    while let Ok((msg, ep)) = socket.recv() {
                        if msg == b"start" {
                            info!("got start message");
                            target = Some(ep);
                            seq_number = 0;
                            // LED blue is "running"
                            leds.set(false, true, true);
                        } else if msg == b"stop" {
                            info!("got stop message");
                            target = None;
                            leds.set(false, true, false);
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
                net.poll(monotonics::now());
            }
        });
    }
}
