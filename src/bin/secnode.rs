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
use smoltcp::socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer, State as TcpState};
use smoltcp::socket::udp::{Socket as UdpSocket, PacketBuffer as UdpPacketBuffer};
use smoltcp::socket::dhcpv4::{Socket as DhcpSocket, Event as DhcpEvent};
use smoltcp::storage::PacketMetadata;
use smoltcp::wire::DhcpRepr;

use stm32_eth::{dma::{RxRingEntry, TxRingEntry}, Parts, PartsIn, EthPins};

use stm_ethernet::{Leds, Net, read_serno};
use stm_ethernet::secnode::{self, SecNode};

const TIME_GRANULARITY: u32 = 1000;  // 1000 Hz granularity
const PORT: u16 = 10767;
const MAX_CLIENTS: usize = 1;

// const NTP_PORT: u16 = 123;
// // Simple packet requesting current timestamp
// const NTP_REQUEST: &[u8] = b"\xE3\x00\x06\xEC\x00\x00\x00\x00\x00\x00\x00\x00\
//                              \x31\x4E\x31\x34\x00\x00\x00\x00\x00\x00\x00\x00\
//                              \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
//                              \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

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
        node: SecNode<MAX_CLIENTS>,
        net: Net<TIME_GRANULARITY>,
        sock: SocketHandle,
        use_dhcp: bool,
    }

    #[local]
    struct Local {
        leds: Leds,
    }

    #[init(local = [
        rx_ring: [RxRingEntry; 16] = [RxRingEntry::RX_INIT; 16],
        tx_ring: [TxRingEntry; 4] = [TxRingEntry::INIT; 4],
        rx_buffer: [u8; 1500*16] = [0; 1500*16],
        tx_buffer: [u8; 1500*16] = [0; 1500*16],
        sockets_storage: [SocketStorage<'static>; MAX_CLIENTS] = [SocketStorage::EMPTY; MAX_CLIENTS],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p = cx.device;

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

        // set up ring buffers for network handling tokens
        let Parts { mut dma, .. } = stm32_eth::new_with_mii(
            PartsIn { dma: p.ETHERNET_DMA, mac: p.ETHERNET_MAC, mmc: p.ETHERNET_MMC, ptp: p.ETHERNET_PTP },
            cx.local.rx_ring,
            cx.local.tx_ring,
            clocks,
            pins,
            mdio,
            mdc,
        ).unwrap();

        // determine MAC address from board's serial number
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

        // set up buffers for packet content and metadata

        // create the UDP socket
        let tcp_socket = TcpSocket::new(
            TcpSocketBuffer::new(&mut cx.local.rx_buffer[..]),
            TcpSocketBuffer::new(&mut cx.local.tx_buffer[..])
        );
        let mut sockets = SocketSet::new(&mut cx.local.sockets_storage[..]);

        let tcp_handle = sockets.add(tcp_socket);

        let node = secnode::create();

        // use systick monotonic clock for now
        let mono = Systick::new(cx.core.SYST, clocks.hclk().raw());

        info!("------------------------------------------------------------------------");
        if use_dhcp {
            dhcp::spawn().unwrap();
        } else {
            start::spawn().unwrap();
        }
        let net = Net { sockets, iface, dma, ntp_time: None };
        (Shared { node, net, sock: tcp_handle, use_dhcp },
         Local { leds },
         init::Monotonics(mono))
    }

    #[task(shared = [net])]
    fn dhcp(mut cx: dhcp::Context) {
        // give the remote partner time to realize the link is up,
        // so we don't run into the 10sec DHCP discover interval
        while monotonics::now().ticks() < 2000 { }

        info!("Starting DHCP");

        let mut storage = [SocketStorage::EMPTY; 2];
        let mut sockets = SocketSet::new(&mut storage[..]);

        let dhcp_socket = DhcpSocket::new();
        let dhcp_handle = sockets.add(dhcp_socket);

        let mut rx_buf = [0; 1500];
        let mut tx_buf = [0; 1500];
        let mut rx_meta_buf = [PacketMetadata::EMPTY; 1];
        let mut tx_meta_buf = [PacketMetadata::EMPTY; 1];
        let ntp_socket = UdpSocket::new(
            UdpPacketBuffer::new(&mut rx_meta_buf[..], &mut rx_buf[..]),
            UdpPacketBuffer::new(&mut tx_meta_buf[..], &mut tx_buf[..])
        );
        let _ntp_handle = sockets.add(ntp_socket);

        cx.shared.net.lock(|net| {
            // use a dedicated socket storage here, we don't need them later
            // let mut ntp_addr = None;

            loop {
                let time = Instant::from_millis(monotonics::now().ticks() as i64);
                net.iface.poll(time, &mut &mut net.dma, &mut sockets);

                let event = net.sockets.get_mut::<DhcpSocket>(dhcp_handle).poll();
                if let Some(DhcpEvent::Configured(config)) = event {
                    let addr = config.address;
                    net.iface.update_ip_addrs(|addrs| addrs.push(addr.into()).unwrap());

                    if let Some(router) = config.router {
                        net.iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        net.iface.routes_mut().remove_default_ipv4_route();
                    }

                    if let Ok(repr) = DhcpRepr::parse(&config.packet.unwrap()) {
                        for opt in repr.additional_options {
                            if opt.kind == 42 {
                                info!("NTP: {:?}", opt.data);
                                // ntp_addr = Some(IpAddress::from_bytes(&opt.data));
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        });
        start::spawn().unwrap();
    }

    #[task(shared = [net, &sock, &use_dhcp])]
    fn start(cx: start::Context) {
        let start::SharedResources { mut net, sock, use_dhcp } = cx.shared;

        net.lock(|net| {
            let ip_addr = net.iface.ipv4_addr().unwrap();
            info!("IP setup done ({}), binding to {}:{}",
                  if *use_dhcp { "dhcp" } else { "static" }, ip_addr, PORT);

            let socket = net.sockets.get_mut::<TcpSocket>(*sock);
            socket.set_nagle_enabled(false);
            socket.listen(PORT).unwrap();

            net.dma.enable_interrupt();
        });
        poll::spawn_after(500.millis().into()).unwrap();
    }

    #[task(binds = ETH, local = [leds, connected: bool = false],
           shared = [node, net, &sock])]
    fn eth(cx: eth::Context) {
        let eth::LocalResources { leds, connected } = cx.local;
        let eth::SharedResources { node, net, sock } = cx.shared;
        let mut buf = [0; 1024];

        let _reason = stm32_eth::eth_interrupt_handler();
        // debug!("Got an ethernet interrupt! Reason: {}", _reason);

        (node, net).lock(|node, net| {
            let now = monotonics::now();
            net.poll(now);
            let time = net.get_time(now);
            let socket = net.sockets.get_mut::<TcpSocket>(*sock);

            if socket.is_active() {
                if !*connected {
                    *connected = true;
                    node.client_connected(0);
                    leds.set(false, true, true);
                }
            } else if *connected {
                *connected = false;
                node.client_finished(0);
                leds.set(false, true, false);
            }

            if let Ok(recv_bytes) = socket.recv_slice(&mut buf) {
                if recv_bytes > 0 {
                    info!("Got {} bytes", recv_bytes);
                    let result = node.process(
                        time, &mut buf[..recv_bytes],
                        0 as usecop::ClientId,
                        |_, callback: &dyn Fn(&mut dyn usecop::io::Write)| {
                            callback(&mut Writer(socket));
                        }
                    );
                    if let Err(e) = result {
                        warn!("Error processing data: {:?}", e);
                    }
                }
            }

            if !socket.is_listening() && !socket.is_open() || socket.state() == TcpState::CloseWait {
                socket.abort();
                socket.listen(PORT).ok();
                warn!("Disconnected... Reopening listening socket.");
            }

            net.poll(monotonics::now());
        });
    }

    #[task(shared = [node, net, &sock])]
    fn poll(cx: poll::Context) {
        let poll::SharedResources { node, net, sock } = cx.shared;

        (node, net).lock(|node, net| {
            let now = monotonics::now();
            let time = net.get_time(now);
            node.poll(time, |_, callback: &dyn Fn(&mut dyn usecop::io::Write)| {
                let socket = net.sockets.get_mut::<TcpSocket>(*sock);
                callback(&mut Writer(socket));
            });
            net.poll(now);
        });
        poll::spawn_after(500.millis().into()).unwrap();
    }
}

struct Writer<'w, 's>(&'w mut TcpSocket<'s>);

impl<'w, 's> usecop::io::Write for Writer<'w, 's> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, usecop::io::Error> {
        self.0.send_slice(buf)
              .map_err(|_| usecop::io::Error::new(usecop::io::ErrorKind::Other, ""))
    }

    fn flush(&mut self) -> Result<(), usecop::io::Error> {
        Ok(())
    }
}
