#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal::{
    self as hal,
    prelude::*,
    gpio::GpioExt,
    pac::{Peripherals, TIM2},
    rcc::RccExt,
};
use systick_monotonic::Systick;

use arrayvec::ArrayVec;
use byteorder::{ByteOrder, LE};

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpCidr, IpAddress, IpEndpoint};
use smoltcp::iface::{SocketSet, SocketHandle, Config, Interface, SocketStorage};
use smoltcp::socket::udp::{Socket as UdpSocket, PacketBuffer as UdpPacketBuffer, UdpMetadata};
use smoltcp::socket::dhcpv4::{Socket as DhcpSocket, Event as DhcpEvent};
use smoltcp::storage::PacketMetadata;

use stm32_eth::{dma::{EthernetDMA, RxRingEntry, TxRingEntry}, Parts, PartsIn, EthPins};

use stm_ethernet::{Leds, read_serno, read_rand};

const TIME_GRANULARITY: u32 = 1000;  // 1000 Hz granularity
const PORT: u16 = 54321;
// absolute maximum events that can fit into a single packet
const MAX_PER_PKT: usize = 234;
// aimed filling of packets, leaving a bit of room to balance
const EVENTS_PER_PKT: u32 = 220;
// maximum interval between packets: 20 ms
const MAX_INTERVAL: u32 = 200_000;
// minimum interval between packets to avoid complete buffer saturation
const MIN_INTERVAL: u32 = 1_215;

fn monotonic_millis() -> u32 {
    app::monotonics::now()
        .duration_since_epoch()
        .to_millis()
        .try_into()
        .unwrap()
}

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
        sockets: SocketSet<'static>,
        iface: Interface,
        dma: EthernetDMA<'static, 'static>,
        use_dhcp: bool,
    }

    #[local]
    struct Local {
        gen: Generator,
        udp_handle: SocketHandle,
    }

    #[init(local = [
        rx_ring: [RxRingEntry; 16] = [RxRingEntry::RX_INIT; 16],
        tx_ring: [TxRingEntry; 4] = [TxRingEntry::INIT; 4],
        udp_rx_meta_buffer: [PacketMetadata<UdpMetadata>; 4] = [PacketMetadata::EMPTY; 4],
        udp_tx_meta_buffer: [PacketMetadata<UdpMetadata>; 16] = [PacketMetadata::EMPTY; 16],
        udp_rx_data_buffer: [u8; 1500*4] = [0; 1500*4],
        udp_tx_data_buffer: [u8; 1500*16] = [0; 1500*16],
        sockets_storage: [SocketStorage<'static>; 2] = [SocketStorage::EMPTY, SocketStorage::EMPTY],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p = cx.device;

        setup_rng(&p);
        setup_10mhz(&p);

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
            &mut cx.local.rx_ring[..],
            &mut cx.local.tx_ring[..],
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
        let udp_socket = UdpSocket::new(
            UdpPacketBuffer::new(&mut cx.local.udp_rx_meta_buffer[..], &mut cx.local.udp_rx_data_buffer[..]),
            UdpPacketBuffer::new(&mut cx.local.udp_tx_meta_buffer[..], &mut cx.local.udp_tx_data_buffer[..])
        );
        let mut sockets = SocketSet::new(&mut cx.local.sockets_storage[..]);

        let udp_handle = sockets.add(udp_socket);

        let gen = Generator::new(p.TIM2, leds);

        // use systick monotonic clock for now
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());

        info!("------------------------------------------------------------------------");
        if use_dhcp {
            dhcp::spawn().unwrap();
        } else {
            main::spawn().unwrap();
        }
        (Shared { sockets, iface, dma, use_dhcp },
         Local { gen, udp_handle },
         init::Monotonics(mono))
    }

    #[task(shared = [sockets, iface, dma])]
    fn dhcp(cx: dhcp::Context) {
        let dhcp::SharedResources { sockets, iface, dma } = cx.shared;

        // give the remote partner time to realize the link is up,
        // so we don't run into the 10sec DHCP discover interval
        while monotonic_millis() < 2000 { }

        (sockets, iface, dma).lock(|sockets, iface, mut dma| {
            info!("Starting DHCP");
            let dhcp_socket = DhcpSocket::new();
            let dhcp_handle = sockets.add(dhcp_socket);

            loop {
                let time = Instant::from_millis(monotonic_millis());
                iface.poll(time, &mut dma, sockets);

                let event = sockets.get_mut::<DhcpSocket>(dhcp_handle).poll();
                if let Some(DhcpEvent::Configured(config)) = event {
                    let addr = config.address;
                    iface.update_ip_addrs(|addrs| addrs.push(addr.into()).unwrap());

                    if let Some(router) = config.router {
                        iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        iface.routes_mut().remove_default_ipv4_route();
                    }

                    break;
                }
            }

        });
        main::spawn().unwrap();
    }

    #[task(local = [gen, udp_handle], shared = [sockets, iface, dma, &use_dhcp])]
    fn main(cx: main::Context) {
        let main::LocalResources { gen, udp_handle } = cx.local;
        let main::SharedResources { sockets, iface, dma, use_dhcp } = cx.shared;
        let mut cmd_buf = [0; 128];

        (sockets, iface, dma).lock(|sockets, iface, mut dma| {
            let ip_addr = iface.ipv4_addr().unwrap();
            info!("IP setup done ({}), binding to {}:{}",
                  if *use_dhcp { "dhcp" } else { "static" }, ip_addr, PORT);
            sockets.get_mut::<UdpSocket>(*udp_handle).bind((ip_addr, PORT)).unwrap();
            gen.leds.set(false, true, false);

            loop {
                // process packets
                {
                    let socket = sockets.get_mut::<UdpSocket>(*udp_handle);
                    while let Ok((n, ep)) = socket.recv_slice(&mut cmd_buf) {
                        gen.process_command(socket, &cmd_buf[..n], ep.endpoint);
                    }
                    gen.maybe_send_data(socket);
                }
                // handle ethernet
                let time = Instant::from_millis(monotonic_millis());
                iface.poll(time, &mut dma, sockets);
            }
        });
    }
}

struct Generator {
    endpoint: IpEndpoint,
    timer: TIM2,
    leds: Leds,

    mcpd_id: u8,
    run_id: u16,
    interval: u32,     // 10MHz times between events
    pkt_interval: u32, // 10MHz times between packets

    buf_no: u16,
    run: bool,
    time: u64,
    lastpkt: u64,
    npkt: [u32; MAX_PER_PKT+1],
}

impl Generator {
    fn new(timer: TIM2, leds: Leds) -> Self {
        // default rate: 1000 events/sec
        Generator { timer, leds, endpoint: (IpAddress::v4(0, 0, 0, 0), PORT).into(),
                    mcpd_id: 0, run_id: 0, interval: 10_000, pkt_interval: 100_000,
                    buf_no: 0, time: 0, lastpkt: 0, run: false, npkt: [0; MAX_PER_PKT+1] }
    }

    fn process_command(&mut self, sock: &mut UdpSocket, msg: &[u8], ep: IpEndpoint) {
        // parse body: all command buffers end in 0xffff
        let req_body = msg[20..].chunks(2).map(|c| LE::read_u16(c))
                                          .take_while(|&v| v != 0xffff)
                                          .collect::<ArrayVec<u16, 32>>();
        let mut body = ArrayVec::<u16, 24>::new();
        let cmd = LE::read_u16(&msg[8..]);
        match cmd {
            22 => { // get MCPD-8 capabilities and mode
                info!("Get capabilities: {:?}", req_body[..]);
                body.push(7); body.push(2);  // TOF + Pos/Amp selected
            }
            23 => { // set MCPD-8 mode
                info!("Set bus mode: {:?}", req_body[..]);
                body.push(2);  // TOF + Pos/Amp selected
            }
            31 => { // write MCPD register
                info!("Write MCPD register: {:?}", req_body[..]);
                if req_body[0] == 1 && req_body[1] == 103 {
                    body.push(1); body.push(103); body.push(2);
                }
            }
            32 => { // read MCPD register
                info!("Read MCPD register: {:?}", req_body[..]);
                if req_body[0] == 1 && req_body[1] == 102 {
                    body.push(1); body.push(102); body.push(2);
                } else if req_body[0] == 1 && req_body[1] == 103 {
                    body.push(1); body.push(102); body.push(2);
                }
            }
            36 => { // get hw_types
                info!("Get hardware types");
                for _ in 0..8 { body.push(103); }
            }
            51 => { // get version information
                info!("Get version information");
                body.push(3); body.push(4); body.push(0x0102);
            }
            52 => { // read MPSD-8 register
                info!("Read MPSD register: {:?}", req_body[..]);
                body.push(req_body[0]);
                body.push(req_body[1]);
                body.push(match req_body[1] {
                    0 => 7,
                    1 => 2,
                    2 => 0x0101,
                    _ => 0,
                });
            }
            0 | 2 => { // reset or stop DAQ
                info!("Stopped");
                self.stop();
            }
            1 | 3 => { // start or continue DAQ
                info!("Started");
                if self.endpoint.addr.is_unspecified() {
                    // to avoid needing reconfiguration of the IP all the time
                    self.endpoint = ep;
                }
                self.start();
            }
            4 => { // set MCPD id
                info!("MCPD id set to {}", req_body[0]);
            }
            5 => { // set protocol parameters
                info!("Set protocol parameters");
                if req_body[0] != 0 {
                    info!("  requested IP change to {}.{}.{}.{}",
                          req_body[0], req_body[1], req_body[2], req_body[3]);
                }
                if req_body[4] != 0 {
                    let port = if req_body[9] != 0 { req_body[9] } else { 54321 };
                    info!("  events should go to {}.{}.{}.{}:{}",
                          req_body[4], req_body[5], req_body[6], req_body[7], port);
                    self.endpoint = (IpAddress::v4(req_body[4] as u8,
                                                   req_body[5] as u8,
                                                   req_body[6] as u8,
                                                   req_body[7] as u8), port).into();
                }
                if req_body[4] == 0 && req_body[5] == 0 {
                    info!("  events should go to sender");
                    self.endpoint = ep;
                }
                if req_body[10] != 0 {
                    info!("  accept commands only from {}.{}.{}.{}:{}",
                          req_body[10], req_body[11], req_body[12], req_body[13],
                          if req_body[8] != 0 { req_body[8] } else { 54321 });
                }
                if req_body[10] == 0 && req_body[11] == 0 {
                    info!("  accept commands only from sender");
                }
            }
            6 => { // set timing setup
                info!("Set timing setup: {:?}", req_body[..]);
            }
            7 => { // set master clock value
                info!("Master clock is {}",
                      req_body[0] as u64 | (req_body[1] as u64) << 16 | (req_body[2] as u64) << 32);
            }
            8 => { // set run ID
                self.run_id = req_body[0];
                info!("Run ID set to {}", self.run_id);
            }
            9 => { // set counter cells
                info!("Set counter cells: {:?}", req_body[..]);
            }
            10 => { // set aux timer
                info!("Set aux timer: {:?}", req_body[..]);
            }
            11 => { // set parameter source
                info!("Set param source: {:?}", req_body[..]);
            }
            12 => { // get all parameters
                info!("CMD 12 not implemented, returning zeros");
                for _ in 0..21 {
                    body.push(0);
                }
            }
            13 => { // set gain
                info!("set gain: {:?}", req_body[..]);
                body.push(req_body[0]);
                body.push(req_body[1]);
                body.push(req_body[2]);
            }
            14 => { // set threshhold
                info!("Set threshhold: {:?}", req_body[..]);
                body.push(req_body[0]);
                body.push(req_body[1]);
            }
            16 => { // set mode
                info!("Set threshhold: {:?}", req_body[..]);
                body.push(req_body[0]);
                body.push(req_body[1]);
            }
            0xF1F0 => { // generator parameters -- generator specific command
                let rate = (req_body[1] as u32) << 16 | req_body[0] as u32;
                if rate == 0 {
                    self.interval = 400_000;
                } else {
                    self.interval = 10_000_000 / rate;
                }
                self.pkt_interval = self.interval * EVENTS_PER_PKT;
                if self.pkt_interval > MAX_INTERVAL {
                    self.pkt_interval = MAX_INTERVAL;
                } else if self.pkt_interval < MIN_INTERVAL {
                    self.pkt_interval = MIN_INTERVAL;
                }
                info!("Configure: set rate to {}/s, events/packet to {}",
                      10_000_000 / self.interval, self.pkt_interval / self.interval);
            }
            _ => { // unknown
                info!("CMD {} not handled...", cmd);
                return;
            }
        }
        body.push(0xffff);

        // send back reply
        if let Ok(buf) = sock.send(20 + 2*body.len(), ep) {
            LE::write_u16(&mut buf[0..], (10 + body.len()) as u16);
            buf[2..12].copy_from_slice(&msg[2..12]);
            LE::write_u16(&mut buf[12..], self.time as u16);
            LE::write_u16(&mut buf[14..], (self.time >> 16) as u16);
            LE::write_u16(&mut buf[16..], (self.time >> 32) as u16);
            buf[18..20].copy_from_slice(&[0, 0]);
            for (i, val) in body.into_iter().enumerate() {
                LE::write_u16(&mut buf[20+i*2..22+i*2], val);
            }
            let cksum = buf.chunks(2).fold((0, 0), |acc, item| (acc.0 ^ item[0],
                                                                acc.1 ^ item[1]));
            buf[18] = cksum.0;
            buf[19] = cksum.1;
        }
    }

    fn maybe_send_data(&mut self, sock: &mut UdpSocket) {
        if !self.run {
            return;
        }

        // keep track of 64-bit time
        let low_time = self.timer.cnt.read().bits();
        let overflow = if low_time < self.time as u32 { 1 << 32 } else { 0 };
        self.time = ((self.time & 0xFFFF_FFFF_0000_0000) + overflow) | low_time as u64;

        // calculate time since last packet
        let elapsed = (self.time - self.lastpkt) as u32;
        if elapsed < self.pkt_interval {
            return;
        }
        let mut nevents = (elapsed / self.interval) as usize;
        if nevents > MAX_PER_PKT {
            nevents = MAX_PER_PKT;
        }

        // now we can send a packet
        match sock.send(42 + 6*nevents, self.endpoint) {
            Ok(buf) => {
                self.buf_no += 1;
                LE::write_u16(&mut buf[0..], 21 + 3*nevents as u16);
                LE::write_u16(&mut buf[2..], 0);
                LE::write_u16(&mut buf[4..], 21);
                LE::write_u16(&mut buf[6..], self.buf_no);
                LE::write_u16(&mut buf[8..], self.run_id);
                buf[10] = 1;  // DAQ running
                buf[11] = self.mcpd_id;
                LE::write_u16(&mut buf[12..], self.time as u16);
                LE::write_u16(&mut buf[14..], (self.time >> 16) as u16);
                LE::write_u16(&mut buf[16..], (self.time >> 32) as u16);
                buf[18..42].copy_from_slice(&[0; 24]);
                let mut offset = 42 + 2;
                for _ in 0..nevents {
                    let (y, x) = loop {
                        let random = read_rand();
                        let y = (random >> 22) as u16;
                        if y < 960 {
                            break (y, random as u16 & 0b11100111);
                        }
                    };
                    LE::write_u16(&mut buf[offset..], y << 3);
                    LE::write_u16(&mut buf[offset+2..], x << 7);
                    offset += 6;
                }
                self.lastpkt = self.time - (elapsed % self.interval) as u64;
                self.npkt[nevents] += 1;
            }
            Err(e) => warn!("send: {:?}", e),
        }
    }

    fn start(&mut self) {
        self.run = true;
        self.time = 0;
        self.lastpkt = 0;
        self.npkt.iter_mut().for_each(|p| *p = 0);
        // LED blue is "running"
        self.leds.set(false, true, true);
        // reset the timer
        self.timer.cnt.write(|w| w.bits(0));
        self.timer.cr1.write(|w| w.cen().set_bit());
    }

    fn stop(&mut self) {
        self.run = false;
        self.timer.cr1.write(|w| w.cen().clear_bit());
        self.leds.set(false, true, false);
        let npkt = self.npkt.iter().sum::<u32>();
        let nevt = self.npkt.iter().enumerate().map(|(i, &n)| i as u64 * n as u64).sum::<u64>();
        info!("generated {} pkts, {} evts in {} ticks", npkt, nevt, self.time);
        info!("event rate: {}/s", nevt as f64 / (self.time as f64 / 10_000_000.));
        info!("per size:");
        for (sz, &n) in self.npkt.iter().enumerate() {
            if n != 0 {
                info!("{} - {}", sz, n);
            }
        }
    }
}

fn setup_rng(p: &Peripherals) {
    p.RCC.ahb2enr.modify(|_, w| w.rngen().set_bit());
    p.RNG.cr.modify(|_, w| w.rngen().set_bit());
}

fn setup_10mhz(p: &Peripherals) {
    p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
    p.TIM2.psc.write(|w| w.psc().bits(8)); // 90 MHz/9
    p.TIM2.egr.write(|w| w.ug().set_bit());
}
