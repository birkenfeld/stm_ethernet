#![no_std]
#![no_main]

extern crate panic_itm;

use cortex_m::{iprintln, interrupt, peripheral};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::{
    gpio::GpioExt, hal::digital::v2::InputPin,
    stm32::{Peripherals, CorePeripherals, GPIOB, SYST, TIM2, RNG},
    time::U32Ext,
    rcc::RccExt,
};

use core::cell::Cell;
use arrayvec::ArrayVec;
use byteorder::{ByteOrder, LE};

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder, Routes};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer, RawSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::dhcp::Dhcpv4Client;
use log::{Record, Metadata, LevelFilter, info, warn};

use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry};

const PORT: u16 = 54321;
// absolute maximum events that can fit into a single packet
const MAX_PER_PKT: usize = 234;
// aimed filling of packets, leaving a bit of room to balance
const EVENTS_PER_PKT: u32 = 220;
// maximum interval between packets: 20 ms
const MAX_INTERVAL: u32 = 200_000;
// minimum interval between packets to avoid complete buffer saturation
const MIN_INTERVAL: u32 = 1_215;

struct ItmLogger;

fn itm() -> &'static mut peripheral::itm::Stim {
    unsafe { &mut (*peripheral::ITM::ptr()).stim[0] }
}

impl log::Log for ItmLogger {
    fn log(&self, record: &Record) {
        iprintln!(itm(), "[{}] {}", record.level(), record.args());
    }

    fn enabled(&self, _: &Metadata) -> bool { true }
    fn flush(&self) {}
}

static LOGGER: ItmLogger = ItmLogger;
static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    // enable logging if someone is listening on ITM
    if itm().is_fifo_ready() {
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(LevelFilter::Info);
    }

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_rng(&p);
    setup_10mhz(&p);

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(180.mhz()).hclk(180.mhz()).freeze();

    setup_systick(&mut cp.SYST);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    let pins = EthPins {
        ref_clk: gpioa.pa1,
        md_io: gpioa.pa2,
        md_clk: gpioc.pc1,
        crs: gpioa.pa7,
        tx_en: gpiog.pg11,
        tx_d0: gpiog.pg13,
        tx_d1: gpiob.pb13,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };

    // DHCP if user button pressed
    let dhcp_btn = gpioc.pc13.into_pull_down_input();
    let use_dhcp = dhcp_btn.is_high().unwrap();

    let _led_green = gpiob.pb0.into_push_pull_output();
    let _let_blue = gpiob.pb7.into_push_pull_output();
    let _let_red = gpiob.pb14.into_push_pull_output();

    // red LED: indicate "booting"
    set_leds(true, false, false);

    // set up ring buffers for network handling tokens
    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 16] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..],
        PhyAddress::_0,
        clocks,
        pins
    ).unwrap();

    // determine MAC address from board's serial number
    let serial = read_serno();
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);

    // select the default Mesytec IP if static configuration
    let mut ip_addrs = if !use_dhcp {
        [IpCidr::new(IpAddress::v4(192, 168, 168, 121), 24)]
    } else {
        [IpCidr::new(IpAddress::v4(0, 0, 0, 0), 0)]
    };
    let mut neighbor_storage = [None; 16];
    let mut routes_storage = [None; 2];
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(NeighborCache::new(&mut neighbor_storage[..]))
        .routes(Routes::new(&mut routes_storage[..]))
        .finalize();

    // set up buffers for packet content and metadata
    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 16];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*16];
    let mut dhcp_rx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_tx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_rx_data_buffer = [0; 1500];
    let mut dhcp_tx_data_buffer = [0; 1500*2];

    // create the UDP socket
    let udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    // set up buffers for DHCP handling
    let dhcp_rx_buffer = RawSocketBuffer::new(&mut dhcp_rx_meta_buffer[..], &mut dhcp_rx_data_buffer[..]);
    let dhcp_tx_buffer = RawSocketBuffer::new(&mut dhcp_tx_meta_buffer[..], &mut dhcp_tx_data_buffer[..]);
    let mut dhcp = Dhcpv4Client::new(&mut sockets, dhcp_rx_buffer, dhcp_tx_buffer, Instant::from_millis(0));

    let udp_handle = sockets.add(udp_socket);
    let mut cmd_buf = [0; 128];

    let mut gen = Generator::new(p.TIM2);

    info!("------------------------------------------------------------------------");

    if use_dhcp {
        // give the remote partner time to realize the link is up,
        // so we don't run into the 10sec DHCP discover interval
        while interrupt::free(|cs| ETH_TIME.borrow(cs).get()) < 2000 { }

        let mut got_offer = false;
        loop {
            let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
            if let Err(e) = iface.poll(&mut sockets, time) {
                warn!("poll: {}", e);
            }
            match dhcp.poll(&mut iface, &mut sockets, time) {
                Err(e) => warn!("dhcp: {}", e),
                Ok(Some(config)) => if let Some(cidr) = config.address {
                    iface.update_ip_addrs(|a| *a.first_mut().unwrap() = cidr.into());
                    if got_offer {
                        break;
                    }
                    got_offer = true;
                },
                _ => ()
            }
        }
    }

    let ip_addr = iface.ipv4_addr().unwrap();
    info!("IP setup done ({}), binding to {}:{}",
          if use_dhcp { "dhcp" } else { "static" }, ip_addr, PORT);
    sockets.get::<UdpSocket>(udp_handle).bind((ip_addr, PORT)).unwrap();
    set_leds(false, true, false);

    loop {
        // process packets
        {
            let mut socket = sockets.get::<UdpSocket>(udp_handle);
            while let Ok((n, ep)) = socket.recv_slice(&mut cmd_buf) {
                gen.process_command(&mut socket, &cmd_buf[..n], ep);
            }
            gen.maybe_send_data(&mut socket);
        }
        // handle ethernet
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        if let Err(e) = iface.poll(&mut sockets, time) {
            warn!("poll: {}", e);
        }
    }
}

struct Generator {
    endpoint: IpEndpoint,
    timer: TIM2,

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
    fn new(timer: TIM2) -> Self {
        // default rate: 1000 events/sec
        Generator { timer, endpoint: (IpAddress::v4(0, 0, 0, 0), PORT).into(),
                    mcpd_id: 0, run_id: 0, interval: 10_000, pkt_interval: 100_000,
                    buf_no: 0, time: 0, lastpkt: 0, run: false, npkt: [0; MAX_PER_PKT+1] }
    }

    fn process_command(&mut self, sock: &mut UdpSocket, msg: &[u8], ep: IpEndpoint) {
        // parse body: all command buffers end in 0xffff
        let req_body = msg[20..].chunks(2).map(|c| LE::read_u16(c))
                                          .take_while(|&v| v != 0xffff)
                                          .collect::<ArrayVec<[u16; 32]>>();
        let mut body = ArrayVec::<[u16; 24]>::new();
        let cmd = LE::read_u16(&msg[8..]);
        match cmd {
            22 => { // get MCPD-8 capabilities and mode
                info!("Get capabilities: {:?}", req_body);
                body.push(7); body.push(2);  // TOF + Pos/Amp selected
            }
            23 => { // set MCPD-8 mode
                info!("Set bus mode: {:?}", req_body);
                body.push(2);  // TOF + Pos/Amp selected
            }
            31 => { // write MCPD register
                info!("Write MCPD register: {:?}", req_body);
                if req_body[0] == 1 && req_body[1] == 103 {
                    body.push(1); body.push(103); body.push(2);
                }
            }
            32 => { // read MCPD register
                info!("Read MCPD register: {:?}", req_body);
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
                info!("Read MPSD register: {:?}", req_body);
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
                info!("Set timing setup: {:?}", req_body);
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
                info!("Set counter cells: {:?}", req_body);
            }
            10 => { // set aux timer
                info!("Set aux timer: {:?}", req_body);
            }
            11 => { // set parameter source
                info!("Set param source: {:?}", req_body);
            }
            12 => { // get all parameters
                info!("CMD 12 not implemented, returning zeros");
                for _ in 0..21 {
                    body.push(0);
                }
            }
            13 => { // set gain
                info!("set gain: {:?}", req_body);
                body.push(req_body[0]);
                body.push(req_body[1]);
                body.push(req_body[2]);
            }
            14 => { // set threshhold
                info!("Set threshhold: {:?}", req_body);
                body.push(req_body[0]);
                body.push(req_body[1]);
            }
            16 => { // set mode
                info!("Set threshhold: {:?}", req_body);
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
            Err(e) => warn!("send: {}", e),
        }
    }

    fn start(&mut self) {
        self.run = true;
        self.time = 0;
        self.lastpkt = 0;
        self.npkt.iter_mut().for_each(|p| *p = 0);
        // LED blue is "running"
        set_leds(false, true, true);
        // reset the timer
        self.timer.cnt.write(|w| unsafe { w.bits(0) });
        self.timer.cr1.write(|w| w.cen().set_bit());
    }

    fn stop(&mut self) {
        self.run = false;
        self.timer.cr1.write(|w| w.cen().clear_bit());
        set_leds(false, true, false);
        let npkt = self.npkt.iter().sum::<u32>();
        let nevt = self.npkt.iter().enumerate().map(|(i, &n)| i as u64 * n as u64).sum::<u64>();
        info!("generated {} pkts, {} evts in {} ticks", npkt, nevt, self.time);
        info!("event rate: {:.0}/s", nevt as f64 / (self.time as f64 / 10_000_000.));
        info!("per size:");
        for (sz, &n) in self.npkt.iter().enumerate() {
            if n != 0 {
                info!("{} - {}", sz, n);
            }
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    // systick is used for advancing the Ethernet clock for timeouts etc.
    syst.set_reload(22_500 - 1); // every ms
    syst.enable_counter();
    syst.enable_interrupt();
}

fn set_leds(red: bool, green: bool, blue: bool) {
    let gpiob = unsafe { &(*GPIOB::ptr()) };
    gpiob.odr.modify(|_, w| w.odr0().bit(green).odr7().bit(blue).odr14().bit(red));
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

fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

fn read_rand() -> u32 {
    // read a random number from the hardware generator
    unsafe {
        (*RNG::ptr()).dr.read().bits()
    }
}

#[exception]
fn SysTick() {
    interrupt::free(|cs| {
        let time = ETH_TIME.borrow(cs);
        time.set(time.get().wrapping_add(1));
    });
}
