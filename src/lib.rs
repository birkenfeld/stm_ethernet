#![no_std]

use fugit::Instant;
use stm32f4xx_hal::gpio::{ErasedPin, Output};
use smoltcp::iface::{SocketSet, Interface};
use smoltcp::time::Instant as SmoltcpInstant;
use stm32_eth::dma::EthernetDMA;

pub struct Leds {
    r: ErasedPin<Output>,
    g: ErasedPin<Output>,
    b: ErasedPin<Output>,
}

impl Leds {
    pub fn new(r: ErasedPin<Output>, g: ErasedPin<Output>, b: ErasedPin<Output>) -> Self {
        Self { r, g, b }
    }

    pub fn set(&mut self, r: bool, g: bool, b: bool) {
        if r { self.r.set_high(); } else { self.r.set_low(); }
        if g { self.g.set_high(); } else { self.g.set_low(); }
        if b { self.b.set_high(); } else { self.b.set_low(); }
    }
}

pub fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

pub struct Net<const TIME_GRANULARITY: u32> {
    pub sockets: SocketSet<'static>,
    pub iface: Interface,
    pub dma: EthernetDMA<'static, 'static>,
    pub ntp_time: Option<f64>,
}

impl<const TIME_GRANULARITY: u32> Net<TIME_GRANULARITY> {
    pub fn poll<const NOM: u32, const DENOM: u32>(&mut self, now: Instant<u64, NOM, DENOM>) {
        let time = SmoltcpInstant::from_millis(now.ticks() as i64);
        self.iface.poll(time, &mut &mut self.dma, &mut self.sockets);
    }
}
