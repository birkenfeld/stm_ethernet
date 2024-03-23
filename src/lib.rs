#![no_std]

pub mod secnode;

use stm32f4xx_hal::{
    gpio::{ErasedPin, Output},
};

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
