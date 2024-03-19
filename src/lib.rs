#![no_std]

use stm32f4xx_hal::{
    gpio::{ErasedPin, Output},
    pac::RNG,
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

pub fn read_rand() -> u32 {
    // read a random number from the hardware generator
    unsafe {
        (*RNG::ptr()).dr.read().bits()
    }
}
