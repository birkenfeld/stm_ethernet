// use stm32f4xx_hal::{
//     hal_02::adc::Channel,
//     adc::{Adc, config::{AdcConfig, SampleTime, Sequence, Eoc, Scan, Clock}},
// };
use usecop::{ModuleInternals, Result};

pub type SecNode<const N: usize> = usecop::node::SecNode<MyModules, N>;

pub fn create<const N: usize>() -> SecNode<N> {
    SecNode::new("rpi", "microSECoP demo on STM32F4", MyModules {
        temp: Temp { conversion: 0.001721,
                     internals: ModuleInternals::new("chip temperature", 5.0) },
    })
}

#[derive(usecop_derive::Modules)]
pub struct MyModules {
    temp: Temp,
}

#[derive(Clone, Copy, usecop_derive::DataInfo)]
enum TempStatus {
    Idle = 100,
}

#[derive(usecop_derive::Module)]
#[secop(interface = "Readable")]
#[secop(param(name = "value", doc = "main value of the temperature",
              readonly = true,
              datainfo(double(unit="degC"))))]
#[secop(param(name = "status", doc = "status of readout",
              readonly = true,
              datainfo(tuple(member(rust="TempStatus"),
                             member(str(maxchars=18))))))]
#[secop(param(name = "conversion", doc = "conversion factor",
              readonly = false, generate_accessors = true,
              datainfo(double())))]
#[secop(command(name = "buzz", doc = "buzz it!",
                argument(str(maxchars=10)),
                result(str(maxchars=10))))]
struct Temp {
    internals: ModuleInternals,
    conversion: f64,
}

impl Temp {
    fn read_value(&mut self) -> Result<f64> {
        let refv = 3.3;
        let adc_value: u16 = 0;
        let vbe = f64::from(adc_value) * refv / 4096.0;
        Ok(27.0 - (vbe - 0.706) / self.conversion)
    }

    fn read_status(&mut self) -> Result<(TempStatus, &str)> {
        Ok((TempStatus::Idle, "all good, trust me"))
    }

    fn do_buzz<'a>(&mut self, arg: &'a mut str) -> Result<&'a str> {
        defmt::info!("got buzzed: <{}>", arg);
        Ok(arg)
    }
}
