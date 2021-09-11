# hdc1080-embedded-rs - Driver for sensor temperature and humidity


Made on the model of a crate for [HDC20xx](https://github.com/eldruin/hdc20xx-rs)


[HDC1080 referenc](https://www.ti.com/lit/ds/symlink/hdc1080.pdf)

Designed for use with [`embedded-hal`]

## Features
- Reading data from a sensor with one request `read()`
- Separate commands for reading temperature and humidity `temperature()` and `humidity()`
- Getting information about the sensor: `get_device_id()`, `get_man_id()`, `get_serial_id()`
- Low battery check: `battery_low()`

## Usage
```rust
#![deny(unsafe_code)]
#![no_main]
#![no_std]
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate nb;
extern crate stm32g0xx_hal as hal;

use hal::{prelude::*, stm32, time::Hertz, i2c::Config};
use nb::block;
use rt::entry;
use embedded_hdc1080_rs::Hdc1080;

fn main() {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut delay = dp.TIM14.delay(&mut rcc);
    let mut led = gpioc.pc6.into_push_pull_output();
    let sda = gpioa.pa10.into_open_drain_output();
    let scl = gpioa.pa9.into_open_drain_output();
    let mut temp:f32;
    let mut hum:f32;

    let mut timer = dp.TIM17.timer(&mut rcc);

    let conf:Config = Config::new(100.khz());

    let mut i2c = dp
        .I2C1
        .i2c(sda, scl, conf, &mut rcc);
    let mut dev = Hdc1080::new(i2c, delay).unwrap();
    
    dev.init().unwrap();
    //hprintln!("device ID {:?}", dev.get_device_id().unwrap());
    //hprintln!("manufacturer id {:?}", dev.get_man_id().unwrap());
    //hprintln!("Serial ID {:?}", dev.get_serial_id().unwrap());
    //hprintln!("curent config {:?}", dev.read_config().unwrap());

    timer.start(500.ms());
    loop {
        led.toggle().unwrap();
        let (temp, hum) = dev.read().unwrap();
        //hprintln!("temperature {}", temp);
        //hprintln!("humidity {}", hum);
        block!(timer.wait()).unwrap();
    }
}
```

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in the github project](https://github.com/maxmarvil/hdc1080-embedded-rs/issues).

## License

