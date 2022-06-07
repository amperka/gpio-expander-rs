use std::error::Error;
use std::thread;
use std::time::Duration;

use gpio_expander::{prelude::*, GpioExpander};
use rppal::i2c::I2c;

fn main() -> Result<(), Box<dyn Error>> {
    // Initializing an I²C Peripheral from HAL
    let i2c = I2c::with_bus(1)?;

    // Initializing GpioExpander with default I²C address
    let mut expander = GpioExpander::new(i2c, None);
    let expander_pins = expander.pins();

    // Pin 0 into output mode
    let mut led = expander_pins.p00.into_output()?;

    loop {
        led.set_high()?;
        thread::sleep(Duration::from_secs(1));
        led.set_low()?;
        thread::sleep(Duration::from_secs(1));
    }
}
