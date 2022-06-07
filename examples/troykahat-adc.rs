use std::error::Error;
use std::thread;
use std::time::Duration;

use gpio_expander::GpioExpander;
use rppal::i2c::I2c;

fn main() -> Result<(), Box<dyn Error>> {
    // Initializing an I²C Peripheral from HAL
    let i2c = I2c::with_bus(1)?;

    // Initializing GpioExpander with default I²C address
    let mut expander = GpioExpander::new(i2c, None);
    let expander_pins = expander.pins();

    // Pin 0 into input mode
    let adc_in = expander_pins.p00.into_input()?;

    loop {
        println!("Pin 0 = {}", adc_in.get_analog()?);
        thread::sleep(Duration::from_millis(500));
    }
}
