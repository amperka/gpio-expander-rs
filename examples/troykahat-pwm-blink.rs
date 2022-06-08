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

    // Pin 0 into output mode
    let mut led = expander_pins.p00.into_output()?;

    let mut up = true;
    let mut n = 0;
    let mut delay_change = 10;

    loop {
        thread::sleep(Duration::from_millis(delay_change));

        if up {
            n += 200;
            if n == 65400 {
                up = false
            }
        } else {
            n -= 200;
            if n == 0 {
                up = true
            }
        }

        led.set_pwm(n)?;

        delay_change = (200 / (n as u64 + 4)) + 1;
    }
}
