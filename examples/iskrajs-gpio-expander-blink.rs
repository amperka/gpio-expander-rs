// Example for template github.com/amperka/iskrajs-rust-template

#![no_main]
#![no_std]
#![allow(unused_imports)]

use cortex_m_rt::entry;
use gpio_expander::{prelude::*, GpioExpander};
use panic_halt as _;
use stm32f4xx_hal::{
    gpio::{Output, Pin, PushPull},
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    if let (Some(device_periph), Some(_core_periph)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Configuration clocks
        let rcc = device_periph.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(168.MHz())
            .hclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .require_pll48clk()
            .freeze();

        // Configure I²C pins
        let gpiob = device_periph.GPIOB.split();
        let scl = gpiob
            .pb8
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        let sda = gpiob
            .pb9
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();

        // Initializing an I²C Peripheral from HAL
        let i2c = device_periph.I2C1.i2c((scl, sda), 400.kHz(), &clocks);

        // Initializing GpioExpander with default I²C address
        let mut expander = GpioExpander::new(i2c, None);
        let expander_pins = expander.pins();

        // Pin 0 into output mode
        let led = expander_pins.p00.into_output().unwrap();

        // Initializing SysClk delay
        let mut delay = core_periph.SYST.delay(&clocks);

        loop {
            led.set_high();
            delay.delay_ms(1000_u32);
            led.set_low();
            delay.delay_ms(1000_u32);
        }
    }

    loop {}
}
