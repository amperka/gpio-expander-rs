//! This is a crate that provides a general abstraction for an I²C port expander used in the [`GPIO Port Expander (Troyka Module)`], [`Troyka HAT`] and [`Slot Expander Expansion Board`] products. This abstraction is not necessarily the most performant, but it allows pins to be used in the same way as direct GPIOs. Because pin types also implement [`embedded-hal`] digital I/O characteristics, they can also be passed to subsequent drivers (for example, as a reset or chip select pin).
//!
//! Это крейт, предоставляющий общую абстракцию для I²C расширителя портов, используемый в продуктах [`Расширитель GPIO-портов (Troyka-модуль)`], [`Troyka HAT`] и [`Плата расширения Slot Expander`]. Эта абстракция не обязательно самая производительная, но она позволяет использовать контакты так же, как прямые GPIO из HAL. Поскольку типы выводов также реализуют характеристики [`embedded-hal`] цифрового ввода-вывода, они также могут быть переданы последующим драйверам (например, в качестве вывода сброса или выбора микросхемы).
//!
//! ## Example (for TroykaHAT)
//!
//! ```no_run
//! use std::error::Error;
//! use std::thread;
//! use std::time::Duration;
//!
//! use gpio_expander::{prelude::*, GpioExpander};
//! use rppal::i2c::I2c;
//!
//! fn main() -> Result<(), Box<dyn Error>> {
//!     // Initializing an I²C Peripheral from HAL
//!     let i2c = I2c::with_bus(1)?;
//!
//!     // Initializing GpioExpander with default I²C address
//!     let mut expander = GpioExpander::new(i2c, None);
//!     let expander_pins = expander.pins();
//!
//!     // Pin 0 into output mode
//!     let mut led = expander_pins.p00.into_output()?;
//!
//!     loop {
//!         led.set_high()?;
//!         thread::sleep(Duration::from_secs(1));
//!         led.set_low()?;
//!         thread::sleep(Duration::from_secs(1));
//!     }
//! }
//! ```
//!
//! More examples in the [`examples`] folder.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//! [`Расширитель GPIO-портов (Troyka-модуль)`]: https://amperka.ru/product/troyka-gpio-expander
//! [`Troyka HAT`]: https://amperka.ru/product/raspberry-pi-troyka-hat
//! [`Плата расширения Slot Expander`]: https://amperka.ru/product/slot-expander
//! [`GPIO Port Expander (Troyka Module)`]: https://amperka.ru/product/troyka-gpio-expander
//! [`Troyka HAT`]: https://amperka.com/modules/troyka-hat
//! [`Slot Expander Expansion Board`]: https://amperka.ru/product/slot-expander
//! [`examples`]: https://github.com/amperka/gpio-expander-rs/examples

#![deny(missing_docs)]
#![deny(unused_results)]
#![forbid(unsafe_code)]
#![cfg_attr(not(test), no_std)]

use core::marker::PhantomData;
use embedded_hal::{blocking::i2c, digital::v2};

/// Default I²C address
///
/// Адрес I²C по умолчанию
pub const DEFAULT_I2C_ADDRESS: u8 = 0x2a;

// GpioExpander commands
//
// Команды GpioExpander
#[allow(dead_code)]
#[repr(u8)]
enum GpioExpanderCommand {
    // Get an ID
    //
    // Получить идентификатор
    Uid = 0,

    // Reset
    //
    // Сброс
    Reset,

    // Change I²C address
    //
    // Измененить адрес I²C
    ChangeI2CAddress,

    // Save the current I²C address in flash memory
    //
    // Сохранить текущий I²C адрес во флэш-памяти
    SaveI2CAddress,

    // Set input mode (Digital and Analog)
    //
    // Установить пин как вход (Цифровой и Аналоговы)
    Input,

    // Set input pullup mode
    //
    // Установить пин как Pull Up вход
    InputPullUp,

    // Set input pulldown mode
    //
    // Установить пин как Pull Down вход
    InputPullDown,

    // Set output mode (Digital and Pwm)
    //
    // Установить пин как выход (Цифровой и ШИМ)
    Output,

    // Read the boolean state of the pin
    //
    // Прочитать булево состояние пина
    DigitalRead,

    // Set a high pin level
    //
    // Установить высокий уровень пина
    DigitalWriteHigh,

    // Set a low pin level
    //
    // Установить низкий уровень пина
    DigitalWriteLow,

    // Set the PWM value
    //
    // Установить значение ШИМ
    PwmWrite,

    // Read the analog pin state
    //
    // Прочитать аналоговое состояние пина
    AnalogRead,

    // Set the PWM frequency (for all pins)
    //
    // Установить частоту ШИМ (для всех пинов)
    PwmFreq,

    // Set the ADC reading speed (for all pins)
    //
    // Установить скорость считывания АЦП (для всех пинов)
    AdcSpeed,
}

/// Floating input (type state)
///
/// Плавающий вход (тип состояние)
#[derive(Debug)]
pub struct Floating;

/// Input mode (type state)
///
/// Выход (тип состояние)
#[derive(Debug)]
pub struct Input;

/// Output mode (type state)
///
/// Вход (тип состояние)
#[derive(Debug)]
pub struct Output;

/// GpioExpander device driver
///
/// Драйвер устройства GpioExpander
#[derive(Debug)]
pub struct GpioExpander<MUTEX>(MUTEX);

impl<I2C, ERROR> GpioExpander<shared_bus::NullMutex<Driver<I2C>>>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
{
    /// Create new instance of the GpioExpander
    ///
    /// Создание нового экземпляра GpioExpander
    pub fn new(i2c: I2C, address: Option<u8>) -> Self {
        Self::with_mutex(i2c, address)
    }
}

impl<I2C, MUTEX, ERROR> GpioExpander<MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    /// Create new instance of the GpioExpander
    ///
    /// Создание нового экземпляра GpioExpander
    pub fn with_mutex(i2c: I2C, address: Option<u8>) -> Self {
        Self(shared_bus::BusMutex::create(Driver::new(
            i2c,
            address.unwrap_or(DEFAULT_I2C_ADDRESS),
        )))
    }

    /// Get all I/O pins
    ///
    /// Получить все пины
    pub fn pins(&mut self) -> Pins<I2C, MUTEX, ERROR> {
        Pins {
            p00: Pin::new(0, &self.0),
            p01: Pin::new(1, &self.0),
            p02: Pin::new(2, &self.0),
            p03: Pin::new(3, &self.0),
            p04: Pin::new(4, &self.0),
            p05: Pin::new(5, &self.0),
            p06: Pin::new(6, &self.0),
            p07: Pin::new(7, &self.0),
            p08: Pin::new(8, &self.0),
            p09: Pin::new(9, &self.0),
        }
    }

    /// Reset GpioExpander
    ///
    /// Сбросить GpioExpander
    pub fn reset(&mut self) -> Result<(), ERROR> {
        self.0.lock(|drv| drv.reset())?;

        Ok(())
    }

    /// Change I²C address
    ///
    /// Измененить I²C адрес
    pub fn set_address(&mut self, new_address: u8) -> Result<(), ERROR> {
        self.0.lock(|drv| drv.set_i2c_address(new_address))?;

        Ok(())
    }

    /// Save the current I²C address in flash memory
    ///
    /// Сохранить текущий I²C адрес во флэш-памяти
    pub fn save_address(&mut self) -> Result<(), ERROR> {
        self.0.lock(|drv| drv.save_i2c_address())?;

        Ok(())
    }
}

/// Driver
pub struct Driver<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, ERROR> Driver<I2C>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
{
    pub(crate) fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    pub(crate) fn pin_to_input(&mut self, pin: u8) -> Result<(), ERROR> {
        let raw_pin_num = (1u16 << pin as u16).to_be_bytes();

        self.i2c.write(
            self.address,
            &[
                GpioExpanderCommand::Input as u8,
                raw_pin_num[0],
                raw_pin_num[1],
            ],
        )?;

        Ok(())
    }

    pub(crate) fn pin_to_pull_up_input(&mut self, pin: u8) -> Result<(), ERROR> {
        let raw_pin_num = (1u16 << pin as u16).to_be_bytes();

        self.i2c.write(
            self.address,
            &[
                GpioExpanderCommand::InputPullUp as u8,
                raw_pin_num[0],
                raw_pin_num[1],
            ],
        )?;

        Ok(())
    }

    pub(crate) fn pin_to_pull_down_input(&mut self, pin: u8) -> Result<(), ERROR> {
        let raw_pin_num = (1u16 << pin as u16).to_be_bytes();

        self.i2c.write(
            self.address,
            &[
                GpioExpanderCommand::InputPullDown as u8,
                raw_pin_num[0],
                raw_pin_num[1],
            ],
        )?;

        Ok(())
    }

    pub(crate) fn pin_to_output(&mut self, pin: u8) -> Result<(), ERROR> {
        let raw_pin_num = (1u16 << pin as u16).to_be_bytes();

        self.i2c.write(
            self.address,
            &[
                GpioExpanderCommand::Output as u8,
                raw_pin_num[0],
                raw_pin_num[1],
            ],
        )?;

        Ok(())
    }

    pub(crate) fn digital_read(&mut self, pin: u8) -> Result<bool, ERROR> {
        let mask = 1u16 << pin as u16;

        let mut read_buf = [0u8; 2];
        self.i2c
            .write(self.address, &[GpioExpanderCommand::DigitalRead as u8])?;
        self.i2c.read(self.address, &mut read_buf)?;

        Ok((u16::from_be_bytes(read_buf) & mask) != 0)
    }

    pub(crate) fn digital_write(&mut self, pin: u8, value: bool) -> Result<(), ERROR> {
        let raw_pin_num = (1u16 << pin as u16).to_be_bytes();

        if value {
            self.i2c.write(
                self.address,
                &[
                    GpioExpanderCommand::DigitalWriteHigh as u8,
                    raw_pin_num[0],
                    raw_pin_num[1],
                ],
            )?;
        } else {
            self.i2c.write(
                self.address,
                &[
                    GpioExpanderCommand::DigitalWriteLow as u8,
                    raw_pin_num[0],
                    raw_pin_num[1],
                ],
            )?;
        }

        Ok(())
    }

    pub(crate) fn analog_read(&mut self, pin: u8) -> Result<u16, ERROR> {
        let write_buf = [GpioExpanderCommand::AnalogRead as u8, pin];
        let mut read_buf = [0u8; 2];

        self.i2c
            .write_read(self.address, &write_buf, &mut read_buf)?;

        Ok(u16::from_be_bytes(read_buf) as u16)
    }

    pub(crate) fn pwm_write(&mut self, pin: u8, value: u16) -> Result<(), ERROR> {
        let raw_value: [u8; 2] = value.to_be_bytes();

        self.i2c.write(
            self.address,
            &[
                GpioExpanderCommand::PwmWrite as u8,
                pin,
                raw_value[0],
                raw_value[1],
            ],
        )?;

        Ok(())
    }

    pub(crate) fn reset(&mut self) -> Result<(), ERROR> {
        self.i2c
            .write(self.address, &[GpioExpanderCommand::Reset as u8])?;

        Ok(())
    }

    pub(crate) fn set_i2c_address(&mut self, new_address: u8) -> Result<(), ERROR> {
        self.i2c.write(
            self.address,
            &[GpioExpanderCommand::ChangeI2CAddress as u8, new_address],
        )?;

        self.address = new_address;

        Ok(())
    }

    pub(crate) fn save_i2c_address(&mut self) -> Result<(), ERROR> {
        self.i2c
            .write(self.address, &[GpioExpanderCommand::SaveI2CAddress as u8])?;

        Ok(())
    }
}

/// GPIO
///
/// Пины ввода/вывода
pub struct Pins<'a, I2C, MUTEX, ERROR>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    /// I/O pin 0
    pub p00: Pin<'a, Floating, MUTEX>,

    /// I/O pin 1
    pub p01: Pin<'a, Floating, MUTEX>,

    /// I/O pin 2
    pub p02: Pin<'a, Floating, MUTEX>,

    /// I/O pin 3
    pub p03: Pin<'a, Floating, MUTEX>,

    /// I/O pin 4
    pub p04: Pin<'a, Floating, MUTEX>,

    /// I/O pin 5
    pub p05: Pin<'a, Floating, MUTEX>,

    /// I/O pin 6
    pub p06: Pin<'a, Floating, MUTEX>,

    /// I/O pin 7
    pub p07: Pin<'a, Floating, MUTEX>,

    /// I/O pin 8
    pub p08: Pin<'a, Floating, MUTEX>,

    /// I/O pin 9
    pub p09: Pin<'a, Floating, MUTEX>,
}

/// Pin
pub struct Pin<'a, MODE, MUTEX> {
    pin: u8,
    driver: &'a MUTEX,
    _mode: PhantomData<MODE>,
}

impl<'a, MODE, MUTEX, I2C, ERROR> Pin<'a, MODE, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    pub(crate) fn new(pin: u8, driver: &'a MUTEX) -> Self {
        Self {
            pin,
            driver,
            _mode: PhantomData,
        }
    }
}

impl<'a, MODE, MUTEX, I2C, ERROR> Pin<'a, MODE, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    /// Set pin as input
    ///
    /// Установить пин как вход
    pub fn into_input(&self) -> Result<Pin<'a, Input, MUTEX>, ERROR> {
        self.driver.lock(|drv| drv.pin_to_input(self.pin))?;

        Ok(Pin {
            pin: self.pin,
            driver: self.driver,
            _mode: PhantomData,
        })
    }

    /// Set pin as pull up input
    ///
    /// Установить пин как Pull Up вход
    pub fn into_pull_up_input(&self) -> Result<Pin<'a, Input, MUTEX>, ERROR> {
        self.driver.lock(|drv| drv.pin_to_pull_up_input(self.pin))?;

        Ok(Pin {
            pin: self.pin,
            driver: self.driver,
            _mode: PhantomData,
        })
    }

    /// Set pin as pull down input
    ///
    /// Установить пин как Pull Down вход
    pub fn into_pull_down_input(&self) -> Result<Pin<'a, Input, MUTEX>, ERROR> {
        self.driver
            .lock(|drv| drv.pin_to_pull_down_input(self.pin))?;

        Ok(Pin {
            pin: self.pin,
            driver: self.driver,
            _mode: PhantomData,
        })
    }

    /// Set pin as output
    ///
    /// Установить пин как выход
    pub fn into_output(&self) -> Result<Pin<'a, Output, MUTEX>, ERROR> {
        self.driver.lock(|drv| drv.pin_to_output(self.pin))?;

        Ok(Pin {
            pin: self.pin,
            driver: self.driver,
            _mode: PhantomData,
        })
    }
}

impl<'a, MUTEX, I2C, ERROR> Pin<'a, Output, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    /// Set pin PWM value
    ///
    /// Установить значение ШИМ пина
    pub fn set_pwm(&mut self, value: u16) -> Result<(), ERROR> {
        self.driver.lock(|drv| drv.pwm_write(self.pin, value))?;

        Ok(())
    }
}

impl<'a, MUTEX, I2C, ERROR> v2::OutputPin for Pin<'a, Output, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    type Error = ERROR;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.driver.lock(|drv| drv.digital_write(self.pin, true))?;

        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.driver.lock(|drv| drv.digital_write(self.pin, false))?;

        Ok(())
    }
}

impl<'a, MUTEX, I2C, ERROR> v2::StatefulOutputPin for Pin<'a, Output, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self.driver.lock(|drv| drv.digital_read(self.pin))
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        let is_set_low = !self.is_set_high()?;
        Ok(is_set_low)
    }
}

impl<'a, MUTEX> v2::toggleable::Default for Pin<'a, Output, MUTEX> where
    Pin<'a, Output, MUTEX>: embedded_hal::digital::v2::StatefulOutputPin
{
}

impl<'a, MUTEX, I2C, ERROR> Pin<'a, Input, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    /// Get the analog value of a pin
    ///
    /// Получить аналоговое значение пина
    pub fn get_analog(&self) -> Result<u16, ERROR> {
        self.driver.lock(|drv| drv.analog_read(self.pin))
    }
}

impl<'a, MUTEX, I2C, ERROR> v2::InputPin for Pin<'a, Input, MUTEX>
where
    I2C: i2c::WriteRead<Error = ERROR> + i2c::Write<Error = ERROR> + i2c::Read<Error = ERROR>,
    MUTEX: shared_bus::BusMutex<Bus = Driver<I2C>>,
{
    type Error = ERROR;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.driver.lock(|drv| drv.digital_read(self.pin))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        let is_low = !self.is_high()?;
        Ok(is_low)
    }
}

/// Convenience re-export of multiple traits.
///
/// This allows a HAL user to conveniently import this module and have all the
/// helper traits already imported.
///
/// Without the prelude we would have to import the following traits:
///
/// ```
/// use embedded_hal::digital::v2::OutputPin; // for the set_high() function.
/// // And more use-statements with more complex code.
/// ```
///
/// These imports are a bit unintuitive, because we can create the objects
/// without the import. But we need these traits to access most of their
/// functions.
///
/// The prelude module keeps the import section cleaner:
/// ```
/// use gpio_expander::prelude::*;
/// ```
pub mod prelude {
    pub use embedded_hal::digital::v2::InputPin as _embedded_hal_digital_v2_InputPin;
    pub use embedded_hal::digital::v2::OutputPin as _embedded_hal_digital_v2_OutputPin;
    pub use embedded_hal::digital::v2::StatefulOutputPin as _embedded_hal_digital_v2_StatefulOutputPin;
    pub use embedded_hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_v2_ToggleableOutputPin;
}

#[cfg(test)]
mod tests {
    use super::{prelude::*, GpioExpander};
    use embedded_hal_mock::i2c;

    #[test]
    fn input() {
        let expectations = [
            // p00.into_input()
            i2c::Transaction::write(0x2a, vec![0b00000100, 0b00000000, 0b00000001]),
            // p01.into_input()
            i2c::Transaction::write(0x2a, vec![0b00000100, 0b00000000, 0b00000010]),
            // read pins state (for p00.is_high())
            i2c::Transaction::write(0x2a, vec![0b00001000]),
            i2c::Transaction::read(0x2a, vec![0b00000011, 0b01001111]),
            // read pins state (for p01.is_low())
            i2c::Transaction::write(0x2a, vec![0b00001000]),
            i2c::Transaction::read(0x2a, vec![0b00000011, 0b01101111]),
        ];
        let mut bus = i2c::Mock::new(&expectations);

        let mut expander = GpioExpander::new(bus.clone(), None);
        let expander_pins = expander.pins();

        let p00 = expander_pins.p00.into_input().unwrap();
        let p01 = expander_pins.p01.into_input().unwrap();
        assert!(p00.is_high().unwrap());
        assert!(!p01.is_low().unwrap());

        bus.done();
    }

    #[test]
    fn output() {
        let expectations = [
            // p00.into_output()
            i2c::Transaction::write(0x2a, vec![0b00000111, 0b00000000, 0b00000001]),
            // p01.into_output()
            i2c::Transaction::write(0x2a, vec![0b00000111, 0b00000000, 0b00000010]),
            // p00.set_high()
            i2c::Transaction::write(0x2a, vec![0b00001001, 0b00000000, 0b00000001]),
            // p01.set_low()
            i2c::Transaction::write(0x2a, vec![0b00001010, 0b00000000, 0b00000010]),
        ];
        let mut bus = i2c::Mock::new(&expectations);

        let mut expander = GpioExpander::new(bus.clone(), None);
        let expander_pins = expander.pins();

        let mut p00 = expander_pins.p00.into_output().unwrap();
        let mut p01 = expander_pins.p01.into_output().unwrap();
        p00.set_high().unwrap();
        p01.set_low().unwrap();

        bus.done();
    }

    #[test]
    fn pwm() {
        let expectations = [
            // p00.into_output()
            i2c::Transaction::write(0x2a, vec![0b00000111, 0b00000000, 0b00000001]),
            // p00.set_pwm(2000)
            i2c::Transaction::write(0x2a, vec![0b00001011, 0b00000000, 0b00000111, 0b11010000]),
        ];
        let mut bus = i2c::Mock::new(&expectations);

        let mut expander = GpioExpander::new(bus.clone(), None);
        let expander_pins = expander.pins();

        let mut p00 = expander_pins.p00.into_output().unwrap();
        p00.set_pwm(2000).unwrap();

        bus.done();
    }

    #[test]
    fn adc() {
        let expectations = [
            // p00.into_input()
            i2c::Transaction::write(0x2a, vec![0b00000100, 0b00000000, 0b00000001]),
            // p00.get_analog() == 1771
            i2c::Transaction::write_read(
                0x2a,
                vec![0b00001100, 0b00000000],
                vec![0b00000110, 0b11101011],
            ),
        ];
        let mut bus = i2c::Mock::new(&expectations);

        let mut expander = GpioExpander::new(bus.clone(), None);
        let expander_pins = expander.pins();

        let p00 = expander_pins.p00.into_input().unwrap();
        assert!(p00.get_analog().unwrap() == 1771);

        bus.done();
    }

    #[test]
    fn save_address() {
        let expectations = [
            // set i2c address
            i2c::Transaction::write(0x2a, vec![0b00000010, 0b00101010]),
            // save i2c address
            i2c::Transaction::write(0x2a, vec![0b00000011]),
        ];
        let mut bus = i2c::Mock::new(&expectations);

        let mut expander = GpioExpander::new(bus.clone(), None);
        expander.set_address(0x2a).unwrap();
        expander.save_address().unwrap();

        bus.done();
    }

    #[test]
    fn reset() {
        let expectations = [
            // p00.into_output()
            i2c::Transaction::write(0x2a, vec![0b00000111, 0b00000000, 0b00000001]),
            // p00.set_high()
            i2c::Transaction::write(0x2a, vec![0b00001001, 0b00000000, 0b00000001]),
            // reset GpioExpander
            i2c::Transaction::write(0x2a, vec![0b00000001]),
        ];
        let mut bus = i2c::Mock::new(&expectations);

        let mut expander = GpioExpander::new(bus.clone(), None);
        let expander_pins = expander.pins();

        let mut p00 = expander_pins.p00.into_output().unwrap();
        p00.set_high().unwrap();

        expander.reset().unwrap();

        bus.done();
    }
}
