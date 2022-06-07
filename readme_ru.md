`gpio-expander`
===============

Это крейт, предоставляющий общую абстракцию для I²C расширителя портов, используемый в продуктах [`Расширитель GPIO-портов (Troyka-модуль)`], [`Troyka HAT`] и [`Плата расширения Slot Expander`]. Эта абстракция не обязательно самая производительная, но она позволяет использовать контакты так же, как прямые GPIO из HAL. Поскольку типы выводов также реализуют характеристики [`embedded-hal`] пинов ввода/вывода, они также могут быть переданы последующим драйверам (например, в качестве вывода сброса или выбора микросхемы).

## Пример

```rust
use std::error::Error;
use std::thread;
use std::time::Duration;

use gpio_expander::{prelude::*, GpioExpander};
use rppal::i2c::I2c;

fn main() -> Result<(), Box<dyn Error>> {
    // Инициализация периферийного устройства I²C из HAL
    let i2c = I2c::with_bus(1)?;

    // Инициализация GpioExpander с адресом I²C по умолчанию
    let mut expander = GpioExpander::new(i2c, None);
    let expander_pins = expander.pins();

    // Настройка пина 0 как выход
    let mut led = expander_pins.p00.into_output()?;

    loop {
        led.set_high()?;
        thread::sleep(Duration::from_secs(1));
        led.set_low()?;
        thread::sleep(Duration::from_secs(1));
    }
}
```

Больше примеров в папке [`examples`].

## Документация

Документацию можно найти на [docs.rs](https://docs.rs/gpio-expander/).

## Использование в мультипотоке

`gpio-expander` использует `BusMutex` из [`shared-bus`] под капотом. Это означает вы также можете сделать выводы доступными для разных задач/потоков, учитывая, что вы предоставляете соответствующий тип мьютекса:

```rust
// Инициализация GpioExpander с адресом I²C по умолчанию, и альтернативным Mutex
let mut expander: GpioExpander<std::sync::Mutex<_>> = GpioExpander::new(i2c, None);
```

Конфигурация по умолчанию, позволяет использовать пины в однопоточном режиме, для многопоточного режима, используйте пример выше (реализация Mutex зависит от используемой вами платформы).

## Лицензия

Лицензируеться под

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) или <http://www.apache.org/licenses/LICENSE-2.0>)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) или <http://opensource.org/licenses/MIT>)

по вашему выбору.

## Вклад

Если вы прямо не укажете иное, любой вклад, преднамеренно отправленный
для включения вами в работу, как определено в лицензии Apache-2.0, должно быть
с двойной лицензией, как указано выше, без каких-либо дополнительных условий.

[`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
[`shared-bus`]: https://crates.io/crates/shared-bus
[`Расширитель GPIO-портов (Troyka-модуль)`]: https://amperka.ru/product/troyka-gpio-expander
[`Troyka HAT`]: https://amperka.ru/product/raspberry-pi-troyka-hat
[`Плата расширения Slot Expander`]: https://amperka.ru/product/slot-expander
[`examples`]: https://github.com/amperka/gpio-expander-rs/examples