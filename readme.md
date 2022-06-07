`gpio-expander`
===============

This is a crate that provides a general abstraction for an I²C port expander used in the [`GPIO Port Expander (Troyka Module)`], [`Troyka HAT`] and [`Slot Expander Expansion Board`] products. This abstraction is not necessarily the most performant, but it allows pins to be used in the same way as direct GPIOs. Because pin types also implement [`embedded-hal`] digital I/O characteristics, they can also be passed to subsequent drivers (for example, as a reset or chip select pin).

## Example

```rust
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
```

More examples in the [`examples`] folder.

## Documentation

The documentation can be found at [docs.rs](https://docs.rs/gpio-expander/).

## Use in multithread

`gpio-expander` uses the `BusMutex` from [`shared-bus`] under the hood.  This means
you can also make the pins shareable across task/thread boundaries, given that
you provide an appropriate mutex type:

```rust
// Initializing GpioExpander with default I²C address, and alternative Mutex
let mut expander: GpioExpander<std::sync::Mutex<_>> = GpioExpander::new(i2c, None);
```

The default configuration allows pins to be used in single-threaded mode, for multi-threaded mode, use the example above (Mutex implementation depends on the platform you are using).

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

[`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
[`shared-bus`]: https://crates.io/crates/shared-bus
[`GPIO Port Expander (Troyka Module)`]: https://amperka.ru/product/troyka-gpio-expander
[`Troyka HAT`]: https://amperka.com/modules/troyka-hat
[`Slot Expander Expansion Board`]: https://amperka.ru/product/slot-expander
[`examples`]: https://github.com/amperka/gpio-expander-rs/examples