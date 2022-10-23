# LCD HiLetGo 2004a I2C driver
Driver to write characters to LCD displays with a HiLetGo 2004 20x4 display connected via i2c. It requires an I2C instance implementing [`embedded_hal::blocking::i2c::Write`] and an instance to delay execution with [`embedded_hal::blocking::delay::DelayMs`].

Fork of Korbinian Maier's [i2c LCD crate](https://github.com/KuabeM/lcd-lcm1602-i2c). Primary difference (aside from supporting a different display) is in this implementation, the display takes a `&mut` rather than a `mut`, meaning users are able to use the display i2c for other purposes as well.

### Usage:
```rust
// Example using rp2040 rp-pico board support crate (also called BSP)
const LCD_ADDRESS: u8 = 0x27; // Address depends on hardware, see datasheet link below

// Create a I2C instance
let sda_pin = pins.gpio2.into_mode::<gpio::FunctionI2C>();
let scl_pin = pins.gpio3.into_mode::<gpio::FunctionI2C>();

let mut i2c = I2C::i2c1(
    pac.I2C1,
    sda_pin,
    scl_pin,
    100.kHz(),
    &mut pac.RESETS,
    &clocks.system_clock,
);

// Init LCD, takes ownership of I2C
let mut lcd = Lcd::new(&mut i2c, Backlight::Off)
    .address(LCD_ADDRESS)
    .cursor_on(true)
    .rows(4)
    .init(&mut delay)
    .unwrap();

loop {
    // Write without ufmt
    _ = lcd.return_home(&mut delay).clear();
    _ = lcd.write_str("write str method");

    // Delay half second
    _ = delay.delay_ms(500);

    // Write with ufmt
    _ = lcd.return_home(&mut delay).clear();
    _ = write!(lcd, "ufmt write method");

    _ = delay.delay_ms(500);
}
```
Datasheet link [here](https://uk.beta-layout.com/download/rk/RK-10290_410.pdf)