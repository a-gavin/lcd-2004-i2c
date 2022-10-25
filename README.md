# LCD HiLetGo 2004a I2C driver
Driver to write characters to LCD displays with a HiLetGo 2004 20x4 display connected via i2c. It requires an I2C instance implementing [`embedded_hal::blocking::i2c::Write`] and an instance to delay execution with [`embedded_hal::blocking::delay::DelayMs`].

Fork of Korbinian Maier's [i2c LCD crate](https://github.com/KuabeM/lcd-lcm1602-i2c). Primary difference (aside from supporting a different display) is in this implementation, the display takes a `&mut` rather than a `mut`, meaning users are able to use the display i2c for other purposes as well.

### Usage:
```rust
const LCD_ADDRESS: u8 = 0x27; // Address depends on hardware, see datasheet link below

// Configure the clocks, delay
let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

let clocks = clocks::init_clocks_and_plls(
    XOSC_CRYSTAL_FREQ,
    ctx.device.XOSC,
    ctx.device.CLOCKS,
    ctx.device.PLL_SYS,
    ctx.device.PLL_USB,
    &mut ctx.device.RESETS,
    &mut watchdog,
)
.ok()
.unwrap();

let delay = Delay::new(
    ctx.core.SYST,
    clocks.system_clock.get_freq().to_Hz(),
);

// Create an I2C instance
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

// Init LCD, takes ownership of delay
let mut lcd = LcdUninit::new(&mut i2c, LCD_ADDRESS, delay)
    .rows(RowMode::Four)
    .init()
    .unwrap();

loop {
    // Write without ufmt
    _ = lcd.return_home();
    _ = lcd.clear();
    _ = lcd.write_str("write str method");

    // Delay half second
    lcd.delay_ms(500);

    // Write with ufmt
    _ = lcd.return_home();
    _ = lcd.clear();
    _ = write!(lcd, "ufmt write method");

    // Delay half second
    lcd.delay_ms(500);
}
```
Datasheet link [here](https://uk.beta-layout.com/download/rk/RK-10290_410.pdf)