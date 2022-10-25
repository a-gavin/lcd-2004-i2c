#![no_std]
//! Driver to write characters to 20x4 LCD displays connected via i2c.
//! It requires an I2C instance implementing [`embedded_hal::blocking::i2c::Write`]
//! and an instance to delay execution with [`embedded_hal::blocking::delay::DelayMs`].
//!
//! Usage:
//! ```
//! // Example using rp2040 rp-pico board support crate (also called BSP)
//! const LCD_ADDRESS: u8 = 0x27; // Address depends on hardware, see datasheet link below
//! 
//! // Configure the clocks, delay
//! let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
//! let clocks = clocks::init_clocks_and_plls(
//!     XOSC_CRYSTAL_FREQ,
//!     ctx.device.XOSC,
//!     ctx.device.CLOCKS,
//!     ctx.device.PLL_SYS,
//!     ctx.device.PLL_USB,
//!     &mut ctx.device.RESETS,
//!     &mut watchdog,
//! )
//! .ok()
//! .unwrap();
//!
//! let delay = Delay::new(
//!     ctx.core.SYST,
//!     clocks.system_clock.get_freq().to_Hz(),
//! );
//!
//! // Create an I2C instance
//! let sda_pin = pins.gpio2.into_mode::<gpio::FunctionI2C>();
//! let scl_pin = pins.gpio3.into_mode::<gpio::FunctionI2C>();
//!
//! let mut i2c = I2C::i2c1(
//!     pac.I2C1,
//!     sda_pin,
//!     scl_pin,
//!     100.kHz(),
//!     &mut pac.RESETS,
//!     &clocks.system_clock,
//! );
//!
//! // Init LCD, takes ownership of delay
//! let mut lcd = LcdUninit::new(&mut i2c, LCD_ADDRESS, delay)
//!     .rows(RowMode::Four)
//!     .init()
//!     .unwrap();
//!
//! loop {
//!     // Write without ufmt
//!     _ = lcd.return_home();
//!     _ = lcd.clear();
//!     _ = lcd.write_str("write str method");
//!
//!     // Delay half second
//!     lcd.delay_ms(500);
//!
//!     // Write with ufmt
//!     _ = lcd.return_home();
//!     _ = lcd.clear();
//!     _ = write!(lcd, "ufmt write method");
//!
//!     // Delay half second
//!     lcd.delay_ms(500);
//! }
//! ```
//! Datasheet used link [here](https://uk.beta-layout.com/download/rk/RK-10290_410.pdf)

use embedded_hal::blocking::{delay::DelayMs, i2c, i2c::Write};

use ufmt_write::uWrite;

// Enums for display operation
#[derive(Copy, Clone, PartialEq)]
pub enum RowMode {
    Two,
    Four,
}

pub enum DisplayControl {
    Off = 0x00,
    CursorBlink = 0x01,
    CursorOn = 0x02,
    DisplayOn = 0x04,
}

#[derive(Copy, Clone)]
pub enum Backlight {
    Off = 0x00,
    On = 0x08,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Mode {
    Cmd = 0x00,
    Data = 0x01,
    DisplayControl = 0x08,
    FunctionSet = 0x20,
}

enum Commands {
    Clear = 0x01,
    ReturnHome = 0x02,
    ShiftCursor = 0x14,
}

enum BitMode {
    Bit4 = 0x00,
    Bit8 = 0x10,
}

const DEFAULT_ROWS: RowMode = RowMode::Four;

/// API to write to the LCD.
// PhantomData<D> used to ensure correct Delay without having to take ownership of delay.
pub struct LcdUninit<'a, I, D>
where
    I: i2c::Write,
    D: DelayMs<u8>,
{
    i2c: &'a mut I,
    address: u8,
    num_rows: RowMode,
    backlight_state: Backlight,
    cursor_on: bool,
    cursor_blink: bool,
    delay: D,
}

impl<'a, I, D> LcdUninit<'a, I, D>
where
    I: i2c::Write,
    D: DelayMs<u8>,
{
    /// Create new instance with only the I2C and address
    pub fn new(i2c: &'a mut I, address: u8, delay: D) -> Self {
        Self {
            i2c,
            backlight_state: Backlight::On,
            address: address,
            num_rows: DEFAULT_ROWS,
            cursor_blink: false,
            cursor_on: false,
            delay: delay,
        }
    }

    /// Number of rows (only 2 or 4 supported)
    pub fn set_num_rows(mut self, num_rows: RowMode) -> Self {
        self.num_rows = num_rows;
        self
    }

    pub fn set_backlight_state(mut self, backlight: Backlight) -> Self {
        self.backlight_state = backlight;
        self
    }

    /// Initializes the hardware.
    ///
    /// Actual procedure is a bit obscure. This one was compiled from this [blog post],
    /// corresponding [code] and the [datasheet].
    ///
    /// [datasheet]: https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf
    /// [code]: https://github.com/jalhadi/i2c-hello-world/blob/main/src/main.rs
    /// [blog post]: https://badboi.dev/rust,/microcontrollers/2020/11/09/i2c-hello-world.html
    pub fn init(self) -> Result<Lcd<'a, I, D>, <I as i2c::Write>::Error> {
        // Consume self to create and initialize Lcd
        let mut lcd: Lcd<'a, I, D> = Lcd {
            i2c: self.i2c,
            backlight_state: self.backlight_state,
            address: self.address,
            num_rows: self.num_rows,
            cursor_blink: self.cursor_blink,
            cursor_on: self.cursor_on,
            delay: self.delay,
        };

        // Initial delay to wait for init after power on.
        lcd.delay.delay_ms(80);

        // Init with 8 bit mode
        let mode_8bit = Mode::FunctionSet as u8 | BitMode::Bit8 as u8;
        for _ in 1..4 {
            lcd.write4bits(mode_8bit)?;
            lcd.delay.delay_ms(5);
        }

        // Switch to 4 bit mode
        let mode_4bit = Mode::FunctionSet as u8 | BitMode::Bit4 as u8;
        lcd.write4bits(mode_4bit)?;

        // Display setup
        // Set mode either 2 or four lines
        // TODO: Verify for 20x4 screen
        lcd.set_num_rows(lcd.num_rows)?;
        lcd.clear()?;

        // Entry right: shifting cursor moves to right
        lcd.send(0x04, Mode::Cmd)?;
        lcd.set_backlight_state(self.backlight_state)?;

        Ok(lcd)
    }
}

pub struct Lcd<'a, I, D>
where
    I: i2c::Write,
    D: DelayMs<u8>,
{
    i2c: &'a mut I,
    address: u8,
    num_rows: RowMode,
    backlight_state: Backlight,
    cursor_on: bool,
    cursor_blink: bool,
    delay: D,
}

impl<'a, I, D> Lcd<'a, I, D>
where
    I: i2c::Write,
    D: DelayMs<u8>,
{
    /// Number of rows (only 2 or 4 supported)
    pub fn set_num_rows(&mut self, num_rows: RowMode) -> Result<(), <I as i2c::Write>::Error> {
        if self.num_rows == num_rows {
            return Ok(());
        }
        self.num_rows = num_rows;

        let lines = match self.num_rows {
            RowMode::Two => 0x08,
            RowMode::Four => 0x00,
        };
        self.send(Mode::FunctionSet as u8 | lines, Mode::Cmd)?;

        Ok(())
    }

    pub fn set_cursor_on(&mut self, on: bool) -> Result<(), <I as i2c::Write>::Error> {
        self.cursor_on = on;

        if self.cursor_on {
            let mut display_ctrl = DisplayControl::DisplayOn as u8;
            display_ctrl |= DisplayControl::CursorOn as u8;

            if self.cursor_blink {
                display_ctrl |= DisplayControl::CursorBlink as u8;
            }

            self.send(Mode::DisplayControl as u8 | display_ctrl, Mode::Cmd)?;
        }

        Ok(())
    }

    pub fn set_cursor_blink(&mut self, on: bool) -> Result<(), <I as i2c::Write>::Error> {
        self.cursor_blink = on;

        if self.cursor_blink {
            let mut display_ctrl = DisplayControl::DisplayOn as u8;
            display_ctrl |= DisplayControl::CursorOn as u8 | DisplayControl::CursorBlink as u8;

            self.send(Mode::DisplayControl as u8 | display_ctrl, Mode::Cmd)?;
        }

        Ok(())
    }

    pub fn set_backlight_state(
        &mut self,
        backlight: Backlight,
    ) -> Result<(), <I as i2c::Write>::Error> {
        self.backlight_state = backlight;
        self.i2c.write(
            self.address,
            &[DisplayControl::DisplayOn as u8 | backlight as u8],
        )
    }

    /// Write string to display.
    pub fn write_str(&mut self, data: &str) -> Result<(), <I as i2c::Write>::Error> {
        for c in data.chars() {
            self.send(c as u8, Mode::Data)?;
        }
        Ok(())
    }

    /// Clear the display
    pub fn clear(&mut self) -> Result<(), <I as i2c::Write>::Error> {
        self.send(Commands::Clear as u8, Mode::Cmd)?;
        Ok(())
    }

    /// Return cursor to upper left corner, i.e. (0,0).
    pub fn return_home(&mut self) -> Result<(), <I as i2c::Write>::Error> {
        self.send(Commands::ReturnHome as u8, Mode::Cmd)?;
        self.delay.delay_ms(10);
        Ok(())
    }

    /// Set the cursor to (rows, col). Coordinates are zero-based.
    pub fn set_cursor(
        &mut self,
        row: u8,
        col: u8,
    ) -> Result<(), <I as i2c::Write>::Error> {
        self.return_home()?;
        let shift: u8 = row * 40 + col;
        for _i in 0..shift {
            self.send(Commands::ShiftCursor as u8, Mode::Cmd)?;
        }
        Ok(())
    }

    // Hacky fix but works for now
    pub fn delay_ms(&mut self, ms: u8) {
        self.delay.delay_ms(ms);
    }

    // Private funcs
    fn write4bits(&mut self, data: u8) -> Result<(), <I as i2c::Write>::Error> {
        self.i2c.write(
            self.address,
            &[data | DisplayControl::DisplayOn as u8 | self.backlight_state as u8],
        )?;
        self.i2c.write(
            self.address,
            &[DisplayControl::Off as u8 | self.backlight_state as u8],
        )?;
        Ok(())
    }

    fn send(&mut self, data: u8, mode: Mode) -> Result<(), <I as i2c::Write>::Error> {
        let high_bits: u8 = data & 0xf0;
        let low_bits: u8 = (data << 4) & 0xf0;
        self.write4bits(high_bits | mode as u8)?;
        self.write4bits(low_bits | mode as u8)?;
        Ok(())
    }
}

impl<'a, I, D> uWrite for Lcd<'a, I, D>
where
    I: Write,
    D: DelayMs<u8>,
{
    type Error = <I as Write>::Error;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_str(s)
    }
}