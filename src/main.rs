//! SPDX-License-Identifier: MIT OR Apache-2.0
//!
//! Copyright (c) 2021–2024 The rp-rs Developers
//! Copyright (c) 2021 rp-rs organization
//! Copyright (c) 2025 Raspberry Pi Ltd.
//!
//! # BME280 + SSD1315 OLED Example
//!
//! This application reads temperature, humidity, and pressure from a BME280
//! sensor and displays them on an SSD1315 OLED, both connected via I2C.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write as FmtWrite;

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::i2c::RefCellDevice;
use heapless::String;
#[cfg(target_arch = "riscv32")]
use panic_halt as _;
#[cfg(target_arch = "arm")]
use panic_probe as _;

// BME280 sensor
use bme280::i2c::BME280;

// OLED display imports
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

// Alias for our HAL crate
use hal::entry;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};

#[cfg(rp2350)]
use rp235x_hal as hal;

#[cfg(rp2040)]
use rp2040_hal as hal;

// use bsp::entry;
// use bsp::hal;
// use rp_pico as bsp;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[unsafe(link_section = ".boot2")]
#[used]
#[cfg(rp2040)]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
#[cfg(rp2350)]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp2040 and rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    info!("Program start");
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    #[cfg(rp2040)]
    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    #[cfg(rp2350)]
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Set up I2C for the OLED display and BME280 sensor
    // Using GPIO4 (SDA) and GPIO5 (SCL) - adjust if your wiring is different
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400_u32.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Wrap I2C in RefCell for sharing between devices
    let i2c_ref_cell = RefCell::new(i2c);

    // Create the I2C display interface using shared bus
    let interface = I2CDisplayInterface::new(RefCellDevice::new(&i2c_ref_cell));

    // Create the SSD1306 driver (compatible with SSD1315)
    // Adjust DisplaySize if your display is different (e.g., DisplaySize128x32)
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Initialize the display
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();

    // Initialize BME280 sensor
    // Try secondary address 0x77 (SDO pin connected to VCC)
    // If your sensor uses 0x76 (SDO to GND), use BME280::new_primary() instead
    let mut bme280 = BME280::new_secondary(RefCellDevice::new(&i2c_ref_cell));
    bme280.init(&mut timer).unwrap();

    info!("BME280 and OLED initialized!");

    // Create text style - using smaller font to fit more data
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        // Read sensor measurements
        let measurements = bme280.measure(&mut timer).unwrap();

        // Format strings for display
        let mut temp_str: String<32> = String::new();
        let mut hum_str: String<32> = String::new();
        let mut pres_str: String<32> = String::new();

        // Format temperature (convert to integer parts for display)
        let temp_int = measurements.temperature as i32;
        let temp_frac = ((measurements.temperature - temp_int as f32) * 10.0) as i32;
        core::write!(temp_str, "Temp: {}.{}C", temp_int, temp_frac.abs()).unwrap();

        // Format humidity
        let hum_int = measurements.humidity as i32;
        let hum_frac = ((measurements.humidity - hum_int as f32) * 10.0) as i32;
        core::write!(hum_str, "Hum:  {}.{}%", hum_int, hum_frac.abs()).unwrap();

        // Format pressure (in hPa)
        let pres_hpa = measurements.pressure / 100.0; // Convert Pa to hPa
        let pres_int = pres_hpa as i32;
        let pres_frac = ((pres_hpa - pres_int as f32) * 10.0) as i32;
        core::write!(pres_str, "Pres: {}.{} hPa", pres_int, pres_frac.abs()).unwrap();

        // Clear display and draw new values
        display.clear(BinaryColor::Off).unwrap();

        Text::new(&temp_str, Point::new(5, 15), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new(&hum_str, Point::new(5, 30), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new(&pres_str, Point::new(5, 45), text_style)
            .draw(&mut display)
            .unwrap();

        // Flush the buffer to the display
        display.flush().unwrap();

        info!(
            "Temp: {}C, Humidity: {}%, Pressure: {} hPa",
            measurements.temperature, measurements.humidity, pres_hpa
        );

        // Toggle LED to show activity
        led_pin.set_high().unwrap();
        timer.delay_ms(100);
        led_pin.set_low().unwrap();
        timer.delay_ms(1900); // Update every 2 seconds
    }
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
