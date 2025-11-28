//! SPDX-License-Identifier: MIT OR Apache-2.0
//!
//! # BME280 + SSD1315 OLED + WiFi NTP Example for Pico 2 W
//!
//! This application:
//! - Connects to WiFi
//! - Gets time from NTP server (Tehran timezone UTC+3:30)
//! - Reads temperature, humidity, and pressure from BME280
//! - Displays everything on an SSD1315 OLED

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write as FmtWrite;

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::i2c::RefCellDevice;
use heapless::String;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

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

// ============================================================================
// WiFi Configuration - CHANGE THESE TO YOUR NETWORK CREDENTIALS
// ============================================================================
const WIFI_NETWORK: &str = "SSID";
const WIFI_PASSWORD: &str = "PASSWORD";

// NTP server (time.google.com)
const NTP_SERVER: (u8, u8, u8, u8) = (216, 239, 35, 0);
const NTP_PORT: u16 = 123;

// Tehran timezone offset: UTC+3:30 = 3 hours and 30 minutes = 12600 seconds
const TEHRAN_OFFSET_SECONDS: i64 = 3 * 3600 + 30 * 60;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

// CYW43 WiFi task
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

// Network stack task
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

/// Simple NTP request - returns Unix timestamp
async fn get_ntp_time(stack: Stack<'static>) -> Result<u64, ()> {
    // Create UDP socket for NTP
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buffer = [0u8; 256];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buffer = [0u8; 256];

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    // Bind to any local port
    socket.bind(0).map_err(|_| ())?;

    // NTP packet (48 bytes)
    let mut packet = [0u8; 48];
    packet[0] = 0x1B; // LI=0, VN=3, Mode=3 (client)

    let server = (
        embassy_net::Ipv4Address::new(NTP_SERVER.0, NTP_SERVER.1, NTP_SERVER.2, NTP_SERVER.3),
        NTP_PORT,
    );

    // Send request
    socket.send_to(&packet, server).await.map_err(|_| ())?;

    // Receive response with timeout (5 seconds)
    let result = with_timeout(Duration::from_secs(5), socket.recv_from(&mut packet)).await;
    let (len, _) = match result {
        Ok(Ok(r)) => r,
        _ => return Err(()),
    };

    if len >= 48 {
        // Extract transmit timestamp (bytes 40-43 are seconds since 1900)
        let secs = u32::from_be_bytes([packet[40], packet[41], packet[42], packet[43]]);
        // Convert from NTP epoch (1900) to Unix epoch (1970)
        // NTP epoch offset: 2208988800 seconds
        Ok(secs.saturating_sub(2208988800) as u64)
    } else {
        Err(())
    }
}

/// Convert Unix timestamp to Tehran local time (hours, minutes, seconds)
fn unix_to_tehran_time(unix_time: u64) -> (u8, u8, u8) {
    let tehran_time = unix_time as i64 + TEHRAN_OFFSET_SECONDS;
    let seconds_in_day = tehran_time % 86400;
    let hours = (seconds_in_day / 3600) as u8;
    let minutes = ((seconds_in_day % 3600) / 60) as u8;
    let seconds = (seconds_in_day % 60) as u8;
    (hours, minutes, seconds)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Pico 2 W - BME280 + OLED + WiFi NTP");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    // Include CYW43 firmware
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // Initialize PIO SPI for CYW43 WiFi chip
    // Pico 2 W pin mapping:
    // - PIN_23: CYW43 Power
    // - PIN_24: CYW43 SPI Data
    // - PIN_25: CYW43 SPI CS (also onboard LED control via WiFi chip)
    // - PIN_29: CYW43 SPI Clock
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER, // Important: Use RM2_CLOCK_DIVIDER for Pico 2 W (RP2350)
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    // Initialize CYW43 WiFi chip
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Generate random seed for network stack
    let seed = rng.next_u64();

    // Initialize network stack with DHCP and hostname
    let mut dhcp_config = embassy_net::DhcpConfig::default();
    dhcp_config.hostname = Some(heapless::String::try_from("pico2w").unwrap());

    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        Config::dhcpv4(dhcp_config),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(net_task(runner)).unwrap();

    // ========================================================================
    // Set up I2C for BME280 sensor and OLED display
    // ========================================================================
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c::Config::default());

    // Wrap I2C in RefCell for sharing between devices
    let i2c_ref_cell = RefCell::new(i2c);

    // Initialize BME280 sensor FIRST (before display takes the bus)
    let mut delay = Delay;
    let mut bme280 = BME280::new_secondary(RefCellDevice::new(&i2c_ref_cell));
    bme280.init(&mut delay).unwrap();
    info!("BME280 initialized!");

    // Test read to verify sensor is working
    let test_measurement = bme280.measure(&mut delay).unwrap();
    info!("Initial reading - Temp: {}C", test_measurement.temperature);

    // Create the I2C display interface using shared bus
    let interface = I2CDisplayInterface::new(RefCellDevice::new(&i2c_ref_cell));

    // Create the SSD1306 driver (compatible with SSD1315)
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Initialize the display
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();

    // Create text style
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Show connecting message
    Text::new("Connecting to WiFi...", Point::new(5, 30), text_style)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    // ========================================================================
    // Connect to WiFi
    // ========================================================================
    info!("Connecting to WiFi: {}", WIFI_NETWORK);
    loop {
        match control
            .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
            .await
        {
            Ok(_) => {
                info!("WiFi connected!");
                break;
            }
            Err(e) => {
                warn!("WiFi join failed: status={}", e.status);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    // Wait for link to be up (with timeout)
    info!("Waiting for link...");
    match with_timeout(Duration::from_secs(10), stack.wait_link_up()).await {
        Ok(_) => info!("Link is up!"),
        Err(_) => warn!("Link timeout, continuing anyway..."),
    }

    // Wait for DHCP to configure the IP (with timeout)
    info!("Waiting for DHCP...");
    match with_timeout(Duration::from_secs(15), stack.wait_config_up()).await {
        Ok(_) => info!("DHCP configured!"),
        Err(_) => warn!("DHCP timeout, continuing without IP..."),
    }

    // Log the IP address
    if let Some(config) = stack.config_v4() {
        info!("DHCP configured:");
        info!("  IP: {}", config.address);
        if let Some(gw) = config.gateway {
            info!("  Gateway: {}", gw);
        }
    }

    // Update display with WiFi connected message
    display.clear(BinaryColor::Off).unwrap();
    Text::new("WiFi Connected!", Point::new(5, 20), text_style)
        .draw(&mut display)
        .unwrap();
    Text::new("Getting time...", Point::new(5, 40), text_style)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    // ========================================================================
    // Get initial NTP time
    // ========================================================================
    let mut current_unix_time: u64 = 0;
    let mut last_ntp_update = Instant::now();
    let mut time_valid = false;

    info!("Getting time from NTP server...");
    match get_ntp_time(stack).await {
        Ok(unix_time) => {
            current_unix_time = unix_time;
            time_valid = true;
            last_ntp_update = Instant::now();
            let (h, m, s) = unix_to_tehran_time(unix_time);
            info!("NTP Time received! Tehran time: {:02}:{:02}:{:02}", h, m, s);
        }
        Err(_) => {
            warn!("Failed to get NTP time, will retry...");
        }
    }

    // ========================================================================
    // Main loop - read sensors, update time, display
    // ========================================================================
    loop {
        // Update time from elapsed duration since last NTP sync
        if time_valid {
            let elapsed_secs = last_ntp_update.elapsed().as_secs();
            let adjusted_time = current_unix_time + elapsed_secs;
            let (hours, minutes, seconds) = unix_to_tehran_time(adjusted_time);

            // Read sensor measurements
            let measurements = bme280.measure(&mut delay).unwrap();

            // Format strings for display
            let mut time_str: String<32> = String::new();
            let mut temp_str: String<32> = String::new();
            let mut hum_str: String<32> = String::new();
            let mut pres_str: String<32> = String::new();

            // Format time (Tehran)
            core::write!(
                time_str,
                "Tehran {:02}:{:02}:{:02}",
                hours,
                minutes,
                seconds
            )
            .unwrap();

            // Format temperature
            let temp_int = measurements.temperature as i32;
            let temp_frac = ((measurements.temperature - temp_int as f32) * 10.0) as i32;
            core::write!(temp_str, "Temp: {}.{}C", temp_int, temp_frac.abs()).unwrap();

            // Format humidity
            let hum_int = measurements.humidity as i32;
            let hum_frac = ((measurements.humidity - hum_int as f32) * 10.0) as i32;
            core::write!(hum_str, "Hum:  {}.{}%", hum_int, hum_frac.abs()).unwrap();

            // Format pressure (in hPa)
            let pres_hpa = measurements.pressure / 100.0;
            let pres_int = pres_hpa as i32;
            let pres_frac = ((pres_hpa - pres_int as f32) * 10.0) as i32;
            core::write!(pres_str, "Pres: {}.{} hPa", pres_int, pres_frac.abs()).unwrap();

            // Clear display and draw new values
            display.clear(BinaryColor::Off).unwrap();

            Text::new(&time_str, Point::new(5, 12), text_style)
                .draw(&mut display)
                .unwrap();
            Text::new(&temp_str, Point::new(5, 28), text_style)
                .draw(&mut display)
                .unwrap();
            Text::new(&hum_str, Point::new(5, 44), text_style)
                .draw(&mut display)
                .unwrap();
            Text::new(&pres_str, Point::new(5, 60), text_style)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();

            // Log to defmt
            info!(
                "Tehran Time: {:02}:{:02}:{:02} | Temp: {}C, Humidity: {}%, Pressure: {} hPa",
                hours, minutes, seconds, measurements.temperature, measurements.humidity, pres_hpa
            );

            // Re-sync NTP every 10 minutes
            if elapsed_secs > 600 {
                info!("Re-syncing NTP time...");
                if let Ok(unix_time) = get_ntp_time(stack).await {
                    current_unix_time = unix_time;
                    last_ntp_update = Instant::now();
                    let (h, m, s) = unix_to_tehran_time(unix_time);
                    info!(
                        "NTP re-sync successful! Tehran time: {:02}:{:02}:{:02}",
                        h, m, s
                    );
                }
            }
        } else {
            // Try to get NTP time again
            if let Ok(unix_time) = get_ntp_time(stack).await {
                current_unix_time = unix_time;
                time_valid = true;
                last_ntp_update = Instant::now();
                let (h, m, s) = unix_to_tehran_time(unix_time);
                info!("NTP Time received! Tehran time: {:02}:{:02}:{:02}", h, m, s);
            } else {
                // Show sensor data without time
                let measurements = bme280.measure(&mut delay).unwrap();
                display.clear(BinaryColor::Off).unwrap();
                Text::new("Time: --:--:--", Point::new(5, 12), text_style)
                    .draw(&mut display)
                    .unwrap();

                let mut temp_str: String<32> = String::new();
                let temp_int = measurements.temperature as i32;
                let temp_frac = ((measurements.temperature - temp_int as f32) * 10.0) as i32;
                core::write!(temp_str, "Temp: {}.{}C", temp_int, temp_frac.abs()).unwrap();
                Text::new(&temp_str, Point::new(5, 28), text_style)
                    .draw(&mut display)
                    .unwrap();

                let mut hum_str: String<32> = String::new();
                let hum_int = measurements.humidity as i32;
                let hum_frac = ((measurements.humidity - hum_int as f32) * 10.0) as i32;
                core::write!(hum_str, "Hum:  {}.{}%", hum_int, hum_frac.abs()).unwrap();
                Text::new(&hum_str, Point::new(5, 44), text_style)
                    .draw(&mut display)
                    .unwrap();

                let mut pres_str: String<32> = String::new();
                let pres_hpa = measurements.pressure / 100.0;
                let pres_int = pres_hpa as i32;
                let pres_frac = ((pres_hpa - pres_int as f32) * 10.0) as i32;
                core::write!(pres_str, "Pres: {}.{} hPa", pres_int, pres_frac.abs()).unwrap();
                Text::new(&pres_str, Point::new(5, 60), text_style)
                    .draw(&mut display)
                    .unwrap();

                display.flush().unwrap();
                info!(
                    "Temp: {}C, Humidity: {}%, Pressure: {} hPa (no time sync)",
                    measurements.temperature, measurements.humidity, pres_hpa
                );
            }
        }

        // Blink LED via WiFi chip to show activity
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(50)).await;
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(450)).await;
    }
}
