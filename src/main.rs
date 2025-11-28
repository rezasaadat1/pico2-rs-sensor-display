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

// DS3231 RTC
use ds323x::{DateTimeAccess, Datelike, Ds323x, NaiveDate, Timelike};

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

// NTP servers to try (in order)
// pool.ntp.org: 162.159.200.1, time.cloudflare.com: 162.159.200.123
// time.google.com: 216.239.35.0, time.apple.com: 17.253.34.123
const NTP_SERVERS: [(u8, u8, u8, u8); 4] = [
    (162, 159, 200, 1), // pool.ntp.org (Cloudflare)
    (129, 6, 15, 28),   // time.nist.gov (US NIST)
    (216, 239, 35, 0),  // time.google.com
    (17, 253, 34, 123), // time.apple.com
];
const NTP_PORT: u16 = 123;

// Tehran timezone offset: UTC+3:30 = 3 hours and 30 minutes = 12600 seconds
const TEHRAN_OFFSET_SECONDS: i64 = 3 * 3600 + 30 * 60;

// WiFi connection settings
const WIFI_MAX_RETRIES: u8 = 5;
const WIFI_RETRY_DELAY_SECS: u64 = 2;

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

/// Simple NTP request - tries multiple servers, returns Unix timestamp
async fn get_ntp_time(stack: Stack<'static>) -> Result<u64, ()> {
    // Check if we have an IP address
    if stack.config_v4().is_none() {
        warn!("NTP: No IPv4 config yet");
        return Err(());
    }

    // Try each NTP server
    for ntp_server in NTP_SERVERS.iter() {
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
        if socket.bind(0).is_err() {
            warn!("NTP: Failed to bind socket");
            continue;
        }

        // NTP packet (48 bytes)
        let mut packet = [0u8; 48];
        packet[0] = 0x1B; // LI=0, VN=3, Mode=3 (client)

        let server = (
            embassy_net::Ipv4Address::new(ntp_server.0, ntp_server.1, ntp_server.2, ntp_server.3),
            NTP_PORT,
        );

        info!(
            "NTP: Trying {}.{}.{}.{}:{}",
            ntp_server.0, ntp_server.1, ntp_server.2, ntp_server.3, NTP_PORT
        );

        // Send request
        if socket.send_to(&packet, server).await.is_err() {
            warn!("NTP: Failed to send packet");
            continue;
        }

        // Receive response with timeout (3 seconds per server)
        let result = with_timeout(Duration::from_secs(3), socket.recv_from(&mut packet)).await;
        let (len, _) = match result {
            Ok(Ok(r)) => r,
            Ok(Err(_)) => {
                warn!("NTP: Receive error");
                continue;
            }
            Err(_) => {
                warn!("NTP: Timeout");
                continue;
            }
        };

        if len >= 48 {
            // Extract transmit timestamp (bytes 40-43 are seconds since 1900)
            let secs = u32::from_be_bytes([packet[40], packet[41], packet[42], packet[43]]);
            // Convert from NTP epoch (1900) to Unix epoch (1970)
            // NTP epoch offset: 2208988800 seconds
            info!(
                "NTP: Success from {}.{}.{}.{}",
                ntp_server.0, ntp_server.1, ntp_server.2, ntp_server.3
            );
            return Ok(secs.saturating_sub(2208988800) as u64);
        } else {
            warn!("NTP: Response too short");
        }
    }

    // All servers failed
    warn!("NTP: All servers failed");
    Err(())
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

/// Convert Unix timestamp to full date/time components (year, month, day, hour, min, sec)
/// Adjusted for Tehran timezone (UTC+3:30)
fn unix_to_datetime(unix_time: u64) -> (i32, u8, u8, u8, u8, u8) {
    let tehran_time = unix_time as i64 + TEHRAN_OFFSET_SECONDS;

    // Calculate days since Unix epoch
    let mut days = (tehran_time / 86400) as i32;
    let seconds_in_day = (tehran_time % 86400) as i32;

    let hours = (seconds_in_day / 3600) as u8;
    let minutes = ((seconds_in_day % 3600) / 60) as u8;
    let seconds = (seconds_in_day % 60) as u8;

    // Calculate year, month, day from days since epoch
    // Unix epoch is January 1, 1970
    let mut year = 1970;

    loop {
        let days_in_year = if is_leap_year(year) { 366 } else { 365 };
        if days < days_in_year {
            break;
        }
        days -= days_in_year;
        year += 1;
    }

    // Calculate month and day
    let months_days = if is_leap_year(year) {
        [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    } else {
        [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    };

    let mut month = 1u8;
    for &m_days in months_days.iter() {
        if days < m_days {
            break;
        }
        days -= m_days;
        month += 1;
    }

    let day = (days + 1) as u8; // Days are 1-indexed

    (year, month, day, hours, minutes, seconds)
}

fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

/// Convert date/time components to Unix timestamp (Tehran timezone already applied in RTC)
/// This reverses the Tehran offset to get back to UTC Unix time
fn datetime_to_unix(year: i32, month: u8, day: u8, hour: u8, min: u8, sec: u8) -> u64 {
    // Calculate days since Unix epoch (1970-01-01)
    let mut days: i64 = 0;

    // Add days for complete years
    for y in 1970..year {
        days += if is_leap_year(y) { 366 } else { 365 };
    }

    // Add days for complete months in current year
    let months_days = if is_leap_year(year) {
        [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    } else {
        [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    };

    for m in 0..(month as usize - 1) {
        days += months_days[m] as i64;
    }

    // Add days in current month (day is 1-indexed)
    days += (day - 1) as i64;

    // Calculate total seconds in Tehran time
    let tehran_secs = days * 86400 + (hour as i64) * 3600 + (min as i64) * 60 + (sec as i64);

    // Convert back to UTC Unix time by subtracting Tehran offset
    (tehran_secs - TEHRAN_OFFSET_SECONDS) as u64
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

    // Configure I2C at 400 kHz (same as previous rp235x-hal version)
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

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

    // Initialize DS3231 RTC
    let mut rtc = Ds323x::new_ds3231(RefCellDevice::new(&i2c_ref_cell));
    // Clear the oscillator stopped flag if set
    if rtc.has_been_stopped().unwrap_or(false) {
        info!("DS3231 oscillator was stopped, clearing flag");
        let _ = rtc.clear_has_been_stopped_flag();
    }
    info!("DS3231 RTC initialized!");

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
    // Connect to WiFi (with limited retries)
    // ========================================================================
    let mut wifi_connected = false;
    info!("Connecting to WiFi: {}", WIFI_NETWORK);

    for attempt in 1..=WIFI_MAX_RETRIES {
        info!("WiFi connection attempt {}/{}", attempt, WIFI_MAX_RETRIES);
        match control
            .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
            .await
        {
            Ok(_) => {
                info!("WiFi connected!");
                wifi_connected = true;
                break;
            }
            Err(e) => {
                warn!("WiFi join failed: status={}", e.status);
                if attempt < WIFI_MAX_RETRIES {
                    Timer::after(Duration::from_secs(WIFI_RETRY_DELAY_SECS)).await;
                }
            }
        }
    }

    if wifi_connected {
        // Wait for link to be up (with timeout)
        info!("Waiting for link...");
        match with_timeout(Duration::from_secs(10), stack.wait_link_up()).await {
            Ok(_) => info!("Link is up!"),
            Err(_) => {
                warn!("Link timeout");
                wifi_connected = false;
            }
        }

        // Wait for DHCP to configure the IP (with timeout)
        if wifi_connected {
            info!("Waiting for DHCP...");
            match with_timeout(Duration::from_secs(15), stack.wait_config_up()).await {
                Ok(_) => info!("DHCP configured!"),
                Err(_) => {
                    warn!("DHCP timeout");
                    wifi_connected = false;
                }
            }
        }

        // Log the IP address
        if let Some(config) = stack.config_v4() {
            info!("DHCP configured:");
            info!("  IP: {}", config.address);
            if let Some(gw) = config.gateway {
                info!("  Gateway: {}", gw);
            }
        }
    }

    // Update display based on WiFi status
    display.clear(BinaryColor::Off).unwrap();
    if wifi_connected {
        Text::new("WiFi Connected!", Point::new(5, 20), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new("Getting time...", Point::new(5, 40), text_style)
            .draw(&mut display)
            .unwrap();
    } else {
        Text::new("WiFi Failed!", Point::new(5, 20), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new("Using RTC time...", Point::new(5, 40), text_style)
            .draw(&mut display)
            .unwrap();
    }
    display.flush().unwrap();

    // ========================================================================
    // Get initial time (NTP if WiFi connected, otherwise RTC)
    // ========================================================================
    let mut current_unix_time: u64 = 0;
    let mut last_ntp_update = Instant::now();
    let mut time_valid = false;

    if wifi_connected {
        info!("Getting time from NTP server...");
        match get_ntp_time(stack).await {
            Ok(unix_time) => {
                current_unix_time = unix_time;
                time_valid = true;
                last_ntp_update = Instant::now();
                let (h, m, s) = unix_to_tehran_time(unix_time);
                info!("NTP Time received! Tehran time: {:02}:{:02}:{:02}", h, m, s);

                // Set DS3231 RTC with NTP time (Tehran timezone)
                let (year, month, day, hour, min, sec) = unix_to_datetime(unix_time);
                info!(
                    "Setting DS3231 RTC to: {}-{:02}-{:02} {:02}:{:02}:{:02}",
                    year, month, day, hour, min, sec
                );

                if let Some(datetime) = NaiveDate::from_ymd_opt(year, month as u32, day as u32)
                    .and_then(|d| d.and_hms_opt(hour as u32, min as u32, sec as u32))
                {
                    match rtc.set_datetime(&datetime) {
                        Ok(_) => info!("DS3231 RTC time set successfully!"),
                        Err(_) => warn!("Failed to set DS3231 RTC time"),
                    }
                } else {
                    warn!("Invalid datetime for DS3231");
                }
            }
            Err(_) => {
                warn!("Failed to get NTP time, falling back to RTC...");
            }
        }
    }

    // If NTP failed or WiFi not connected, try to read time from DS3231 RTC
    if !time_valid {
        info!("Reading time from DS3231 RTC...");
        match rtc.datetime() {
            Ok(datetime) => {
                let year = datetime.year();
                let month = datetime.month() as u8;
                let day = datetime.day() as u8;
                let hour = datetime.hour() as u8;
                let min = datetime.minute() as u8;
                let sec = datetime.second() as u8;

                info!(
                    "RTC Time: {}-{:02}-{:02} {:02}:{:02}:{:02}",
                    year, month, day, hour, min, sec
                );

                // Convert RTC time back to Unix timestamp
                current_unix_time = datetime_to_unix(year, month, day, hour, min, sec);
                time_valid = true;
                last_ntp_update = Instant::now();

                let (h, m, s) = unix_to_tehran_time(current_unix_time);
                info!("Using RTC time - Tehran: {:02}:{:02}:{:02}", h, m, s);
            }
            Err(_) => {
                warn!("Failed to read DS3231 RTC time - clock may not be set");
            }
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

                    // Update DS3231 RTC with new NTP time
                    let (year, month, day, hour, min, sec) = unix_to_datetime(unix_time);
                    if let Some(datetime) = NaiveDate::from_ymd_opt(year, month as u32, day as u32)
                        .and_then(|d| d.and_hms_opt(hour as u32, min as u32, sec as u32))
                    {
                        let _ = rtc.set_datetime(&datetime);
                        info!("DS3231 RTC re-synced");
                    }
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

                // Set DS3231 RTC with NTP time
                let (year, month, day, hour, min, sec) = unix_to_datetime(unix_time);
                if let Some(datetime) = NaiveDate::from_ymd_opt(year, month as u32, day as u32)
                    .and_then(|d| d.and_hms_opt(hour as u32, min as u32, sec as u32))
                {
                    match rtc.set_datetime(&datetime) {
                        Ok(_) => info!("DS3231 RTC time set successfully!"),
                        Err(_) => warn!("Failed to set DS3231 RTC time"),
                    }
                }
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
