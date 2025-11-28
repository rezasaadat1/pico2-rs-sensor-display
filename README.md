# Pico 2 W Sensor Display with RTC

A Rust embedded project for the Raspberry Pi Pico 2 W that displays environmental sensor data and time on an OLED display.

## Features

- **WiFi Connectivity** - Connects to WiFi to sync time via NTP
- **NTP Time Sync** - Gets accurate time from NTP servers (with Tehran timezone UTC+3:30)
- **DS3231 RTC Backup** - Battery-backed RTC maintains time when WiFi is unavailable
- **BME280 Sensor** - Reads temperature, humidity, and pressure
- **SSD1306/SSD1315 OLED Display** - 128x64 pixel display showing all data
- **Automatic Fallback** - Uses RTC time if WiFi connection fails

## Hardware Requirements

| Component | Description | I2C Address |
|-----------|-------------|-------------|
| Raspberry Pi Pico 2 W | RP2350-based board with WiFi | - |
| BME280 | Temperature, humidity, pressure sensor | 0x77 (secondary) |
| DS3231 | Real-time clock module | 0x68 |
| SSD1306/SSD1315 | 128x64 OLED display | 0x3C |

## Wiring

All I2C devices share the same bus:

| Pico 2 W Pin | Function | Connected To |
|--------------|----------|--------------|
| GPIO 4 | I2C0 SDA | BME280, DS3231, OLED SDA |
| GPIO 5 | I2C0 SCL | BME280, DS3231, OLED SCL |
| 3.3V | Power | All sensors VCC |
| GND | Ground | All sensors GND |

## Configuration

Edit the WiFi credentials in `src/main.rs`:

```rust
const WIFI_NETWORK: &str = "YOUR_SSID";
const WIFI_PASSWORD: &str = "YOUR_PASSWORD";
```

### Timezone

The default timezone is Tehran (UTC+3:30). To change it, modify:

```rust
const TEHRAN_OFFSET_SECONDS: i64 = 3 * 3600 + 30 * 60; // UTC+3:30
```

## Building

### Prerequisites

1. Install Rust and cargo
2. Add the ARM Cortex-M target:

   ```bash
   rustup target add thumbv8m.main-none-eabihf
   ```

### Compile

```bash
cargo build --release
```

The output binary will be in `target/thumbv8m.main-none-eabihf/release/`

## Flashing

### Using probe-rs (Recommended)

Install probe-rs:

```bash
cargo install probe-rs-tools
```

Flash and run with RTT logging:

```bash
cargo run --release
```

Or flash only:

```bash
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/pico2-rs-sensor-display
```

### Using picotool

Alternatively, use `picotool` to flash the firmware:

```bash
picotool load -x target/thumbv8m.main-none-eabihf/release/pico2-rs-sensor-display -t elf
```

Or use the VS Code task: **Run Project**

## Operation

1. On boot, the device attempts to connect to WiFi (5 retries)
2. If WiFi connects successfully:
   - Fetches time from NTP servers
   - Sets the DS3231 RTC with NTP time
   - Re-syncs NTP every 10 minutes
3. If WiFi fails:
   - Falls back to reading time from DS3231 RTC
   - Continues displaying sensor data with RTC time
4. Display shows:
   - Tehran time (HH:MM:SS)
   - Temperature (°C)
   - Humidity (%)
   - Pressure (hPa)
5. Onboard LED blinks to indicate activity

## NTP Servers

The project tries multiple NTP servers in order:

- pool.ntp.org (Cloudflare): 162.159.200.1
- time.nist.gov: 129.6.15.28
- time.google.com: 216.239.35.0
- time.apple.com: 17.253.34.123

## Dependencies

- [embassy](https://github.com/embassy-rs/embassy) - Async embedded framework
- [cyw43](https://crates.io/crates/cyw43) - WiFi driver for CYW43 chip
- [bme280](https://crates.io/crates/bme280) - BME280 sensor driver
- [ds323x](https://crates.io/crates/ds323x) - DS3231/DS3232/DS3234 RTC driver
- [ssd1306](https://crates.io/crates/ssd1306) - SSD1306/SSD1315 OLED driver
- [embedded-graphics](https://crates.io/crates/embedded-graphics) - 2D graphics library

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
