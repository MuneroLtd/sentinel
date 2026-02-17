# PROJECT SENTINEL

## PortaPack H4M Custom Firmware - Minimum Viable Product Specification

**Version:** 0.1  
**Date:** February 2026  
**Status:** DRAFT  
**Author:** Munero / Dan  
**Hardware:** HackRF One R10C + PortaPack H4M + ESP32-S3

---

## 1. Executive Summary

Project Sentinel is a custom firmware for the HackRF One + PortaPack H4M that transforms the device from a collection of isolated radio tools into an intelligent, context-aware RF monitoring platform. The MVP proves three foundational capabilities that Mayhem cannot deliver: background processing via RTOS, inter-app communication via an event bus, and ESP32 co-processor integration for sensors and networking.

The MVP is not a feature-complete product. It is the minimum implementation needed to validate the architecture and demonstrate that the hardware can support the full vision described in the project roadmap.

---

## 2. Problem Statement

The Mayhem firmware treats the PortaPack as a collection of standalone tools. Each app runs in isolation with no background processing, no shared state, and no persistent knowledge. The device forgets everything between sessions and cannot perform multiple tasks concurrently. The ESP32 GPIO expansion on the H4M is underutilised, serving only as a basic sensor bridge rather than a compute co-processor.

These limitations are architectural, not feature gaps. Adding more apps to Mayhem does not solve them. A new foundation is required.

---

## 3. MVP Objectives

The MVP must prove the following technical claims:

| # | Claim | Success Criteria |
|---|-------|-----------------|
| 1 | FreeRTOS can replace the cooperative scheduler without breaking radio functionality | FM audio reception works with identical quality to stock Mayhem while background tasks run concurrently |
| 2 | Multiple tasks can share the radio hardware safely via a task scheduler | Background RF energy logger runs while the user interacts with the spectrum analyser app |
| 3 | An event bus enables inter-app communication | A signal detection event from the background scanner triggers a visible alert on the home screen |
| 4 | The ESP32-S3 can serve as a co-processor over I2C/SPI | GPS coordinates from the ESP32 appear on the PortaPack display, and IQ samples stream to the ESP32 over SPI at >1 Mbit/s |
| 5 | The ESP32 can bridge data to a LAN | Decoded signal events are published via MQTT over Wi-Fi and received by a subscriber on the local network |

---

## 4. Out of Scope for MVP

- ML-based signal classification (post-MVP, requires trained models)
- Protocol Decoder Framework with loadable definitions (post-MVP)
- RF Mapper / heatmap generation (post-MVP, depends on GPS integration)
- Smart Replay with signal analysis (post-MVP)
- Collaborative multi-device mode (post-MVP)
- Automated capture workflows / scripting engine (post-MVP)
- Custom UI theme or redesigned menu system (post-MVP)
- Full Mayhem app compatibility (only core apps ported for MVP)

---

## 5. System Architecture

### 5.1 Hardware Configuration

| Component | Details |
|-----------|---------|
| HackRF One R10C | 1 MHz - 6 GHz SDR transceiver, 8-bit ADC, 20 Msps, half-duplex, USB 2.0 |
| PortaPack H4M | LPC4320 dual-core MCU (M4 + M0 @ 204 MHz), 200KB SRAM, 320x240 LCD, GPIO header with I2C |
| ESP32-S3 Dev Board | Dual-core 240 MHz, 520KB SRAM, Wi-Fi, Bluetooth, SPI slave + I2C, USB OTG |
| GPS Module (NEO-6M) | Connected to ESP32-S3 via UART |
| BME280 Sensor | Temperature, humidity, pressure via I2C to ESP32-S3 |
| microSD Card | FAT32, 32GB+, stores apps, knowledge base, captures, config |

### 5.2 Bus Architecture

| Bus | Between | Purpose | Speed |
|-----|---------|---------|-------|
| I2C | LPC4320 <> ESP32-S3 | Sensor data, commands, status | 400 kbit/s |
| SPI | LPC4320 <> ESP32-S3 | IQ sample streaming | 10-20 Mbit/s target |
| UART | ESP32-S3 <> NEO-6M | GPS NMEA data | 9600 baud |
| I2C | ESP32-S3 <> BME280 | Environmental sensors | 400 kbit/s |
| USB 2.0 | HackRF <> Host PC | DFU flashing, HackRF mode | 480 Mbit/s |

SPI availability on the H4M GPIO header is unconfirmed and must be verified in Phase 0. If the exposed GPIO pins cannot be remapped to an SPI peripheral on the LPC4320, the fallback is UART at 2-5 Mbaud, which provides approximately 2-5 Mbit/s throughput. This is sufficient for decimated narrowband IQ streaming but not full 20 Msps wideband capture.

### 5.3 Software Architecture

#### 5.3.1 LPC4320 Firmware (Sentinel Core)

Fork of Mayhem firmware with the cooperative scheduler replaced by FreeRTOS. The M0 core continues to run baseband DSP. The M4 core runs the RTOS kernel with the following tasks:

| Task | Priority | Purpose |
|------|----------|---------|
| Baseband Interface | Highest | Moves samples between M0 core and app/streaming pipelines |
| Radio Manager | High | Arbitrates radio hardware access between foreground app and background scanner |
| Event Bus | High | Routes typed messages between tasks via ring buffers |
| Background Scanner | Medium | Sweeps configurable frequency ranges, measures energy, emits detection events |
| ESP32 Comms | Medium | I2C command/response + SPI IQ streaming to co-processor |
| Knowledge Base Writer | Low | Flushes in-memory RF observations to SD card |
| UI Renderer | Low | Drives display and handles touch/button input |
| App Task | Normal | Runs the currently active foreground app |

#### 5.3.2 ESP32-S3 Firmware (Sentinel Bridge)

Built on ESP-IDF with FreeRTOS (native to ESP32). Responsibilities:

- Receive GPS data from NEO-6M, parse NMEA, serve coordinates to LPC4320 over I2C
- Read BME280 sensor data, serve to LPC4320 over I2C
- Receive IQ samples over SPI, buffer in RAM
- Forward received IQ samples over UDP to configurable homelab endpoint
- Publish event bus messages from LPC4320 to MQTT broker over Wi-Fi
- Serve basic web status page showing device state, GPS position, recent events
- Maintain backward compatibility with existing ESP32PP I2C protocol for sensor apps

#### 5.3.3 Event Bus Specification

The event bus is a typed message passing system using fixed-size ring buffers in shared SRAM. Each message is a 32-byte struct:

| Field | Size | Description |
|-------|------|-------------|
| type | 1 byte | Event type enum (SIGNAL_DETECTED, GPS_UPDATE, SENSOR_DATA, APP_STATE, etc.) |
| timestamp | 4 bytes | Milliseconds since boot (from RTOS tick) |
| source | 1 byte | Originating task ID |
| payload | 26 bytes | Type-specific data (frequency, signal strength, coordinates, etc.) |

Tasks subscribe to event types. The bus supports up to 8 subscribers and buffers 64 messages. Overflow drops oldest unread messages. Total memory cost: 2KB for the bus + subscriber table.

---

## 6. MVP Feature Set

### 6.1 Home Dashboard

Replaces the Mayhem home menu with a live status screen:

- **Top bar:** battery %, GPS fix status, Wi-Fi status, time from GPS
- **Centre:** mini spectrum waterfall (reduced resolution, 128 bins) showing current scanner band
- **Activity feed:** last 5 events from the event bus (signal detections, sensor readings)
- **Quick launch:** tap any event to open relevant app, or tap waterfall to open spectrum analyser

The dashboard renders as a low-priority UI task that updates at approximately 2-4 fps. It consumes the event bus passively and does not use the radio hardware directly.

### 6.2 Background RF Scanner

A medium-priority RTOS task that sweeps a configurable frequency range when the radio is not in use by a foreground app.

- **Scan configuration:** stored in `/SENTINEL/scan_config.json` on SD card. Defines one or more frequency ranges, dwell time per step (default 50ms), step size, and energy threshold for detection.
- **Detection logic:** when measured energy on a frequency exceeds the configured threshold, emit a SIGNAL_DETECTED event on the bus containing frequency, signal strength (RSSI), timestamp, and GPS coordinates (if available).
- **Radio arbitration:** when a foreground app requests the radio (e.g., user opens FM receiver), the scanner yields immediately and suspends. It resumes when the radio is released.
- **Baseline learning:** after 24+ hours of operation in a location, the scanner builds a noise floor baseline per frequency step. Detections are then relative to baseline rather than absolute threshold. Baseline stored on SD card per GPS grid square.

### 6.3 RF Knowledge Base

A persistent log of RF observations stored on the SD card in a compact binary format.

- **In-memory buffer:** 4KB ring buffer holding the most recent observations. Flushed to SD card every 60 seconds or when buffer reaches 75% capacity.
- **On-disk format:** flat binary file with 16-byte records: timestamp (4B), frequency (4B), signal strength (2B), GPS lat (3B), GPS lon (3B). Approximately 5.4MB per 24 hours of continuous scanning at 1 observation per second.
- **Indexing:** separate index file per day with frequency-sorted offsets for fast lookup. No complex database engine required.
- **Query interface:** simple API accessible from apps: `get_observations(freq_min, freq_max, time_start, time_end)` returns matching records from current day.

### 6.4 ESP32 Co-processor Integration

The ESP32-S3 operates as a subordinate processor managed by the LPC4320 firmware.

#### 6.4.1 Sensor Pipeline

- GPS: position updates published as GPS_UPDATE events at 1 Hz
- BME280: temperature, humidity, pressure published as SENSOR_DATA events at 0.1 Hz
- All sensor data available to any subscribed task or app via the event bus

#### 6.4.2 IQ Sample Streaming

When enabled, the LPC4320 taps the baseband pipeline on the M0 core, decimates to a configurable rate (target: 48 ksps for narrowband voice, 500 ksps for wideband), and streams via SPI to the ESP32-S3. The ESP32 buffers and forwards over UDP to a configurable IP:port on the local network.

- **Use case:** remote SDR. The PortaPack acts as a portable antenna and tuner, while a GNU Radio instance on the homelab does the heavy DSP processing.
- **Constraint:** SPI throughput determines maximum streaming sample rate. At 10 Mbit/s SPI, approximately 625 ksps of 16-bit IQ is achievable.

#### 6.4.3 MQTT Event Publishing

The ESP32 subscribes to the I2C event feed from the LPC4320 and publishes selected event types to an MQTT broker. Topic structure:

- `sentinel/{device_id}/signal_detected`
- `sentinel/{device_id}/gps`
- `sentinel/{device_id}/sensors`
- `sentinel/{device_id}/status`

Payload format is JSON. QoS 0 (fire and forget) for high-frequency events, QoS 1 for status changes.

#### 6.4.4 Web Status Page

Minimal HTTP server on the ESP32 serving a single-page status dashboard. Displays current GPS position, recent signal detections, sensor readings, and device status. No remote control in MVP, read-only monitoring only.

---

## 7. Ported Mayhem Apps

The MVP does not port all Mayhem apps. The following core apps are ported to run as RTOS tasks and participate in the event bus:

| App | Purpose | Event Bus Integration |
|-----|---------|----------------------|
| Spectrum Analyser | Wideband frequency visualisation | Publishes peak frequency events, subscribes to SIGNAL_DETECTED to auto-tune |
| FM Audio Receiver | Analogue FM reception with audio output | Publishes tuned frequency, receives GPS_UPDATE for display |
| ADS-B Receiver | Aircraft transponder decoding at 1090 MHz | Publishes AIRCRAFT_POSITION events, receives GPS_UPDATE for own-position display |
| Scanner | Frequency scanning across defined ranges | Publishes SIGNAL_DETECTED events, shares data with background scanner |
| Signal Capture/Replay | Record and playback IQ samples to/from SD card | GPS-tags captures automatically via GPS_UPDATE events |

Additional Mayhem apps can be ported incrementally post-MVP. The RTOS task interface and event bus API will be documented to enable community contributions.

---

## 8. Development Phases

### Phase 0: Toolchain and Hardware Verification (Days 1-3)

- Set up GCC ARM cross-compilation toolchain (arm-none-eabi-gcc)
- Clone Mayhem source, build from source, flash to device, verify identical operation
- Probe H4M GPIO header with multimeter to identify pin mapping to LPC4320 peripherals
- Confirm SPI peripheral availability on exposed pins (consult LPC4320 SCU register map)
- Wire ESP32-S3 to GPIO header (I2C minimum, SPI if confirmed)
- Flash stock ESP32PP firmware to ESP32-S3, verify I2C communication works

**Exit criteria:** Mayhem builds and runs from source. GPIO pinout documented. ESP32 communicates over I2C.

### Phase 1: RTOS Foundation (Week 1)

- Integrate FreeRTOS kernel into Mayhem build system
- Replace cooperative scheduler with RTOS task dispatcher on M4 core
- M0 baseband core untouched initially, communication via existing shared memory
- Port UI rendering loop to an RTOS task
- Port FM audio receiver app to an RTOS task
- Add UART serial debug logging over USB for diagnostics
- Verify FM reception quality matches stock Mayhem (A/B comparison)

**Exit criteria:** FM radio plays cleanly. Serial debug output visible. Multiple RTOS tasks running concurrently.

### Phase 2: Event Bus and Background Scanner (Week 2)

- Implement event bus ring buffer and subscriber table
- Implement Radio Manager task for hardware arbitration
- Implement Background Scanner task with configurable sweep
- Implement Knowledge Base Writer task with SD card persistence
- Build Home Dashboard screen consuming event bus
- Signal detection events display on dashboard while scanner runs

**Exit criteria:** Dashboard shows live signal detections. Scanner pauses when FM app opens, resumes when closed. Observation log written to SD card.

### Phase 3: ESP32 Co-processor (Week 2-3)

- Write Sentinel Bridge firmware for ESP32-S3 (ESP-IDF + FreeRTOS)
- Implement I2C command protocol between LPC4320 and ESP32-S3
- GPS data flowing from NEO-6M through ESP32 to PortaPack display
- BME280 sensor data flowing through to event bus
- Implement SPI IQ sample tap in M0 baseband pipeline (or UART fallback)
- Stream decimated IQ samples to ESP32, forward over UDP
- Verify IQ stream received and decodable by GNU Radio on homelab
- Implement MQTT event publishing
- Implement basic web status page

**Exit criteria:** GPS coordinates on PortaPack screen. IQ samples received on homelab. Signal detections in MQTT. Web page shows device status.

### Phase 4: App Porting and Integration (Week 3-4)

- Port Spectrum Analyser, ADS-B, Scanner, and Capture apps to RTOS tasks
- Integrate event bus into each ported app
- ADS-B shows own-position from GPS
- Signal captures auto-tagged with GPS coordinates
- End-to-end test: device scanning in background, user opens ADS-B, aircraft and own position visible, signal detections continue logging to SD, events publishing to MQTT

**Exit criteria:** All five ported apps functional. Background scanning works alongside foreground apps. Full event pipeline operational from radio through to MQTT.

---

## 9. Memory Budget

The LPC4320 has approximately 200KB of SRAM across multiple banks. This budget must accommodate all concurrent tasks. Estimated allocation:

| Component | Estimated RAM | Notes |
|-----------|--------------|-------|
| FreeRTOS kernel | ~8 KB | Kernel objects, idle task, timer task |
| Task stacks (8 tasks) | ~32 KB | 4 KB per task stack (tuneable per task) |
| Display buffer | ~38 KB | 320 x 240 x 16-bit (existing, cannot reduce) |
| Baseband buffers | ~32 KB | Existing M0/M4 shared buffers |
| Event bus | ~2 KB | 64 messages x 32 bytes + subscriber table |
| Knowledge base (in-memory) | ~4 KB | Ring buffer for recent observations |
| SPI TX buffer | ~8 KB | DMA ring buffer for IQ streaming to ESP32 |
| App working memory | ~40 KB | Variable per app, shared pool |
| USB/SD card drivers | ~16 KB | Existing drivers, minimal change |
| Headroom | ~20 KB | Safety margin for stack growth, fragmentation |
| **TOTAL** | **~200 KB** | **Fully allocated** |

This is tight but feasible. Task stack sizes must be profiled empirically and tuned down where possible. The app working memory pool is the flex point: complex apps get more, simple apps get less, managed by the RTOS memory allocator.

---

## 10. Risk Register

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| SPI not available on GPIO header pins | High | Medium | Fall back to UART at 2-5 Mbaud. Reduces IQ streaming throughput but all other features unaffected. |
| RTOS migration breaks baseband timing | High | Medium | M0 core runs independently; only M4 scheduling changes. Baseband tested in isolation before integration. |
| Memory budget overrun | High | Medium | Profile early, tune stack sizes, defer non-essential features. App working memory is the pressure release valve. |
| First flash does not boot | Medium | High | HackRF DFU recovery mode always available. Stock Mayhem can be reflashed in under 60 seconds. |
| ESP32 SPI slave timing issues | Medium | Medium | Start with I2C-only integration. SPI added incrementally with DMA and careful clock configuration. |
| FM audio quality regression | Low | Low | A/B comparison with stock Mayhem is Phase 1 exit criteria. Any regression blocks further work until resolved. |

---

## 11. Testing Strategy

All testing is manual, performed on physical hardware. No simulation environment exists for this platform.

- **Phase gates:** each phase has defined exit criteria. No phase begins until the previous phase criteria are met.
- **Serial debug logging:** all RTOS tasks emit structured log messages over USB serial. Log level configurable at runtime.
- **A/B comparison:** FM reception quality tested against stock Mayhem using identical antenna, frequency, and gain settings.
- **IQ validation:** streamed samples captured on homelab and compared against known signal characteristics (carrier frequency accuracy, modulation shape).
- **Endurance:** Phase 4 includes a 4-hour continuous run with background scanning, GPS logging, and MQTT publishing to verify memory stability (no leaks, no stack overflows).
- **Rollback:** stock Mayhem firmware kept on SD card for immediate recovery at any point.

---

## 12. Tools and Dependencies

| Tool | Purpose |
|------|---------|
| arm-none-eabi-gcc | Cross-compiler for LPC4320 (M4 + M0 cores) |
| CMake 3.16+ | Build system (Mayhem standard) |
| FreeRTOS 10.x | RTOS kernel (MIT licensed, LPC43xx port available) |
| ESP-IDF 5.x | ESP32-S3 development framework (includes FreeRTOS) |
| hackrf_spiflash / hackrf.app | Firmware flashing to HackRF |
| esptool.py / web flasher | Firmware flashing to ESP32-S3 |
| Mosquitto | MQTT broker on homelab for testing |
| GNU Radio | IQ stream validation on homelab |
| minicom / picocom | Serial debug terminal |
| Multimeter | GPIO pinout verification |

---

## 13. Deliverables

On completion of all four phases, the MVP delivers:

- Sentinel Core firmware binary (.bin) flashable to HackRF + PortaPack H4M
- Sentinel Bridge firmware binary (.bin) flashable to ESP32-S3
- SD card file structure with default scan configuration and app binaries
- GPIO wiring diagram for ESP32-S3 to H4M connection
- Build instructions for both firmware targets
- Architecture documentation sufficient for community contributors to add apps

All source code will be structured as a Mayhem fork with clear separation between Sentinel additions and upstream Mayhem code, enabling future upstream merges where applicable.

---

## Appendix A: Post-MVP Roadmap

Features deferred from MVP, in priority order:

1. **Protocol Decoder Framework** - loadable protocol definitions from SD card, community-contributed protocol files
2. **Signal Sentinel alerting** - anomaly detection against learned baselines, configurable alert rules
3. **RF Mapper** - GPS-tagged signal strength heatmaps, KML export
4. **Smart Replay** - signal analysis before replay, modulation identification, rolling code detection
5. **ML Signal Classification** - TFLite Micro on ESP32-S3, automatic modulation recognition
6. **Automated Capture Workflows** - JSON-defined trigger/action chains, overnight unattended operation
7. **Collaborative Mode** - multi-device event sharing over Wi-Fi, signal triangulation
8. **Spectrum Timelapse** - long-duration low-resolution waterfall recording and playback
9. **Remote Control** - full device control via ESP32 web interface
10. **Community App SDK** - documented API, example apps, build templates for third-party development

## Appendix B: Event Type Registry

| Event Type | ID | Payload Contents |
|------------|-----|-----------------|
| SIGNAL_DETECTED | 0x01 | frequency (4B), rssi (2B), bandwidth_est (2B), gps_lat (4B), gps_lon (4B), duration_ms (2B), pad (8B) |
| GPS_UPDATE | 0x02 | latitude (4B), longitude (4B), altitude (2B), speed (2B), heading (2B), satellites (1B), fix_quality (1B), pad (10B) |
| SENSOR_DATA | 0x03 | temperature_c (2B), humidity_pct (2B), pressure_hpa (2B), light_lux (2B), pad (18B) |
| APP_STATE | 0x04 | app_id (1B), state (1B), tuned_freq (4B), pad (20B) |
| RADIO_REQUEST | 0x05 | requesting_task (1B), freq (4B), bandwidth (4B), mode (1B), pad (16B) |
| RADIO_RELEASE | 0x06 | releasing_task (1B), pad (25B) |
| AIRCRAFT_POSITION | 0x07 | icao (3B), lat (4B), lon (4B), altitude_ft (2B), heading (2B), speed_kts (2B), callsign_hash (2B), pad (7B) |
| CAPTURE_START | 0x08 | freq (4B), sample_rate (4B), filename_hash (4B), pad (14B) |
| CAPTURE_STOP | 0x09 | freq (4B), duration_ms (4B), file_size_kb (4B), pad (14B) |
| SYSTEM_STATUS | 0x0A | battery_pct (1B), sd_free_mb (2B), uptime_s (4B), cpu_load_pct (1B), pad (18B) |

## Appendix C: SD Card Directory Structure

```
/SENTINEL/
  scan_config.json          # Background scanner configuration
  knowledge_base/
    2026-02-17.bin          # Daily observation log (binary)
    2026-02-17.idx          # Daily frequency-sorted index
    baselines/
      N51W001.baseline      # Noise floor baseline per GPS grid square
  captures/
    capture_433920_20260217_143022.iq   # IQ captures with freq/date/time naming
  logs/
    sentinel.log            # Debug log (circular, max 1MB)
  config/
    device.json             # Device ID, MQTT broker, Wi-Fi credentials
    mqtt_topics.json        # Event type to MQTT topic mapping
    stream.json             # IQ streaming config (target IP, port, decimation)
/APPS/                      # External app binaries (Mayhem compatible location)
/ADS-B/                     # ADS-B data files (Mayhem compatible)
/FREQMAN/                   # Frequency manager files (Mayhem compatible)
```

## Appendix D: I2C Command Protocol (LPC4320 <> ESP32-S3)

| Command | Direction | ID | Payload | Response |
|---------|-----------|-----|---------|----------|
| GET_GPS | LPC -> ESP | 0x01 | none | lat(4B), lon(4B), alt(2B), sats(1B), fix(1B) |
| GET_SENSORS | LPC -> ESP | 0x02 | none | temp(2B), hum(2B), pres(2B), light(2B) |
| SET_WIFI | LPC -> ESP | 0x03 | ssid + password (null-terminated) | status(1B) |
| SET_MQTT | LPC -> ESP | 0x04 | broker IP(4B), port(2B) | status(1B) |
| PUBLISH_EVENT | LPC -> ESP | 0x05 | event struct (32B) | ack(1B) |
| START_IQ_STREAM | LPC -> ESP | 0x06 | target_ip(4B), port(2B), decim(2B) | status(1B) |
| STOP_IQ_STREAM | LPC -> ESP | 0x07 | none | status(1B) |
| GET_STATUS | LPC -> ESP | 0x08 | none | wifi_connected(1B), mqtt_connected(1B), gps_fix(1B), uptime(4B) |
| ESP_EVENT | ESP -> LPC | 0x10 | event struct (32B) | ack(1B) |
