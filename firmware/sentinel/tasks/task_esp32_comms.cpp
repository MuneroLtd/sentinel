// SPDX-License-Identifier: MIT
// Project Sentinel — ESP32 Bridge Communication Task (task_esp32_comms)
//
// Polls the ESP32 co-processor via I2C using the Project Sentinel HAL.
// Does NOT use portapack:: namespace or any Mayhem code.
//
// I2C command protocol (SPEC Appendix D):
//   CMD 0x01 — Get GPS data        → read 26 bytes
//   CMD 0x02 — Get BME280 sensors  → read 8 bytes
//   CMD 0x03 — Get ESP32 status    → read 4 bytes
//   CMD 0x05 — Send event to ESP32 → write 33 bytes (1 cmd + 32 payload)
//
// Poll rates:
//   GPS    : 1 Hz  (every 1000 ms)
//   BME280 : 0.1 Hz (every 10 000 ms)
//
// The task also forwards SIGNAL_DETECTED and SYSTEM_STATUS events to the
// ESP32 bridge so it can relay them via MQTT.

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "../event_bus/event_bus.hpp"
#include "../event_bus/event_types.hpp"
#include "../tasks/task_ids.hpp"
#include "../hal/i2c.hpp"
#include "../bsp/portapack_pins.hpp"

#include <cstdint>
#include <cstring>

extern "C" void sentinel_log(const char* tag, const char* fmt, ...);

// ---------------------------------------------------------------------------
// ESP32 I2C command codes (SPEC Appendix D)
// ---------------------------------------------------------------------------
namespace {

static constexpr uint8_t CMD_GET_GPS     = 0x01u;
static constexpr uint8_t CMD_GET_SENSORS = 0x02u;
static constexpr uint8_t CMD_GET_STATUS  = 0x03u;
static constexpr uint8_t CMD_SEND_EVENT  = 0x05u;

// Retry parameters
static constexpr int     MAX_RETRIES     = 3;
static constexpr uint32_t RETRY_DELAY_MS = 5u;

// ---------------------------------------------------------------------------
// GPS response layout (26 bytes, SPEC Appendix D)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) GpsResponse {
    int32_t  lat_i32;       // degrees × 1 000 000
    int32_t  lon_i32;       // degrees × 1 000 000
    int16_t  alt_i16;       // metres
    uint16_t speed_u16;     // km/h × 10
    uint16_t heading_u16;   // degrees × 10
    uint8_t  sats_u8;       // satellites in view
    uint8_t  fix_u8;        // 0=no fix, 1=GPS fix, 2=DGPS
    uint8_t  pad[10];
};
static_assert(sizeof(GpsResponse) == 26, "GpsResponse must be 26 bytes");

// ---------------------------------------------------------------------------
// BME280 response layout (8 bytes, SPEC Appendix D)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) SensorResponse {
    int16_t  temp_i16;      // °C × 10
    uint16_t humidity_u16;  // % × 10
    uint32_t pressure_u32;  // Pa
};
static_assert(sizeof(SensorResponse) == 8, "SensorResponse must be 8 bytes");

// ---------------------------------------------------------------------------
// ESP32 status response (4 bytes, SPEC Appendix D)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) Esp32StatusResponse {
    uint8_t  wifi_ok;       // 1 = connected
    uint8_t  mqtt_ok;       // 1 = connected
    uint16_t uptime_s;      // seconds since ESP32 boot
};
static_assert(sizeof(Esp32StatusResponse) == 4, "Esp32StatusResponse must be 4 bytes");

// ---------------------------------------------------------------------------
// esp32_cmd
//
// Send a 1-byte command to the ESP32 and optionally read rx_len bytes back.
// Retries up to MAX_RETRIES times with RETRY_DELAY_MS between attempts.
// Returns true on success.
// ---------------------------------------------------------------------------
static bool esp32_cmd(uint8_t cmd, uint8_t* rx_buf, size_t rx_len)
{
    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        bool ok;

        if (rx_len > 0 && rx_buf != nullptr) {
            // Write command byte, then read response (repeated-start)
            ok = i2c_write_read(ESP32_I2C_BUS,
                                ESP32_I2C_ADDR,
                                &cmd, 1u,
                                rx_buf, rx_len);
        } else {
            // Write-only (used for CMD_SEND_EVENT where we pre-built the buffer)
            ok = i2c_write(ESP32_I2C_BUS, ESP32_I2C_ADDR, &cmd, 1u);
        }

        if (ok) return true;

        if (attempt < MAX_RETRIES - 1) {
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// esp32_send_event
//
// Sends a 32-byte BusEvent to the ESP32 via CMD 0x05.
// Wire format: [0x05][32 bytes of BusEvent] = 33 bytes total.
// ---------------------------------------------------------------------------
static bool esp32_send_event(const sentinel::BusEvent& ev)
{
    // Build a 33-byte frame: command byte + raw event
    uint8_t frame[33];
    frame[0] = CMD_SEND_EVENT;
    __builtin_memcpy(&frame[1], &ev, sizeof(ev));  // sizeof(BusEvent) == 32

    return i2c_write(ESP32_I2C_BUS, ESP32_I2C_ADDR, frame, sizeof(frame));
}

// ---------------------------------------------------------------------------
// poll_gps
// ---------------------------------------------------------------------------
static void poll_gps()
{
    uint8_t raw[sizeof(GpsResponse)];
    if (!esp32_cmd(CMD_GET_GPS, raw, sizeof(raw))) {
        sentinel_log("ESP32", "GPS poll failed");
        return;
    }

    GpsResponse resp;
    __builtin_memcpy(&resp, raw, sizeof(resp));

    if (resp.fix_u8 == 0u) {
        return;  // No fix — don't publish stale data
    }

    sentinel::BusEvent ev = sentinel::BusEvent::make(
        sentinel::EventType::GPS_UPDATE,
        sentinel::TaskID::ESP32_COMMS);
    ev.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    auto& p = ev.as<sentinel::GPSUpdatePayload>();
    p.latitude    = resp.lat_i32;
    p.longitude   = resp.lon_i32;
    p.altitude_m  = resp.alt_i16;
    p.speed_kmh   = resp.speed_u16;
    p.heading_deg = resp.heading_u16;
    p.satellites  = resp.sats_u8;
    p.fix_quality = resp.fix_u8;
    __builtin_memset(p.pad, 0, sizeof(p.pad));

    sentinel::EventBus::instance().publish(ev);
}

// ---------------------------------------------------------------------------
// poll_sensors
// ---------------------------------------------------------------------------
static void poll_sensors()
{
    uint8_t raw[sizeof(SensorResponse)];
    if (!esp32_cmd(CMD_GET_SENSORS, raw, sizeof(raw))) {
        sentinel_log("ESP32", "Sensor poll failed");
        return;
    }

    SensorResponse resp;
    __builtin_memcpy(&resp, raw, sizeof(resp));

    sentinel::BusEvent ev = sentinel::BusEvent::make(
        sentinel::EventType::SENSOR_DATA,
        sentinel::TaskID::ESP32_COMMS);
    ev.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    auto& p = ev.as<sentinel::SensorDataPayload>();
    p.temperature_c_x10 = resp.temp_i16;
    p.humidity_pct_x10  = resp.humidity_u16;
    p.pressure_pa       = resp.pressure_u32;
    p.light_lux         = 0u;   // BME280 has no light sensor
    __builtin_memset(p.pad, 0, sizeof(p.pad));

    sentinel::EventBus::instance().publish(ev);
}

// ---------------------------------------------------------------------------
// poll_esp32_status (optional — for debug / watchdog)
// ---------------------------------------------------------------------------
static void poll_esp32_status()
{
    uint8_t raw[sizeof(Esp32StatusResponse)];
    if (!esp32_cmd(CMD_GET_STATUS, raw, sizeof(raw))) {
        sentinel_log("ESP32", "Status poll failed");
        return;
    }

    Esp32StatusResponse resp;
    __builtin_memcpy(&resp, raw, sizeof(resp));

    sentinel_log("ESP32", "wifi=%d mqtt=%d uptime=%us",
                 (int)resp.wifi_ok,
                 (int)resp.mqtt_ok,
                 (unsigned)resp.uptime_s);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// task_esp32_comms_entry
//
// Priority: ESP32_COMMS (4).  Stack: 768 words (3 KB).
// ---------------------------------------------------------------------------
extern "C" void task_esp32_comms_entry(void* /*pvParameters*/)
{
    sentinel_log("ESP32", "ESP32 comms task started");

    // Initialise I2C bus for ESP32
    i2c_init(ESP32_I2C_BUS, 400000u);

    // -----------------------------------------------------------------------
    // Subscribe to events we need to forward to the ESP32
    // -----------------------------------------------------------------------
    static StaticQueue_t s_queue_static;
    static uint8_t       s_queue_storage[16 * sizeof(sentinel::BusEvent)];
    const QueueHandle_t  rx_queue = xQueueCreateStatic(
        16,
        sizeof(sentinel::BusEvent),
        s_queue_storage,
        &s_queue_static);

    const uint32_t mask =
        EVENT_MASK(sentinel::EventType::SIGNAL_DETECTED) |
        EVENT_MASK(sentinel::EventType::SYSTEM_STATUS);

    const int sub_id = sentinel::EventBus::instance().subscribe(rx_queue, mask);
    if (sub_id < 0) {
        sentinel_log("ESP32", "ERROR: EventBus subscribe failed");
        vTaskSuspend(nullptr);
    }

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------
    TickType_t last_gps_tick    = xTaskGetTickCount();
    TickType_t last_sensor_tick = xTaskGetTickCount();
    TickType_t last_status_tick = xTaskGetTickCount();

    static constexpr uint32_t GPS_INTERVAL_MS    = 1000u;
    static constexpr uint32_t SENSOR_INTERVAL_MS = 10000u;
    static constexpr uint32_t STATUS_INTERVAL_MS = 60000u;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();

        // --- Poll GPS at 1 Hz ---
        if ((now - last_gps_tick) >= pdMS_TO_TICKS(GPS_INTERVAL_MS)) {
            last_gps_tick = now;
            poll_gps();
        }

        // --- Poll BME280 sensors at 0.1 Hz ---
        if ((now - last_sensor_tick) >= pdMS_TO_TICKS(SENSOR_INTERVAL_MS)) {
            last_sensor_tick = now;
            poll_sensors();
        }

        // --- Poll ESP32 status occasionally ---
        if ((now - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
            last_status_tick = now;
            poll_esp32_status();
        }

        // --- Forward bus events to ESP32 (non-blocking drain) ---
        sentinel::BusEvent ev{};
        while (xQueueReceive(rx_queue, &ev, 0) == pdTRUE) {
            if (!esp32_send_event(ev)) {
                sentinel_log("ESP32", "send_event failed (type=0x%02x)", (unsigned)ev.type);
            }
        }

        // Sleep for the GPS poll period
        vTaskDelay(pdMS_TO_TICKS(GPS_INTERVAL_MS));
    }

    // Unreachable
    sentinel::EventBus::instance().unsubscribe(sub_id);
}
