// SPDX-License-Identifier: MIT
// Project Sentinel — ESP32 Bridge I2C Driver Implementation
//
// Uses hal/i2c.hpp directly — no portapack:: dependencies.
// All I2C wire transfers use i2c_write_read() which issues a write phase
// followed by a repeated-START read phase in a single transaction.

#include "i2c_esp32.hpp"

#include "../hal/i2c.hpp"
#include "../bsp/portapack_pins.hpp"
#include "../event_bus/event_types.hpp"

#include "FreeRTOS.h"
#include "task.h"   // vTaskDelay

#include <cstdint>
#include <cstring>
#include <cstddef>

extern "C" void sentinel_log(const char* tag, const char* fmt, ...);

// ---------------------------------------------------------------------------
// Internal constants
// ---------------------------------------------------------------------------
namespace {

// Command codes — SPEC Appendix D
static constexpr uint8_t CMD_GET_GPS        = 0x01u;
static constexpr uint8_t CMD_GET_SENSORS    = 0x02u;
static constexpr uint8_t CMD_GET_STATUS     = 0x03u;
static constexpr uint8_t CMD_SEND_EVENT     = 0x05u;
static constexpr uint8_t CMD_SET_WIFI       = 0x10u;
static constexpr uint8_t CMD_SET_MQTT       = 0x11u;

static constexpr int     RETRIES            = 3;
static constexpr uint32_t RETRY_DELAY_MS    = 5u;

// ---------------------------------------------------------------------------
// GPS response layout (26 bytes, SPEC Appendix D)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) GpsResponse {
    int32_t  lat_i32;       // degrees × 1 000 000
    int32_t  lon_i32;       // degrees × 1 000 000
    int16_t  alt_i16;       // metres
    uint16_t speed_u16;     // km/h × 10
    uint16_t heading_u16;   // degrees × 10
    uint8_t  sats_u8;
    uint8_t  fix_u8;        // 0 = no fix
    uint8_t  pad[10];
};
static_assert(sizeof(GpsResponse) == 26, "GpsResponse size mismatch");

// ---------------------------------------------------------------------------
// BME280 sensor response layout (8 bytes, SPEC Appendix D)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) SensorResponse {
    int16_t  temp_i16;      // °C × 10
    uint16_t humidity_u16;  // % × 10
    uint32_t pressure_u32;  // Pa
};
static_assert(sizeof(SensorResponse) == 8, "SensorResponse size mismatch");

// ---------------------------------------------------------------------------
// ESP32 status response (4 bytes)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) StatusResponse {
    uint8_t  wifi_ok;
    uint8_t  mqtt_ok;
    uint16_t uptime_s;
};
static_assert(sizeof(StatusResponse) == 4, "StatusResponse size mismatch");

// ---------------------------------------------------------------------------
// Wi-Fi configuration command payload (65 bytes total)
//   [0]      = CMD_SET_WIFI
//   [1..32]  = SSID (null-terminated, 32 bytes)
//   [33..64] = Password (null-terminated, 32 bytes)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) SetWifiFrame {
    uint8_t cmd;
    char    ssid[32];
    char    password[32];
};
static_assert(sizeof(SetWifiFrame) == 65, "SetWifiFrame size mismatch");

// ---------------------------------------------------------------------------
// MQTT configuration command payload (8 bytes total)
//   [0]    = CMD_SET_MQTT
//   [1..4] = broker_ip (big-endian)
//   [5..6] = port (big-endian)
//   [7]    = reserved
// ---------------------------------------------------------------------------
struct __attribute__((packed)) SetMqttFrame {
    uint8_t  cmd;
    uint32_t broker_ip;     // big-endian IPv4
    uint16_t port;          // big-endian
    uint8_t  reserved;
};
static_assert(sizeof(SetMqttFrame) == 8, "SetMqttFrame size mismatch");

// ---------------------------------------------------------------------------
// do_write_read
//
// Send 'wr_len' bytes then read 'rd_len' bytes from the ESP32.
// Retries on NAK up to RETRIES times.
// ---------------------------------------------------------------------------
static bool do_write_read(const uint8_t* wr, size_t wr_len,
                           uint8_t* rd, size_t rd_len)
{
    for (int attempt = 0; attempt < RETRIES; ++attempt) {
        bool ok;
        if (rd_len > 0 && rd != nullptr) {
            ok = i2c_write_read(ESP32_I2C_BUS, ESP32_I2C_ADDR,
                                wr, wr_len,
                                rd, rd_len);
        } else {
            ok = i2c_write(ESP32_I2C_BUS, ESP32_I2C_ADDR, wr, wr_len);
        }
        if (ok) return true;
        if (attempt < RETRIES - 1) {
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }
    return false;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API implementation
// ---------------------------------------------------------------------------

namespace sentinel::esp32 {

bool init(void)
{
    i2c_init(ESP32_I2C_BUS, 400000u);

    // Probe the ESP32 by reading its status — if it ACKs, it's alive.
    bool wifi_ok, mqtt_ok;
    uint16_t uptime = 0;
    const bool alive = get_status(&wifi_ok, &mqtt_ok, &uptime);

    if (alive) {
        sentinel_log("ESP32_DRV", "ESP32 init OK wifi=%d mqtt=%d uptime=%u",
                     (int)wifi_ok, (int)mqtt_ok, (unsigned)uptime);
    } else {
        sentinel_log("ESP32_DRV", "ESP32 init FAILED — not responding on I2C1:0x42");
    }
    return alive;
}

bool get_gps(GPSUpdatePayload& out)
{
    uint8_t cmd = CMD_GET_GPS;
    GpsResponse resp{};

    if (!do_write_read(&cmd, 1u,
                       reinterpret_cast<uint8_t*>(&resp), sizeof(resp))) {
        return false;
    }

    if (resp.fix_u8 == 0u) {
        return false;  // No fix — data is not valid
    }

    out.latitude    = resp.lat_i32;
    out.longitude   = resp.lon_i32;
    out.altitude_m  = resp.alt_i16;
    out.speed_kmh   = resp.speed_u16;
    out.heading_deg = resp.heading_u16;
    out.satellites  = resp.sats_u8;
    out.fix_quality = resp.fix_u8;
    __builtin_memset(out.pad, 0, sizeof(out.pad));

    return true;
}

bool get_sensors(SensorDataPayload& out)
{
    uint8_t cmd = CMD_GET_SENSORS;
    SensorResponse resp{};

    if (!do_write_read(&cmd, 1u,
                       reinterpret_cast<uint8_t*>(&resp), sizeof(resp))) {
        return false;
    }

    out.temperature_c_x10 = resp.temp_i16;
    out.humidity_pct_x10  = resp.humidity_u16;
    out.pressure_pa       = resp.pressure_u32;
    out.light_lux         = 0u;
    __builtin_memset(out.pad, 0, sizeof(out.pad));

    return true;
}

bool send_event(const BusEvent& ev)
{
    static_assert(sizeof(BusEvent) == 32, "BusEvent must be 32 bytes");

    // Frame: [CMD_SEND_EVENT][32 bytes BusEvent] = 33 bytes
    uint8_t frame[33];
    frame[0] = CMD_SEND_EVENT;
    __builtin_memcpy(&frame[1], &ev, sizeof(ev));

    return do_write_read(frame, sizeof(frame), nullptr, 0u);
}

bool set_wifi(const char* ssid, const char* password)
{
    SetWifiFrame frame{};
    frame.cmd = CMD_SET_WIFI;

    // Copy strings with bounded length, ensure null-termination
    __builtin_strncpy(frame.ssid,     ssid,     sizeof(frame.ssid)     - 1u);
    __builtin_strncpy(frame.password, password, sizeof(frame.password) - 1u);
    frame.ssid    [sizeof(frame.ssid)     - 1u] = '\0';
    frame.password[sizeof(frame.password) - 1u] = '\0';

    return do_write_read(reinterpret_cast<const uint8_t*>(&frame),
                         sizeof(frame),
                         nullptr, 0u);
}

bool set_mqtt(uint32_t broker_ip, uint16_t port)
{
    SetMqttFrame frame{};
    frame.cmd       = CMD_SET_MQTT;
    frame.broker_ip = broker_ip;
    frame.port      = port;
    frame.reserved  = 0u;

    return do_write_read(reinterpret_cast<const uint8_t*>(&frame),
                         sizeof(frame),
                         nullptr, 0u);
}

bool get_status(bool* wifi_ok, bool* mqtt_ok, uint16_t* uptime_s)
{
    uint8_t cmd = CMD_GET_STATUS;
    StatusResponse resp{};

    if (!do_write_read(&cmd, 1u,
                       reinterpret_cast<uint8_t*>(&resp), sizeof(resp))) {
        return false;
    }

    if (wifi_ok)  *wifi_ok  = (resp.wifi_ok  != 0u);
    if (mqtt_ok)  *mqtt_ok  = (resp.mqtt_ok  != 0u);
    if (uptime_s) *uptime_s = resp.uptime_s;

    return true;
}

} // namespace sentinel::esp32
