#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// I2C command byte codes (SPEC Appendix D)
// ---------------------------------------------------------------------------
#define CMD_GET_GPS          0x01  // LPC4320 → ESP32: request current GPS fix
#define CMD_GET_SENSORS      0x02  // LPC4320 → ESP32: request sensor readings
#define CMD_SET_WIFI         0x03  // LPC4320 → ESP32: update Wi-Fi credentials
#define CMD_SET_MQTT         0x04  // LPC4320 → ESP32: update MQTT broker config
#define CMD_PUBLISH_EVENT    0x05  // LPC4320 → ESP32: publish sentinel event via MQTT
#define CMD_START_IQ_STREAM  0x06  // LPC4320 → ESP32: begin UDP IQ streaming
#define CMD_STOP_IQ_STREAM   0x07  // LPC4320 → ESP32: halt UDP IQ streaming
#define CMD_GET_STATUS       0x08  // LPC4320 → ESP32: request bridge status
#define CMD_ESP_EVENT        0x10  // ESP32 → LPC4320: unsolicited event notification

// ---------------------------------------------------------------------------
// I2C response/ACK bytes
// ---------------------------------------------------------------------------
#define I2C_ACK_OK           0x00
#define I2C_ACK_ERR          0xFF

// ---------------------------------------------------------------------------
// Response payload sizes (bytes)
// ---------------------------------------------------------------------------

// CMD_GET_GPS (0x01) → 12 bytes:
//   lat[4]  lon[4]  alt[2]  sats[1]  fix[1]
#define RESP_GPS_SIZE        12

// CMD_GET_SENSORS (0x02) → 8 bytes:
//   temp_c_x10[2]  humidity_x10[2]  pressure_hpa_x10[2]  light[2]
#define RESP_SENSORS_SIZE    8

// CMD_GET_STATUS (0x08) → 7 bytes:
//   wifi_connected[1]  mqtt_connected[1]  gps_fix[1]  uptime_s[4]
#define RESP_STATUS_SIZE     7

// CMD_SET_WIFI (0x03) payload from master:
//   null-terminated SSID (max 32 chars) + null-terminated password (max 64 chars)
#define CMD_SET_WIFI_MAX     96    // 32 + 1 + 63 (conservatively)

// CMD_SET_MQTT (0x04) payload from master:
//   broker_ip[4]  port[2]
#define CMD_SET_MQTT_SIZE    6

// CMD_START_IQ_STREAM (0x06) payload from master:
//   target_ip[4]  target_port[2]  decimation[2]
#define CMD_START_IQ_SIZE    8

// ---------------------------------------------------------------------------
// Sentinel event struct — 32 bytes, packed.
// Must EXACTLY mirror BusEvent in the LPC4320 firmware (event_types.hpp):
//   uint8_t  type         [offset 0,  size 1]
//   uint32_t timestamp_ms [offset 1,  size 4]
//   uint8_t  source       [offset 5,  size 1]
//   uint8_t  payload[26]  [offset 6,  size 26]
//   Total                          = 32 bytes
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  type;            // Event type identifier (see EVENT_TYPE_* below)
    uint32_t timestamp_ms;    // Milliseconds since LPC4320 boot
    uint8_t  source;          // Source subsystem identifier
    uint8_t  payload[26];     // Type-specific payload
} sentinel_event_t;

// Compile-time assertion: struct must be exactly 32 bytes
_Static_assert(sizeof(sentinel_event_t) == 32,
    "sentinel_event_t must be 32 bytes to match LPC4320 BusEvent");

// ---------------------------------------------------------------------------
// Event type codes
// ---------------------------------------------------------------------------
#define EVENT_TYPE_SIGNAL_DETECTED  0x01
#define EVENT_TYPE_GPS_UPDATE       0x02
#define EVENT_TYPE_SENSOR_DATA      0x03
#define EVENT_TYPE_SYSTEM_STATUS    0x0A

// ---------------------------------------------------------------------------
// SIGNAL_DETECTED payload layout (payload[26] interpretation):
//   freq_hz[4]   center frequency in Hz
//   rssi[2]      RSSI in dBm × 10 (signed)
//   bandwidth[4] signal bandwidth in Hz
//   duration_ms[2]
//   reserved[14]
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint32_t freq_hz;
    int16_t  rssi_x10;
    uint32_t bandwidth_hz;
    uint16_t duration_ms;
    uint8_t  reserved[14];
} signal_detected_payload_t;

_Static_assert(sizeof(signal_detected_payload_t) == 26,
    "signal_detected_payload_t must be 26 bytes");

// ---------------------------------------------------------------------------
// SYSTEM_STATUS payload layout (payload[26] interpretation):
//   battery_pct[1]   0-100
//   reserved[25]
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t battery_pct;
    uint8_t reserved[25];
} system_status_payload_t;

_Static_assert(sizeof(system_status_payload_t) == 26,
    "system_status_payload_t must be 26 bytes");
