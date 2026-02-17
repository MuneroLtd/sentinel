#pragma once
#include <cstdint>
#include <cstring>

// ============================================================
// Project Sentinel - Event Bus Type Definitions
// All agents/tasks must use these definitions unchanged.
// 32-byte fixed-size message (SPEC Appendix B).
// ============================================================

namespace sentinel {

// Event type enum (1 byte, SPEC Appendix B IDs)
enum class EventType : uint8_t {
    NONE              = 0x00,
    SIGNAL_DETECTED   = 0x01,
    GPS_UPDATE        = 0x02,
    SENSOR_DATA       = 0x03,
    APP_STATE         = 0x04,
    RADIO_REQUEST     = 0x05,
    RADIO_RELEASE     = 0x06,
    AIRCRAFT_POSITION = 0x07,
    CAPTURE_START     = 0x08,
    CAPTURE_STOP      = 0x09,
    SYSTEM_STATUS     = 0x0A,
};

// Task IDs (source field in BusEvent)
enum class TaskID : uint8_t {
    NONE            = 0x00,
    APP_TASK        = 0x01,
    RADIO_MANAGER   = 0x02,
    BG_SCANNER      = 0x03,
    KNOWLEDGE_BASE  = 0x04,
    ESP32_COMMS     = 0x05,
    UI_RENDERER     = 0x06,
    LOGGER          = 0x07,
};

// ---- Payload structs (26 bytes each, must fit in BusEvent::payload) ----

struct __attribute__((packed)) SignalDetectedPayload {
    uint32_t frequency_hz;      // Centre frequency in Hz
    int16_t  rssi_dbm;          // Signal strength (dBm × 10 for 0.1dB precision)
    uint16_t bandwidth_est_hz;  // Estimated bandwidth in Hz (0 = unknown)
    int32_t  gps_lat;           // Latitude × 1e6 (signed, degrees)
    int32_t  gps_lon;           // Longitude × 1e6 (signed, degrees)
    uint16_t duration_ms;       // How long signal was present
    uint8_t  pad[8];
};
static_assert(sizeof(SignalDetectedPayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) GPSUpdatePayload {
    int32_t  latitude;      // Degrees × 1e6 (signed)
    int32_t  longitude;     // Degrees × 1e6 (signed)
    int16_t  altitude_m;    // Metres above sea level
    uint16_t speed_kmh;     // Speed × 10 (0.1 km/h precision)
    uint16_t heading_deg;   // Heading × 10 (0.1 degree precision)
    uint8_t  satellites;    // Number of satellites in view
    uint8_t  fix_quality;   // 0=no fix, 1=GPS, 2=DGPS
    uint8_t  pad[10];
};
static_assert(sizeof(GPSUpdatePayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) SensorDataPayload {
    int16_t  temperature_c_x10; // °C × 10 (e.g. 214 = 21.4°C)
    uint16_t humidity_pct_x10;  // % × 10 (e.g. 560 = 56.0%)
    uint32_t pressure_pa;       // Pascals (e.g. 101300 = 1013 hPa)
    uint16_t light_lux;         // Lux (0 if not available)
    uint8_t  pad[18];
};
static_assert(sizeof(SensorDataPayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) AppStatePayload {
    uint8_t  app_id;            // AppID enum (see app_ids.hpp)
    uint8_t  state;             // 0=stopped, 1=running, 2=suspended
    uint32_t tuned_freq_hz;     // Currently tuned frequency (0 if N/A)
    uint8_t  pad[20];
};
static_assert(sizeof(AppStatePayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) RadioRequestPayload {
    uint8_t  requesting_task;   // TaskID enum
    uint32_t freq_hz;
    uint32_t bandwidth_hz;
    uint8_t  mode;              // RadioMode enum (0=RX, 1=TX, 2=ENERGY_DETECT)
    uint8_t  pad[16];
};
static_assert(sizeof(RadioRequestPayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) RadioReleasePayload {
    uint8_t  releasing_task;    // TaskID enum
    uint8_t  pad[25];
};
static_assert(sizeof(RadioReleasePayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) AircraftPositionPayload {
    uint8_t  icao[3];           // 24-bit ICAO address
    int32_t  lat;               // Degrees × 1e6
    int32_t  lon;               // Degrees × 1e6
    int16_t  altitude_ft;       // Altitude (feet / 25, range ±819km)
    uint16_t heading_deg;       // Heading × 10
    uint16_t speed_kts;         // Speed in knots
    uint16_t callsign_hash;     // CRC16 of callsign string
    uint8_t  pad[7];
};
static_assert(sizeof(AircraftPositionPayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) CaptureStartPayload {
    uint32_t freq_hz;
    uint32_t sample_rate_sps;
    uint32_t filename_hash;     // CRC32 of filename
    uint8_t  pad[14];
};
static_assert(sizeof(CaptureStartPayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) CaptureStopPayload {
    uint32_t freq_hz;
    uint32_t duration_ms;
    uint32_t file_size_kb;
    uint8_t  pad[14];
};
static_assert(sizeof(CaptureStopPayload) == 26, "Payload must be 26 bytes");

struct __attribute__((packed)) SystemStatusPayload {
    uint8_t  battery_pct;       // 0-100
    uint16_t sd_free_mb;        // Free SD space in MB (max 65535 MB)
    uint32_t uptime_s;          // Seconds since boot
    uint8_t  cpu_load_pct;      // 0-100, averaged over last tick
    uint8_t  heap_free_pct;     // FreeRTOS heap free %
    uint8_t  pad[18];
};
static_assert(sizeof(SystemStatusPayload) == 26, "Payload must be 26 bytes");

// ---- Master bus event (32 bytes fixed) ----

struct __attribute__((packed)) BusEvent {
    uint8_t  type;          // EventType
    uint32_t timestamp_ms;  // xTaskGetTickCount() × portTICK_PERIOD_MS
    uint8_t  source;        // TaskID
    uint8_t  payload[26];   // One of the *Payload structs above

    // Convenience constructors
    static BusEvent make(EventType t, TaskID src) {
        BusEvent ev{};
        ev.type   = static_cast<uint8_t>(t);
        ev.source = static_cast<uint8_t>(src);
        return ev;
    }

    template<typename T>
    const T& as() const {
        static_assert(sizeof(T) == 26, "Payload type must be 26 bytes");
        return *reinterpret_cast<const T*>(payload);
    }

    template<typename T>
    T& as() {
        static_assert(sizeof(T) == 26, "Payload type must be 26 bytes");
        return *reinterpret_cast<T*>(payload);
    }
};
static_assert(sizeof(BusEvent) == 32, "BusEvent must be exactly 32 bytes");

// Bitmask helper for subscribe()
constexpr uint32_t event_mask(EventType t) {
    return 1u << static_cast<uint8_t>(t);
}
#define EVENT_MASK(t) sentinel::event_mask(t)

} // namespace sentinel
