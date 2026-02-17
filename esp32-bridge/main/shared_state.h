#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "protocols/sentinel_i2c.h"
#include "sentinel_config.h"

// ---------------------------------------------------------------------------
// Shared state — all fields protected by g_state_mutex.
// Every task reads/writes through shared_state_lock() / shared_state_unlock().
// ---------------------------------------------------------------------------

typedef struct {
    // --- GPS (from NEO-6M via UART) ---
    int32_t  gps_lat;         // Degrees × 1e6 (e.g. 51.505° → 51505000)
    int32_t  gps_lon;         // Degrees × 1e6
    int16_t  gps_alt_m;       // Altitude in metres MSL
    uint8_t  gps_sats;        // Satellites tracked
    uint8_t  gps_fix;         // 0 = no fix, 1 = GPS fix, 2 = DGPS fix, etc.
    float    gps_speed_kmh;   // Speed over ground km/h
    float    gps_heading;     // True heading degrees

    // --- Environmental sensors (from BME280) ---
    int16_t  temp_c_x10;      // Temperature: degrees C × 10 (e.g. 23.4° → 234)
    uint16_t humidity_x10;    // Relative humidity: % × 10 (e.g. 55.3% → 553)
    uint32_t pressure_pa;     // Pressure in Pascals (e.g. 101325)

    // --- Connectivity status ---
    bool     wifi_connected;
    bool     mqtt_connected;

    // --- IQ streaming ---
    bool     iq_streaming;
    uint32_t iq_target_ip;    // IPv4 in network byte order
    uint16_t iq_target_port;
    uint16_t iq_decimation;   // Decimation factor (1 = no decimation)

    // --- Uptime ---
    uint32_t uptime_s;        // Seconds since boot (updated by a timer)

    // --- Recent signal detection events (ring buffer, size RECENT_EVENTS_LEN) ---
    sentinel_event_t recent_events[RECENT_EVENTS_LEN];
    uint8_t          event_write_idx; // Next write position (mod RECENT_EVENTS_LEN)

    // --- Device ID (from NVS or generated) ---
    char     device_id[32];

    // --- MQTT broker config (from NVS) ---
    char     mqtt_host[64];
    uint16_t mqtt_port;

    // --- Wi-Fi SSID (for display) ---
    char     wifi_ssid[33];
} shared_state_t;

// ---------------------------------------------------------------------------
// Global instances — declared extern, defined in shared_state.c
// ---------------------------------------------------------------------------
extern shared_state_t     g_state;
extern SemaphoreHandle_t  g_state_mutex;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * @brief  Initialise shared state to defaults and create the mutex.
 *         Must be called once from app_main() before starting any tasks.
 */
void shared_state_init(void);

/**
 * @brief  Acquire the state mutex (blocking, portMAX_DELAY).
 *         Always paired with shared_state_unlock().
 */
static inline void shared_state_lock(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
}

/**
 * @brief  Release the state mutex.
 */
static inline void shared_state_unlock(void)
{
    xSemaphoreGive(g_state_mutex);
}

/**
 * @brief  Append a sentinel_event_t to the recent_events ring buffer.
 *         Acquires mutex internally.
 */
void shared_state_add_event(const sentinel_event_t *event);

/**
 * @brief  Increment uptime_s by one.  Called from a 1-second timer in main.c.
 */
void shared_state_tick_uptime(void);
