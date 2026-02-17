#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "sentinel_config.h"
#include "shared_state.h"

static const char *TAG = "gps";

// ---------------------------------------------------------------------------
// NMEA line buffer
// ---------------------------------------------------------------------------
#define NMEA_MAX_LEN  128

// ---------------------------------------------------------------------------
// NMEA helpers
// ---------------------------------------------------------------------------

/**
 * @brief Validate NMEA checksum.
 *        Format: $<data>*<XX>\r\n  where XX is XOR of bytes between $ and *
 */
static bool nmea_checksum_valid(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') {
        return false;
    }

    const char *star = strchr(sentence, '*');
    if (star == NULL || strlen(star) < 3) {
        return false;
    }

    uint8_t calc = 0;
    for (const char *p = sentence + 1; p < star; p++) {
        calc ^= (uint8_t)*p;
    }

    uint8_t received = (uint8_t)strtol(star + 1, NULL, 16);
    return (calc == received);
}

/**
 * @brief Return the Nth comma-separated token from sentence into buf.
 *        field_idx=0 returns the sentence type (e.g. "$GNGGA").
 *        Returns false if the field index is out of range.
 */
static bool nmea_field(const char *sentence, int field_idx,
                       char *buf, size_t buf_len)
{
    if (sentence == NULL || buf == NULL || buf_len == 0) {
        return false;
    }

    int current = 0;
    const char *p = sentence;
    const char *field_start = p;

    while (*p != '\0') {
        if (*p == ',' || *p == '*' || *p == '\r' || *p == '\n') {
            if (current == field_idx) {
                size_t len = (size_t)(p - field_start);
                if (len >= buf_len) len = buf_len - 1;
                memcpy(buf, field_start, len);
                buf[len] = '\0';
                return true;
            }
            current++;
            field_start = p + 1;
        }
        p++;
    }

    // Handle last field (no trailing comma)
    if (current == field_idx) {
        size_t len = strlen(field_start);
        // Strip *XX checksum if present
        const char *star = strchr(field_start, '*');
        if (star != NULL) {
            len = (size_t)(star - field_start);
        }
        if (len >= buf_len) len = buf_len - 1;
        memcpy(buf, field_start, len);
        buf[len] = '\0';
        return true;
    }

    return false;
}

/**
 * @brief Convert NMEA lat/lon (DDDMM.MMMM format) to degrees × 1e6.
 *        e.g. "5130.5000" N → 51.508333° → 51508333
 */
static int32_t nmea_coord_to_deg_e6(const char *coord, char hemi)
{
    if (coord == NULL || coord[0] == '\0') {
        return 0;
    }

    // Find the decimal point to determine degree digit count
    const char *dot = strchr(coord, '.');
    if (dot == NULL) {
        return 0;
    }

    // Degrees: all digits before the last 2 digits before '.'
    int dot_pos = (int)(dot - coord);
    int deg_digits = dot_pos - 2;  // 2 digits are always minutes integer part
    if (deg_digits <= 0) {
        return 0;
    }

    char deg_str[8] = {0};
    memcpy(deg_str, coord, (size_t)deg_digits);
    int32_t degrees = (int32_t)atoi(deg_str);

    // Minutes: 2 integer digits + decimal portion
    double minutes = atof(coord + deg_digits);

    // Total degrees with 6 decimal places
    double total = (double)degrees + minutes / 60.0;
    int32_t result = (int32_t)(total * 1.0e6 + 0.5);

    // Apply hemisphere sign
    if (hemi == 'S' || hemi == 'W') {
        result = -result;
    }

    return result;
}

// ---------------------------------------------------------------------------
// $GNGGA parser
// Fields: $GNGGA,hhmmss.ss,Lat,N/S,Lon,E/W,fix,sats,hdop,alt,M,...*cs
//   [0]  = sentence type
//   [1]  = UTC time
//   [2]  = Latitude
//   [3]  = N/S indicator
//   [4]  = Longitude
//   [5]  = E/W indicator
//   [6]  = Fix quality (0=no, 1=GPS, 2=DGPS)
//   [7]  = Satellites used
//   [8]  = HDOP
//   [9]  = Altitude (MSL, metres)
// ---------------------------------------------------------------------------
static void parse_gngga(const char *sentence)
{
    char lat_str[16]  = {0};
    char ns_str[4]    = {0};
    char lon_str[16]  = {0};
    char ew_str[4]    = {0};
    char fix_str[4]   = {0};
    char sats_str[4]  = {0};
    char alt_str[12]  = {0};

    if (!nmea_field(sentence, 2, lat_str, sizeof(lat_str))) return;
    if (!nmea_field(sentence, 3, ns_str,  sizeof(ns_str)))  return;
    if (!nmea_field(sentence, 4, lon_str, sizeof(lon_str))) return;
    if (!nmea_field(sentence, 5, ew_str,  sizeof(ew_str)))  return;
    if (!nmea_field(sentence, 6, fix_str, sizeof(fix_str))) return;
    if (!nmea_field(sentence, 7, sats_str,sizeof(sats_str)))return;
    if (!nmea_field(sentence, 9, alt_str, sizeof(alt_str))) return;

    uint8_t fix  = (uint8_t)atoi(fix_str);
    uint8_t sats = (uint8_t)atoi(sats_str);

    if (fix == 0 || lat_str[0] == '\0' || lon_str[0] == '\0') {
        shared_state_lock();
        g_state.gps_fix  = 0;
        g_state.gps_sats = sats;
        shared_state_unlock();
        return;
    }

    int32_t lat = nmea_coord_to_deg_e6(lat_str, ns_str[0]);
    int32_t lon = nmea_coord_to_deg_e6(lon_str, ew_str[0]);
    int16_t alt = (int16_t)(atof(alt_str) + 0.5);

    shared_state_lock();
    g_state.gps_lat  = lat;
    g_state.gps_lon  = lon;
    g_state.gps_alt_m = alt;
    g_state.gps_sats = sats;
    g_state.gps_fix  = fix;
    shared_state_unlock();
}

// ---------------------------------------------------------------------------
// $GNRMC parser
// Fields: $GNRMC,hhmmss.ss,A/V,Lat,N/S,Lon,E/W,speed,heading,date,...*cs
//   [0]  = sentence type
//   [1]  = UTC time
//   [2]  = Status (A=active, V=void)
//   [3]  = Latitude
//   [4]  = N/S indicator
//   [5]  = Longitude
//   [6]  = E/W indicator
//   [7]  = Speed over ground (knots)
//   [8]  = Track angle (degrees true)
// ---------------------------------------------------------------------------
static void parse_gnrmc(const char *sentence)
{
    char status_str[4]  = {0};
    char lat_str[16]    = {0};
    char ns_str[4]      = {0};
    char lon_str[16]    = {0};
    char ew_str[4]      = {0};
    char speed_str[12]  = {0};
    char heading_str[12]= {0};

    if (!nmea_field(sentence, 2, status_str,  sizeof(status_str)))  return;
    if (!nmea_field(sentence, 3, lat_str,     sizeof(lat_str)))     return;
    if (!nmea_field(sentence, 4, ns_str,      sizeof(ns_str)))      return;
    if (!nmea_field(sentence, 5, lon_str,     sizeof(lon_str)))     return;
    if (!nmea_field(sentence, 6, ew_str,      sizeof(ew_str)))      return;
    if (!nmea_field(sentence, 7, speed_str,   sizeof(speed_str)))   return;
    if (!nmea_field(sentence, 8, heading_str, sizeof(heading_str))) return;

    if (status_str[0] != 'A') {
        // Void / no fix — don't update position from RMC
        return;
    }

    int32_t lat = nmea_coord_to_deg_e6(lat_str, ns_str[0]);
    int32_t lon = nmea_coord_to_deg_e6(lon_str, ew_str[0]);

    // Speed: knots → km/h
    float speed_knots = (float)atof(speed_str);
    float speed_kmh   = speed_knots * 1.852f;

    float heading = (float)atof(heading_str);

    shared_state_lock();
    // Only update position if we already have a fix from GGA
    if (g_state.gps_fix > 0) {
        g_state.gps_lat = lat;
        g_state.gps_lon = lon;
    }
    g_state.gps_speed_kmh = speed_kmh;
    g_state.gps_heading   = heading;
    shared_state_unlock();
}

// ---------------------------------------------------------------------------
// Dispatch NMEA sentence to the correct parser
// ---------------------------------------------------------------------------
static void dispatch_nmea(const char *sentence)
{
    if (!nmea_checksum_valid(sentence)) {
        return;
    }

    // Match on the talker+sentence identifier at the start
    if (strncmp(sentence, "$GNGGA", 6) == 0 ||
        strncmp(sentence, "$GPGGA", 6) == 0) {
        parse_gngga(sentence);
    } else if (strncmp(sentence, "$GNRMC", 6) == 0 ||
               strncmp(sentence, "$GPRMC", 6) == 0) {
        parse_gnrmc(sentence);
    }
    // Other sentences ($GPGSV, $GPGSA, etc.) are silently ignored
}

// ---------------------------------------------------------------------------
// Task entry point
// ---------------------------------------------------------------------------
void task_gps_run(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Initialising GPS UART%d at %d baud (RX=%d TX=%d)",
             GPS_UART_NUM, GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);

    const uart_config_t uart_cfg = {
        .baud_rate  = GPS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM,
                                 GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM,
                                        GPS_RX_BUF_LEN, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "GPS UART ready — waiting for NMEA data");

    char line[NMEA_MAX_LEN];
    int  line_pos = 0;
    uint8_t byte;

    TickType_t last_log_tick = xTaskGetTickCount();
    const TickType_t log_interval = pdMS_TO_TICKS(10000);  // 10 s

    for (;;) {
        int got = uart_read_bytes(GPS_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (got <= 0) {
            // Yield on timeout so other tasks can run
            taskYIELD();
            continue;
        }

        if (byte == '$') {
            // Start of a new sentence — reset line buffer
            line_pos = 0;
            line[0] = '$';
            line_pos = 1;
        } else if (byte == '\n') {
            // End of sentence
            if (line_pos > 0 && line_pos < NMEA_MAX_LEN) {
                line[line_pos] = '\0';
                dispatch_nmea(line);
            }
            line_pos = 0;
        } else if (byte == '\r') {
            // Ignore carriage return
        } else {
            if (line_pos > 0 && line_pos < NMEA_MAX_LEN - 1) {
                line[line_pos++] = (char)byte;
            }
        }

        // Periodic status log
        TickType_t now = xTaskGetTickCount();
        if ((now - last_log_tick) >= log_interval) {
            last_log_tick = now;

            shared_state_lock();
            uint8_t fix  = g_state.gps_fix;
            uint8_t sats = g_state.gps_sats;
            int32_t lat  = g_state.gps_lat;
            int32_t lon  = g_state.gps_lon;
            shared_state_unlock();

            if (fix) {
                ESP_LOGI(TAG, "Fix=%u sats=%u lat=%ld.%06ld lon=%ld.%06ld",
                         fix, sats,
                         (long)(lat / 1000000), (long)labs(lat % 1000000),
                         (long)(lon / 1000000), (long)labs(lon % 1000000));
            } else {
                ESP_LOGI(TAG, "No GPS fix (sats tracked: %u)", sats);
            }
        }
    }
    vTaskDelete(NULL);
}
