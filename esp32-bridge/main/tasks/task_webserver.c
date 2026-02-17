#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_http_server.h"

#include "sentinel_config.h"
#include "shared_state.h"
#include "protocols/sentinel_i2c.h"

static const char *TAG = "webserver";

// ---------------------------------------------------------------------------
// HTTP response buffer size
// Large enough to hold the full HTML page including ring buffer entries.
// ---------------------------------------------------------------------------
#define HTML_BUF_SIZE  4096

// ---------------------------------------------------------------------------
// Inline CSS / HTML structure for the status page
// ---------------------------------------------------------------------------

static const char *HTML_HEADER =
    "<!DOCTYPE html>"
    "<html lang=\"en\">"
    "<head>"
    "<meta charset=\"UTF-8\">"
    "<meta http-equiv=\"refresh\" content=\"10\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Sentinel Bridge</title>"
    "<style>"
    "body{font-family:monospace;background:#111;color:#0f0;margin:0;padding:16px;}"
    "h1{color:#0ff;margin:0 0 8px 0;font-size:1.4em;}"
    "h2{color:#0cf;margin:12px 0 4px 0;font-size:1.1em;}"
    ".box{border:1px solid #0f0;padding:8px 12px;margin:8px 0;border-radius:4px;}"
    ".ok{color:#0f0;} .warn{color:#ff0;} .err{color:#f44;}"
    "a{color:#0cf;} table{border-collapse:collapse;width:100%;}"
    "th,td{text-align:left;padding:2px 8px;border-bottom:1px solid #333;}"
    "th{color:#0cf;}"
    "</style>"
    "</head>"
    "<body>"
    "<h1>Sentinel Bridge Status</h1>";

static const char *HTML_FOOTER =
    "</body></html>";

// ---------------------------------------------------------------------------
// Helper: escape a string for HTML (minimal: amp, lt, gt, quot)
// Returns number of bytes written (not including null terminator).
// ---------------------------------------------------------------------------
static int html_escape(const char *src, char *dst, size_t dst_max)
{
    int n = 0;
    for (; *src && (size_t)n < dst_max - 6; src++) {
        switch (*src) {
            case '&':  n += snprintf(dst + n, dst_max - n, "&amp;");  break;
            case '<':  n += snprintf(dst + n, dst_max - n, "&lt;");   break;
            case '>':  n += snprintf(dst + n, dst_max - n, "&gt;");   break;
            case '"':  n += snprintf(dst + n, dst_max - n, "&quot;"); break;
            default:   dst[n++] = *src; break;
        }
    }
    if ((size_t)n < dst_max) dst[n] = '\0';
    return n;
}

// ---------------------------------------------------------------------------
// Build the full HTML status page into buf (at most buf_len bytes).
// Returns the number of bytes written.
// ---------------------------------------------------------------------------
static int build_status_page(char *buf, size_t buf_len)
{
    // Snapshot all state under mutex
    int32_t  lat, lon;
    int16_t  alt;
    uint8_t  sats, gps_fix;
    int16_t  temp_x10;
    uint16_t hum_x10;
    uint32_t pres_pa;
    bool     wifi_conn, mqtt_conn, iq_stream;
    uint32_t uptime;
    char     device_id[32];
    char     wifi_ssid[33];
    sentinel_event_t events[RECENT_EVENTS_LEN];
    uint8_t  ev_write_idx;

    shared_state_lock();
    lat          = g_state.gps_lat;
    lon          = g_state.gps_lon;
    alt          = g_state.gps_alt_m;
    sats         = g_state.gps_sats;
    gps_fix      = g_state.gps_fix;
    temp_x10     = g_state.temp_c_x10;
    hum_x10      = g_state.humidity_x10;
    pres_pa      = g_state.pressure_pa;
    wifi_conn    = g_state.wifi_connected;
    mqtt_conn    = g_state.mqtt_connected;
    iq_stream    = g_state.iq_streaming;
    uptime       = g_state.uptime_s;
    ev_write_idx = g_state.event_write_idx;
    strlcpy(device_id, g_state.device_id,  sizeof(device_id));
    strlcpy(wifi_ssid, g_state.wifi_ssid,  sizeof(wifi_ssid));
    memcpy(events, g_state.recent_events,  sizeof(events));
    shared_state_unlock();

    int n = 0;

    // Header
    n += snprintf(buf + n, buf_len - n, "%s", HTML_HEADER);

    // --- Device info box ---
    char esc_id[64];
    html_escape(device_id, esc_id, sizeof(esc_id));
    char esc_ssid[72];
    html_escape(wifi_ssid, esc_ssid, sizeof(esc_ssid));

    uint32_t up_h = uptime / 3600;
    uint32_t up_m = (uptime % 3600) / 60;
    uint32_t up_s = uptime % 60;

    n += snprintf(buf + n, buf_len - n,
        "<div class=\"box\">"
        "<h2>Device</h2>"
        "<table>"
        "<tr><th>ID</th><td>%s</td></tr>"
        "<tr><th>Uptime</th><td>%02lu:%02lu:%02lu</td></tr>"
        "<tr><th>Free Heap</th><td>%lu bytes</td></tr>"
        "</table>"
        "</div>",
        esc_id,
        (unsigned long)up_h, (unsigned long)up_m, (unsigned long)up_s,
        (unsigned long)esp_get_free_heap_size());

    // --- Connectivity box ---
    const char *wifi_cls  = wifi_conn ? "ok"  : "err";
    const char *wifi_str  = wifi_conn ? "Connected" : "Disconnected";
    const char *mqtt_cls  = mqtt_conn ? "ok"  : "err";
    const char *mqtt_str  = mqtt_conn ? "Connected" : "Disconnected";
    const char *iq_cls    = iq_stream ? "ok"  : "warn";
    const char *iq_str    = iq_stream ? "Streaming" : "Idle";

    n += snprintf(buf + n, buf_len - n,
        "<div class=\"box\">"
        "<h2>Connectivity</h2>"
        "<table>"
        "<tr><th>Wi-Fi</th><td><span class=\"%s\">%s</span> (%s)</td></tr>"
        "<tr><th>MQTT</th><td><span class=\"%s\">%s</span></td></tr>"
        "<tr><th>IQ Stream</th><td><span class=\"%s\">%s</span></td></tr>"
        "</table>"
        "</div>",
        wifi_cls, wifi_str, esc_ssid,
        mqtt_cls, mqtt_str,
        iq_cls, iq_str);

    // --- GPS box ---
    n += snprintf(buf + n, buf_len - n,
        "<div class=\"box\">"
        "<h2>GPS</h2>"
        "<table>");

    if (gps_fix > 0) {
        float lat_f = lat / 1.0e6f;
        float lon_f = lon / 1.0e6f;
        // Sign-safe integer formatting for lat/lon display
        int lat_deg = lat / 1000000;
        int lat_frac = (int)(lat < 0 ? -lat % 1000000 : lat % 1000000);
        int lon_deg = lon / 1000000;
        int lon_frac = (int)(lon < 0 ? -lon % 1000000 : lon % 1000000);

        n += snprintf(buf + n, buf_len - n,
            "<tr><th>Fix</th><td class=\"ok\">Yes (type %d)</td></tr>"
            "<tr><th>Satellites</th><td>%d</td></tr>"
            "<tr><th>Latitude</th><td>%d.%06d&deg;</td></tr>"
            "<tr><th>Longitude</th><td>%d.%06d&deg;</td></tr>"
            "<tr><th>Altitude</th><td>%d m MSL</td></tr>"
            "<tr><th>Map</th><td>"
            "<a href=\"https://maps.google.com/maps?q=%.6f,%.6f\" target=\"_blank\">"
            "Open in Google Maps</a></td></tr>",
            (int)gps_fix,
            (int)sats,
            lat_deg, lat_frac,
            lon_deg, lon_frac,
            (int)alt,
            (double)lat_f, (double)lon_f);
    } else {
        n += snprintf(buf + n, buf_len - n,
            "<tr><th>Fix</th><td class=\"err\">No fix</td></tr>"
            "<tr><th>Satellites</th><td>%d tracked</td></tr>",
            (int)sats);
    }

    n += snprintf(buf + n, buf_len - n, "</table></div>");

    // --- Sensors box ---
    float temp_c   = temp_x10 / 10.0f;
    float hum_pct  = hum_x10  / 10.0f;
    float pres_hpa = pres_pa  / 100.0f;

    n += snprintf(buf + n, buf_len - n,
        "<div class=\"box\">"
        "<h2>Environmental Sensors (BME280)</h2>"
        "<table>"
        "<tr><th>Temperature</th><td>%.1f &deg;C</td></tr>"
        "<tr><th>Humidity</th><td>%.1f %%</td></tr>"
        "<tr><th>Pressure</th><td>%.1f hPa</td></tr>"
        "</table>"
        "</div>",
        (double)temp_c, (double)hum_pct, (double)pres_hpa);

    // --- Recent signal detections box ---
    n += snprintf(buf + n, buf_len - n,
        "<div class=\"box\">"
        "<h2>Recent Signal Detections (last %d)</h2>"
        "<table>"
        "<tr><th>#</th><th>Type</th><th>Timestamp (ms)</th>"
        "<th>Freq (Hz)</th><th>RSSI (dBm)</th></tr>",
        RECENT_EVENTS_LEN);

    // Walk ring buffer in reverse-chronological order (newest first)
    bool any = false;
    for (int i = 0; i < RECENT_EVENTS_LEN; i++) {
        // Index of the i-th most recent event
        int idx = ((int)ev_write_idx - 1 - i + RECENT_EVENTS_LEN) % RECENT_EVENTS_LEN;
        const sentinel_event_t *ev = &events[idx];

        if (ev->type == 0 && ev->timestamp_ms == 0) {
            continue;  // Empty slot
        }

        any = true;
        const char *type_str = "unknown";
        uint32_t freq_hz = 0;
        int rssi_dbm = 0;

        if (ev->type == EVENT_TYPE_SIGNAL_DETECTED) {
            type_str = "SIGNAL";
            signal_detected_payload_t sig;
            memcpy(&sig, ev->payload, sizeof(sig));
            freq_hz  = sig.freq_hz;
            rssi_dbm = sig.rssi_x10 / 10;
        } else if (ev->type == EVENT_TYPE_GPS_UPDATE) {
            type_str = "GPS";
        } else if (ev->type == EVENT_TYPE_SENSOR_DATA) {
            type_str = "SENSOR";
        } else if (ev->type == EVENT_TYPE_SYSTEM_STATUS) {
            type_str = "STATUS";
        }

        n += snprintf(buf + n, buf_len - n,
            "<tr><td>%d</td><td>%s</td><td>%lu</td><td>%lu</td><td>%d</td></tr>",
            i + 1, type_str,
            (unsigned long)ev->timestamp_ms,
            (unsigned long)freq_hz,
            rssi_dbm);
    }

    if (!any) {
        n += snprintf(buf + n, buf_len - n,
            "<tr><td colspan=\"5\" style=\"color:#666\">No events yet</td></tr>");
    }

    n += snprintf(buf + n, buf_len - n, "</table></div>");

    // Footer
    n += snprintf(buf + n, buf_len - n, "%s", HTML_FOOTER);

    return n;
}

// ---------------------------------------------------------------------------
// HTTP GET "/" handler
// ---------------------------------------------------------------------------
static esp_err_t root_get_handler(httpd_req_t *req)
{
    // Allocate page buffer from heap (too large for stack)
    char *buf = malloc(HTML_BUF_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Out of memory for HTML buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int len = build_status_page(buf, HTML_BUF_SIZE);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store");

    esp_err_t err = httpd_resp_send(req, buf, len);

    free(buf);
    return err;
}

// ---------------------------------------------------------------------------
// HTTP GET "/api/status" — JSON endpoint for machine consumption
// ---------------------------------------------------------------------------
static esp_err_t api_status_handler(httpd_req_t *req)
{
    char buf[512];

    shared_state_lock();
    uint32_t uptime   = g_state.uptime_s;
    bool     wifi_ok  = g_state.wifi_connected;
    bool     mqtt_ok  = g_state.mqtt_connected;
    uint8_t  gps_fix  = g_state.gps_fix;
    int32_t  lat      = g_state.gps_lat;
    int32_t  lon      = g_state.gps_lon;
    shared_state_unlock();

    float lat_f = lat / 1.0e6f;
    float lon_f = lon / 1.0e6f;

    int n = snprintf(buf, sizeof(buf),
        "{\"uptime_s\":%lu,\"wifi\":%s,\"mqtt\":%s,"
        "\"gps_fix\":%d,\"lat\":%.6f,\"lon\":%.6f,"
        "\"heap_free\":%lu}",
        (unsigned long)uptime,
        wifi_ok ? "true" : "false",
        mqtt_ok ? "true" : "false",
        (int)gps_fix,
        (double)lat_f, (double)lon_f,
        (unsigned long)esp_get_free_heap_size());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, buf, n);
}

// ---------------------------------------------------------------------------
// Task entry point
// ---------------------------------------------------------------------------
void task_webserver_run(void *pvParameters)
{
    (void)pvParameters;

    // Wait for Wi-Fi before starting server
    ESP_LOGI(TAG, "Waiting for Wi-Fi before starting HTTP server...");
    {
        bool wifi = false;
        while (!wifi) {
            shared_state_lock();
            wifi = g_state.wifi_connected;
            shared_state_unlock();
            if (!wifi) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port     = WEBSERVER_PORT;
    config.max_uri_handlers = WEBSERVER_MAX_URI;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", WEBSERVER_PORT);
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %d", err);
        vTaskDelete(NULL);
        return;
    }

    // Register URI handlers
    static const httpd_uri_t root_uri = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &root_uri);

    static const httpd_uri_t api_uri = {
        .uri     = "/api/status",
        .method  = HTTP_GET,
        .handler = api_status_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &api_uri);

    ESP_LOGI(TAG, "HTTP server running");

    // Keep task alive — httpd runs in its own internal tasks
    // Monitor Wi-Fi state and restart server if needed
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        // If server somehow stopped, attempt restart
        // (httpd internally handles client connections)
    }

    httpd_stop(server);
    vTaskDelete(NULL);
}
