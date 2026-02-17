#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "sentinel_config.h"
#include "shared_state.h"
#include "protocols/sentinel_i2c.h"

static const char *TAG = "mqtt";

// ---------------------------------------------------------------------------
// Extern — MQTT event queue created in main.c
// ---------------------------------------------------------------------------
extern QueueHandle_t g_mqtt_event_queue;

// ---------------------------------------------------------------------------
// Module-level MQTT client handle
// ---------------------------------------------------------------------------
static esp_mqtt_client_handle_t s_client = NULL;
static volatile bool             s_connected = false;

// ---------------------------------------------------------------------------
// Reconnect backoff state (ms)
// ---------------------------------------------------------------------------
#define BACKOFF_INITIAL_MS   2000
#define BACKOFF_MAX_MS       60000

static uint32_t s_backoff_ms = BACKOFF_INITIAL_MS;

// ---------------------------------------------------------------------------
// Build MQTT URI from g_state (caller must hold mutex or use cached values)
// ---------------------------------------------------------------------------
static void build_uri(char *uri, size_t uri_len,
                      const char *host, uint16_t port)
{
    snprintf(uri, uri_len, "mqtt://%s:%u", host, port);
}

// ---------------------------------------------------------------------------
// Publish helpers
// ---------------------------------------------------------------------------

static void mqtt_publish(const char *topic, const char *payload,
                         int qos, int retain)
{
    if (!s_connected || s_client == NULL) {
        return;
    }
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload,
                                         (int)strlen(payload), qos, retain);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Publish failed on %s", topic);
    }
}

// ---------------------------------------------------------------------------
// Event type routing
// ---------------------------------------------------------------------------

static void publish_signal_detected(const sentinel_event_t *event,
                                    const char *device_id)
{
    if (sizeof(signal_detected_payload_t) > sizeof(event->payload)) {
        return;
    }

    signal_detected_payload_t sig;
    memcpy(&sig, event->payload, sizeof(sig));

    // Snapshot GPS under mutex
    int32_t lat, lon;
    shared_state_lock();
    lat = g_state.gps_lat;
    lon = g_state.gps_lon;
    shared_state_unlock();

    float lat_f = lat / 1.0e6f;
    float lon_f = lon / 1.0e6f;
    float rssi_f = sig.rssi_x10 / 10.0f;

    char topic[MQTT_TOPIC_MAX_LEN];
    char payload[MQTT_PAYLOAD_MAX_LEN];

    snprintf(topic, sizeof(topic), "sentinel/%s/signal_detected", device_id);
    snprintf(payload, sizeof(payload),
             "{\"freq\":%lu,\"rssi\":%.1f,\"lat\":%.6f,\"lon\":%.6f,\"ts\":%lu}",
             (unsigned long)sig.freq_hz,
             (double)rssi_f,
             (double)lat_f,
             (double)lon_f,
             (unsigned long)event->timestamp_ms);

    mqtt_publish(topic, payload, MQTT_QOS_LOW, 0);
    ESP_LOGD(TAG, "signal_detected freq=%lu rssi=%.1f",
             (unsigned long)sig.freq_hz, (double)rssi_f);
}

static void publish_gps_update(const sentinel_event_t *event,
                               const char *device_id)
{
    char topic[MQTT_TOPIC_MAX_LEN];
    char payload[MQTT_PAYLOAD_MAX_LEN];

    shared_state_lock();
    int32_t lat  = g_state.gps_lat;
    int32_t lon  = g_state.gps_lon;
    int16_t alt  = g_state.gps_alt_m;
    uint8_t sats = g_state.gps_sats;
    uint8_t fix  = g_state.gps_fix;
    shared_state_unlock();

    float lat_f = lat / 1.0e6f;
    float lon_f = lon / 1.0e6f;

    snprintf(topic, sizeof(topic), "sentinel/%s/gps", device_id);
    snprintf(payload, sizeof(payload),
             "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%d,\"sats\":%d,\"fix\":%d}",
             (double)lat_f, (double)lon_f, (int)alt, (int)sats, (int)fix);

    mqtt_publish(topic, payload, MQTT_QOS_LOW, 0);
}

static void publish_sensor_data(const sentinel_event_t *event,
                                const char *device_id)
{
    char topic[MQTT_TOPIC_MAX_LEN];
    char payload[MQTT_PAYLOAD_MAX_LEN];

    shared_state_lock();
    int16_t  temp = g_state.temp_c_x10;
    uint16_t hum  = g_state.humidity_x10;
    uint32_t pres = g_state.pressure_pa;
    shared_state_unlock();

    float temp_c    = temp / 10.0f;
    float hum_pct   = hum  / 10.0f;
    float pres_hpa  = pres / 100.0f;

    snprintf(topic, sizeof(topic), "sentinel/%s/sensors", device_id);
    snprintf(payload, sizeof(payload),
             "{\"temp_c\":%.1f,\"humidity_pct\":%.1f,\"pressure_hpa\":%.1f}",
             (double)temp_c, (double)hum_pct, (double)pres_hpa);

    mqtt_publish(topic, payload, MQTT_QOS_LOW, 0);
}

static void publish_system_status(const sentinel_event_t *event,
                                  const char *device_id)
{
    char topic[MQTT_TOPIC_MAX_LEN];
    char payload[MQTT_PAYLOAD_MAX_LEN];

    uint8_t battery_pct = 0;
    if (sizeof(system_status_payload_t) <= sizeof(event->payload)) {
        system_status_payload_t st;
        memcpy(&st, event->payload, sizeof(st));
        battery_pct = st.battery_pct;
    }

    shared_state_lock();
    uint32_t uptime   = g_state.uptime_s;
    bool     wifi_ok  = g_state.wifi_connected;
    shared_state_unlock();

    uint32_t free_heap = esp_get_free_heap_size();

    snprintf(topic, sizeof(topic), "sentinel/%s/status", device_id);
    snprintf(payload, sizeof(payload),
             "{\"uptime_s\":%lu,\"battery_pct\":%d,"
             "\"heap_free\":%lu,\"wifi\":%s}",
             (unsigned long)uptime,
             (int)battery_pct,
             (unsigned long)free_heap,
             wifi_ok ? "true" : "false");

    mqtt_publish(topic, payload, MQTT_QOS_HIGH, 1);  // retained
}

// ---------------------------------------------------------------------------
// Route a sentinel_event_t to the appropriate publisher
// ---------------------------------------------------------------------------
static void route_event(const sentinel_event_t *event, const char *device_id)
{
    switch (event->type) {
        case EVENT_TYPE_SIGNAL_DETECTED:
            publish_signal_detected(event, device_id);
            break;
        case EVENT_TYPE_GPS_UPDATE:
            publish_gps_update(event, device_id);
            break;
        case EVENT_TYPE_SENSOR_DATA:
            publish_sensor_data(event, device_id);
            break;
        case EVENT_TYPE_SYSTEM_STATUS:
            publish_system_status(event, device_id);
            break;
        default:
            ESP_LOGW(TAG, "Unknown event type 0x%02X — discarding", event->type);
            break;
    }
}

// ---------------------------------------------------------------------------
// MQTT event handler callback
// ---------------------------------------------------------------------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    (void)handler_args;
    (void)base;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            s_connected = true;
            s_backoff_ms = BACKOFF_INITIAL_MS;

            shared_state_lock();
            g_state.mqtt_connected = true;
            char device_id[32];
            strlcpy(device_id, g_state.device_id, sizeof(device_id));
            shared_state_unlock();

            // Publish initial retained status
            {
                char topic[MQTT_TOPIC_MAX_LEN];
                char payload[MQTT_PAYLOAD_MAX_LEN];
                shared_state_lock();
                uint32_t uptime = g_state.uptime_s;
                shared_state_unlock();
                snprintf(topic, sizeof(topic), "sentinel/%s/status", device_id);
                snprintf(payload, sizeof(payload),
                         "{\"uptime_s\":%lu,\"battery_pct\":0,"
                         "\"heap_free\":%lu,\"wifi\":true}",
                         (unsigned long)uptime,
                         (unsigned long)esp_get_free_heap_size());
                mqtt_publish(topic, payload, MQTT_QOS_HIGH, 1);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected — backoff %lu ms",
                     (unsigned long)s_backoff_ms);
            s_connected = false;
            shared_state_lock();
            g_state.mqtt_connected = false;
            shared_state_unlock();
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Connect / reconnect the MQTT client
// ---------------------------------------------------------------------------
static void mqtt_connect(const char *host, uint16_t port,
                         const char *device_id)
{
    if (host[0] == '\0') {
        ESP_LOGW(TAG, "No MQTT host configured — waiting");
        return;
    }

    char uri[96];
    build_uri(uri, sizeof(uri), host, port);
    ESP_LOGI(TAG, "Connecting to MQTT: %s (client_id: sentinel_%s)",
             uri, device_id);

    char client_id[48];
    snprintf(client_id, sizeof(client_id), "sentinel_%s", device_id);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri        = uri,
        .credentials.client_id     = client_id,
        .network.reconnect_timeout_ms = 5000,
        .buffer.size               = MQTT_PAYLOAD_MAX_LEN * 4,
    };

    if (s_client != NULL) {
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }

    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_client));
}

// ---------------------------------------------------------------------------
// Task entry point
// ---------------------------------------------------------------------------
void task_mqtt_run(void *pvParameters)
{
    (void)pvParameters;

    // Snapshot config
    char host[64]      = {0};
    char device_id[32] = {0};
    uint16_t port      = MQTT_DEFAULT_PORT;

    // Wait briefly for Wi-Fi to stabilise
    vTaskDelay(pdMS_TO_TICKS(3000));

    shared_state_lock();
    strlcpy(host,      g_state.mqtt_host,  sizeof(host));
    strlcpy(device_id, g_state.device_id,  sizeof(device_id));
    port = g_state.mqtt_port;
    shared_state_unlock();

    mqtt_connect(host, port, device_id);

    sentinel_event_t event;

    for (;;) {
        // Drain the event queue with a 5-second timeout
        if (xQueueReceive(g_mqtt_event_queue, &event,
                          pdMS_TO_TICKS(5000)) == pdTRUE) {
            if (s_connected) {
                route_event(&event, device_id);
            } else {
                ESP_LOGD(TAG, "Not connected — event type 0x%02X dropped",
                         event.type);
            }
            continue;
        }

        // --- Periodic tasks on queue timeout ---

        // Check if MQTT host config changed (updated via CMD_SET_MQTT)
        char new_host[64]  = {0};
        uint16_t new_port  = MQTT_DEFAULT_PORT;
        char new_dev[32]   = {0};

        shared_state_lock();
        strlcpy(new_host, g_state.mqtt_host,  sizeof(new_host));
        new_port = g_state.mqtt_port;
        strlcpy(new_dev,  g_state.device_id,  sizeof(new_dev));
        shared_state_unlock();

        if (strcmp(new_host, host) != 0 || new_port != port) {
            ESP_LOGI(TAG, "MQTT config changed — reconnecting");
            strlcpy(host, new_host, sizeof(host));
            strlcpy(device_id, new_dev, sizeof(device_id));
            port = new_port;
            s_connected = false;
            mqtt_connect(host, port, device_id);
        }

        // Reconnect with backoff if disconnected
        if (!s_connected && host[0] != '\0') {
            ESP_LOGI(TAG, "Attempting MQTT reconnect (backoff %lu ms)",
                     (unsigned long)s_backoff_ms);
            vTaskDelay(pdMS_TO_TICKS(s_backoff_ms));

            // Exponential backoff
            s_backoff_ms *= 2;
            if (s_backoff_ms > BACKOFF_MAX_MS) {
                s_backoff_ms = BACKOFF_MAX_MS;
            }

            // Re-check Wi-Fi before attempting connect
            bool wifi_ok = false;
            shared_state_lock();
            wifi_ok = g_state.wifi_connected;
            shared_state_unlock();

            if (wifi_ok) {
                mqtt_connect(host, port, device_id);
            }
        }
    }
    vTaskDelete(NULL);
}
