#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "sentinel_config.h"
#include "shared_state.h"

// ---------------------------------------------------------------------------
// Task forward declarations
// ---------------------------------------------------------------------------
void task_i2c_slave_run(void *pvParameters);
void task_gps_run(void *pvParameters);
void task_bme280_run(void *pvParameters);
void task_mqtt_run(void *pvParameters);
void task_iq_stream_run(void *pvParameters);
void task_webserver_run(void *pvParameters);

// ---------------------------------------------------------------------------
// MQTT event queue — shared between i2c_slave (producer) and mqtt (consumer)
// ---------------------------------------------------------------------------
QueueHandle_t g_mqtt_event_queue;

// ---------------------------------------------------------------------------
// Logging tag
// ---------------------------------------------------------------------------
static const char *TAG = "main";

// ---------------------------------------------------------------------------
// Wi-Fi event group bits
// ---------------------------------------------------------------------------
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_num = 0;

// ---------------------------------------------------------------------------
// Uptime timer callback — fires every second
// ---------------------------------------------------------------------------
static void uptime_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    shared_state_tick_uptime();
}

// ---------------------------------------------------------------------------
// NVS helpers
// ---------------------------------------------------------------------------

/**
 * @brief Read a string value from NVS.  Returns false if the key is missing
 *        or the read fails; does not modify buf in that case.
 */
static bool nvs_read_str(nvs_handle_t handle, const char *key,
                         char *buf, size_t buf_len)
{
    size_t required = buf_len;
    esp_err_t err = nvs_get_str(handle, key, buf, &required);
    return (err == ESP_OK);
}

/**
 * @brief Attempt to load Wi-Fi credentials from NVS.
 *        Falls back to WIFI_FALLBACK_SSID / WIFI_FALLBACK_PASS.
 */
static void load_wifi_config(char *ssid, size_t ssid_len,
                             char *pass, size_t pass_len)
{
    nvs_handle_t h;
    bool got_ssid = false, got_pass = false;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        got_ssid = nvs_read_str(h, NVS_WIFI_SSID, ssid, ssid_len);
        got_pass = nvs_read_str(h, NVS_WIFI_PASS, pass, pass_len);
        nvs_close(h);
    }

    if (!got_ssid || ssid[0] == '\0') {
        strlcpy(ssid, WIFI_FALLBACK_SSID, ssid_len);
        strlcpy(pass, WIFI_FALLBACK_PASS, pass_len);
        ESP_LOGW(TAG, "No Wi-Fi credentials in NVS — using fallback AP");
    } else {
        ESP_LOGI(TAG, "Loaded Wi-Fi SSID from NVS: %s", ssid);
        if (!got_pass) {
            pass[0] = '\0';
        }
    }
}

/**
 * @brief Load device ID from NVS.  If absent, generate one from MAC address.
 */
static void load_device_id(char *buf, size_t buf_len)
{
    nvs_handle_t h;
    bool got_id = false;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        got_id = nvs_read_str(h, NVS_DEVICE_ID, buf, buf_len);
        nvs_close(h);
    }

    if (!got_id || buf[0] == '\0') {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(buf, buf_len, "sentinel_%02x%02x%02x",
                 mac[3], mac[4], mac[5]);
        ESP_LOGI(TAG, "Generated device ID from MAC: %s", buf);
    } else {
        ESP_LOGI(TAG, "Loaded device ID from NVS: %s", buf);
    }
}

/**
 * @brief Load MQTT broker config from NVS into g_state.
 */
static void load_mqtt_config(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    shared_state_lock();
    nvs_read_str(h, NVS_MQTT_HOST,
                 g_state.mqtt_host, sizeof(g_state.mqtt_host));

    uint16_t port = MQTT_DEFAULT_PORT;
    size_t sz = sizeof(port);
    nvs_get_blob(h, NVS_MQTT_PORT, &port, &sz);
    g_state.mqtt_port = port;
    shared_state_unlock();

    nvs_close(h);
    ESP_LOGI(TAG, "MQTT broker: %s:%u", g_state.mqtt_host, g_state.mqtt_port);
}

// ---------------------------------------------------------------------------
// Wi-Fi event handler
// ---------------------------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();

        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            shared_state_lock();
            g_state.wifi_connected = false;
            shared_state_unlock();

            if (s_wifi_retry_num < WIFI_MAX_RETRY) {
                esp_wifi_connect();
                s_wifi_retry_num++;
                ESP_LOGI(TAG, "Wi-Fi retry %d/%d", s_wifi_retry_num, WIFI_MAX_RETRY);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "Wi-Fi connection failed after %d retries", WIFI_MAX_RETRY);
            }
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        s_wifi_retry_num = 0;

        shared_state_lock();
        g_state.wifi_connected = true;
        shared_state_unlock();

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ---------------------------------------------------------------------------
// Wi-Fi initialisation
// ---------------------------------------------------------------------------
static void wifi_init_sta(void)
{
    char ssid[33] = {0};
    char pass[65] = {0};
    load_wifi_config(ssid, sizeof(ssid), pass, sizeof(pass));

    // Store SSID in state for web display
    shared_state_lock();
    strlcpy(g_state.wifi_ssid, ssid, sizeof(g_state.wifi_ssid));
    shared_state_unlock();

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {0};
    strlcpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    if (pass[0] == '\0') {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA initialised, connecting to SSID: %s", ssid);

    // Wait for connection (non-blocking for init — tasks can handle reconnect)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi: %s", ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect — tasks will retry in background");
    } else {
        ESP_LOGW(TAG, "Wi-Fi connect timed out — tasks will retry in background");
    }
}

// ---------------------------------------------------------------------------
// NVS initialisation with erase-on-corruption fallback
// ---------------------------------------------------------------------------
static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash corrupted or version mismatch — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS flash initialised");
}

// ---------------------------------------------------------------------------
// app_main — ESP-IDF entry point
// ---------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "=== Sentinel Bridge starting ===");
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    // 1. NVS flash
    nvs_init();

    // 2. Shared state
    shared_state_init();

    // 3. Device ID
    {
        char device_id[32];
        load_device_id(device_id, sizeof(device_id));
        shared_state_lock();
        strlcpy(g_state.device_id, device_id, sizeof(g_state.device_id));
        shared_state_unlock();
    }

    // 4. MQTT config from NVS
    load_mqtt_config();

    // 5. Create MQTT event queue (shared between i2c_slave and mqtt tasks)
    g_mqtt_event_queue = xQueueCreate(MQTT_EVENT_QUEUE_DEPTH,
                                      sizeof(sentinel_event_t));
    if (g_mqtt_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT event queue");
        esp_restart();
    }

    // 6. Wi-Fi (blocking up to 15 s, tasks retry in background)
    wifi_init_sta();

    // 7. Uptime timer — 1-second period
    TimerHandle_t uptime_timer = xTimerCreate("uptime", pdMS_TO_TICKS(1000),
                                              pdTRUE, NULL, uptime_timer_cb);
    if (uptime_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create uptime timer");
    } else {
        xTimerStart(uptime_timer, 0);
    }

    // 8. Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");

    xTaskCreate(task_i2c_slave_run, "i2c_slave",  STACK_I2C_SLAVE,
                NULL, PRIO_I2C_SLAVE, NULL);

    xTaskCreate(task_gps_run,       "gps",         STACK_GPS,
                NULL, PRIO_GPS, NULL);

    xTaskCreate(task_bme280_run,    "bme280",      STACK_BME280,
                NULL, PRIO_BME280, NULL);

    xTaskCreate(task_mqtt_run,      "mqtt",        STACK_MQTT,
                NULL, PRIO_MQTT, NULL);

    xTaskCreate(task_iq_stream_run, "iq_stream",   STACK_IQ_STREAM,
                NULL, PRIO_IQ_STREAM, NULL);

    xTaskCreate(task_webserver_run, "webserver",   STACK_WEBSERVER,
                NULL, PRIO_WEBSERVER, NULL);

    ESP_LOGI(TAG, "All tasks started. Free heap: %lu bytes",
             (unsigned long)esp_get_free_heap_size());
}
