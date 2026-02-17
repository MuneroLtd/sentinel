#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "sentinel_config.h"
#include "shared_state.h"
#include "protocols/sentinel_i2c.h"

static const char *TAG = "i2c_slave";

// ---------------------------------------------------------------------------
// Extern — MQTT event queue created in main.c
// ---------------------------------------------------------------------------
extern QueueHandle_t g_mqtt_event_queue;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * @brief Write ACK/NAK byte back to master.
 */
static void send_ack(uint8_t ack_byte)
{
    i2c_slave_write_buffer(I2C_SLAVE_PORT, &ack_byte, 1, pdMS_TO_TICKS(10));
}

/**
 * @brief Block-read exactly `len` bytes from the slave receive buffer.
 *        Returns true on success, false on timeout.
 */
static bool slave_read(uint8_t *buf, size_t len, TickType_t timeout_ticks)
{
    size_t received = 0;
    TickType_t start = xTaskGetTickCount();

    while (received < len) {
        int got = i2c_slave_read_buffer(I2C_SLAVE_PORT,
                                        buf + received,
                                        len - received,
                                        pdMS_TO_TICKS(5));
        if (got > 0) {
            received += (size_t)got;
        }
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            if (received < len) {
                ESP_LOGW(TAG, "slave_read timeout: wanted %u got %u",
                         (unsigned)len, (unsigned)received);
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

/**
 * CMD_GET_GPS (0x01)
 * Response: lat[4] lon[4] alt[2] sats[1] fix[1]  = 12 bytes
 */
static void handle_get_gps(void)
{
    uint8_t buf[RESP_GPS_SIZE];

    shared_state_lock();
    int32_t lat   = g_state.gps_lat;
    int32_t lon   = g_state.gps_lon;
    int16_t alt   = g_state.gps_alt_m;
    uint8_t sats  = g_state.gps_sats;
    uint8_t fix   = g_state.gps_fix;
    shared_state_unlock();

    // Little-endian packing
    buf[0]  = (uint8_t)(lat & 0xFF);
    buf[1]  = (uint8_t)((lat >> 8) & 0xFF);
    buf[2]  = (uint8_t)((lat >> 16) & 0xFF);
    buf[3]  = (uint8_t)((lat >> 24) & 0xFF);

    buf[4]  = (uint8_t)(lon & 0xFF);
    buf[5]  = (uint8_t)((lon >> 8) & 0xFF);
    buf[6]  = (uint8_t)((lon >> 16) & 0xFF);
    buf[7]  = (uint8_t)((lon >> 24) & 0xFF);

    buf[8]  = (uint8_t)(alt & 0xFF);
    buf[9]  = (uint8_t)((alt >> 8) & 0xFF);

    buf[10] = sats;
    buf[11] = fix;

    i2c_slave_write_buffer(I2C_SLAVE_PORT, buf, RESP_GPS_SIZE, pdMS_TO_TICKS(50));
    send_ack(I2C_ACK_OK);
}

/**
 * CMD_GET_SENSORS (0x02)
 * Response: temp_c_x10[2] humidity_x10[2] pressure_pa[2] light[2]  = 8 bytes
 * Note: pressure transmitted as hPa×10 (uint16) to fit in 2 bytes.
 */
static void handle_get_sensors(void)
{
    uint8_t buf[RESP_SENSORS_SIZE];

    shared_state_lock();
    int16_t  temp = g_state.temp_c_x10;
    uint16_t hum  = g_state.humidity_x10;
    // Convert Pa to hPa×10 for compact transport (101325 Pa → 10132)
    uint16_t pres = (uint16_t)(g_state.pressure_pa / 10);
    shared_state_unlock();

    buf[0] = (uint8_t)(temp & 0xFF);
    buf[1] = (uint8_t)((temp >> 8) & 0xFF);

    buf[2] = (uint8_t)(hum & 0xFF);
    buf[3] = (uint8_t)((hum >> 8) & 0xFF);

    buf[4] = (uint8_t)(pres & 0xFF);
    buf[5] = (uint8_t)((pres >> 8) & 0xFF);

    // Light sensor not populated on this rev — send 0xFFFF as sentinel
    buf[6] = 0xFF;
    buf[7] = 0xFF;

    i2c_slave_write_buffer(I2C_SLAVE_PORT, buf, RESP_SENSORS_SIZE, pdMS_TO_TICKS(50));
    send_ack(I2C_ACK_OK);
}

/**
 * CMD_SET_WIFI (0x03)
 * Payload: null-terminated SSID (max 32 B) + null-terminated password (max 64 B)
 */
static void handle_set_wifi(void)
{
    uint8_t payload[CMD_SET_WIFI_MAX];
    memset(payload, 0, sizeof(payload));

    if (!slave_read(payload, CMD_SET_WIFI_MAX, pdMS_TO_TICKS(200))) {
        send_ack(I2C_ACK_ERR);
        return;
    }

    // Split SSID and password at the first null terminator
    char *ssid = (char *)payload;
    char *pass = NULL;
    size_t ssid_len = strnlen(ssid, 32);
    if (ssid_len < CMD_SET_WIFI_MAX - 1) {
        pass = (char *)payload + ssid_len + 1;
    } else {
        pass = (char *)payload + 32;  // Fallback: assume fixed layout
        payload[CMD_SET_WIFI_MAX - 1] = '\0';
    }

    ESP_LOGI(TAG, "CMD_SET_WIFI: SSID='%s'", ssid);

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        send_ack(I2C_ACK_ERR);
        return;
    }
    nvs_set_str(h, NVS_WIFI_SSID, ssid);
    nvs_set_str(h, NVS_WIFI_PASS, pass ? pass : "");
    nvs_commit(h);
    nvs_close(h);

    // Update display copy in state
    shared_state_lock();
    strlcpy(g_state.wifi_ssid, ssid, sizeof(g_state.wifi_ssid));
    shared_state_unlock();

    send_ack(I2C_ACK_OK);

    // Trigger reconnect by restarting — simple and reliable for config change
    ESP_LOGI(TAG, "Wi-Fi credentials updated — restarting in 1 s");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/**
 * CMD_SET_MQTT (0x04)
 * Payload: broker_ip[4] (network byte order) + port[2] (little-endian)
 */
static void handle_set_mqtt(void)
{
    uint8_t payload[CMD_SET_MQTT_SIZE];

    if (!slave_read(payload, CMD_SET_MQTT_SIZE, pdMS_TO_TICKS(100))) {
        send_ack(I2C_ACK_ERR);
        return;
    }

    uint32_t ip_nbo;
    memcpy(&ip_nbo, payload, 4);

    uint16_t port = (uint16_t)(payload[4] | ((uint16_t)payload[5] << 8));
    if (port == 0) {
        port = MQTT_DEFAULT_PORT;
    }

    // Convert IP to dotted string for NVS storage and MQTT URI
    uint8_t a = (ip_nbo >> 24) & 0xFF;
    uint8_t b = (ip_nbo >> 16) & 0xFF;
    uint8_t c = (ip_nbo >>  8) & 0xFF;
    uint8_t d = (ip_nbo >>  0) & 0xFF;
    char host_str[20];
    snprintf(host_str, sizeof(host_str), "%u.%u.%u.%u", a, b, c, d);

    ESP_LOGI(TAG, "CMD_SET_MQTT: host=%s port=%u", host_str, port);

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, NVS_MQTT_HOST, host_str);
        nvs_set_blob(h, NVS_MQTT_PORT, &port, sizeof(port));
        nvs_commit(h);
        nvs_close(h);
    }

    shared_state_lock();
    strlcpy(g_state.mqtt_host, host_str, sizeof(g_state.mqtt_host));
    g_state.mqtt_port = port;
    shared_state_unlock();

    send_ack(I2C_ACK_OK);
    // The MQTT task monitors g_state.mqtt_host for changes and will reconnect
}

/**
 * CMD_PUBLISH_EVENT (0x05)
 * Payload: sentinel_event_t (32 bytes)
 */
static void handle_publish_event(void)
{
    sentinel_event_t event;

    if (!slave_read((uint8_t *)&event, sizeof(sentinel_event_t),
                    pdMS_TO_TICKS(200))) {
        send_ack(I2C_ACK_ERR);
        return;
    }

    ESP_LOGD(TAG, "CMD_PUBLISH_EVENT: type=0x%02x ts=%lu",
             event.type, (unsigned long)event.timestamp_ms);

    // Store in ring buffer for web display
    shared_state_add_event(&event);

    // Enqueue for MQTT task
    if (xQueueSend(g_mqtt_event_queue, &event, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "MQTT queue full — event dropped");
        send_ack(I2C_ACK_ERR);
        return;
    }

    send_ack(I2C_ACK_OK);
}

/**
 * CMD_START_IQ_STREAM (0x06)
 * Payload: target_ip[4] target_port[2] decimation[2]  = 8 bytes
 */
static void handle_start_iq_stream(void)
{
    uint8_t payload[CMD_START_IQ_SIZE];

    if (!slave_read(payload, CMD_START_IQ_SIZE, pdMS_TO_TICKS(100))) {
        send_ack(I2C_ACK_ERR);
        return;
    }

    uint32_t ip;
    memcpy(&ip, payload, 4);
    uint16_t port = (uint16_t)(payload[4] | ((uint16_t)payload[5] << 8));
    uint16_t dec  = (uint16_t)(payload[6] | ((uint16_t)payload[7] << 8));

    if (port == 0) port = IQ_UDP_DEFAULT_PORT;
    if (dec  == 0) dec  = 1;

    ESP_LOGI(TAG, "CMD_START_IQ_STREAM: port=%u dec=%u", port, dec);

    shared_state_lock();
    g_state.iq_target_ip   = ip;
    g_state.iq_target_port = port;
    g_state.iq_decimation  = dec;
    g_state.iq_streaming   = true;
    shared_state_unlock();

    send_ack(I2C_ACK_OK);
}

/**
 * CMD_STOP_IQ_STREAM (0x07)
 * No payload.
 */
static void handle_stop_iq_stream(void)
{
    shared_state_lock();
    g_state.iq_streaming = false;
    shared_state_unlock();

    ESP_LOGI(TAG, "CMD_STOP_IQ_STREAM");
    send_ack(I2C_ACK_OK);
}

/**
 * CMD_GET_STATUS (0x08)
 * Response: wifi[1] mqtt[1] gps_fix[1] uptime[4]  = 7 bytes
 */
static void handle_get_status(void)
{
    uint8_t buf[RESP_STATUS_SIZE];

    shared_state_lock();
    buf[0] = g_state.wifi_connected ? 1 : 0;
    buf[1] = g_state.mqtt_connected ? 1 : 0;
    buf[2] = g_state.gps_fix;
    uint32_t up = g_state.uptime_s;
    shared_state_unlock();

    buf[3] = (uint8_t)(up & 0xFF);
    buf[4] = (uint8_t)((up >> 8) & 0xFF);
    buf[5] = (uint8_t)((up >> 16) & 0xFF);
    buf[6] = (uint8_t)((up >> 24) & 0xFF);

    i2c_slave_write_buffer(I2C_SLAVE_PORT, buf, RESP_STATUS_SIZE, pdMS_TO_TICKS(50));
    send_ack(I2C_ACK_OK);
}

// ---------------------------------------------------------------------------
// Task entry point
// ---------------------------------------------------------------------------
void task_i2c_slave_run(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Initialising I2C slave on SDA=%d SCL=%d addr=0x%02X",
             I2C_SLAVE_SDA, I2C_SLAVE_SCL, I2C_SLAVE_ADDR);

    i2c_config_t conf = {
        .mode             = I2C_MODE_SLAVE,
        .sda_io_num       = I2C_SLAVE_SDA,
        .scl_io_num       = I2C_SLAVE_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_PORT, I2C_MODE_SLAVE,
                                       I2C_SLAVE_RX_BUF_LEN,
                                       I2C_SLAVE_TX_BUF_LEN, 0));

    ESP_LOGI(TAG, "I2C slave ready");

    for (;;) {
        // Read one command byte from master
        uint8_t cmd = 0;
        int got = i2c_slave_read_buffer(I2C_SLAVE_PORT, &cmd, 1,
                                        pdMS_TO_TICKS(100));
        if (got <= 0) {
            // No data — yield and try again
            taskYIELD();
            continue;
        }

        ESP_LOGD(TAG, "Received command: 0x%02X", cmd);

        switch (cmd) {
            case CMD_GET_GPS:
                handle_get_gps();
                break;

            case CMD_GET_SENSORS:
                handle_get_sensors();
                break;

            case CMD_SET_WIFI:
                handle_set_wifi();
                break;

            case CMD_SET_MQTT:
                handle_set_mqtt();
                break;

            case CMD_PUBLISH_EVENT:
                handle_publish_event();
                break;

            case CMD_START_IQ_STREAM:
                handle_start_iq_stream();
                break;

            case CMD_STOP_IQ_STREAM:
                handle_stop_iq_stream();
                break;

            case CMD_GET_STATUS:
                handle_get_status();
                break;

            default:
                ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
                send_ack(I2C_ACK_ERR);
                break;
        }
    }
    // Should never reach here
    vTaskDelete(NULL);
}
