#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/spi_slave.h"

// ---------------------------------------------------------------------------
// I2C slave bus — receives commands from LPC4320 (PortaPack H4M)
// ---------------------------------------------------------------------------
#define I2C_SLAVE_SDA     GPIO_NUM_8
#define I2C_SLAVE_SCL     GPIO_NUM_9
#define I2C_SLAVE_ADDR    0x42
#define I2C_SLAVE_PORT    I2C_NUM_0

// Buffer sizes for I2C slave driver
#define I2C_SLAVE_RX_BUF_LEN  256
#define I2C_SLAVE_TX_BUF_LEN  256

// ---------------------------------------------------------------------------
// GPS NEO-6M — UART
// ---------------------------------------------------------------------------
#define GPS_UART_NUM      UART_NUM_1
#define GPS_TX_PIN        GPIO_NUM_17
#define GPS_RX_PIN        GPIO_NUM_18
#define GPS_BAUD          9600
#define GPS_RX_BUF_LEN    1024

// ---------------------------------------------------------------------------
// BME280 environmental sensor — I2C master (separate bus from slave)
// ---------------------------------------------------------------------------
#define BME280_I2C_PORT   I2C_NUM_1
#define BME280_SDA_PIN    GPIO_NUM_21
#define BME280_SCL_PIN    GPIO_NUM_22
#define BME280_I2C_ADDR   0x76
#define BME280_I2C_FREQ   400000   // 400 kHz

// ---------------------------------------------------------------------------
// SPI slave — IQ streaming from LPC4320
// ---------------------------------------------------------------------------
#define IQ_SPI_HOST       SPI2_HOST
#define IQ_MOSI_PIN       GPIO_NUM_11
#define IQ_MISO_PIN       GPIO_NUM_13
#define IQ_CLK_PIN        GPIO_NUM_12
#define IQ_CS_PIN         GPIO_NUM_10

// DMA buffer: 2048 complex 8-bit I/Q pairs = 4096 bytes
#define IQ_DMA_BUF_SIZE   4096

// ---------------------------------------------------------------------------
// NVS (non-volatile storage) namespace and keys
// ---------------------------------------------------------------------------
#define NVS_NAMESPACE     "sentinel"
#define NVS_WIFI_SSID     "wifi_ssid"
#define NVS_WIFI_PASS     "wifi_pass"
#define NVS_MQTT_HOST     "mqtt_host"
#define NVS_MQTT_PORT     "mqtt_port"
#define NVS_DEVICE_ID     "device_id"

// ---------------------------------------------------------------------------
// Wi-Fi fallback AP credentials (used when NVS has no stored credentials)
// ---------------------------------------------------------------------------
#define WIFI_FALLBACK_SSID  "sentinel_ap"
#define WIFI_FALLBACK_PASS  "sentinel123"
#define WIFI_MAX_RETRY      5

// ---------------------------------------------------------------------------
// MQTT defaults
// ---------------------------------------------------------------------------
#define MQTT_DEFAULT_PORT   1883
#define MQTT_QOS_HIGH       1
#define MQTT_QOS_LOW        0
#define MQTT_TOPIC_MAX_LEN  128
#define MQTT_PAYLOAD_MAX_LEN 256

// ---------------------------------------------------------------------------
// UDP IQ streaming
// ---------------------------------------------------------------------------
#define IQ_UDP_DEFAULT_PORT  5555
#define IQ_UDP_HEADER_MAGIC  0x4953  // 'SI' little-endian

// ---------------------------------------------------------------------------
// Web server
// ---------------------------------------------------------------------------
#define WEBSERVER_PORT      80
#define WEBSERVER_MAX_URI   8

// ---------------------------------------------------------------------------
// Task priorities (ESP-IDF FreeRTOS, higher = more priority)
// ---------------------------------------------------------------------------
#define PRIO_I2C_SLAVE    15
#define PRIO_GPS          10
#define PRIO_BME280       8
#define PRIO_MQTT         8
#define PRIO_IQ_STREAM    12
#define PRIO_WEBSERVER    6

// Task stack sizes (bytes)
#define STACK_I2C_SLAVE   4096
#define STACK_GPS         4096
#define STACK_BME280      3072
#define STACK_MQTT        4096
#define STACK_IQ_STREAM   8192
#define STACK_WEBSERVER   8192

// ---------------------------------------------------------------------------
// Sentinel event ring buffer depth
// ---------------------------------------------------------------------------
#define RECENT_EVENTS_LEN  5

// ---------------------------------------------------------------------------
// MQTT event queue depth
// ---------------------------------------------------------------------------
#define MQTT_EVENT_QUEUE_DEPTH  16
