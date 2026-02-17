#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// LWIP socket API
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "sentinel_config.h"
#include "shared_state.h"

static const char *TAG = "iq_stream";

// ---------------------------------------------------------------------------
// UDP packet header
// 8 bytes: magic[2] seq[2] timestamp_ms[4]
// ---------------------------------------------------------------------------
#define IQ_HDR_SIZE     8
#define IQ_HDR_MAGIC    IQ_UDP_HEADER_MAGIC  // 0x5349 = 'SI'

typedef struct __attribute__((packed)) {
    uint16_t magic;         // 0x5349
    uint16_t sequence;
    uint32_t timestamp_ms;
} iq_udp_header_t;

_Static_assert(sizeof(iq_udp_header_t) == IQ_HDR_SIZE,
    "iq_udp_header_t must be 8 bytes");

// ---------------------------------------------------------------------------
// DMA-capable buffers — must be in DMA-accessible memory
// ---------------------------------------------------------------------------
static WORD_ALIGNED_ATTR uint8_t s_rx_buf[IQ_DMA_BUF_SIZE];
static WORD_ALIGNED_ATTR uint8_t s_tx_buf[IQ_DMA_BUF_SIZE];  // Dummy TX for slave

// ---------------------------------------------------------------------------
// SPI slave init / deinit
// ---------------------------------------------------------------------------

static bool s_spi_inited = false;

static esp_err_t iq_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num   = IQ_MOSI_PIN,
        .miso_io_num   = IQ_MISO_PIN,
        .sclk_io_num   = IQ_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = IQ_DMA_BUF_SIZE,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode          = 0,           // CPOL=0, CPHA=0
        .spics_io_num  = IQ_CS_PIN,
        .queue_size    = 4,
        .flags         = 0,
        .post_setup_cb  = NULL,
        .post_trans_cb  = NULL,
    };

    esp_err_t err = spi_slave_initialize(IQ_SPI_HOST, &buscfg, &slvcfg,
                                         SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_initialize failed: %d", err);
        return err;
    }

    s_spi_inited = true;
    ESP_LOGI(TAG, "SPI slave initialised (MOSI=%d MISO=%d CLK=%d CS=%d)",
             IQ_MOSI_PIN, IQ_MISO_PIN, IQ_CLK_PIN, IQ_CS_PIN);
    return ESP_OK;
}

static void iq_spi_deinit(void)
{
    if (s_spi_inited) {
        spi_slave_free(IQ_SPI_HOST);
        s_spi_inited = false;
    }
}

// ---------------------------------------------------------------------------
// UDP socket helpers
// ---------------------------------------------------------------------------

static int s_udp_sock = -1;

static bool udp_open(uint32_t target_ip_nbo, uint16_t target_port,
                     struct sockaddr_in *dest_addr)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return false;
    }

    memset(dest_addr, 0, sizeof(*dest_addr));
    dest_addr->sin_family      = AF_INET;
    dest_addr->sin_port        = htons(target_port);
    dest_addr->sin_addr.s_addr = target_ip_nbo;

    ESP_LOGI(TAG, "UDP socket created → %s:%u",
             inet_ntoa(dest_addr->sin_addr), target_port);
    return true;
}

static void udp_close(void)
{
    if (s_udp_sock >= 0) {
        close(s_udp_sock);
        s_udp_sock = -1;
    }
}

// ---------------------------------------------------------------------------
// Task entry point
// ---------------------------------------------------------------------------
void task_iq_stream_run(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "IQ stream task ready — waiting for start command");

    // Packet buffer: 8-byte header + IQ_DMA_BUF_SIZE payload
    static uint8_t pkt_buf[IQ_HDR_SIZE + IQ_DMA_BUF_SIZE];

    for (;;) {
        // --- Wait until streaming is requested ---
        {
            bool streaming = false;
            while (!streaming) {
                shared_state_lock();
                streaming = g_state.iq_streaming;
                shared_state_unlock();
                if (!streaming) {
                    vTaskDelay(pdMS_TO_TICKS(250));
                }
            }
        }

        // Snapshot streaming parameters
        uint32_t target_ip;
        uint16_t target_port;
        uint16_t decimation;
        shared_state_lock();
        target_ip   = g_state.iq_target_ip;
        target_port = g_state.iq_target_port;
        decimation  = g_state.iq_decimation;
        shared_state_unlock();

        if (target_ip == 0) {
            ESP_LOGW(TAG, "No target IP — aborting stream");
            shared_state_lock();
            g_state.iq_streaming = false;
            shared_state_unlock();
            continue;
        }

        ESP_LOGI(TAG, "Starting IQ stream: decimation=%u", decimation);

        // Init SPI slave
        if (!s_spi_inited) {
            if (iq_spi_init() != ESP_OK) {
                ESP_LOGE(TAG, "SPI init failed — aborting stream");
                shared_state_lock();
                g_state.iq_streaming = false;
                shared_state_unlock();
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }

        // Open UDP socket
        struct sockaddr_in dest;
        if (!udp_open(target_ip, target_port, &dest)) {
            iq_spi_deinit();
            shared_state_lock();
            g_state.iq_streaming = false;
            shared_state_unlock();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // --- Streaming loop ---
        uint16_t seq          = 0;
        uint32_t bytes_sent   = 0;
        TickType_t log_tick   = xTaskGetTickCount();
        const TickType_t log_interval = pdMS_TO_TICKS(10000);

        spi_slave_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length    = IQ_DMA_BUF_SIZE * 8;  // Length in bits
        trans.rx_buffer = s_rx_buf;
        trans.tx_buffer = s_tx_buf;  // Dummy TX data (slave sends zeros)
        memset(s_tx_buf, 0, sizeof(s_tx_buf));

        for (;;) {
            // Check stop condition
            bool streaming;
            shared_state_lock();
            streaming = g_state.iq_streaming;
            shared_state_unlock();

            if (!streaming) {
                ESP_LOGI(TAG, "Stop command received — halting IQ stream");
                break;
            }

            // Receive one DMA-sized buffer from LPC4320 over SPI
            esp_err_t err = spi_slave_transmit(IQ_SPI_HOST, &trans,
                                               pdMS_TO_TICKS(500));
            if (err == ESP_ERR_TIMEOUT) {
                // No transaction from master — LPC4320 may not be streaming yet
                taskYIELD();
                continue;
            }
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "SPI slave error: %d — stopping stream", err);
                break;
            }

            size_t rx_bytes = trans.trans_len / 8;
            if (rx_bytes == 0) {
                continue;
            }

            // Build UDP packet
            iq_udp_header_t *hdr = (iq_udp_header_t *)pkt_buf;
            hdr->magic        = IQ_HDR_MAGIC;
            hdr->sequence     = seq++;
            hdr->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

            memcpy(pkt_buf + IQ_HDR_SIZE, s_rx_buf, rx_bytes);

            ssize_t sent = sendto(s_udp_sock,
                                  pkt_buf, IQ_HDR_SIZE + rx_bytes, 0,
                                  (struct sockaddr *)&dest, sizeof(dest));
            if (sent < 0) {
                ESP_LOGE(TAG, "sendto failed: errno %d", errno);
                break;
            }

            bytes_sent += (uint32_t)sent;

            // Periodic throughput log
            TickType_t now = xTaskGetTickCount();
            if ((now - log_tick) >= log_interval) {
                float elapsed_s = (float)(now - log_tick) / configTICK_RATE_HZ;
                float kbps = (bytes_sent * 8.0f) / (elapsed_s * 1000.0f);
                ESP_LOGI(TAG, "IQ stream: %.1f kbps (%lu bytes / %.1f s) seq=%u",
                         (double)kbps, (unsigned long)bytes_sent,
                         (double)elapsed_s, (unsigned)seq);
                bytes_sent = 0;
                log_tick   = now;
            }
        }

        // Tear down
        udp_close();
        iq_spi_deinit();

        shared_state_lock();
        g_state.iq_streaming = false;
        shared_state_unlock();

        ESP_LOGI(TAG, "IQ stream stopped");
        // Short delay before re-checking for another start command
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}
