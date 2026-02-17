#include "shared_state.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "shared_state";

// ---------------------------------------------------------------------------
// Global definitions
// ---------------------------------------------------------------------------
shared_state_t    g_state;
SemaphoreHandle_t g_state_mutex;

// ---------------------------------------------------------------------------
void shared_state_init(void)
{
    // Zero-initialise the whole struct first
    memset(&g_state, 0, sizeof(g_state));

    // Set safe defaults
    g_state.mqtt_port        = MQTT_DEFAULT_PORT;
    g_state.iq_target_port   = IQ_UDP_DEFAULT_PORT;
    g_state.iq_decimation    = 1;
    g_state.event_write_idx  = 0;

    // Default device ID — will be overwritten from NVS in main.c
    snprintf(g_state.device_id, sizeof(g_state.device_id), "sentinel_000000");

    // Create mutex
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        // This is fatal — abort early
        ESP_LOGE(TAG, "Failed to create state mutex — aborting");
        esp_restart();
    }

    ESP_LOGI(TAG, "Shared state initialised");
}

// ---------------------------------------------------------------------------
void shared_state_add_event(const sentinel_event_t *event)
{
    if (event == NULL) {
        return;
    }

    shared_state_lock();

    memcpy(&g_state.recent_events[g_state.event_write_idx],
           event,
           sizeof(sentinel_event_t));

    g_state.event_write_idx = (g_state.event_write_idx + 1) % RECENT_EVENTS_LEN;

    shared_state_unlock();
}

// ---------------------------------------------------------------------------
void shared_state_tick_uptime(void)
{
    shared_state_lock();
    g_state.uptime_s++;
    shared_state_unlock();
}
