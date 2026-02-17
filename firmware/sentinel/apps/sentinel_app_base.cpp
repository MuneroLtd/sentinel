// SPDX-License-Identifier: MIT
// Project Sentinel - AppBase implementation

#include "sentinel_app_base.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>

namespace sentinel {

AppBase::AppBase(AppID id, TaskID task_id)
    : app_id_(id), task_id_(task_id)
{
    // NOTE: Do NOT call xQueueCreate or EventBus here!
    // Global AppBase objects are constructed before main() via init_array,
    // long before FreeRTOS heap or EventBus are initialised.
    // Use base_init() after the scheduler infrastructure is ready.
}

void AppBase::base_init()
{
    if (bus_queue_) return;  // already initialised

    // Create a private FreeRTOS queue for receiving bus events
    bus_queue_ = xQueueCreate(BUS_QUEUE_DEPTH, sizeof(BusEvent));
    configASSERT(bus_queue_);

    // Subscribe to all event types (full mask covers bits 0..10)
    constexpr uint32_t ALL_EVENTS = 0x0000'07FFu;
    bus_sub_id_ = EventBus::instance().subscribe(bus_queue_, ALL_EVENTS);
}

AppBase::~AppBase()
{
    if (bus_sub_id_ >= 0) {
        EventBus::instance().unsubscribe(bus_sub_id_);
        bus_sub_id_ = -1;
    }
    if (bus_queue_) {
        vQueueDelete(bus_queue_);
        bus_queue_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Radio helpers
// ---------------------------------------------------------------------------

void AppBase::request_radio(uint32_t freq_hz, uint32_t bw_hz, uint8_t mode)
{
    BusEvent ev = BusEvent::make(EventType::RADIO_REQUEST, task_id_);
    auto& p = ev.as<RadioRequestPayload>();
    p.requesting_task = static_cast<uint8_t>(task_id_);
    p.freq_hz         = freq_hz;
    p.bandwidth_hz    = bw_hz;
    p.mode            = mode;
    EventBus::instance().publish(ev);
}

void AppBase::release_radio()
{
    if (!radio_owned_) return;

    BusEvent ev = BusEvent::make(EventType::RADIO_RELEASE, task_id_);
    auto& p = ev.as<RadioReleasePayload>();
    p.releasing_task = static_cast<uint8_t>(task_id_);
    EventBus::instance().publish(ev);
    radio_owned_ = false;
}

bool AppBase::wait_radio_grant(uint32_t timeout_ms)
{
    // The radio manager will set the RADIO_GRANTED task notification bit
    // when our request is honoured.
    uint32_t notif = 0;
    BaseType_t r = xTaskNotifyWait(0, notify::RADIO_GRANTED,
                                   &notif, pdMS_TO_TICKS(timeout_ms));
    if (r == pdTRUE && (notif & notify::RADIO_GRANTED)) {
        radio_owned_ = true;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// State broadcasting
// ---------------------------------------------------------------------------

void AppBase::publish_app_state(uint8_t state, uint32_t tuned_freq)
{
    BusEvent ev = BusEvent::make(EventType::APP_STATE, task_id_);
    auto& p = ev.as<AppStatePayload>();
    p.app_id       = static_cast<uint8_t>(app_id_);
    p.state        = state;
    p.tuned_freq_hz = tuned_freq;
    EventBus::instance().publish(ev);
}

// ---------------------------------------------------------------------------
// Bus drain
// ---------------------------------------------------------------------------

void AppBase::process_bus_events()
{
    if (!bus_queue_) return;

    BusEvent ev{};
    while (xQueueReceive(bus_queue_, &ev, 0) == pdTRUE) {
        // Update GPS cache opportunistically
        if (static_cast<EventType>(ev.type) == EventType::GPS_UPDATE) {
            const auto& gps = ev.as<GPSUpdatePayload>();
            cached_lat_ = gps.latitude;
            cached_lon_ = gps.longitude;
        }
        on_bus_event(ev);
    }
}

} // namespace sentinel
