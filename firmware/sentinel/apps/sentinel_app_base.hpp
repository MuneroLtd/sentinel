// SPDX-License-Identifier: MIT
// Project Sentinel - Base class for all application modules
#pragma once

#include <cstdint>
#include "app_ids.hpp"
#include "../ui/widgets.hpp"
#include "../event_bus/event_bus.hpp"
#include "../event_bus/event_types.hpp"
#include "../tasks/task_ids.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace sentinel {

class AppBase {
public:
    explicit AppBase(AppID id, TaskID task_id);
    virtual ~AppBase();

    // Deferred init: creates FreeRTOS queue and subscribes to EventBus.
    // Must be called AFTER the scheduler and EventBus are initialised
    // (i.e. from sentinel_main, not from a global constructor).
    void base_init();

    // Called by task_app when this app becomes active
    virtual void on_enter() = 0;
    // Called when leaving this app (e.g. switching to another)
    virtual void on_exit() = 0;
    // Called at ~60 Hz to redraw the screen
    virtual void on_paint() = 0;
    // Called with navigation input
    virtual void on_nav(ui::NavKey k) = 0;
    // Called each scheduler tick; drain bus events inside here
    virtual void on_tick() = 0;

    AppID app_id() const { return app_id_; }

protected:
    // ----- Radio helpers -----

    // Publish a RADIO_REQUEST event for this task, then block until granted
    // or timeout_ms elapses. Returns true if granted.
    void request_radio(uint32_t freq_hz, uint32_t bw_hz, uint8_t mode);
    void release_radio();
    bool wait_radio_grant(uint32_t timeout_ms = 2000);

    // ----- State broadcasting -----

    // Publish APP_STATE event (state: 0=stopped, 1=running, 2=suspended)
    void publish_app_state(uint8_t state, uint32_t tuned_freq = 0);

    // ----- Bus helpers -----

    // Drain the private bus queue; calls on_bus_event() for each item
    void process_bus_events();

    // Override to react to incoming bus events
    virtual void on_bus_event(const BusEvent& ev) { (void)ev; }

    // ----- Members -----

    AppID         app_id_;
    TaskID        task_id_;
    QueueHandle_t bus_queue_{nullptr};  // private receive queue
    int           bus_sub_id_{-1};      // EventBus subscription id
    bool          radio_owned_{false};  // true when radio has been granted

    // Cached GPS position (updated from GPS_UPDATE events)
    int32_t       cached_lat_{0};
    int32_t       cached_lon_{0};

private:
    static constexpr int BUS_QUEUE_DEPTH = 32;
};

} // namespace sentinel
