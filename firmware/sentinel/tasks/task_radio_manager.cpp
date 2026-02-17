/*
 * Project Sentinel - Radio Arbitration Task
 *
 * Manages exclusive access to the PortaPack radio hardware.  Multiple tasks
 * (BG Scanner, App Task, future capture tasks) may request the radio
 * simultaneously; the radio manager serialises access and handles preemption.
 *
 * Protocol (via EventBus):
 *   RADIO_REQUEST (0x05) — task wants radio access
 *     payload.requesting_task:  TaskID of requester
 *     payload.freq_hz:          desired frequency
 *     payload.mode:             0=RX, 1=TX, 2=ENERGY_DETECT
 *
 *   RADIO_RELEASE (0x06) — task is done with radio
 *     payload.releasing_task:   TaskID releasing
 *
 * Responses (task notifications to the requesting task handle):
 *   notify::RADIO_GRANTED   — radio is yours
 *   notify::RADIO_PREEMPTED — radio taken away mid-use, release immediately
 *
 * Priority model:
 *   Higher FreeRTOS task priority => higher radio priority.
 *   A RADIO_REQUEST from a higher-priority task will preempt the current owner
 *   if the owner has a lower priority.
 *
 * Pending queue:
 *   Up to 4 pending requests are held in a simple circular buffer.
 *   When the current owner releases, the first pending request is granted.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "../event_bus/event_bus.hpp"
#include "../event_bus/event_types.hpp"
#include "../tasks/task_ids.hpp"
#include "../radio/radio_ctrl.hpp"

// Forward declaration of sentinel_log (implemented in task_logger.cpp)
extern "C" void sentinel_log(const char* tag, const char* fmt, ...);

// We need access to the global task handle table declared in task_orchestrator.cpp
// Forward-declare the structure here; the full definition is in task_orchestrator.cpp
namespace sentinel {
    extern TaskHandle_t g_task_handles[];
    // Index constants matching TaskID enum values
    static constexpr int TASK_HANDLE_IDX_APP_TASK      = 1;
    static constexpr int TASK_HANDLE_IDX_RADIO_MANAGER = 2;
    static constexpr int TASK_HANDLE_IDX_BG_SCANNER    = 3;
    static constexpr int TASK_HANDLE_IDX_KNOWLEDGE_BASE= 4;
    static constexpr int TASK_HANDLE_IDX_ESP32_COMMS   = 5;
    static constexpr int TASK_HANDLE_IDX_LOGGER        = 7;
} // namespace sentinel

namespace {

// ---------------------------------------------------------------------------
// Pending request entry
// ---------------------------------------------------------------------------
struct PendingRequest {
    TaskHandle_t         handle;
    sentinel::RadioRequestPayload payload;
    bool                 valid;
};

static constexpr int PENDING_QUEUE_DEPTH = 4;

// ---------------------------------------------------------------------------
// Radio manager state
// ---------------------------------------------------------------------------
static sentinel::TaskID  s_current_owner     = sentinel::TaskID::NONE;
static TaskHandle_t      s_owner_handle      = nullptr;
static UBaseType_t       s_owner_priority    = 0;

static PendingRequest s_pending[PENDING_QUEUE_DEPTH]{};
static int            s_pending_head = 0;
static int            s_pending_tail = 0;
static int            s_pending_count = 0;

// ---------------------------------------------------------------------------
// Helper: task handle from TaskID
// Maps TaskID enum to the global task handle array declared in orchestrator.
// ---------------------------------------------------------------------------
static TaskHandle_t handle_for_task_id(sentinel::TaskID id)
{
    // Index matches the TaskID enum value (0=NONE, 1=APP, 2=RADIO_MGR, etc.)
    const int idx = static_cast<int>(id);
    if (idx <= 0 || idx >= 8) return nullptr;
    return sentinel::g_task_handles[idx];
}

// ---------------------------------------------------------------------------
// pending_enqueue / pending_dequeue
// Simple fixed-size FIFO for pending radio requests.
// ---------------------------------------------------------------------------
static bool pending_enqueue(TaskHandle_t handle,
                             const sentinel::RadioRequestPayload& payload)
{
    if (s_pending_count >= PENDING_QUEUE_DEPTH) return false;
    s_pending[s_pending_tail] = { handle, payload, true };
    s_pending_tail = (s_pending_tail + 1) % PENDING_QUEUE_DEPTH;
    ++s_pending_count;
    return true;
}

static bool pending_dequeue(PendingRequest* out)
{
    if (s_pending_count == 0) return false;
    *out = s_pending[s_pending_head];
    s_pending[s_pending_head].valid = false;
    s_pending_head = (s_pending_head + 1) % PENDING_QUEUE_DEPTH;
    --s_pending_count;
    return true;
}

// ---------------------------------------------------------------------------
// grant_radio: configure RF hardware and notify the task that it has access.
// ---------------------------------------------------------------------------
static void grant_radio(TaskHandle_t handle, sentinel::TaskID id,
                        const sentinel::RadioRequestPayload& req)
{
    s_current_owner  = id;
    s_owner_handle   = handle;
    s_owner_priority = (handle != nullptr) ? uxTaskPriorityGet(handle) : 0;

    // Configure the physical radio chain for the requester's parameters
    if (req.freq_hz != 0) {
        sentinel::radio::tune(req.freq_hz);
    }
    if (req.bandwidth_hz != 0) {
        sentinel::radio::set_bandwidth(req.bandwidth_hz);
    }
    if (req.mode == 0) {
        sentinel::radio::rx_mode();
    }

    xTaskNotify(handle, sentinel::notify::RADIO_GRANTED, eSetBits);
    sentinel_log("RADIO_MGR", "GRANTED to task %d freq=%u bw=%u",
                 static_cast<int>(id),
                 static_cast<unsigned>(req.freq_hz),
                 static_cast<unsigned>(req.bandwidth_hz));
}

// ---------------------------------------------------------------------------
// grant_next_pending: if there are pending requests, grant to the first one.
// ---------------------------------------------------------------------------
static void grant_next_pending()
{
    PendingRequest next{};
    if (pending_dequeue(&next) && next.valid) {
        grant_radio(next.handle,
                    static_cast<sentinel::TaskID>(next.payload.requesting_task),
                    next.payload);
    } else {
        // No pending requests — put RF hardware into low-power standby
        sentinel::radio::standby();
        s_current_owner  = sentinel::TaskID::NONE;
        s_owner_handle   = nullptr;
        s_owner_priority = 0;
    }
}

// ---------------------------------------------------------------------------
// handle_radio_request
// ---------------------------------------------------------------------------
static void handle_radio_request(const sentinel::BusEvent& ev)
{
    const auto& p = ev.as<sentinel::RadioRequestPayload>();
    const auto  req_id = static_cast<sentinel::TaskID>(p.requesting_task);
    TaskHandle_t req_handle = handle_for_task_id(req_id);

    if (req_handle == nullptr) {
        sentinel_log("RADIO_MGR", "REQUEST from unknown task %d — ignored",
                     static_cast<int>(p.requesting_task));
        return;
    }

    // Case 1: radio is free — grant immediately
    if (s_current_owner == sentinel::TaskID::NONE) {
        grant_radio(req_handle, req_id, p);
        return;
    }

    // Case 2: requester already owns it — re-grant (idempotent)
    if (s_current_owner == req_id) {
        xTaskNotify(req_handle, sentinel::notify::RADIO_GRANTED, eSetBits);
        return;
    }

    // Case 3: occupied — check if requester has higher priority than owner
    const UBaseType_t req_priority = uxTaskPriorityGet(req_handle);

    if (req_priority > s_owner_priority) {
        // Preempt the current owner
        sentinel_log("RADIO_MGR", "PREEMPTING task %d (pri %u) for task %d (pri %u)",
                     static_cast<int>(s_current_owner),
                     static_cast<unsigned>(s_owner_priority),
                     static_cast<int>(req_id),
                     static_cast<unsigned>(req_priority));

        // Put current owner back in the pending queue (at head if possible,
        // otherwise it gets bumped — high-priority preemptor is more important)
        if (s_owner_handle != nullptr) {
            sentinel::RadioRequestPayload owner_req{};
            owner_req.requesting_task = static_cast<uint8_t>(s_current_owner);
            // We don't have the owner's original request details at this point;
            // the owner will need to re-issue RADIO_REQUEST after RADIO_PREEMPTED.
            // Notify owner of preemption.
            xTaskNotify(s_owner_handle, sentinel::notify::RADIO_PREEMPTED, eSetBits);
        }

        grant_radio(req_handle, req_id, p);
    } else {
        // Lower or equal priority — queue the request
        if (!pending_enqueue(req_handle, p)) {
            // Queue full — deny silently (requester should retry)
            sentinel_log("RADIO_MGR", "Pending queue full — dropping request from task %d",
                         static_cast<int>(req_id));
        } else {
            sentinel_log("RADIO_MGR", "QUEUED request from task %d (pending: %d)",
                         static_cast<int>(req_id), s_pending_count);
        }
    }
}

// ---------------------------------------------------------------------------
// handle_radio_release
// ---------------------------------------------------------------------------
static void handle_radio_release(const sentinel::BusEvent& ev)
{
    const auto& p  = ev.as<sentinel::RadioReleasePayload>();
    const auto  rel_id = static_cast<sentinel::TaskID>(p.releasing_task);

    if (rel_id != s_current_owner) {
        sentinel_log("RADIO_MGR", "RELEASE from task %d but owner is %d — ignored",
                     static_cast<int>(rel_id),
                     static_cast<int>(s_current_owner));
        return;
    }

    sentinel_log("RADIO_MGR", "RELEASED by task %d", static_cast<int>(rel_id));
    grant_next_pending();
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// task_radio_manager_fn
//
// Entry point for the radio arbitration task. Priority: RADIO_MANAGER (5).
//
// Creates a local receive queue, subscribes to RADIO_REQUEST and RADIO_RELEASE
// events on the EventBus, then loops processing events.
// ---------------------------------------------------------------------------
extern "C" void task_radio_manager_fn(void* /*pvParameters*/)
{
    sentinel_log("RADIO_MGR", "Radio manager task started");

    // Initialise the RF hardware chain (Si5351C, RFFC5072, MAX2837, MAX5864, CPLD)
    if (!sentinel::radio::init()) {
        sentinel_log("RADIO_MGR", "ERROR: radio::init() failed — task halting");
        vTaskSuspend(nullptr);
    }
    sentinel_log("RADIO_MGR", "Radio hardware initialised");

    // Create a local event receive queue.
    // Depth 16: radio events arrive infrequently; this is ample headroom.
    static StaticQueue_t s_queue_static;
    static uint8_t       s_queue_storage[16 * sizeof(sentinel::BusEvent)];
    const QueueHandle_t  rx_queue = xQueueCreateStatic(
        16,
        sizeof(sentinel::BusEvent),
        s_queue_storage,
        &s_queue_static);

    // Subscribe to radio control events
    const uint32_t mask =
        EVENT_MASK(sentinel::EventType::RADIO_REQUEST) |
        EVENT_MASK(sentinel::EventType::RADIO_RELEASE);

    const int sub_id = sentinel::EventBus::instance().subscribe(rx_queue, mask);
    if (sub_id < 0) {
        sentinel_log("RADIO_MGR", "ERROR: failed to subscribe to EventBus");
        vTaskSuspend(nullptr);
    }

    sentinel_log("RADIO_MGR", "Subscribed to EventBus (id=%d)", sub_id);

    sentinel::BusEvent ev{};

    for (;;) {
        // Block until an event arrives (or 1s timeout for watchdog/debug)
        if (xQueueReceive(rx_queue, &ev, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        const auto type = static_cast<sentinel::EventType>(ev.type);

        switch (type) {
            case sentinel::EventType::RADIO_REQUEST:
                handle_radio_request(ev);
                break;
            case sentinel::EventType::RADIO_RELEASE:
                handle_radio_release(ev);
                break;
            default:
                break;
        }
    }

    // Unreachable, but tidy up if ever reached
    sentinel::EventBus::instance().unsubscribe(sub_id);
}
