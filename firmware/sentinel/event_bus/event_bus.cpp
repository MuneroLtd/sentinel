/*
 * Project Sentinel - EventBus Implementation
 *
 * Typed publish-subscribe message bus over FreeRTOS queues.
 * Singleton; thread-safe via FreeRTOS mutex; ISR-safe publish variant.
 *
 * Design constraints (from SPEC):
 *   - Max 8 subscribers
 *   - BusEvent is exactly 32 bytes
 *   - publish() is non-blocking: drops message and increments dropped_ counter
 *     rather than blocking when a subscriber queue is full
 *   - publish_from_isr() uses FromISR queue API, no mutex (ISR context)
 *
 * Memory cost breakdown:
 *   - Subscriber table:  8 × (4 + 4 + 1) bytes = 72 bytes (padded ~80)
 *   - Mutex handle:      4 bytes
 *   - dropped_ counter:  4 bytes
 *   Total: ~88 bytes static storage in the singleton
 */

#include "event_bus.hpp"

// FreeRTOS portable headers
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"

namespace sentinel {

// ---------------------------------------------------------------------------
// Singleton accessor
// ---------------------------------------------------------------------------

EventBus& EventBus::instance()
{
    static EventBus bus;
    return bus;
}

// ---------------------------------------------------------------------------
// init()
//
// Creates the mutex that protects the subscriber table. Must be called before
// vTaskStartScheduler(). Returns false if mutex creation fails — the caller
// should treat this as a fatal error (insufficient heap).
// ---------------------------------------------------------------------------

bool EventBus::init()
{
    lock_ = xSemaphoreCreateMutex();
    if (lock_ == nullptr) {
        return false;
    }

    // Zero-initialise the subscriber table defensively.
    for (int i = 0; i < MAX_SUBSCRIBERS; ++i) {
        subscribers_[i].queue  = nullptr;
        subscribers_[i].mask   = 0;
        subscribers_[i].active = false;
    }

    dropped_ = 0;
    return true;
}

// ---------------------------------------------------------------------------
// subscribe()
//
// Registers a subscriber queue and event type mask. The caller owns the queue;
// the EventBus only holds a handle. The queue must be sized to accept BusEvent
// items (32 bytes each).
//
// Returns the subscriber index (0..MAX_SUBSCRIBERS-1) on success,
// or -1 if the table is full.
// ---------------------------------------------------------------------------

int EventBus::subscribe(QueueHandle_t rx_queue, uint32_t event_type_mask)
{
    if (rx_queue == nullptr || event_type_mask == 0) {
        return -1;
    }

    xSemaphoreTake(lock_, portMAX_DELAY);

    int slot = -1;
    for (int i = 0; i < MAX_SUBSCRIBERS; ++i) {
        if (!subscribers_[i].active) {
            slot = i;
            break;
        }
    }

    if (slot >= 0) {
        subscribers_[slot].queue  = rx_queue;
        subscribers_[slot].mask   = event_type_mask;
        subscribers_[slot].active = true;
    }

    xSemaphoreGive(lock_);
    return slot;
}

// ---------------------------------------------------------------------------
// unsubscribe()
//
// Marks a subscriber slot inactive. Silently ignores out-of-range IDs.
// The caller is responsible for draining or deleting their queue after this.
// ---------------------------------------------------------------------------

void EventBus::unsubscribe(int subscriber_id)
{
    if (subscriber_id < 0 || subscriber_id >= MAX_SUBSCRIBERS) {
        return;
    }

    xSemaphoreTake(lock_, portMAX_DELAY);
    subscribers_[subscriber_id].active = false;
    subscribers_[subscriber_id].queue  = nullptr;
    subscribers_[subscriber_id].mask   = 0;
    xSemaphoreGive(lock_);
}

// ---------------------------------------------------------------------------
// publish()
//
// Timestamps the event with the current RTOS tick (converted to ms), then
// fans out to all active subscribers whose mask includes this event type.
//
// Uses xQueueSend() with timeout 0 (non-blocking). If a subscriber's queue
// is full the event is dropped and dropped_ is incremented. This design
// prevents a slow subscriber from stalling the publisher or other subscribers.
//
// The mutex is held only while iterating the subscriber table, not while
// calling xQueueSend — this keeps the critical section short and avoids
// priority inversion with subscriber tasks.
// ---------------------------------------------------------------------------

void EventBus::publish(const BusEvent& event)
{
    // Stamp the event with current time. We cast away const to fill the
    // timestamp — BusEvent is a value type copied by the caller anyway.
    BusEvent stamped = event;
    stamped.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Snapshot the active subscriber list under the lock so we don't hold
    // the mutex while calling xQueueSend (which may wake higher-priority tasks).
    Subscriber snapshot[MAX_SUBSCRIBERS];
    int count = 0;

    xSemaphoreTake(lock_, portMAX_DELAY);
    for (int i = 0; i < MAX_SUBSCRIBERS; ++i) {
        if (subscribers_[i].active) {
            snapshot[count++] = subscribers_[i];
        }
    }
    xSemaphoreGive(lock_);

    // Compute the event type bit for mask matching.
    const uint32_t type_bit = 1u << stamped.type;

    for (int i = 0; i < count; ++i) {
        if (snapshot[i].mask & type_bit) {
            if (xQueueSend(snapshot[i].queue, &stamped, 0) != pdTRUE) {
                ++dropped_;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// publish_from_isr()
//
// ISR-safe variant. Skips the mutex entirely (mutexes cannot be taken from
// an ISR) and uses xQueueSendFromISR(). The caller must pass a valid pointer
// to pxHigherPriorityTaskWoken; call portYIELD_FROM_ISR() afterwards if it
// is set to pdTRUE.
//
// Note: timestamps are still written; xTaskGetTickCountFromISR() is ISR-safe.
// ---------------------------------------------------------------------------

void EventBus::publish_from_isr(const BusEvent& event,
                                BaseType_t* pxHigherPriorityTaskWoken)
{
    BusEvent stamped = event;
    stamped.timestamp_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    const uint32_t type_bit = 1u << stamped.type;

    // No mutex in ISR context — iterate the live array directly.
    // Race condition is acceptable: a subscriber that just registered may
    // miss one event; a subscriber that just unsubscribed may receive one
    // extra event. Both are benign.
    for (int i = 0; i < MAX_SUBSCRIBERS; ++i) {
        if (subscribers_[i].active && (subscribers_[i].mask & type_bit)) {
            BaseType_t woken = pdFALSE;
            if (xQueueSendFromISR(subscribers_[i].queue, &stamped, &woken) != pdTRUE) {
                ++dropped_;
            }
            if (woken == pdTRUE) {
                *pxHigherPriorityTaskWoken = pdTRUE;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// subscriber_count()
//
// Returns the number of currently active subscribers. Thread-safe.
// ---------------------------------------------------------------------------

int EventBus::subscriber_count() const
{
    // Take the lock read-only to count active slots.
    // Cast away const: FreeRTOS mutex is not logically const but is required
    // for this conceptually-const inspection.
    xSemaphoreTake(const_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < MAX_SUBSCRIBERS; ++i) {
        if (subscribers_[i].active) {
            ++count;
        }
    }
    xSemaphoreGive(const_cast<SemaphoreHandle_t>(lock_));
    return count;
}

} // namespace sentinel
