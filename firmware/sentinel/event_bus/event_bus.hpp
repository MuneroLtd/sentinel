#pragma once
#include "event_types.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

// ============================================================
// Project Sentinel - Event Bus Interface
// Typed publish-subscribe over FreeRTOS queues.
// Thread-safe. ISR-safe publish variant provided.
// Max 8 subscribers. Bus queue depth 64 messages.
// Total SRAM cost: ~2.1KB.
// ============================================================

namespace sentinel {

class EventBus {
public:
    static constexpr int     MAX_SUBSCRIBERS  = 8;
    static constexpr int     BUS_QUEUE_DEPTH  = 64;
    static constexpr TickType_t PUBLISH_TIMEOUT = pdMS_TO_TICKS(5);

    // Singleton - call once after FreeRTOS scheduler starts
    static EventBus& instance();

    // Must be called before vTaskStartScheduler()
    // Returns false if queue/mutex creation fails (fatal)
    bool init();

    // Subscribe to events matching mask (bitwise OR of EVENT_MASK(type))
    // Returns subscriber_id >= 0 on success, -1 if subscriber table full
    // The caller's queue must accept BusEvent items
    int subscribe(QueueHandle_t rx_queue, uint32_t event_type_mask);

    // Remove subscription by ID returned from subscribe()
    void unsubscribe(int subscriber_id);

    // Publish from a task context
    // Fans out to all matching subscriber queues (non-blocking, drops if full)
    void publish(const BusEvent& event);

    // Publish from an ISR context
    void publish_from_isr(const BusEvent& event,
                          BaseType_t* pxHigherPriorityTaskWoken);

    // Convenience: build and publish in one call
    template<typename PayloadT>
    void publish(EventType type, TaskID source, const PayloadT& payload) {
        static_assert(sizeof(PayloadT) == 26, "Payload must be 26 bytes");
        BusEvent ev = BusEvent::make(type, source);
        __builtin_memcpy(ev.payload, &payload, 26);
        publish(ev);
    }

    // Stats (for serial debug)
    uint32_t dropped_count() const { return dropped_; }
    int      subscriber_count() const;

private:
    EventBus() = default;
    ~EventBus() = default;
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    struct Subscriber {
        QueueHandle_t queue     = nullptr;
        uint32_t      mask      = 0;
        bool          active    = false;
    };

    Subscriber      subscribers_[MAX_SUBSCRIBERS]{};
    SemaphoreHandle_t lock_         = nullptr;
    uint32_t          dropped_      = 0;
};

} // namespace sentinel
