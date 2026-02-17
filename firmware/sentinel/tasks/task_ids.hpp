#pragma once
#include <cstdint>

// ============================================================
// Project Sentinel - Task Priorities and Stack Sizes
// FreeRTOS task configuration.
// configMAX_PRIORITIES = 8 (see FreeRTOSConfig.h)
// ============================================================

namespace sentinel {

// FreeRTOS task priorities (0 = idle, 7 = highest)
struct TaskPriority {
    static constexpr UBaseType_t IDLE             = 0;
    static constexpr UBaseType_t KNOWLEDGE_BASE   = 1;
    static constexpr UBaseType_t UI_RENDERER      = 2;
    static constexpr UBaseType_t APP_TASK         = 3;
    static constexpr UBaseType_t BG_SCANNER       = 3;
    static constexpr UBaseType_t ESP32_COMMS      = 4;
    static constexpr UBaseType_t LOGGER           = 4;
    static constexpr UBaseType_t RADIO_MANAGER    = 5;
    static constexpr UBaseType_t EVENT_BUS        = 5;
    // Reserved highest: FreeRTOS timer task at configMAX_PRIORITIES-1
};

// Stack sizes in words (1 word = 4 bytes on Cortex-M4)
struct TaskStackWords {
    static constexpr uint32_t KNOWLEDGE_BASE  = 512;   // 2KB
    static constexpr uint32_t UI_RENDERER     = 1024;  // 4KB
    static constexpr uint32_t APP_TASK        = 1024;  // 4KB
    static constexpr uint32_t BG_SCANNER      = 512;   // 2KB
    static constexpr uint32_t ESP32_COMMS     = 768;   // 3KB
    static constexpr uint32_t LOGGER          = 256;   // 1KB
    static constexpr uint32_t RADIO_MANAGER   = 512;   // 2KB
};

// Task notification bit assignments (xTaskNotifyWait bitmasks)
// Each task defines which bits it uses for internal signalling
namespace notify {
    static constexpr uint32_t RADIO_GRANTED   = (1u << 0);
    static constexpr uint32_t RADIO_PREEMPTED = (1u << 1);
    static constexpr uint32_t UI_REFRESH      = (1u << 2);
    static constexpr uint32_t SD_FLUSH        = (1u << 3);
    static constexpr uint32_t ESP32_READY     = (1u << 4);
    static constexpr uint32_t SHUTDOWN        = (1u << 31);
}

} // namespace sentinel
