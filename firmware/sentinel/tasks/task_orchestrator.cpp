// SPDX-License-Identifier: MIT
// Project Sentinel — Task Orchestrator
//
// Creates all FreeRTOS tasks using xTaskCreateStatic() so that no heap
// allocation occurs at task creation time.  All TCBs and stacks live in
// BSS (zero-initialised by the C runtime before main()).
//
// Call sentinel_create_tasks() from main.cpp before vTaskStartScheduler().
//
// Task creation order: lowest priority first (KNOWLEDGE_BASE → RADIO_MANAGER)
// so that if the scheduler somehow starts during creation the highest-priority
// task does not starve earlier ones during the creation window.

#include "FreeRTOS.h"
#include "task.h"

#include "../tasks/task_ids.hpp"
#include "../event_bus/event_types.hpp"
#include "../hal/uart.hpp"

#include <cstdint>

// ---------------------------------------------------------------------------
// External task entry points
// Each is defined in its own translation unit.
// ---------------------------------------------------------------------------
extern "C" {
    void task_knowledge_base_entry(void*);   // task_knowledge_base.cpp
    void task_app_entry(void*);              // task_app.cpp
    void task_bg_scanner_entry(void*);       // task_bg_scanner.cpp
    void task_esp32_comms_entry(void*);      // task_esp32_comms.cpp
    void task_logger_entry(void*);           // task_logger.cpp (fn alias below)
    void task_radio_manager_entry(void*);    // task_radio_manager.cpp (fn alias below)
}

// The existing task .cpp files use _fn suffix; provide _entry aliases here.
// task_logger.cpp      exports task_logger_fn
// task_knowledge_base  exports task_knowledge_base_fn
// task_radio_manager   exports task_radio_manager_fn
// We declare weak aliases so the linker resolves them to the canonical names.

extern "C" void task_knowledge_base_fn(void*);
extern "C" void task_logger_fn(void*);
extern "C" void task_radio_manager_fn(void*);

// Provide the _entry entry-point symbols as thin wrappers — avoids modifying
// the existing clean task files while matching the expected naming convention.
extern "C" void task_knowledge_base_entry(void* p) { task_knowledge_base_fn(p); }
extern "C" void task_logger_entry(void* p)         { task_logger_fn(p); }
extern "C" void task_radio_manager_entry(void* p)  { task_radio_manager_fn(p); }

// ---------------------------------------------------------------------------
// Global task handle table — indexed by TaskID enum value
//   [0] = NONE (unused)
//   [1] = APP_TASK
//   [2] = RADIO_MANAGER
//   [3] = BG_SCANNER
//   [4] = KNOWLEDGE_BASE
//   [5] = ESP32_COMMS
//   [6] = UI_RENDERER  (alias for APP_TASK handle — same task)
//   [7] = LOGGER
// ---------------------------------------------------------------------------
namespace sentinel {
    TaskHandle_t g_task_handles[8]{};
} // namespace sentinel

// ---------------------------------------------------------------------------
// FreeRTOS heap — placed in AHB SRAM to free SRAM0 for task stacks/BSS.
// configAPPLICATION_ALLOCATED_HEAP = 1 tells heap_4.c to use this extern
// array instead of allocating its own static array in BSS.
// ---------------------------------------------------------------------------
extern "C" {
    __attribute__((section(".ahbram"), aligned(8)))
    uint8_t ucHeap[configTOTAL_HEAP_SIZE];
}

// ---------------------------------------------------------------------------
// Static TCBs and stacks (all in BSS — no heap allocation)
// ---------------------------------------------------------------------------
namespace {

// Task stacks placed in AHB SRAM to free SRAM0 (only 96 KB on LPC4320).
// TCBs stay in SRAM0 (small, ~168 bytes each).
#define AHBRAM_STACK __attribute__((section(".ahbram"), aligned(8)))

// KNOWLEDGE_BASE — priority 1, 512 words
static StaticTask_t s_kb_tcb;
AHBRAM_STACK static StackType_t s_kb_stack[sentinel::TaskStackWords::KNOWLEDGE_BASE];

// UI_RENDERER / APP_TASK — priority 3, 1024 words
static StaticTask_t s_app_tcb;
AHBRAM_STACK static StackType_t s_app_stack[sentinel::TaskStackWords::APP_TASK];

// BG_SCANNER — priority 3, 512 words
static StaticTask_t s_scan_tcb;
AHBRAM_STACK static StackType_t s_scan_stack[sentinel::TaskStackWords::BG_SCANNER];

// ESP32_COMMS — priority 4, 768 words
static StaticTask_t s_esp32_tcb;
AHBRAM_STACK static StackType_t s_esp32_stack[sentinel::TaskStackWords::ESP32_COMMS];

// LOGGER — priority 4, 256 words
static StaticTask_t s_logger_tcb;
AHBRAM_STACK static StackType_t s_logger_stack[sentinel::TaskStackWords::LOGGER];

// RADIO_MANAGER — priority 5, 512 words
static StaticTask_t s_radio_tcb;
AHBRAM_STACK static StackType_t s_radio_stack[sentinel::TaskStackWords::RADIO_MANAGER];

// FreeRTOS idle task — required by configSUPPORT_STATIC_ALLOCATION = 1
static StaticTask_t s_idle_tcb;
AHBRAM_STACK static StackType_t s_idle_stack[configMINIMAL_STACK_SIZE];

// FreeRTOS timer daemon task — required by configUSE_TIMERS = 1
static StaticTask_t s_timer_tcb;
AHBRAM_STACK static StackType_t s_timer_stack[configTIMER_TASK_STACK_DEPTH];

} // anonymous namespace

// ---------------------------------------------------------------------------
// sentinel_create_tasks
//
// Creates all application tasks. Must be called before vTaskStartScheduler().
// Panics (halts) if any task creation fails — that would indicate a build
// configuration error (wrong stack size constant, etc.).
// ---------------------------------------------------------------------------
extern "C" void sentinel_create_tasks(void)
{
    TaskHandle_t h;

    // 1. KNOWLEDGE_BASE — priority 1
    h = xTaskCreateStatic(
        task_knowledge_base_entry,
        "KnowledgeBase",
        sentinel::TaskStackWords::KNOWLEDGE_BASE,
        nullptr,
        sentinel::TaskPriority::KNOWLEDGE_BASE,
        s_kb_stack,
        &s_kb_tcb);
    configASSERT(h != nullptr);
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::KNOWLEDGE_BASE)] = h;

    // 2. UI_RENDERER / APP_TASK — priority 3
    h = xTaskCreateStatic(
        task_app_entry,
        "AppTask",
        sentinel::TaskStackWords::APP_TASK,
        nullptr,
        sentinel::TaskPriority::APP_TASK,
        s_app_stack,
        &s_app_tcb);
    configASSERT(h != nullptr);
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::APP_TASK)]    = h;
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::UI_RENDERER)] = h;

    // 3. BG_SCANNER — priority 3
    h = xTaskCreateStatic(
        task_bg_scanner_entry,
        "BgScanner",
        sentinel::TaskStackWords::BG_SCANNER,
        nullptr,
        sentinel::TaskPriority::BG_SCANNER,
        s_scan_stack,
        &s_scan_tcb);
    configASSERT(h != nullptr);
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::BG_SCANNER)] = h;

    // 4. ESP32_COMMS — priority 4
    h = xTaskCreateStatic(
        task_esp32_comms_entry,
        "Esp32Comms",
        sentinel::TaskStackWords::ESP32_COMMS,
        nullptr,
        sentinel::TaskPriority::ESP32_COMMS,
        s_esp32_stack,
        &s_esp32_tcb);
    configASSERT(h != nullptr);
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::ESP32_COMMS)] = h;

    // 5. LOGGER — priority 4
    h = xTaskCreateStatic(
        task_logger_entry,
        "Logger",
        sentinel::TaskStackWords::LOGGER,
        nullptr,
        sentinel::TaskPriority::LOGGER,
        s_logger_stack,
        &s_logger_tcb);
    configASSERT(h != nullptr);
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::LOGGER)] = h;

    // 6. RADIO_MANAGER — priority 5 (highest user task)
    h = xTaskCreateStatic(
        task_radio_manager_entry,
        "RadioMgr",
        sentinel::TaskStackWords::RADIO_MANAGER,
        nullptr,
        sentinel::TaskPriority::RADIO_MANAGER,
        s_radio_stack,
        &s_radio_tcb);
    configASSERT(h != nullptr);
    sentinel::g_task_handles[static_cast<int>(sentinel::TaskID::RADIO_MANAGER)] = h;
}

// ---------------------------------------------------------------------------
// FreeRTOS static memory callbacks
// Required when configSUPPORT_STATIC_ALLOCATION = 1.
// ---------------------------------------------------------------------------

extern "C" void vApplicationGetIdleTaskMemory(
    StaticTask_t** ppxIdleTaskTCBBuffer,
    StackType_t**  ppxIdleTaskStackBuffer,
    uint32_t*      pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &s_idle_tcb;
    *ppxIdleTaskStackBuffer = s_idle_stack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

extern "C" void vApplicationGetTimerTaskMemory(
    StaticTask_t** ppxTimerTaskTCBBuffer,
    StackType_t**  ppxTimerTaskStackBuffer,
    uint32_t*      pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &s_timer_tcb;
    *ppxTimerTaskStackBuffer = s_timer_stack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

// ---------------------------------------------------------------------------
// FreeRTOS hook functions
// ---------------------------------------------------------------------------

// Idle hook: enter WFI sleep to save power.
// Called from the idle task when there is no work to do.
extern "C" void vApplicationIdleHook(void)
{
    __asm volatile("wfi" ::: "memory");
}

// Stack overflow hook: log the offending task name via UART and halt.
// Configured by configCHECK_FOR_STACK_OVERFLOW = 2 (canary method).
extern "C" void vApplicationStackOverflowHook(TaskHandle_t /*xTask*/, char* pcTaskName)
{
    uart_puts("\r\n*** FATAL: STACK OVERFLOW: ");
    uart_puts(pcTaskName);
    uart_puts(" ***\r\n");

    // Disable all interrupts and spin — recovery is not possible
    taskDISABLE_INTERRUPTS();
    for (;;) {
        __asm volatile("nop");
    }
}
