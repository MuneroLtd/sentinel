#pragma once

// ============================================================
// Project Sentinel - FreeRTOS Configuration
// Target: LPC4320 Cortex-M4 @ 204MHz
// Port: GCC/ARM_CM4F (hardware FPU enabled)
// ============================================================

// Scheduler
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      204000000UL
#define configTICK_RATE_HZ                      1000U       // 1ms resolution
#define configMAX_PRIORITIES                    8
#define configMINIMAL_STACK_SIZE                128         // Idle task (words)
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0           // 32-bit tick counter

// Memory
#define configTOTAL_HEAP_SIZE                   (14 * 1024) // 14KB
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         1
#define configAPPLICATION_ALLOCATED_HEAP        0

// Features
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configQUEUE_REGISTRY_SIZE               10
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               7           // Highest priority
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            256

// Debugging (enable during development, disable for production)
#define configUSE_TRACE_FACILITY                1
#define configGENERATE_RUN_TIME_STATS           1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
#define configCHECK_FOR_STACK_OVERFLOW          2           // Method 2: canary

// Co-routines (not used)
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

// Hook function prototypes (implemented in task_app.cpp)
#define configASSERT(x) do { if (!(x)) { taskDISABLE_INTERRUPTS(); for(;;); } } while(0)

// Run-time stats counter (use DWT cycle counter on Cortex-M4)
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()    sentinel_init_runtime_counter()
#define portGET_RUN_TIME_COUNTER_VALUE()             sentinel_get_runtime_counter()

// Required API includes
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  1
#define INCLUDE_pcTaskGetTaskName               1

// Cortex-M4 interrupt priority settings
// Interrupt priorities: 0 = highest. Must use same grouping as NVIC.
// FreeRTOS requires lowest priority interrupts for SVC and PendSV.
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY     \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - 4))    // 4-bit priority
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - 4))

// Runtime stats helpers (implemented in sentinel_runtime.cpp)
#ifdef __cplusplus
extern "C" {
#endif
void sentinel_init_runtime_counter(void);
uint32_t sentinel_get_runtime_counter(void);
#ifdef __cplusplus
}
#endif
