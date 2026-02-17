/*
 * Project Sentinel - FreeRTOS Run-Time Stats Counter
 *
 * Implements the two hooks required by FreeRTOS when
 * configGENERATE_RUN_TIME_STATS = 1 (see FreeRTOSConfig.h):
 *
 *   portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  -> sentinel_init_runtime_counter()
 *   portGET_RUN_TIME_COUNTER_VALUE()           -> sentinel_get_runtime_counter()
 *
 * Implementation uses the Cortex-M4 DWT (Data Watchpoint and Trace) unit's
 * CYCCNT register, which increments on every CPU clock cycle.
 *
 * At 204 MHz the counter rolls over every ~21 seconds, which is fine for
 * FreeRTOS run-time stats (they track relative percentages, not absolute time).
 *
 * The counter ticks at 204 MHz while the FreeRTOS tick runs at 1 kHz, so
 * run-time percentages have ~204 000x more resolution than tick-based
 * counters — accurate enough for identifying hot tasks.
 *
 * References:
 *   ARM DDI 0403E.d — ARMv7-M Architecture Reference Manual, Section C1.8
 *   LPC43xx User Manual UM10503 — Chapter 3 (Cortex-M4 core)
 */

#include "FreeRTOS.h"
#include "task.h"

// ---------------------------------------------------------------------------
// DWT register block base address (Cortex-M4, ARMv7-M §C1.8)
// ---------------------------------------------------------------------------
namespace dwt {

// CoreDebug base address
static constexpr uint32_t DEMCR_ADDR  = 0xE000EDFC;
// DWT base
static constexpr uint32_t DWT_BASE    = 0xE0001000;
static constexpr uint32_t DWT_CTRL    = DWT_BASE + 0x000;
static constexpr uint32_t DWT_CYCCNT  = DWT_BASE + 0x004;

// DEMCR bit: enables the DWT/ITM/ETM blocks
static constexpr uint32_t DEMCR_TRCENA = (1u << 24);

// DWT_CTRL bit: enables the cycle counter
static constexpr uint32_t DWT_CTRL_CYCCNTENA = (1u << 0);

inline volatile uint32_t& reg(uint32_t addr)
{
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

} // namespace dwt

// ---------------------------------------------------------------------------
// sentinel_init_runtime_counter
//
// Called once by the FreeRTOS kernel (via portCONFIGURE_TIMER_FOR_RUN_TIME_STATS)
// before vTaskStartScheduler() returns and task switching begins.
//
// Steps:
//   1. Enable the DWT block via DEMCR.TRCENA
//   2. Reset CYCCNT to 0
//   3. Enable CYCCNT via DWT_CTRL.CYCCNTENA
// ---------------------------------------------------------------------------
extern "C" void sentinel_init_runtime_counter(void)
{
    // Step 1: Enable trace / DWT block
    dwt::reg(dwt::DEMCR_ADDR) |= dwt::DEMCR_TRCENA;

    // Step 2: Reset the cycle counter to start from a known value
    dwt::reg(dwt::DWT_CYCCNT) = 0u;

    // Step 3: Enable cycle counting
    dwt::reg(dwt::DWT_CTRL) |= dwt::DWT_CTRL_CYCCNTENA;
}

// ---------------------------------------------------------------------------
// sentinel_get_runtime_counter
//
// Called by the FreeRTOS kernel (via portGET_RUN_TIME_COUNTER_VALUE) on every
// context switch to attribute CPU cycles to the task being switched out.
//
// Returns the current DWT CYCCNT value.  FreeRTOS only cares that this value
// is monotonically increasing between calls; rollover is handled internally.
// ---------------------------------------------------------------------------
extern "C" uint32_t sentinel_get_runtime_counter(void)
{
    return dwt::reg(dwt::DWT_CYCCNT);
}
