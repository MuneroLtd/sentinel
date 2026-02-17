// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Vector Table
//
// Places a minimal vector table at the start of the M0 SRAM image.
// The linker script (m0.ld) puts the .vectors section first in M0_SRAM.
// Cortex-M0 reads word[0] as initial SP, word[1] as reset handler.

#include <cstdint>

// Entry point (defined in main_m0.cpp)
extern "C" void m0_main(void);

// ISR handlers (defined in main_m0.cpp)
extern "C" void SysTick_Handler(void);
extern "C" void SGPIO_IRQHandler(void);

// Stack top symbol provided by m0.ld
extern uint32_t _stack_top;

// Default handler for unused interrupts — spin forever
extern "C" void Default_Handler(void) {
    while (true) { __asm volatile("bkpt #0"); }
}

// Cortex-M0 vector table (48 entries: 16 system + 32 external IRQs)
__attribute__((section(".vectors"), used))
const uint32_t m0_vector_table[48] = {
    reinterpret_cast<uint32_t>(&_stack_top),           // [0]  Initial SP
    reinterpret_cast<uint32_t>(&m0_main),              // [1]  Reset handler
    reinterpret_cast<uint32_t>(&Default_Handler),      // [2]  NMI
    reinterpret_cast<uint32_t>(&Default_Handler),      // [3]  HardFault
    0u, 0u, 0u, 0u, 0u, 0u, 0u,                       // [4-10]  Reserved
    reinterpret_cast<uint32_t>(&Default_Handler),      // [11] SVCall
    0u, 0u,                                            // [12-13] Reserved
    reinterpret_cast<uint32_t>(&Default_Handler),      // [14] PendSV
    reinterpret_cast<uint32_t>(&SysTick_Handler),      // [15] SysTick
    // External IRQs [16-47] — 32 entries
    reinterpret_cast<uint32_t>(&SGPIO_IRQHandler),     // [16] IRQ0  - SGPIO exchange
    reinterpret_cast<uint32_t>(&Default_Handler),      // [17] IRQ1  - M4CORE
    reinterpret_cast<uint32_t>(&Default_Handler),      // [18] IRQ2  - GPDMA (unused)
    0u, 0u, 0u, 0u, 0u, 0u, 0u,                       // [19-25] IRQ3-9
    0u, 0u, 0u, 0u, 0u, 0u, 0u,                       // [26-32] IRQ10-16
    0u, 0u, 0u, 0u, 0u, 0u, 0u,                       // [33-39] IRQ17-23
    0u, 0u, 0u, 0u, 0u, 0u, 0u,                       // [40-46] IRQ24-30
    0u,                                                // [47] IRQ31
};
