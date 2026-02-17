// SPDX-License-Identifier: MIT
// Project Sentinel — M4 Application Core Entry Point
//
// Called by startup assembly after it has:
//   - Set up the initial MSP from the vector table
//   - Copied .data from flash to SRAM
//   - Zeroed .bss
//
// Execution order:
//   1.  clocks_init()         — PLL1 → 204 MHz; peripheral + SGPIO clocks on
//   2.  uart_init()           — USART0 for debug output
//   3.  SGPIO pin mux         — 8 data + 1 clock pin for IQ capture from CPLD
//   4.  sgpio_init()          — Configure SGPIO slices for 8-bit parallel
//   5.  Shared IPC region     — zeroed (SharedMemory at SHARED_RAM_BASE)
//   6.  M0 firmware copy      — linked-in blob → 0x10080000 M0 SRAM
//   7.  sgpio_start()         — Start SGPIO capture before M0 wakes
//   8.  M0 release from reset — CREG M0APP memory map + RGU reset release
//   9.  EventBus::init()      — create inter-task message infrastructure
//  10.  sentinel_create_tasks()— create all FreeRTOS tasks (static alloc)
//  11.  vTaskStartScheduler() — hand control to FreeRTOS; never returns

#include "hal/clocks.hpp"
#include "hal/uart.hpp"
#include "hal/gpio.hpp"
#include "hal/sgpio.hpp"
#include "hal/lpc4320_regs.hpp"
#include "bsp/portapack_pins.hpp"
#include "event_bus/event_bus.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// Shared IPC
#include "ipc/ipc_protocol.hpp"

#include <cstring>
#include <cstdint>

// ---------------------------------------------------------------------------
// M0 binary symbols — provided by the linker script (sentinel_m4.ld).
// The baseband binary is linked into the .m0_binary section in Flash.
// The linker script exports _m0_binary_start and _m0_binary_end.
// ---------------------------------------------------------------------------
extern "C" {
    extern const uint8_t _m0_binary_start[];
    extern const uint8_t _m0_binary_end[];
}

// ---------------------------------------------------------------------------
// M0 SRAM load address (LPC4320 SRAM1 — visible to both cores)
// The M0APP reset handler vector at [0] and initial SP at [-4] must be
// at this address when the M0 is released.
// ---------------------------------------------------------------------------
static constexpr uint32_t M0_SRAM_BASE = 0x10080000u;

// ---------------------------------------------------------------------------
// Forward declaration: task orchestrator (task_orchestrator.cpp)
// ---------------------------------------------------------------------------
extern "C" void sentinel_create_tasks();

// ---------------------------------------------------------------------------
// sentinel_main — called by startup code as the C entry point
// ---------------------------------------------------------------------------
extern "C" void sentinel_main() {

    // -------------------------------------------------------------------------
    // 1. Clock initialisation — must be first; all subsequent code assumes
    //    204 MHz core clock and enabled peripheral clocks.
    // -------------------------------------------------------------------------
    clocks_init();

    // -------------------------------------------------------------------------
    // 2. UART pin mux + UART init for debug output
    // -------------------------------------------------------------------------
    scu_set_pinmode(UART0_TX_SCU_GRP, UART0_TX_SCU_PIN, UART0_TX_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_IBUF);
    scu_set_pinmode(UART0_RX_SCU_GRP, UART0_RX_SCU_PIN, UART0_RX_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_IBUF);

    uart_init(115200);

    uart_puts("\r\n[SENTINEL] Boot\r\n");
    uart_printf("[SENTINEL] Core clock: %u Hz\r\n", clocks_get_cpu_hz());

    // -------------------------------------------------------------------------
    // 3. SGPIO pin mux — 8 data pins + 1 clock pin for baseband IQ capture
    //    Must be done before sgpio_init() and before M0 release.
    //    All SGPIO pins: input buffer enabled, no pull, high-speed slew.
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] Configuring SGPIO pins...\r\n");

    static constexpr uint32_t SGPIO_PIN_MODE = SCU_MODE_INACT | SCU_MODE_IBUF | SCU_MODE_ZIF_DIS;

    // SGPIO0–SGPIO7 data lines
    scu_set_pinmode(SGPIO0_SCU_GRP, SGPIO0_SCU_PIN, SGPIO0_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO1_SCU_GRP, SGPIO1_SCU_PIN, SGPIO1_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO2_SCU_GRP, SGPIO2_SCU_PIN, SGPIO2_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO3_SCU_GRP, SGPIO3_SCU_PIN, SGPIO3_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO4_SCU_GRP, SGPIO4_SCU_PIN, SGPIO4_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO5_SCU_GRP, SGPIO5_SCU_PIN, SGPIO5_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO6_SCU_GRP, SGPIO6_SCU_PIN, SGPIO6_SCU_FUNC, SGPIO_PIN_MODE);
    scu_set_pinmode(SGPIO7_SCU_GRP, SGPIO7_SCU_PIN, SGPIO7_SCU_FUNC, SGPIO_PIN_MODE);

    // SGPIO8 clock line (from CPLD) — CLK2 is a dedicated pin, not a P-group pin.
    // SCU register at SCU_BASE + 0xC08. CLK pins use func 1 for clock input.
    {
        volatile uint32_t* sfsclk2 = reinterpret_cast<volatile uint32_t*>(
            SCU_BASE + SCU_SFSCLK2_OFFSET);
        *sfsclk2 = 1u | SGPIO_PIN_MODE;  // func 1 = CLK input to SGPIO
    }

    // -------------------------------------------------------------------------
    // 4. SGPIO peripheral init — configure slices for 8-bit parallel capture
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] Initialising SGPIO...\r\n");
    sgpio_init();

    // -------------------------------------------------------------------------
    // 5. Zero the shared IPC memory region
    //    This guarantees both cores see a clean state before the M0 starts.
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] Clearing IPC shared memory...\r\n");
    ::memset(reinterpret_cast<void*>(sentinel::ipc::SHARED_RAM_BASE),
             0,
             sentinel::ipc::SHARED_RAM_SIZE);

    // -------------------------------------------------------------------------
    // 6. Copy M0 firmware into M0 SRAM (0x10080000)
    //    The baseband binary is linked in as a binary object section.
    //    We copy it to the start of M0's view of SRAM1.
    // -------------------------------------------------------------------------
    {
        const uint8_t* src   = _m0_binary_start;
        const uint8_t* end   = _m0_binary_end;
        const size_t   fw_sz = static_cast<size_t>(end - src);

        uart_printf("[SENTINEL] Copying M0 firmware: %u bytes to 0x%X\r\n",
                    static_cast<uint32_t>(fw_sz),
                    static_cast<uint32_t>(M0_SRAM_BASE));

        // Guard against oversized firmware (SRAM1 = 72 KB, shared IPC uses top 8 KB)
        // Safe load region: 0x10080000 – 0x10088000 = 32 KB below shared region
        static constexpr size_t M0_SRAM_LIMIT = 0x8000u; // 32 KB
        if (fw_sz > M0_SRAM_LIMIT) {
            uart_puts("[SENTINEL] ERROR: M0 firmware too large!\r\n");
            while (true) { } // Halt
        }

        ::memcpy(reinterpret_cast<void*>(M0_SRAM_BASE), src, fw_sz);

        // Data memory barrier — ensure all writes to SRAM complete before
        // the M0 starts fetching from that memory.
        __asm volatile("dsb" ::: "memory");
        __asm volatile("isb" ::: "memory");
    }

    // -------------------------------------------------------------------------
    // 7. Start SGPIO capture before M0 release — ensures data is flowing
    //    when the M0 enables its exchange interrupt.
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] Starting SGPIO capture...\r\n");
    sgpio_start();

    // -------------------------------------------------------------------------
    // 8. Release M0APP from reset via CREG + RGU
    //    Sequence (UM10503 §4.5):
    //      a) Write M0 start address to CREG_M0APPMEMMAP
    //      b) Assert M0APP reset via RGU (so changes take effect)
    //      c) Deassert M0APP reset to start the core
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] Releasing M0 core...\r\n");

    // Set M0APP memory map to our SRAM load address
    *CREG_M0APPMEMMAP = M0_SRAM_BASE;

    // Assert M0APP reset
    *RGU_RESET_CTRL1 = RGU_M0APP_RST;
    // Short delay for reset to propagate
    for (volatile uint32_t i = 0; i < 10000; i++) { __asm volatile("nop"); }

    // Deassert M0APP reset (write 0 to the reset bit)
    *RGU_RESET_CTRL1 = 0u;

    uart_puts("[SENTINEL] M0 core started.\r\n");

    // -------------------------------------------------------------------------
    // 9. Initialise the event bus (creates FreeRTOS mutex — must be before
    //    vTaskStartScheduler but can be done without the scheduler running
    //    since xSemaphoreCreateMutex works before the scheduler starts).
    // -------------------------------------------------------------------------
    if (!sentinel::EventBus::instance().init()) {
        uart_puts("[SENTINEL] FATAL: EventBus init failed!\r\n");
        while (true) { }
    }

    uart_puts("[SENTINEL] EventBus ready.\r\n");

    // -------------------------------------------------------------------------
    // 10. Create all application tasks (static allocation, no heap needed)
    // -------------------------------------------------------------------------
    sentinel_create_tasks();
    uart_puts("[SENTINEL] Tasks created. Starting scheduler...\r\n");

    // -------------------------------------------------------------------------
    // 11. Start the FreeRTOS scheduler — should never return.
    //    If it does, something is catastrophically wrong.
    // -------------------------------------------------------------------------
    vTaskStartScheduler();

    // -------------------------------------------------------------------------
    // Should never reach here.
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] FATAL: Scheduler returned!\r\n");
    while (true) {
        __asm volatile("wfe");
    }
}
