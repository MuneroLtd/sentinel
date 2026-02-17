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
#include "bsp/lcd_ili9341.hpp"
#include "ui/font.hpp"
#include "ui/color.hpp"
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
// RF-themed Matrix rainfall boot animation
// Green falling characters using radio/RF symbols and digits.
// Runs for ~2 seconds before the main boot sequence continues.
// ---------------------------------------------------------------------------
static void boot_animation() {
    using namespace sentinel::ui;

    // Simple xorshift32 PRNG
    static uint32_t rng_state = 0xDEADBEEF;
    auto rng = [&]() -> uint32_t {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 17;
        rng_state ^= rng_state << 5;
        return rng_state;
    };

    // RF-themed character set: digits, units, symbols
    static const char rf_chars[] =
        "0123456789"
        "MHZGHZDBM"
        "RFIQFFT"
        "><|~^v"
        "SENTINEL";

    static constexpr int COLS = LCD_WIDTH / FONT_W;   // 53 columns
    static constexpr int ROWS = LCD_HEIGHT / FONT_H;  // 30 rows
    static constexpr int NUM_DROPS = 32;  // active falling columns

    // Track position (row) and speed for each active drop
    struct Drop {
        uint8_t col;      // which screen column
        int8_t  row;      // current head position (can be negative = pending)
        uint8_t speed;    // rows per frame (1-3)
        uint8_t trail;    // trail length (4-12)
    };

    Drop drops[NUM_DROPS];
    for (int i = 0; i < NUM_DROPS; i++) {
        drops[i].col   = rng() % COLS;
        drops[i].row   = -(static_cast<int8_t>(rng() % 20));  // staggered start
        drops[i].speed = 1 + (rng() % 3);
        drops[i].trail = 4 + (rng() % 9);
    }

    lcd_clear(LCD_BLACK);

    // Animate for ~80 frames
    for (int frame = 0; frame < 80; frame++) {
        for (int i = 0; i < NUM_DROPS; i++) {
            Drop& d = drops[i];

            // Draw the bright head character
            if (d.row >= 0 && d.row < ROWS) {
                char ch = rf_chars[rng() % (sizeof(rf_chars) - 1)];
                font_draw_char(
                    d.col * FONT_W, d.row * FONT_H,
                    ch, colors::WHITE, colors::BLACK);
            }

            // Dim the previous head position to green
            if (d.row - 1 >= 0 && d.row - 1 < ROWS) {
                char ch = rf_chars[rng() % (sizeof(rf_chars) - 1)];
                font_draw_char(
                    d.col * FONT_W, (d.row - 1) * FONT_H,
                    ch, colors::GREEN, colors::BLACK);
            }

            // Dark green for older trail
            if (d.row - 3 >= 0 && d.row - 3 < ROWS) {
                char ch = rf_chars[rng() % (sizeof(rf_chars) - 1)];
                font_draw_char(
                    d.col * FONT_W, (d.row - 3) * FONT_H,
                    ch, colors::DARK_GREEN, colors::BLACK);
            }

            // Erase the tail
            int tail_row = d.row - d.trail;
            if (tail_row >= 0 && tail_row < ROWS) {
                lcd_fill_rect(
                    d.col * FONT_W, tail_row * FONT_H,
                    FONT_W, FONT_H, LCD_BLACK);
            }

            // Advance
            d.row += d.speed;

            // Reset when fully off-screen
            if (d.row - d.trail >= ROWS) {
                d.col   = rng() % COLS;
                d.row   = -(static_cast<int8_t>(rng() % 15));
                d.speed = 1 + (rng() % 3);
                d.trail = 4 + (rng() % 9);
            }
        }

        // ~25ms per frame at 96 MHz (busy wait, no timer yet)
        for (volatile uint32_t d = 0; d < 250000; d++) {
            __asm volatile("nop");
        }
    }

    // Final screen: clear and show Sentinel branding
    lcd_clear(LCD_BLACK);

    // Centre "SENTINEL" title
    static const char title[] = "S E N T I N E L";
    int title_x = (LCD_WIDTH - static_cast<int>(sizeof(title) - 1) * FONT_W) / 2;
    font_draw_str(title_x, 100, title, colors::GREEN, colors::BLACK);

    // Tagline
    static const char tagline[] = "RF Awareness Platform";
    int tag_x = (LCD_WIDTH - static_cast<int>(sizeof(tagline) - 1) * FONT_W) / 2;
    font_draw_str(tag_x, 120, tagline, colors::DARK_GREEN, colors::BLACK);

    // Version
    static const char ver[] = "v0.1.9";
    int ver_x = (LCD_WIDTH - static_cast<int>(sizeof(ver) - 1) * FONT_W) / 2;
    font_draw_str(ver_x, 140, ver, colors::GREY, colors::BLACK);

    // Hold branding for ~1 second at 96 MHz
    for (volatile uint32_t d = 0; d < 5000000; d++) {
        __asm volatile("nop");
    }
}

// ---------------------------------------------------------------------------
// sentinel_main — called by startup code as the C entry point
// ---------------------------------------------------------------------------
extern "C" void sentinel_main() {

    // -------------------------------------------------------------------------
    // 0. Early LED setup — configure LEDs for boot status indication.
    //    Uses IRC clock (~12 MHz), GPIO2 is on APB which is always clocked.
    // -------------------------------------------------------------------------
    {
        // Configure LED1 (P4_1 → GPIO2[1]) as output
        scu_set_pinmode(LED1_SCU_GRP, LED1_SCU_PIN, LED1_SCU_FUNC, SCU_MODE_INACT);
        gpio_set_dir(LED1_GPIO_PORT, LED1_GPIO_PIN, true);

        // Configure LED2 (P4_2 → GPIO2[2]) as output
        scu_set_pinmode(LED2_SCU_GRP, LED2_SCU_PIN, LED2_SCU_FUNC, SCU_MODE_INACT);
        gpio_set_dir(LED2_GPIO_PORT, LED2_GPIO_PIN, true);

        // Configure LED3 (P6_12 → GPIO2[8]) as output
        scu_set_pinmode(LED3_SCU_GRP, LED3_SCU_PIN, LED3_SCU_FUNC, SCU_MODE_INACT);
        gpio_set_dir(LED3_GPIO_PORT, LED3_GPIO_PIN, true);

        // LED2 on = "boot in progress"
        gpio_write(LED2_GPIO_PORT, LED2_GPIO_PIN, true);
        gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, false);
    }

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
    // 2b. LCD initialisation — ILI9341 via 8-bit parallel bus through CPLD.
    //     lcd_init() configures SCU pin mux (including P2_0/P2_1 which are
    //     shared with UART0), GPIO directions, CPLD IO registers, and the
    //     full ILI9341 command sequence.
    //     WARNING: UART output ceases after this point (P2_0/P2_1 reassigned).
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] Initialising LCD (UART will stop after this)...\r\n");

    // === GPIO READBACK DIAGNOSTIC ===
    // Test WRX (GPIO1[10]) pin BEFORE lcd_init to verify GPIO works.
    // First, manually configure just the WRX pin for GPIO output.
    scu_set_pinmode(LCD_WRX_SCU_GRP, LCD_WRX_SCU_PIN, LCD_WRX_SCU_FUNC,
                    (1u << 4) | (1u << 6));  // 0x50: EPUN=1, EZI=1
    LPC_GPIO->SET[1] = (1u << 10);   // preload HIGH
    LPC_GPIO->DIR[1] |= (1u << 10);  // output

    bool wrx_ok = true;

    // Test 1: should be HIGH (we just set it)
    for (volatile int d = 0; d < 1000; d++) { __asm volatile("nop"); }
    if (!(LPC_GPIO->PIN[1] & (1u << 10))) wrx_ok = false;

    // Test 2: drive LOW
    LPC_GPIO->CLR[1] = (1u << 10);
    for (volatile int d = 0; d < 1000; d++) { __asm volatile("nop"); }
    if (LPC_GPIO->PIN[1] & (1u << 10)) wrx_ok = false;

    // Test 3: drive HIGH again
    LPC_GPIO->SET[1] = (1u << 10);
    for (volatile int d = 0; d < 1000; d++) { __asm volatile("nop"); }
    if (!(LPC_GPIO->PIN[1] & (1u << 10))) wrx_ok = false;

    // Also verify SCU register for P2_9
    volatile uint32_t* scu_p2_9 = reinterpret_cast<volatile uint32_t*>(0x40086124u);
    uint32_t scu_val = *scu_p2_9;
    bool scu_ok = ((scu_val & 0x7F) == 0x50);

    // Also verify DIR bit
    bool dir_ok = (LPC_GPIO->DIR[1] & (1u << 10)) != 0;

    if (wrx_ok && scu_ok && dir_ok) {
        // GPIO1[10] WRX toggles correctly - blink LED1 3x fast (pass)
        for (int i = 0; i < 3; i++) {
            gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, true);
            for (volatile uint32_t d = 0; d < 200000; d++) { __asm volatile("nop"); }
            gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, false);
            for (volatile uint32_t d = 0; d < 200000; d++) { __asm volatile("nop"); }
        }
    } else {
        // FAIL: blink LED3 with error code
        // Count: 1=WRX toggle, 2=SCU config, 3=DIR bit
        int code = !wrx_ok ? 1 : (!scu_ok ? 2 : 3);
        for (int round = 0; round < 5; round++) {
            for (int i = 0; i < code; i++) {
                gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, true);
                for (volatile uint32_t d = 0; d < 500000; d++) { __asm volatile("nop"); }
                gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, false);
                for (volatile uint32_t d = 0; d < 500000; d++) { __asm volatile("nop"); }
            }
            for (volatile uint32_t d = 0; d < 1500000; d++) { __asm volatile("nop"); }
        }
    }

    // === LCD INIT (with slow timing for diagnostic) ===
    lcd_init();

    // Blink LED1 5x to confirm lcd_init completed
    for (int i = 0; i < 5; i++) {
        gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, true);
        for (volatile uint32_t d = 0; d < 500000; d++) { __asm volatile("nop"); }
        gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, false);
        for (volatile uint32_t d = 0; d < 500000; d++) { __asm volatile("nop"); }
    }

    // Fill screen with RED — slow but diagnostic
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_RED);

    // Blink LED1 5 more times to confirm fill completed
    for (int i = 0; i < 5; i++) {
        gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, true);
        for (volatile uint32_t d = 0; d < 500000; d++) { __asm volatile("nop"); }
        gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, false);
        for (volatile uint32_t d = 0; d < 500000; d++) { __asm volatile("nop"); }
    }

    // Skip boot_animation for now — diagnostic mode
    // boot_animation();

    // -------------------------------------------------------------------------
    // 3. SGPIO pin mux — 8 data pins + 1 clock pin for baseband IQ capture
    //    Must be done before sgpio_init() and before M0 release.
    //    All SGPIO pins: input buffer enabled, no pull, high-speed slew.
    // -------------------------------------------------------------------------
    // NOTE: All uart_puts/printf removed after lcd_init — UART pins are now LCD bus.

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
    sgpio_init();

    // -------------------------------------------------------------------------
    // 5. Zero the shared IPC memory region
    //    This guarantees both cores see a clean state before the M0 starts.
    // -------------------------------------------------------------------------
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

        // Guard against oversized firmware (SRAM1 = 72 KB, shared IPC uses top 8 KB)
        // Safe load region: 0x10080000 – 0x10088000 = 32 KB below shared region
        static constexpr size_t M0_SRAM_LIMIT = 0x8000u; // 32 KB
        if (fw_sz > M0_SRAM_LIMIT) {
            while (true) { } // Halt — M0 firmware too large
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
    sgpio_start();

    // -------------------------------------------------------------------------
    // 8. Release M0APP from reset via CREG + RGU
    //    Sequence (UM10503 §4.5):
    //      a) Write M0 start address to CREG_M0APPMEMMAP
    //      b) Assert M0APP reset via RGU (so changes take effect)
    //      c) Deassert M0APP reset to start the core
    // -------------------------------------------------------------------------
    // Set M0APP memory map to our SRAM load address
    *CREG_M0APPMEMMAP = M0_SRAM_BASE;

    // Assert M0APP reset
    *RGU_RESET_CTRL1 = RGU_M0APP_RST;
    // Short delay for reset to propagate
    for (volatile uint32_t i = 0; i < 10000; i++) { __asm volatile("nop"); }

    // Deassert M0APP reset (write 0 to the reset bit)
    *RGU_RESET_CTRL1 = 0u;

    // -------------------------------------------------------------------------
    // 9. Initialise the event bus (creates FreeRTOS mutex — must be before
    //    vTaskStartScheduler but can be done without the scheduler running
    //    since xSemaphoreCreateMutex works before the scheduler starts).
    // -------------------------------------------------------------------------
    if (!sentinel::EventBus::instance().init()) {
        while (true) { } // FATAL: EventBus init failed
    }

    // -------------------------------------------------------------------------
    // 10. Create all application tasks (static allocation, no heap needed)
    // -------------------------------------------------------------------------
    sentinel_create_tasks();

    // Turn off LED2 ("boot in progress"), turn on LED3 ("boot complete")
    gpio_write(LED2_GPIO_PORT, LED2_GPIO_PIN, false);
    gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, true);

    // -------------------------------------------------------------------------
    // 11. Start the FreeRTOS scheduler — should never return.
    //    If it does, something is catastrophically wrong.
    // -------------------------------------------------------------------------
    vTaskStartScheduler();

    // -------------------------------------------------------------------------
    // Should never reach here.
    // -------------------------------------------------------------------------
    // FATAL: Scheduler returned
    while (true) {
        __asm volatile("wfe");
    }
}
