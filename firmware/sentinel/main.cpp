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
    //     v0.2.5 diagnostic: Full Mayhem init sequence, full-screen fill,
    //     LCD ID read, UART register dump, clear LED staging.
    //     WARNING: UART output ceases after pin reassignment.
    // -------------------------------------------------------------------------
    uart_puts("[SENTINEL] v0.2.5 LCD Diagnostic\r\n");

    // Dump GPIO register state BEFORE pin reassignment
    uart_puts("[SENTINEL] Pre-LCD GPIO state:\r\n");
    uart_printf("  GPIO1 DIR=0x%08X PIN=0x%08X\r\n", LPC_GPIO->DIR[1], LPC_GPIO->PIN[1]);
    uart_printf("  GPIO3 DIR=0x%08X PIN=0x%08X\r\n", LPC_GPIO->DIR[3], LPC_GPIO->PIN[3]);
    uart_printf("  GPIO5 DIR=0x%08X PIN=0x%08X\r\n", LPC_GPIO->DIR[5], LPC_GPIO->PIN[5]);

    // Dump SCU registers for LCD control pins
    {
        auto read_scu = [](uint8_t grp, uint8_t pin) -> uint32_t {
            volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
                SCU_BASE + grp * 0x80u + pin * 4u);
            return *reg;
        };
        uart_printf("  SCU P2_0 =0x%03X P2_1 =0x%03X P2_3 =0x%03X\r\n",
            read_scu(2,0), read_scu(2,1), read_scu(2,3));
        uart_printf("  SCU P2_4 =0x%03X P2_9 =0x%03X P2_13=0x%03X\r\n",
            read_scu(2,4), read_scu(2,9), read_scu(2,13));
        uart_printf("  SCU P7_0 =0x%03X P7_1 =0x%03X P7_7 =0x%03X\r\n",
            read_scu(7,0), read_scu(7,1), read_scu(7,7));
    }

    // LED blink helper: slow, distinct blinks on LED1
    auto led_stage = [](int count) {
        for (volatile uint32_t d = 0; d < 10000000; d++) __asm volatile("nop"); // 1s pause
        for (int i = 0; i < count; i++) {
            gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, true);
            for (volatile uint32_t d = 0; d < 5000000; d++) __asm volatile("nop"); // 500ms on
            gpio_write(LED1_GPIO_PORT, LED1_GPIO_PIN, false);
            for (volatile uint32_t d = 0; d < 5000000; d++) __asm volatile("nop"); // 500ms off
        }
    };

    // Delay helpers (calibrated for 204 MHz: ~41 iter/us)
    auto delay_us_local = [](uint32_t us) {
        volatile uint32_t count = us * 41u;
        while (count--) __asm volatile("nop");
    };
    auto delay_ms_local = [&](uint32_t ms) { delay_us_local(ms * 1000u); };

    {
        static constexpr uint32_t PIN_MODE = (1u << 4) | (1u << 6); // 0x50: EPUN, EZI

        // ===== STAGE 1: Pin setup =====
        uart_puts("[SENTINEL] Stage 1: Pin setup\r\n");

        // Configure SCU pin mux for all LCD pins
        scu_set_pinmode(LCD_WRX_SCU_GRP, LCD_WRX_SCU_PIN, LCD_WRX_SCU_FUNC, PIN_MODE);
        scu_set_pinmode(LCD_RDX_SCU_GRP, LCD_RDX_SCU_PIN, LCD_RDX_SCU_FUNC, PIN_MODE);
        scu_set_pinmode(LCD_ADDR_SCU_GRP, LCD_ADDR_SCU_PIN, LCD_ADDR_SCU_FUNC, PIN_MODE);
        scu_set_pinmode(LCD_DIR_SCU_GRP, LCD_DIR_SCU_PIN, LCD_DIR_SCU_FUNC, PIN_MODE);
        scu_set_pinmode(LCD_STBX_SCU_GRP, LCD_STBX_SCU_PIN, LCD_STBX_SCU_FUNC, PIN_MODE);
        scu_set_pinmode(LCD_TE_SCU_GRP, LCD_TE_SCU_PIN, LCD_TE_SCU_FUNC, PIN_MODE);
        for (uint8_t p = 0; p < 8; p++)
            scu_set_pinmode(LCD_D0_SCU_GRP, p, LCD_DATA_SCU_FUNC, PIN_MODE);

        // Set GPIO MASK for data bus port — ONLY bits [15:8] writable via MPIN
        LPC_GPIO->MASK[LCD_DATA_GPIO_PORT] = ~LCD_DATA_MASK;

        // Preload safe values BEFORE setting directions (prevent glitches)
        LPC_GPIO->MPIN[LCD_DATA_GPIO_PORT] = 0;
        LPC_GPIO->SET[LCD_RDX_GPIO_PORT] = (1u << LCD_RDX_GPIO_PIN);   // RDX high
        LPC_GPIO->SET[LCD_WRX_GPIO_PORT] = (1u << LCD_WRX_GPIO_PIN);   // WRX high
        LPC_GPIO->SET[LCD_STBX_GPIO_PORT] = (1u << LCD_STBX_GPIO_PIN); // IO_STB high
        LPC_GPIO->CLR[LCD_ADDR_GPIO_PORT] = (1u << LCD_ADDR_GPIO_PIN); // ADDR low
        LPC_GPIO->DIR[LCD_DATA_GPIO_PORT] &= ~LCD_DATA_MASK;           // bus = inputs
        LPC_GPIO->SET[LCD_DIR_GPIO_PORT] = (1u << LCD_DIR_GPIO_PIN);   // DIR = read

        // Now set control pins as outputs
        gpio_set_dir(LCD_DIR_GPIO_PORT, LCD_DIR_GPIO_PIN, true);
        gpio_set_dir(LCD_RDX_GPIO_PORT, LCD_RDX_GPIO_PIN, true);
        gpio_set_dir(LCD_WRX_GPIO_PORT, LCD_WRX_GPIO_PIN, true);
        gpio_set_dir(LCD_STBX_GPIO_PORT, LCD_STBX_GPIO_PIN, true);
        gpio_set_dir(LCD_ADDR_GPIO_PORT, LCD_ADDR_GPIO_PIN, true);

        // Dump GPIO state after pin setup
        uart_puts("[SENTINEL] Post-setup GPIO state:\r\n");
        uart_printf("  GPIO1 DIR=0x%08X PIN=0x%08X\r\n", LPC_GPIO->DIR[1], LPC_GPIO->PIN[1]);
        uart_printf("  GPIO3 DIR=0x%08X PIN=0x%08X MASK=0x%08X\r\n",
            LPC_GPIO->DIR[3], LPC_GPIO->PIN[3], LPC_GPIO->MASK[3]);
        uart_printf("  GPIO5 DIR=0x%08X PIN=0x%08X\r\n", LPC_GPIO->DIR[5], LPC_GPIO->PIN[5]);

        led_stage(1);  // LED1 × 1 = pin setup done

        // ===== STAGE 2: GPIO verification =====
        uart_puts("[SENTINEL] Stage 2: GPIO verify\r\n");
        bool all_ok = true;
        auto delay_short = []() {
            for (volatile int d = 0; d < 200; d++) __asm volatile("nop");
        };

        // Test WRX
        LPC_GPIO->CLR[1] = (1u << 10); delay_short();
        if (LPC_GPIO->PIN[1] & (1u << 10)) all_ok = false;
        LPC_GPIO->SET[1] = (1u << 10); delay_short();
        if (!(LPC_GPIO->PIN[1] & (1u << 10))) all_ok = false;

        // Test DIR
        LPC_GPIO->CLR[1] = (1u << 13); delay_short();
        if (LPC_GPIO->PIN[1] & (1u << 13)) all_ok = false;
        LPC_GPIO->SET[1] = (1u << 13); delay_short();
        if (!(LPC_GPIO->PIN[1] & (1u << 13))) all_ok = false;

        // Test ADDR
        LPC_GPIO->SET[5] = (1u << 1); delay_short();
        if (!(LPC_GPIO->PIN[5] & (1u << 1))) all_ok = false;
        LPC_GPIO->CLR[5] = (1u << 1); delay_short();
        if (LPC_GPIO->PIN[5] & (1u << 1)) all_ok = false;

        // Test IO_STBX
        LPC_GPIO->CLR[5] = (1u << 0); delay_short();
        if (LPC_GPIO->PIN[5] & (1u << 0)) all_ok = false;
        LPC_GPIO->SET[5] = (1u << 0); delay_short();
        if (!(LPC_GPIO->PIN[5] & (1u << 0))) all_ok = false;

        // Test data bus 0xAA / 0x55
        LPC_GPIO->CLR[1] = (1u << 13);
        LPC_GPIO->DIR[3] |= LCD_DATA_MASK;
        LPC_GPIO->MPIN[3] = 0xAA00u; delay_short();
        uint32_t bus_aa = (LPC_GPIO->PIN[3] >> 8) & 0xFF;
        if (bus_aa != 0xAA) all_ok = false;
        LPC_GPIO->MPIN[3] = 0x5500u; delay_short();
        uint32_t bus_55 = (LPC_GPIO->PIN[3] >> 8) & 0xFF;
        if (bus_55 != 0x55) all_ok = false;
        LPC_GPIO->MPIN[3] = 0;
        LPC_GPIO->DIR[3] &= ~LCD_DATA_MASK;
        LPC_GPIO->SET[1] = (1u << 13);

        uart_printf("[SENTINEL] GPIO test: %s (bus=0x%02X,0x%02X)\r\n",
            all_ok ? "PASS" : "FAIL", bus_aa, bus_55);

        if (all_ok) led_stage(2);  // LED1 × 2 = GPIO pass
        else {
            // LED3 × 5 for fail (unmistakable)
            for (int i = 0; i < 5; i++) {
                gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, true);
                for (volatile uint32_t d = 0; d < 5000000; d++) __asm volatile("nop");
                gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, false);
                for (volatile uint32_t d = 0; d < 5000000; d++) __asm volatile("nop");
            }
        }

        // ===== Bus operation helpers =====
        auto cpld_io_write = [&](uint8_t val) {
            LPC_GPIO->MPIN[3] = static_cast<uint32_t>(val) << 8;
            LPC_GPIO->CLR[1] = (1u << 13);           // DIR = write
            LPC_GPIO->DIR[3] |= LCD_DATA_MASK;
            LPC_GPIO->SET[5] = (1u << 1);            // ADDR = 1
            delay_us_local(5);
            LPC_GPIO->CLR[5] = (1u << 0);            // IO_STB low
            delay_us_local(5);
            LPC_GPIO->SET[5] = (1u << 0);            // IO_STB high (rising edge latches)
            delay_us_local(5);
        };

        auto lcd_cmd = [&](uint8_t cmd) {
            LPC_GPIO->MPIN[3] = 0;                    // high byte = 0
            LPC_GPIO->CLR[1] = (1u << 13);            // DIR = write
            LPC_GPIO->DIR[3] |= LCD_DATA_MASK;
            LPC_GPIO->CLR[5] = (1u << 1);             // ADDR = 0 (command)
            delay_us_local(2);
            LPC_GPIO->CLR[1] = (1u << 10);            // WRX low → CPLD latches 0x00
            delay_us_local(2);
            LPC_GPIO->MPIN[3] = static_cast<uint32_t>(cmd) << 8;
            delay_us_local(2);
            LPC_GPIO->SET[1] = (1u << 10);            // WRX high → CPLD sends 0x00:cmd
            delay_us_local(2);
            LPC_GPIO->SET[5] = (1u << 1);             // ADDR = 1 (data mode)
        };

        auto lcd_dat = [&](uint16_t val) {
            LPC_GPIO->MPIN[3] = val & 0xFF00u;        // high byte in bits [15:8]
            delay_us_local(2);
            LPC_GPIO->CLR[1] = (1u << 10);            // WRX low → CPLD latches high byte
            delay_us_local(2);
            LPC_GPIO->MPIN[3] = static_cast<uint32_t>(val & 0xFFu) << 8; // low byte
            delay_us_local(2);
            LPC_GPIO->SET[1] = (1u << 10);            // WRX high → CPLD sends 16-bit
            delay_us_local(2);
        };

        auto lcd_param = [&](uint8_t val) {
            lcd_dat(static_cast<uint16_t>(val));
        };

        auto lcd_cmd_params = [&](uint8_t cmd, const uint8_t* data, int len) {
            lcd_cmd(cmd);
            for (int i = 0; i < len; i++)
                lcd_param(data[i]);
        };

        // ===== STAGE 3: CPLD reset =====
        uart_puts("[SENTINEL] Stage 3: CPLD reset\r\n");
        cpld_io_write(0x02);     // deassert LCD reset (bit 0=0)
        delay_ms_local(1);
        cpld_io_write(0x03);     // assert LCD reset (bit 0=1)
        delay_ms_local(10);
        cpld_io_write(0x02);     // deassert LCD reset
        delay_ms_local(120);

        // Backlight ON
        cpld_io_write(0x82);     // bit 7=backlight, bit 1=audio reset deasserted
        uart_puts("[SENTINEL] Backlight ON\r\n");

        led_stage(3);  // LED1 × 3 = CPLD reset + backlight done

        // ===== STAGE 4: Full Mayhem ILI9341 init sequence =====
        uart_puts("[SENTINEL] Stage 4: Full ILI9341 init\r\n");

        // Software reset
        lcd_cmd(0x01);
        delay_ms_local(120);

        // Power Control B
        { uint8_t d[] = {0x00, 0xD9, 0x30}; lcd_cmd_params(0xCF, d, 3); }
        // Power On Sequence
        { uint8_t d[] = {0x64, 0x03, 0x12, 0x81}; lcd_cmd_params(0xED, d, 4); }
        // Driver Timing A
        { uint8_t d[] = {0x85, 0x10, 0x78}; lcd_cmd_params(0xE8, d, 3); }
        // Power Control A
        { uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02}; lcd_cmd_params(0xCB, d, 5); }
        // Pump Ratio
        { uint8_t d[] = {0x20}; lcd_cmd_params(0xF7, d, 1); }
        // Driver Timing B
        { uint8_t d[] = {0x00, 0x00}; lcd_cmd_params(0xEA, d, 2); }
        // Frame Rate
        { uint8_t d[] = {0x00, 0x1B}; lcd_cmd_params(0xB1, d, 2); }
        // Blanking Porch
        { uint8_t d[] = {0x02, 0x02, 0x0A, 0x14}; lcd_cmd_params(0xB5, d, 4); }
        // Display Function
        { uint8_t d[] = {0x0A, 0xA2, 0x27, 0x00}; lcd_cmd_params(0xB6, d, 4); }
        // Power Control 1
        { uint8_t d[] = {0x1B}; lcd_cmd_params(0xC0, d, 1); }
        // Power Control 2
        { uint8_t d[] = {0x12}; lcd_cmd_params(0xC1, d, 1); }
        // VCOM 1
        { uint8_t d[] = {0x32, 0x3C}; lcd_cmd_params(0xC5, d, 2); }
        // VCOM 2
        { uint8_t d[] = {0x9B}; lcd_cmd_params(0xC7, d, 1); }
        // MADCTL: MY=1, MX=1, ML=1, BGR=1 = 0xD8
        { uint8_t d[] = {0xD8}; lcd_cmd_params(0x36, d, 1); }
        // Pixel Format: 16bpp
        { uint8_t d[] = {0x55}; lcd_cmd_params(0x3A, d, 1); }
        // Interface Control
        { uint8_t d[] = {0x01, 0x30, 0x00}; lcd_cmd_params(0xF6, d, 3); }
        // 3Gamma disable
        { uint8_t d[] = {0x00}; lcd_cmd_params(0xF2, d, 1); }
        // Gamma Curve
        { uint8_t d[] = {0x01}; lcd_cmd_params(0x26, d, 1); }
        // Positive Gamma
        { uint8_t d[] = {0x0F, 0x1D, 0x19, 0x0E, 0x10, 0x07,
                         0x4C, 0x63, 0x3F, 0x03, 0x0D, 0x00,
                         0x26, 0x24, 0x04}; lcd_cmd_params(0xE0, d, 15); }
        // Negative Gamma
        { uint8_t d[] = {0x00, 0x1C, 0x1F, 0x02, 0x0F, 0x03,
                         0x35, 0x25, 0x47, 0x04, 0x0C, 0x0B,
                         0x29, 0x2F, 0x05}; lcd_cmd_params(0xE1, d, 15); }
        // Sleep Out
        lcd_cmd(0x11);
        delay_ms_local(120);
        // Display On
        lcd_cmd(0x29);
        delay_ms_local(20);
        // Tearing Effect Line On
        { uint8_t d[] = {0x00}; lcd_cmd_params(0x35, d, 1); }

        uart_puts("[SENTINEL] Init sequence complete\r\n");
        led_stage(4);  // LED1 × 4 = init done

        // ===== STAGE 5: Full-screen RED fill (240×320 = 76800 pixels) =====
        uart_puts("[SENTINEL] Stage 5: Full-screen RED\r\n");

        // CASET: columns 0–239
        lcd_cmd(0x2A);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x00); lcd_param(0xEF);  // 239

        // PASET: rows 0–319
        lcd_cmd(0x2B);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x01); lcd_param(0x3F);  // 319

        // RAMWR
        lcd_cmd(0x2C);
        for (uint32_t i = 0; i < 76800u; i++) {
            lcd_dat(0xF800);  // RED
        }

        led_stage(5);  // LED1 × 5 = RED fill done

        // ===== STAGE 6: Try other colors to rule out color decoding =====
        // Full-screen WHITE
        lcd_cmd(0x2A);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x00); lcd_param(0xEF);
        lcd_cmd(0x2B);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x01); lcd_param(0x3F);
        lcd_cmd(0x2C);
        for (uint32_t i = 0; i < 76800u; i++) {
            lcd_dat(0xFFFF);  // WHITE
        }
        delay_ms_local(1000);

        // Full-screen BLUE
        lcd_cmd(0x2A);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x00); lcd_param(0xEF);
        lcd_cmd(0x2B);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x01); lcd_param(0x3F);
        lcd_cmd(0x2C);
        for (uint32_t i = 0; i < 76800u; i++) {
            lcd_dat(0x001F);  // BLUE
        }
        delay_ms_local(1000);

        // Full-screen BLACK
        lcd_cmd(0x2A);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x00); lcd_param(0xEF);
        lcd_cmd(0x2B);
        lcd_param(0x00); lcd_param(0x00);
        lcd_param(0x01); lcd_param(0x3F);
        lcd_cmd(0x2C);
        for (uint32_t i = 0; i < 76800u; i++) {
            lcd_dat(0x0000);  // BLACK
        }

        led_stage(6);  // LED1 × 6 = color fills done

        // ===== STAGE 7: LCD ID read attempt =====
        // Send command 0x04 (RDDID), then try to read 3 bytes back
        // via the CPLD read path (RDX strobe).
        uint8_t id_bytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        {
            lcd_cmd(0x04);  // RDDID command

            // Switch to read mode
            LPC_GPIO->DIR[3] &= ~LCD_DATA_MASK;           // data bus = inputs
            LPC_GPIO->SET[1] = (1u << 13);                // DIR = 1 (CPLD drives to MCU)
            delay_us_local(10);

            // Read 4 bytes (1 dummy + 3 ID bytes)
            for (int i = 0; i < 4; i++) {
                LPC_GPIO->CLR[5] = (1u << 4);             // RDX low
                delay_us_local(10);
                id_bytes[i] = static_cast<uint8_t>((LPC_GPIO->PIN[3] >> 8) & 0xFF);
                LPC_GPIO->SET[5] = (1u << 4);             // RDX high
                delay_us_local(10);
            }

            // Switch back to write mode
            LPC_GPIO->CLR[1] = (1u << 13);                // DIR = 0 (MCU drives)
            LPC_GPIO->DIR[3] |= LCD_DATA_MASK;            // data bus = outputs
        }

        // Report LCD ID via LED pattern:
        // If any byte != 0xFF and != 0x00, LCD responded → LED1 × 7
        // If all bytes are 0xFF or 0x00 → LED3 × 7
        bool lcd_responded = false;
        for (int i = 0; i < 4; i++) {
            if (id_bytes[i] != 0xFF && id_bytes[i] != 0x00)
                lcd_responded = true;
        }

        if (lcd_responded) {
            led_stage(7);  // LED1 × 7 = LCD responded
        } else {
            // LED3 × 7 for no response
            for (volatile uint32_t d = 0; d < 10000000; d++) __asm volatile("nop");
            for (int i = 0; i < 7; i++) {
                gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, true);
                for (volatile uint32_t d = 0; d < 5000000; d++) __asm volatile("nop");
                gpio_write(LED3_GPIO_PORT, LED3_GPIO_PIN, false);
                for (volatile uint32_t d = 0; d < 5000000; d++) __asm volatile("nop");
            }
        }

        // ===== STAGE 8: Final — toggle backlight to prove CPLD alive =====
        delay_ms_local(500);
        cpld_io_write(0x02);  // backlight OFF
        delay_ms_local(1000);
        cpld_io_write(0x82);  // backlight ON
        delay_ms_local(500);
        cpld_io_write(0x02);  // backlight OFF
        delay_ms_local(1000);
        cpld_io_write(0x82);  // backlight ON

        // Leave LED2 ON as "still running" indicator
        gpio_write(LED2_GPIO_PORT, LED2_GPIO_PIN, true);
    }

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
