// SPDX-License-Identifier: MIT
// Project Sentinel — ILI9341 320×240 LCD Driver Implementation
//
// 8-bit parallel 8080-I bus via EPM240 CPLD, matching the PortaPack H1/H2/H4M
// hardware.  The CPLD latches two 8-bit bus writes (high byte on WR↓, low byte
// on WR↑) and forwards a single 16-bit word to the ILI9341.
//
// Init sequence ported from portapack-mayhem lcd_ili9341.cpp lcd_init_pp().

#include "lcd_ili9341.hpp"
#include "portapack_pins.hpp"
#include "../hal/gpio.hpp"
#include "../hal/lpc4320_regs.hpp"
#include <cstddef>

// ---------------------------------------------------------------------------
// ILI9341 command bytes
// ---------------------------------------------------------------------------
static constexpr uint8_t ILI9341_SLPIN      = 0x10;
static constexpr uint8_t ILI9341_SLPOUT     = 0x11;
static constexpr uint8_t ILI9341_DISPON     = 0x29;
static constexpr uint8_t ILI9341_CASET      = 0x2A;
static constexpr uint8_t ILI9341_PASET      = 0x2B;
static constexpr uint8_t ILI9341_RAMWR      = 0x2C;

// ---------------------------------------------------------------------------
// Simple delay helpers (calibrated for 204 MHz core)
// Each iteration ≈ 5 cycles → 204M/5 ≈ 40.8M iter/s
// ---------------------------------------------------------------------------
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 41u;
    while (count--) {
        __asm volatile("nop");
    }
}

static void delay_ms(uint32_t ms) {
    delay_us(ms * 1000u);
}

// ---------------------------------------------------------------------------
// CPLD IO register shadow — tracks the last value written.
// Bit 0: LCD reset (1 = in reset)
// Bit 1: Audio codec reset (1 = in reset)
// Bit 7: Backlight (1 = on)
// ---------------------------------------------------------------------------
static uint8_t io_reg = 0x03u;  // Initial: both resets asserted

// ---------------------------------------------------------------------------
// Low-level parallel bus primitives
// These directly manipulate LPC_GPIO registers matching the Mayhem
// portapack_io.hpp implementation.
// ---------------------------------------------------------------------------

// Set GPIO port 3 mask so MPIN only touches data bus bits [15:8]
static inline void data_mask_set() {
    LPC_GPIO->MASK[LCD_DATA_GPIO_PORT] = ~LCD_DATA_MASK;
}

// Write high byte of a 16-bit value: value's bits [15:8] land on bus
static inline void data_write_high(const uint32_t value) {
    LPC_GPIO->MPIN[LCD_DATA_GPIO_PORT] = value;
}

// Write low byte of a 16-bit value: shift bits [7:0] into bus position [15:8]
static inline void data_write_low(const uint32_t value) {
    LPC_GPIO->MPIN[LCD_DATA_GPIO_PORT] = (value << LCD_DATA_SHIFT);
}

// WR strobe (active low)
static inline void lcd_wr_assert()   { LPC_GPIO->CLR[LCD_WRX_GPIO_PORT] = (1u << LCD_WRX_GPIO_PIN); }
static inline void lcd_wr_deassert() { LPC_GPIO->SET[LCD_WRX_GPIO_PORT] = (1u << LCD_WRX_GPIO_PIN); }

// RD strobe (active low)
static inline void lcd_rd_deassert() { LPC_GPIO->SET[LCD_RDX_GPIO_PORT] = (1u << LCD_RDX_GPIO_PIN); }

// IO strobe (active low)
static inline void io_stb_assert()   { LPC_GPIO->CLR[LCD_STBX_GPIO_PORT] = (1u << LCD_STBX_GPIO_PIN); }
static inline void io_stb_deassert() { LPC_GPIO->SET[LCD_STBX_GPIO_PORT] = (1u << LCD_STBX_GPIO_PIN); }

// ADDR: 0 = command, 1 = data
static inline void addr(bool a) {
    if (a)
        LPC_GPIO->SET[LCD_ADDR_GPIO_PORT] = (1u << LCD_ADDR_GPIO_PIN);
    else
        LPC_GPIO->CLR[LCD_ADDR_GPIO_PORT] = (1u << LCD_ADDR_GPIO_PIN);
}

// Bus direction: MCU writes (DIR=0, data pins = outputs)
static inline void dir_write() {
    LPC_GPIO->CLR[LCD_DIR_GPIO_PORT] = (1u << LCD_DIR_GPIO_PIN);
    LPC_GPIO->DIR[LCD_DATA_GPIO_PORT] |= LCD_DATA_MASK;
}

// Bus direction: CPLD drives (data pins = inputs, DIR=1)
static inline void dir_read() {
    LPC_GPIO->DIR[LCD_DATA_GPIO_PORT] &= ~LCD_DATA_MASK;
    LPC_GPIO->SET[LCD_DIR_GPIO_PORT] = (1u << LCD_DIR_GPIO_PIN);
}

// ---------------------------------------------------------------------------
// CPLD IO register write
// Writes an 8-bit value to the CPLD IO register via the IO_STBX strobe.
// ---------------------------------------------------------------------------
static void io_write(bool address, uint8_t value) {
    data_write_low(value);
    dir_write();
    addr(address);
    delay_us(2);
    io_stb_assert();
    delay_us(2);
    io_stb_deassert();
    delay_us(2);
}

// ---------------------------------------------------------------------------
// LCD command: send an 8-bit command via the 16-bit CPLD path
// High byte = 0, low byte = command, ADDR=0 during write.
// ---------------------------------------------------------------------------
static void lcd_command(uint8_t cmd) {
    data_write_high(0);
    dir_write();
    addr(false);
    delay_us(2);
    lcd_wr_assert();
    delay_us(2);
    data_write_low(cmd);
    delay_us(2);
    lcd_wr_deassert();
    delay_us(2);
    addr(true);  // pre-set for data phase
}

// ---------------------------------------------------------------------------
// LCD 16-bit data write: two-phase through CPLD
// High byte latched on WR↓, low byte passed on WR↑.
// Assumes dir_write() already called and ADDR=1.
// ---------------------------------------------------------------------------
static inline __attribute__((always_inline)) void lcd_write_data(uint16_t value) {
    data_write_high(value);
    delay_us(2);
    lcd_wr_assert();
    delay_us(2);
    data_write_low(value);
    delay_us(2);
    lcd_wr_deassert();
    delay_us(2);
}

// ---------------------------------------------------------------------------
// Send command + N data parameters (each param is 8-bit, sent as 16-bit)
// ---------------------------------------------------------------------------
static void lcd_cmd_data(uint8_t cmd, const uint8_t* data, size_t len) {
    lcd_command(cmd);
    dir_write();
    for (size_t i = 0; i < len; i++) {
        lcd_write_data(static_cast<uint16_t>(data[i]));
    }
}

// ---------------------------------------------------------------------------
// SCU pin mode for LCD parallel bus pins: no pullup, input buffer enabled
// EPUN=1 (bit 4), EZI=1 (bit 6), ZIF=0 (bit 7 clear)
// ---------------------------------------------------------------------------
static constexpr uint32_t LCD_PIN_MODE = (1u << 4) | (1u << 6);  // 0x50

// ---------------------------------------------------------------------------
// lcd_init — full pin setup, CPLD reset, ILI9341 init, backlight on
// ---------------------------------------------------------------------------
void lcd_init() {
    // ----- 1. Configure SCU pin mux for all LCD control signals -----

    // LCD_WRX: P2_9 func 0
    scu_set_pinmode(LCD_WRX_SCU_GRP, LCD_WRX_SCU_PIN, LCD_WRX_SCU_FUNC, LCD_PIN_MODE);
    // LCD_RDX: P2_4 func 4
    scu_set_pinmode(LCD_RDX_SCU_GRP, LCD_RDX_SCU_PIN, LCD_RDX_SCU_FUNC, LCD_PIN_MODE);
    // ADDR: P2_1 func 4
    scu_set_pinmode(LCD_ADDR_SCU_GRP, LCD_ADDR_SCU_PIN, LCD_ADDR_SCU_FUNC, LCD_PIN_MODE);
    // DIR: P2_13 func 0
    scu_set_pinmode(LCD_DIR_SCU_GRP, LCD_DIR_SCU_PIN, LCD_DIR_SCU_FUNC, LCD_PIN_MODE);
    // IO_STBX: P2_0 func 4
    scu_set_pinmode(LCD_STBX_SCU_GRP, LCD_STBX_SCU_PIN, LCD_STBX_SCU_FUNC, LCD_PIN_MODE);
    // LCD_TE: P2_3 func 4 (input)
    scu_set_pinmode(LCD_TE_SCU_GRP, LCD_TE_SCU_PIN, LCD_TE_SCU_FUNC, LCD_PIN_MODE);

    // 8-bit data bus: P7_0..P7_7 func 0
    for (uint8_t pin = 0; pin < 8; pin++) {
        scu_set_pinmode(LCD_D0_SCU_GRP, pin, LCD_DATA_SCU_FUNC, LCD_PIN_MODE);
    }

    // ----- 2. Preload output values BEFORE enabling output direction -----
    // Matching Mayhem IO::init() pattern: set-value-then-drive to prevent
    // glitches on strobe lines (a brief LOW on WRX/IO_STBX would trigger
    // a spurious CPLD latch).
    data_mask_set();
    data_write_high(0);
    dir_read();            // data bus = inputs, DIR = high (safe)
    lcd_rd_deassert();     // RDX = high (preload output register)
    lcd_wr_deassert();     // WRX = high (preload output register)
    io_stb_deassert();     // IO_STBX = high (preload output register)
    addr(false);           // ADDR = low (preload)

    // ----- 3. Now enable output direction on control pins -----
    // The output register already holds the correct (deasserted) values,
    // so switching to output drives the correct level immediately.
    gpio_set_dir(LCD_DIR_GPIO_PORT,  LCD_DIR_GPIO_PIN,  true);
    gpio_set_dir(LCD_RDX_GPIO_PORT,  LCD_RDX_GPIO_PIN,  true);
    gpio_set_dir(LCD_WRX_GPIO_PORT,  LCD_WRX_GPIO_PIN,  true);
    gpio_set_dir(LCD_STBX_GPIO_PORT, LCD_STBX_GPIO_PIN, true);
    gpio_set_dir(LCD_ADDR_GPIO_PORT, LCD_ADDR_GPIO_PIN,  true);
    // LCD_TE is an input — leave default (input)

    // ----- 4. CPLD reset sequence via IO register -----
    // Sequence from Mayhem lcd_reset(): deassert → assert → deassert
    // with 1ms / 10ms / 120ms delays between transitions.
    io_reg = 0x03u;  // start with both resets asserted

    // Deassert LCD reset first (bit 0 = 0)
    io_reg &= ~0x01u;  // io_reg = 0x02
    io_write(true, io_reg);
    delay_ms(1);

    // Assert LCD reset (bit 0 = 1) — active reset pulse
    io_reg |= 0x01u;   // io_reg = 0x03
    io_write(true, io_reg);
    delay_ms(10);

    // Deassert LCD reset (bit 0 = 0) — LCD starts initialising
    io_reg &= ~0x01u;  // io_reg = 0x02
    io_write(true, io_reg);
    delay_ms(120);

    // ----- 5. Software reset (some panels need this in addition to HW reset) -----
    lcd_command(0x01);  // SWRESET
    delay_ms(120);

    // ----- 6. ILI9341 init command sequence (from Mayhem lcd_init_pp) -----

    // Power Control B
    { uint8_t d[] = {0x00, 0xD9, 0x30};
      lcd_cmd_data(0xCF, d, sizeof(d)); }

    // Power On Sequence Control
    { uint8_t d[] = {0x64, 0x03, 0x12, 0x81};
      lcd_cmd_data(0xED, d, sizeof(d)); }

    // Driver Timing Control A
    { uint8_t d[] = {0x85, 0x10, 0x78};
      lcd_cmd_data(0xE8, d, sizeof(d)); }

    // Power Control A
    { uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
      lcd_cmd_data(0xCB, d, sizeof(d)); }

    // Pump Ratio Control
    { uint8_t d[] = {0x20};
      lcd_cmd_data(0xF7, d, sizeof(d)); }

    // Driver Timing Control B
    { uint8_t d[] = {0x00, 0x00};
      lcd_cmd_data(0xEA, d, sizeof(d)); }

    // Frame Rate Control (Normal Mode)
    { uint8_t d[] = {0x00, 0x1B};
      lcd_cmd_data(0xB1, d, sizeof(d)); }

    // Blanking Porch Control
    { uint8_t d[] = {0x02, 0x02, 0x0A, 0x14};
      lcd_cmd_data(0xB5, d, sizeof(d)); }

    // Display Function Control (TFT, normally white)
    { uint8_t d[] = {0x0A, 0xA2, 0x27, 0x00};
      lcd_cmd_data(0xB6, d, sizeof(d)); }

    // Power Control 1: VRH = 4.60V
    { uint8_t d[] = {0x1B};
      lcd_cmd_data(0xC0, d, sizeof(d)); }

    // Power Control 2
    { uint8_t d[] = {0x12};
      lcd_cmd_data(0xC1, d, sizeof(d)); }

    // VCOM Control 1
    { uint8_t d[] = {0x32, 0x3C};
      lcd_cmd_data(0xC5, d, sizeof(d)); }

    // VCOM Control 2
    { uint8_t d[] = {0x9B};
      lcd_cmd_data(0xC7, d, sizeof(d)); }

    // Memory Access Control: MY=1, MX=1, MV=0, ML=1, BGR=1 → 0xD8
    // Matches Mayhem PortaPack exactly (portrait 240×320).
    { uint8_t d[] = {0xD8};
      lcd_cmd_data(0x36, d, sizeof(d)); }

    // Pixel Format: 16bpp
    { uint8_t d[] = {0x55};
      lcd_cmd_data(0x3A, d, sizeof(d)); }

    // Interface Control
    { uint8_t d[] = {0x01, 0x30, 0x00};
      lcd_cmd_data(0xF6, d, sizeof(d)); }

    // 3-Gamma Function Disable
    { uint8_t d[] = {0x00};
      lcd_cmd_data(0xF2, d, sizeof(d)); }

    // Gamma Curve Selected
    { uint8_t d[] = {0x01};
      lcd_cmd_data(0x26, d, sizeof(d)); }

    // Positive Gamma Correction
    { uint8_t d[] = {0x0F, 0x1D, 0x19, 0x0E, 0x10, 0x07,
                     0x4C, 0x63, 0x3F, 0x03, 0x0D, 0x00,
                     0x26, 0x24, 0x04};
      lcd_cmd_data(0xE0, d, sizeof(d)); }

    // Negative Gamma Correction
    { uint8_t d[] = {0x00, 0x1C, 0x1F, 0x02, 0x0F, 0x03,
                     0x35, 0x25, 0x47, 0x04, 0x0C, 0x0B,
                     0x29, 0x2F, 0x05};
      lcd_cmd_data(0xE1, d, sizeof(d)); }

    // Sleep Out
    lcd_command(ILI9341_SLPOUT);
    delay_ms(120);

    // Display On
    lcd_command(ILI9341_DISPON);

    // Tearing Effect Line On (mode 0 = V-blanking only)
    { uint8_t d[] = {0x00};
      lcd_cmd_data(0x35, d, sizeof(d)); }

    // ----- 6. Enable backlight -----
    io_reg |= (1u << 7);
    io_write(true, io_reg);

    // ----- 7. Clear screen -----
    lcd_clear(LCD_BLACK);
}

// ---------------------------------------------------------------------------
// lcd_set_window — CASET + PASET + RAMWR
// ---------------------------------------------------------------------------
void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column Address Set (0x2A)
    {
        uint8_t d[4] = {
            static_cast<uint8_t>(x0 >> 8),
            static_cast<uint8_t>(x0 & 0xFF),
            static_cast<uint8_t>(x1 >> 8),
            static_cast<uint8_t>(x1 & 0xFF)
        };
        lcd_cmd_data(ILI9341_CASET, d, sizeof(d));
    }
    // Page Address Set (0x2B)
    {
        uint8_t d[4] = {
            static_cast<uint8_t>(y0 >> 8),
            static_cast<uint8_t>(y0 & 0xFF),
            static_cast<uint8_t>(y1 >> 8),
            static_cast<uint8_t>(y1 & 0xFF)
        };
        lcd_cmd_data(ILI9341_PASET, d, sizeof(d));
    }
    // Memory Write command
    lcd_command(ILI9341_RAMWR);
}

// ---------------------------------------------------------------------------
// lcd_draw_pixel
// ---------------------------------------------------------------------------
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color565) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;

    lcd_set_window(x, y, x, y);
    dir_write();
    lcd_write_data(color565);
}

// ---------------------------------------------------------------------------
// lcd_fill_rect — optimised parallel bus pixel loop
// ---------------------------------------------------------------------------
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color565) {
    if (w == 0 || h == 0) return;
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;

    // Clip
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH  - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    lcd_set_window(x, y, x + w - 1u, y + h - 1u);

    dir_write();

    uint32_t count = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);
    while (count--) {
        data_write_high(color565);
        delay_us(2);
        lcd_wr_assert();
        delay_us(2);
        data_write_low(color565);
        delay_us(2);
        lcd_wr_deassert();
        delay_us(2);
    }
}

// ---------------------------------------------------------------------------
// lcd_blit — transfer a pixel buffer to the LCD
// ---------------------------------------------------------------------------
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* pixels) {
    if (w == 0 || h == 0 || !pixels) return;

    lcd_set_window(x, y, x + w - 1u, y + h - 1u);

    dir_write();

    uint32_t count = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);
    const uint16_t* p = pixels;
    while (count--) {
        uint16_t pixel = *p++;
        data_write_high(pixel);
        lcd_wr_assert();
        data_write_low(pixel);
        __asm volatile("nop"); __asm volatile("nop"); __asm volatile("nop");
        lcd_wr_deassert();
    }
}

// ---------------------------------------------------------------------------
// lcd_backlight — control via CPLD IO register bit 7
// ---------------------------------------------------------------------------
void lcd_backlight(bool on) {
    if (on)
        io_reg |= (1u << 7);
    else
        io_reg &= ~(1u << 7);
    io_write(true, io_reg);
}

// ---------------------------------------------------------------------------
// lcd_sleep — enter ILI9341 sleep mode, turn off backlight
// ---------------------------------------------------------------------------
void lcd_sleep() {
    lcd_backlight(false);
    lcd_command(ILI9341_SLPIN);
    delay_ms(5);
}

// ---------------------------------------------------------------------------
// lcd_wake — exit ILI9341 sleep mode, turn on backlight
// ---------------------------------------------------------------------------
void lcd_wake() {
    lcd_command(ILI9341_SLPOUT);
    delay_ms(120);
    lcd_command(ILI9341_DISPON);
    lcd_backlight(true);
}
