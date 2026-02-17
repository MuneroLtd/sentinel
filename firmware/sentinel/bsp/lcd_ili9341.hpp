// SPDX-License-Identifier: MIT
// Project Sentinel — ILI9341 320×240 16-bit TFT LCD Driver
//
// Interface: SPI (SSP1), 8-bit transfers.
// Colour depth: 16 bits per pixel (RGB565).
// Resolution: 320 × 240 landscape (or 240 × 320 portrait, selectable by rotation).
//
// Pin control is via GPIO (CS, DC, RST).  CS is active-low.
// DC high = pixel/parameter data, DC low = command byte.
// RST active-low hardware reset.

#pragma once
#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// Colour packing
// ---------------------------------------------------------------------------

// Pack an 8-bit R/G/B triplet into a 16-bit RGB565 word (big-endian on wire).
inline uint16_t lcd_rgb(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(
        ((static_cast<uint16_t>(r) & 0xF8u) << 8) |
        ((static_cast<uint16_t>(g) & 0xFCu) << 3) |
        (static_cast<uint16_t>(b)            >> 3)
    );
}

// Predefined colours
static constexpr uint16_t LCD_BLACK   = 0x0000u;
static constexpr uint16_t LCD_WHITE   = 0xFFFFu;
static constexpr uint16_t LCD_RED     = 0xF800u;
static constexpr uint16_t LCD_GREEN   = 0x07E0u;
static constexpr uint16_t LCD_BLUE    = 0x001Fu;
static constexpr uint16_t LCD_YELLOW  = 0xFFE0u;
static constexpr uint16_t LCD_CYAN    = 0x07FFu;
static constexpr uint16_t LCD_MAGENTA = 0xF81Fu;

// ---------------------------------------------------------------------------
// LCD dimensions
// ---------------------------------------------------------------------------
static constexpr uint16_t LCD_WIDTH   = 320;
static constexpr uint16_t LCD_HEIGHT  = 240;

// ---------------------------------------------------------------------------
// Driver API
// ---------------------------------------------------------------------------

// Full ILI9341 hardware reset + initialisation sequence.
// Must call gpio_init / ssp_init for the LCD bus before calling this.
void lcd_init();

// Set the active drawing window (column/row address set).
// All subsequent pixel writes go to this window.
void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

// Fill a rectangle with a solid 16-bit colour.
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color565);

// Draw a single pixel.
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color565);

// Bulk blit: write w×h pixels from a 16-bit RGB565 buffer.
// 'pixels' must contain at least w*h entries.
// Pixels are transmitted in row-major order (top-left first).
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* pixels);

// Clear the entire screen to 'color565'.
inline void lcd_clear(uint16_t color565) {
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color565);
}
