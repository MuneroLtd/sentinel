// SPDX-License-Identifier: MIT
// Project Sentinel - Embedded 6x8 bitmap font, ASCII 32-126
#pragma once

#include <cstdint>
#include <cstdarg>
#include "color.hpp"

// Character dimensions
static constexpr int FONT_W = 6;
static constexpr int FONT_H = 8;

// Raw font table: font_6x8[char - 32][col], 6 cols per char, 8 bits per col
// Bit 7 (MSB) = top pixel; bit 0 (LSB) = bottom pixel
extern const uint8_t font_6x8[95][6];  // ASCII 32 (' ') .. 126 ('~')

// Draw a single 6x8 character at (x,y)
void font_draw_char(uint16_t x, uint16_t y, char c,
                    sentinel::ui::Color fg, sentinel::ui::Color bg);

// Draw a null-terminated string
void font_draw_str(uint16_t x, uint16_t y, const char* s,
                   sentinel::ui::Color fg, sentinel::ui::Color bg);

// printf-style string draw
void font_draw_strf(uint16_t x, uint16_t y,
                    sentinel::ui::Color fg, sentinel::ui::Color bg,
                    const char* fmt, ...);
