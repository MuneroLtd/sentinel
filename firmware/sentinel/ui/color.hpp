// SPDX-License-Identifier: MIT
// Project Sentinel - RGB565 colour constants and helpers
#pragma once

#include <cstdint>

namespace sentinel::ui {

using Color = uint16_t;  // RGB565

namespace colors {
    constexpr Color BLACK   = 0x0000;
    constexpr Color WHITE   = 0xFFFF;
    constexpr Color RED     = 0xF800;
    constexpr Color GREEN   = 0x07E0;
    constexpr Color BLUE    = 0x001F;
    constexpr Color CYAN    = 0x07FF;
    constexpr Color MAGENTA = 0xF81F;
    constexpr Color YELLOW  = 0xFFE0;
    constexpr Color ORANGE  = 0xFD20;
    constexpr Color GREY    = 0x7BEF;
    constexpr Color DARK    = 0x18C3;  // dark background (~0x18C3 ≈ #306486)
    constexpr Color DARK_GREY   = 0x4208;  // medium dark grey
    constexpr Color LIGHT_GREY  = 0xC618;  // light grey
    constexpr Color DARK_GREEN  = 0x03E0;  // darker green
    constexpr Color DARK_RED    = 0x7800;  // darker red
    constexpr Color DARK_BLUE   = 0x000F;  // darker blue
    constexpr Color NAVY        = 0x000F;
    constexpr Color LIME        = 0x07E0;
    constexpr Color TEAL        = 0x0410;
    constexpr Color PURPLE      = 0x8010;
    constexpr Color BROWN       = 0xA145;
    constexpr Color PINK        = 0xF81F;
    constexpr Color SILVER      = 0xC618;
    constexpr Color MAROON      = 0x7800;
    constexpr Color OLIVE       = 0x7BE0;
    // Status colours
    constexpr Color STATUS_BG   = 0x18E3;   // status bar background
    constexpr Color STATUS_FG   = 0xFFFF;   // status bar foreground
    constexpr Color HIGHLIGHT   = 0x051F;   // focused widget highlight
    constexpr Color BUTTON_BG   = 0x2945;   // button background
    constexpr Color BUTTON_FG   = 0xFFFF;   // button foreground
    constexpr Color BUTTON_FOC  = 0x051F;   // focused button background
    constexpr Color ROW_EVEN    = 0x1082;   // even table row background
    constexpr Color ROW_ODD     = 0x0841;   // odd table row background
} // namespace colors

// Pack 8-bit RGB into RGB565
constexpr Color rgb(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<Color>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

// Waterfall heat-map: intensity 0-255 -> RGB565 colour
// Maps: 0=black, 64=blue, 128=green, 192=yellow, 255=red (classic waterfall)
Color heat_color(uint8_t intensity);

} // namespace sentinel::ui
