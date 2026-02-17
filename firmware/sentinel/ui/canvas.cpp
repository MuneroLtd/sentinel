// SPDX-License-Identifier: MIT
// Project Sentinel - canvas drawing primitives

#include "canvas.hpp"
#include "font.hpp"
#include "../bsp/lcd_ili9341.hpp"
#include <cstdio>
#include <cstring>
#include <cstdarg>

// Screen dimensions
static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 240;

// ---------------------------------------------------------------------------
// heat_color implementation (placed here to avoid a separate color.cpp)
// ---------------------------------------------------------------------------
sentinel::ui::Color sentinel::ui::heat_color(uint8_t intensity)
{
    // Classic 4-stop waterfall gradient:
    //   0-63:   black -> blue
    //   64-127: blue  -> green
    //   128-191: green -> yellow
    //   192-255: yellow -> red
    uint8_t r, g, b;
    if (intensity < 64) {
        // black -> blue
        r = 0;
        g = 0;
        b = static_cast<uint8_t>(intensity << 2);
    } else if (intensity < 128) {
        // blue -> green
        uint8_t t = static_cast<uint8_t>((intensity - 64) << 2);
        r = 0;
        g = t;
        b = static_cast<uint8_t>(255 - t);
    } else if (intensity < 192) {
        // green -> yellow
        uint8_t t = static_cast<uint8_t>((intensity - 128) << 2);
        r = t;
        g = 255;
        b = 0;
    } else {
        // yellow -> red
        uint8_t t = static_cast<uint8_t>((intensity - 192) << 2);
        r = 255;
        g = static_cast<uint8_t>(255 - t);
        b = 0;
    }
    return rgb(r, g, b);
}

// ---------------------------------------------------------------------------
// Canvas primitives
// ---------------------------------------------------------------------------

namespace sentinel::ui {

void fill_screen(Color c)
{
    lcd_fill_rect(0, 0, SCREEN_W, SCREEN_H, c);
}

void fill_rect(int x, int y, int w, int h, Color c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x >= SCREEN_W || y >= SCREEN_H || w <= 0 || h <= 0) return;
    if (x + w > SCREEN_W) w = SCREEN_W - x;
    if (y + h > SCREEN_H) h = SCREEN_H - y;
    lcd_fill_rect(static_cast<uint16_t>(x), static_cast<uint16_t>(y),
                  static_cast<uint16_t>(w), static_cast<uint16_t>(h), c);
}

void draw_rect(int x, int y, int w, int h, Color c)
{
    draw_hline(x,         y,         w, c);  // top
    draw_hline(x,         y + h - 1, w, c);  // bottom
    draw_vline(x,         y,         h, c);  // left
    draw_vline(x + w - 1, y,         h, c);  // right
}

void draw_hline(int x, int y, int len, Color c)
{
    if (y < 0 || y >= SCREEN_H || len <= 0) return;
    if (x < 0) { len += x; x = 0; }
    if (x >= SCREEN_W) return;
    if (x + len > SCREEN_W) len = SCREEN_W - x;
    if (len <= 0) return;
    lcd_fill_rect(static_cast<uint16_t>(x), static_cast<uint16_t>(y),
                  static_cast<uint16_t>(len), 1, c);
}

void draw_vline(int x, int y, int len, Color c)
{
    if (x < 0 || x >= SCREEN_W || len <= 0) return;
    if (y < 0) { len += y; y = 0; }
    if (y >= SCREEN_H) return;
    if (y + len > SCREEN_H) len = SCREEN_H - y;
    if (len <= 0) return;
    lcd_fill_rect(static_cast<uint16_t>(x), static_cast<uint16_t>(y),
                  1, static_cast<uint16_t>(len), c);
}

void draw_text(int x, int y, const char* s, Color fg, Color bg)
{
    if (!s) return;
    font_draw_str(static_cast<uint16_t>(x), static_cast<uint16_t>(y), s, fg, bg);
}

void draw_textf(int x, int y, Color fg, Color bg, const char* fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    draw_text(x, y, buf, fg, bg);
}

void draw_waterfall_row(int y, const int8_t* bins, int n_bins, int screen_w)
{
    if (!bins || n_bins <= 0 || screen_w <= 0) return;
    if (y < 0 || y >= SCREEN_H) return;

    for (int px = 0; px < screen_w; ++px) {
        // Map pixel column to bin index (nearest-neighbour)
        int bin_idx = (px * n_bins) / screen_w;
        if (bin_idx >= n_bins) bin_idx = n_bins - 1;

        // Convert dBFS (-128..0) to heat intensity (0..255)
        // -128 dBFS -> intensity 0  (cold/black)
        //    0 dBFS -> intensity 255 (hot/red)
        int8_t db = bins[bin_idx];
        uint8_t intensity = static_cast<uint8_t>(
            static_cast<int>(db + 128) & 0xFF);

        Color c = heat_color(intensity);
        lcd_draw_pixel(static_cast<uint16_t>(px), static_cast<uint16_t>(y), c);
    }
}

void draw_progress_bar(int x, int y, int w, int h,
                       int percent, Color fg, Color bar)
{
    // Clamp percent
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    // Outline
    draw_rect(x, y, w, h, fg);

    // Filled portion (1 pixel inset)
    int inner_w = ((w - 2) * percent) / 100;
    if (inner_w > 0) {
        fill_rect(x + 1, y + 1, inner_w, h - 2, bar);
    }
    // Unfilled remainder
    int remaining = (w - 2) - inner_w;
    if (remaining > 0) {
        fill_rect(x + 1 + inner_w, y + 1, remaining, h - 2, colors::BLACK);
    }
}

} // namespace sentinel::ui
