// SPDX-License-Identifier: MIT
// Project Sentinel - Drawing primitives over lcd_* functions
#pragma once

#include <cstdint>
#include <cstdarg>
#include "color.hpp"

namespace sentinel::ui {

// Fill the entire 320x240 screen with colour c
void fill_screen(Color c);

// Filled rectangle
void fill_rect(int x, int y, int w, int h, Color c);

// Outline rectangle (1 pixel border)
void draw_rect(int x, int y, int w, int h, Color c);

// Horizontal line
void draw_hline(int x, int y, int len, Color c);

// Vertical line
void draw_vline(int x, int y, int len, Color c);

// Draw null-terminated string using 6x8 font
void draw_text(int x, int y, const char* s,
               Color fg, Color bg = colors::BLACK);

// printf-style text draw
void draw_textf(int x, int y, Color fg, Color bg,
                const char* fmt, ...);

// Draw a waterfall row at screen row y.
// bins: array of n_bins values in dBFS range [-128..0].
// Bins are stretched/compressed to fit screen_w pixels using nearest-neighbour.
void draw_waterfall_row(int y, const int8_t* bins, int n_bins, int screen_w);

// Draw a horizontal progress bar.
// percent: 0-100
// fg  = border/text colour
// bar = filled bar colour
void draw_progress_bar(int x, int y, int w, int h,
                       int percent, Color fg, Color bar);

} // namespace sentinel::ui
