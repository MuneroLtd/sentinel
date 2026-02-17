// SPDX-License-Identifier: MIT
// Project Sentinel - Minimal widget system (no heap allocation)
#pragma once

#include <cstdint>
#include <cstdarg>
#include "color.hpp"

namespace sentinel::ui {

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

// ---------------------------------------------------------------------------
// Navigation key enum
// ---------------------------------------------------------------------------

enum class NavKey : uint8_t {
    NONE   = 0,
    UP     = 1,
    DOWN   = 2,
    LEFT   = 3,
    RIGHT  = 4,
    SELECT = 5,
};

// ---------------------------------------------------------------------------
// Button widget
// ---------------------------------------------------------------------------

struct Button {
    Rect        r;
    const char* label;
    bool        focused;
    bool        pressed;

    void draw() const;
    bool contains(int px, int py) const { return r.contains(px, py); }
};

// ---------------------------------------------------------------------------
// Text label widget
// ---------------------------------------------------------------------------

struct Label {
    Rect  r;
    char  text[32];
    Color fg;
    Color bg;

    void set(const char* s);
    void setf(const char* fmt, ...);
    void draw() const;
};

// ---------------------------------------------------------------------------
// Status bar (top of screen, 320x16)
// ---------------------------------------------------------------------------

struct StatusBar {
    char left[24];    // left field   (e.g. frequency)
    char centre[24];  // centre field (e.g. mode name)
    char right[24];   // right field  (e.g. GPS / battery)

    void draw() const;
    void update(const char* l, const char* c, const char* r);
};

// ---------------------------------------------------------------------------
// Focus manager for a flat list of buttons
// ---------------------------------------------------------------------------

struct ButtonList {
    Button* buttons;   // pointer to array (stack or static)
    int     count;
    int     focused_idx;

    void draw_all() const;
    void navigate(NavKey k);
    // Returns index of the button that was pressed (pressed==true), or -1
    int  selected_idx() const;
    // Clear pressed state on all buttons
    void clear_pressed();
};

} // namespace sentinel::ui
