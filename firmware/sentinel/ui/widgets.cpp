// SPDX-License-Identifier: MIT
// Project Sentinel - Widget drawing implementations

#include "widgets.hpp"
#include "canvas.hpp"
#include "font.hpp"
#include <cstring>
#include <cstdio>
#include <cstdarg>

namespace sentinel::ui {

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------

void Button::draw() const
{
    Color bg = focused ? colors::BUTTON_FOC : colors::BUTTON_BG;
    Color fg_col = colors::BUTTON_FG;

    fill_rect(r.x, r.y, r.w, r.h, bg);
    draw_rect(r.x, r.y, r.w, r.h, focused ? colors::WHITE : colors::GREY);

    // Centre the label text inside the button
    if (label) {
        int text_len = 0;
        const char* p = label;
        while (*p++) ++text_len;
        int text_px = text_len * FONT_W;
        int tx = r.x + (r.w - text_px) / 2;
        int ty = r.y + (r.h - FONT_H) / 2;
        if (tx < r.x) tx = r.x;
        if (ty < r.y) ty = r.y;
        draw_text(tx, ty, label, fg_col, bg);
    }
}

// ---------------------------------------------------------------------------
// Label
// ---------------------------------------------------------------------------

void Label::set(const char* s)
{
    if (!s) { text[0] = '\0'; return; }
    strncpy(text, s, sizeof(text) - 1);
    text[sizeof(text) - 1] = '\0';
}

void Label::setf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
}

void Label::draw() const
{
    fill_rect(r.x, r.y, r.w, r.h, bg);
    draw_text(r.x, r.y + (r.h - FONT_H) / 2, text, fg, bg);
}

// ---------------------------------------------------------------------------
// StatusBar
// ---------------------------------------------------------------------------

static constexpr int STATUS_H   = 16;
static constexpr int SCREEN_W   = 320;

void StatusBar::draw() const
{
    // Background
    fill_rect(0, 0, SCREEN_W, STATUS_H, colors::STATUS_BG);

    // Left field: left-aligned at x=2
    draw_text(2, (STATUS_H - FONT_H) / 2, left, colors::STATUS_FG, colors::STATUS_BG);

    // Centre field: centred
    int clen = 0;
    const char* p = centre;
    while (*p++) ++clen;
    int cx = (SCREEN_W - clen * FONT_W) / 2;
    draw_text(cx, (STATUS_H - FONT_H) / 2, centre, colors::STATUS_FG, colors::STATUS_BG);

    // Right field: right-aligned
    int rlen = 0;
    p = right;
    while (*p++) ++rlen;
    int rx = SCREEN_W - rlen * FONT_W - 2;
    if (rx < 0) rx = 0;
    draw_text(rx, (STATUS_H - FONT_H) / 2, right, colors::STATUS_FG, colors::STATUS_BG);

    // Bottom separator line
    draw_hline(0, STATUS_H - 1, SCREEN_W, colors::GREY);
}

void StatusBar::update(const char* l, const char* c, const char* r)
{
    if (l) { strncpy(left,   l, sizeof(left)   - 1); left[sizeof(left)-1]     = '\0'; }
    if (c) { strncpy(centre, c, sizeof(centre) - 1); centre[sizeof(centre)-1] = '\0'; }
    if (r) { strncpy(right,  r, sizeof(right)  - 1); right[sizeof(right)-1]   = '\0'; }
}

// ---------------------------------------------------------------------------
// ButtonList
// ---------------------------------------------------------------------------

void ButtonList::draw_all() const
{
    for (int i = 0; i < count; ++i) {
        buttons[i].draw();
    }
}

void ButtonList::navigate(NavKey k)
{
    if (count <= 0) return;

    switch (k) {
        case NavKey::LEFT:
            focused_idx = (focused_idx > 0) ? focused_idx - 1 : count - 1;
            break;
        case NavKey::RIGHT:
            focused_idx = (focused_idx < count - 1) ? focused_idx + 1 : 0;
            break;
        case NavKey::UP:
            focused_idx = (focused_idx > 0) ? focused_idx - 1 : count - 1;
            break;
        case NavKey::DOWN:
            focused_idx = (focused_idx < count - 1) ? focused_idx + 1 : 0;
            break;
        case NavKey::SELECT:
            if (focused_idx >= 0 && focused_idx < count) {
                buttons[focused_idx].pressed = true;
            }
            break;
        default:
            break;
    }

    // Update focus flags
    for (int i = 0; i < count; ++i) {
        buttons[i].focused = (i == focused_idx);
    }
}

int ButtonList::selected_idx() const
{
    for (int i = 0; i < count; ++i) {
        if (buttons[i].pressed) return i;
    }
    return -1;
}

void ButtonList::clear_pressed()
{
    for (int i = 0; i < count; ++i) {
        buttons[i].pressed = false;
    }
}

} // namespace sentinel::ui
