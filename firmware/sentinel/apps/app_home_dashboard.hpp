// SPDX-License-Identifier: MIT
// Project Sentinel - Home dashboard application
//
// Layout (320x240):
//   Row   0-15  : StatusBar  (freq | mode | GPS+bat)
//   Row  16-111 : Mini waterfall  (320x96, scrolling spectrum)
//   Row 112-207 : Activity feed   (5 lines x ~19px)
//   Row 208-239 : 5 quick-launch buttons (Spec/FM/ADS-B/Scan/Cap)
#pragma once

#include "sentinel_app_base.hpp"
#include "../ui/widgets.hpp"
#include <cstdint>

namespace sentinel {

// Forward declarations for app singletons (defined in respective cpp files)
class AppSpectrum;
class AppFM;
class AppAdsb;
class AppScanner;
class AppCapture;

// Global current-app pointer used for navigation stack
extern AppBase* g_current_app;

class AppHomeDashboard : public AppBase {
public:
    AppHomeDashboard();

    void on_enter() override;
    void on_exit()  override;
    void on_paint() override;
    void on_nav(ui::NavKey k) override;
    void on_tick()  override;

protected:
    void on_bus_event(const BusEvent& ev) override;

private:
    // --- Zones ---
    static constexpr int WF_Y      = 16;   // waterfall start row
    static constexpr int WF_H      = 96;   // waterfall height
    static constexpr int FEED_Y    = 112;  // activity feed start row
    static constexpr int FEED_H    = 96;   // activity feed height (5 lines)
    static constexpr int BTN_Y     = 208;  // buttons start row
    static constexpr int BTN_H     = 32;   // button height
    static constexpr int SCREEN_W  = 320;

    // --- Status bar ---
    ui::StatusBar status_bar_{};

    // --- Waterfall ring buffer ---
    // 96 rows deep, 320 bins wide (nearest-neighbour from 256 bins)
    // Ring buffer depth: limited to save SRAM0 (LPC4320 only has 96 KB).
    static constexpr int WF_ROWS  = 24;
    static constexpr int WF_COLS  = 320;
    int8_t  wf_buf_[WF_ROWS][WF_COLS]{};
    int     wf_head_{0};        // next row to overwrite
    uint16_t last_spectrum_seq_{0};

    // --- Activity feed ---
    static constexpr int FEED_LINES     = 5;
    static constexpr int FEED_LINE_H    = 19;
    char    feed_buf_[FEED_LINES][48]{};
    int     feed_head_{0};
    int     feed_count_{0};

    // --- Quick-launch buttons ---
    static constexpr int BTN_COUNT = 5;
    ui::Button   btns_[BTN_COUNT]{};
    ui::ButtonList btn_list_{btns_, BTN_COUNT, 0};

    // --- Internal helpers ---
    void draw_waterfall() const;
    void draw_feed() const;
    void draw_buttons() const;
    void push_feed(const char* msg);
    void advance_waterfall(const int8_t* bins, int n_bins);
};

} // namespace sentinel
