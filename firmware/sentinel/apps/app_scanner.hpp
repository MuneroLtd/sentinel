// SPDX-License-Identifier: MIT
// Project Sentinel - RF scanner application
//
// Layout (320x240):
//   Row   0-15  : StatusBar (current freq | "SCANNER" | detections)
//   Row  16-175 : Spectrum bar chart of last scan sweep (one bar per freq step)
//   Row 176-207 : Last 3 detected signals (freq + RSSI)
//   Row 208-239 : [Start/Stop] [< Step >] [Threshold] [Home]
#pragma once

#include "sentinel_app_base.hpp"
#include "../ui/widgets.hpp"
#include <cstdint>

namespace sentinel {

struct ScanDetection {
    uint32_t freq_hz;
    int16_t  rssi_dbm_x10;
    uint32_t time_ms;
    bool     valid;
};

class AppScanner : public AppBase {
public:
    AppScanner();

    void on_enter() override;
    void on_exit()  override;
    void on_paint() override;
    void on_nav(ui::NavKey k) override;
    void on_tick()  override;

protected:
    void on_bus_event(const BusEvent& ev) override;

private:
    static constexpr int BAR_Y     = 16;
    static constexpr int BAR_H     = 160;
    static constexpr int FEED_Y    = 176;
    static constexpr int FEED_H    = 32;
    static constexpr int CTRL_Y    = 208;
    static constexpr int CTRL_H    = 32;
    static constexpr int SCREEN_W  = 320;
    static constexpr int MAX_STEPS = 320;  // at most 320 bars
    static constexpr int DET_FEED  = 3;    // shown in feed

    // Scan config
    uint32_t start_hz_{430'000'000u};
    uint32_t end_hz_{440'000'000u};
    uint32_t step_hz_{25'000u};
    int16_t  threshold_dbm_x10_{-800};  // -80.0 dBm

    // Runtime state
    bool     scanning_{false};
    uint32_t current_freq_{430'000'000u};
    int      detect_count_{0};

    // Per-step RSSI bar array (one entry per step, mapped to 320 columns)
    int16_t  bar_rssi_[MAX_STEPS]{};   // dBm x10
    int      bar_count_{0};            // number of valid steps in bar_rssi_

    // Detection feed (ring buffer, last 3)
    ScanDetection detections_[DET_FEED]{};
    int           det_head_{0};

    // Status bar
    ui::StatusBar status_bar_{};

    // Control buttons: [Start/Stop] [< Step] [Step >] [Threshold] [Home]
    static constexpr int CTRL_BTN_COUNT = 5;
    ui::Button     ctrl_btns_[CTRL_BTN_COUNT]{};
    ui::ButtonList ctrl_list_{ctrl_btns_, CTRL_BTN_COUNT, 0};

    void load_scan_config();
    void draw_bars() const;
    void draw_detection_feed() const;
    void draw_controls();
    void update_status();
    void push_detection(uint32_t freq, int16_t rssi);
};

} // namespace sentinel
