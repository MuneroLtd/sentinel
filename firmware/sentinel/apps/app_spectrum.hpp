// SPDX-License-Identifier: MIT
// Project Sentinel - Spectrum analyser application
//
// Layout (320x240):
//   Row   0-15  : StatusBar (freq | "SPECTRUM" | span)
//   Row  16-175 : Waterfall (320x160 scrolling, 256 bins -> 320 pixels)
//   Row 176-207 : Instantaneous bar graph (32px tall)
//   Row 208-239 : Controls [< Freq >] [< Gain >] [Home]
#pragma once

#include "sentinel_app_base.hpp"
#include "../ui/widgets.hpp"
#include <cstdint>

namespace sentinel {

class AppSpectrum : public AppBase {
public:
    AppSpectrum();

    void on_enter() override;
    void on_exit()  override;
    void on_paint() override;
    void on_nav(ui::NavKey k) override;
    void on_tick()  override;

protected:
    void on_bus_event(const BusEvent& ev) override;

private:
    static constexpr int WF_Y      = 16;
    static constexpr int WF_H      = 160;
    static constexpr int BAR_Y     = 176;
    static constexpr int BAR_H     = 32;
    static constexpr int CTRL_Y    = 208;
    static constexpr int CTRL_H    = 32;
    static constexpr int SCREEN_W  = 320;
    static constexpr int WF_ROWS   = WF_H;  // one row per pixel line

    // Waterfall ring buffer
    int8_t   wf_buf_[WF_ROWS][SCREEN_W]{};
    int      wf_head_{0};
    uint16_t last_spectrum_seq_{0};

    // Current spectrum snapshot for the bar graph
    int8_t   cur_bins_[256]{};
    bool     cur_bins_valid_{false};

    // Frequency / gain controls
    uint32_t centre_freq_hz_{433'920'000u};
    uint32_t span_hz_{2'000'000u};   // display span
    int8_t   lna_db_{16};
    int8_t   vga_db_{20};
    static constexpr uint32_t FREQ_STEP = 100'000u;  // 100 kHz step

    // Focused control column: 0=freq-, 1=freq+, 2=gain-, 3=gain+, 4=home
    int ctrl_focus_{0};

    // Status bar
    ui::StatusBar status_bar_{};

    // Control buttons
    static constexpr int CTRL_BTN_COUNT = 5;
    ui::Button     ctrl_btns_[CTRL_BTN_COUNT]{};
    ui::ButtonList ctrl_list_{ctrl_btns_, CTRL_BTN_COUNT, 0};

    void draw_waterfall() const;
    void draw_bar_graph() const;
    void draw_controls() const;
    void advance_waterfall();
    void send_tune();
    void update_status();
};

} // namespace sentinel
