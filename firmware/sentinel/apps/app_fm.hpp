// SPDX-License-Identifier: MIT
// Project Sentinel - FM radio application
//
// Layout (320x240):
//   Row   0-15  : StatusBar (freq MHz | "FM RADIO" | RSSI dBm)
//   Row  16-79  : Signal strength bar + RSSI numeric
//   Row  80-175 : Scrolling audio waveform (oscilloscope, 320x96)
//   Row 176-207 : GPS coordinate overlay
//   Row 208-239 : [< Freq >] [Wide/Narrow] [Home]
#pragma once

#include "sentinel_app_base.hpp"
#include "../ui/widgets.hpp"
#include <cstdint>
#include <cstddef>

namespace sentinel {

class AppFM : public AppBase {
public:
    AppFM();

    void on_enter() override;
    void on_exit()  override;
    void on_paint() override;
    void on_nav(ui::NavKey k) override;
    void on_tick()  override;

protected:
    void on_bus_event(const BusEvent& ev) override;

private:
    static constexpr int SIG_Y     = 16;
    static constexpr int SIG_H     = 64;
    static constexpr int WAVE_Y    = 80;
    static constexpr int WAVE_H    = 96;
    static constexpr int GPS_Y     = 176;
    static constexpr int GPS_H     = 32;
    static constexpr int CTRL_Y    = 208;
    static constexpr int CTRL_H    = 32;
    static constexpr int SCREEN_W  = 320;

    // Audio waveform: last 320 samples
    static constexpr int WAVE_SAMPLES = SCREEN_W;
    int16_t  wave_buf_[WAVE_SAMPLES]{};
    uint32_t last_audio_rd_{0};

    // Current RSSI
    int16_t  rssi_dbm_x10_{0};

    // Frequency control
    uint32_t freq_hz_{88'000'000u};
    bool     narrow_{false};
    static constexpr uint32_t FM_STEP = 100'000u;  // 100 kHz

    // GPS fix display
    bool     gps_fix_{false};

    // Status bar
    ui::StatusBar status_bar_{};

    // Control buttons: [< Frq] [Frq >] [Wide/Narrow] [Home]
    static constexpr int CTRL_BTN_COUNT = 4;
    ui::Button     ctrl_btns_[CTRL_BTN_COUNT]{};
    ui::ButtonList ctrl_list_{ctrl_btns_, CTRL_BTN_COUNT, 0};

    void draw_signal_bar() const;
    void draw_waveform() const;
    void draw_gps() const;
    void draw_controls();
    void update_status();
    void send_tune();
};

} // namespace sentinel
