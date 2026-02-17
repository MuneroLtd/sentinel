// SPDX-License-Identifier: MIT
// Project Sentinel - FM radio implementation

#include "app_fm.hpp"
#include "app_home_dashboard.hpp"
#include "../ui/canvas.hpp"
#include "../ui/font.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>
#include <cstdio>

namespace sentinel {

AppFM::AppFM()
    : AppBase(AppID::FM_RADIO, TaskID::APP_TASK)
{
    static const char* labels[CTRL_BTN_COUNT] = { "< Frq", "Frq >", "Wide/Nrw", "Home" };
    for (int i = 0; i < CTRL_BTN_COUNT; ++i) {
        ctrl_btns_[i].r       = { i * 80, CTRL_Y, 80, CTRL_H };
        ctrl_btns_[i].label   = labels[i];
        ctrl_btns_[i].focused = (i == 0);
        ctrl_btns_[i].pressed = false;
    }
    ctrl_list_.focused_idx = 0;
    memset(wave_buf_, 0, sizeof(wave_buf_));
    update_status();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AppFM::on_enter()
{
    // Request radio at FM frequency, 200 kHz BW (WBFM), mode RX=0
    request_radio(freq_hz_, narrow_ ? 12'500u : 200'000u, 0);
    bool granted = wait_radio_grant(2000);

    if (granted) {
        auto& shm = ipc::shared();
        shm.cmd_mode      = ipc::BasebandMode::FM;
        shm.cmd_freq_hz   = freq_hz_;
        shm.cmd_bw_hz     = narrow_ ? 12'500u : 200'000u;
        shm.cmd_fm_narrow = narrow_ ? 1u : 0u;
        ipc::send_command(ipc::Command::SET_MODE);
    }

    publish_app_state(1, freq_hz_);
    update_status();
}

void AppFM::on_exit()
{
    ipc::shared().cmd_mode = ipc::BasebandMode::IDLE;
    ipc::send_command(ipc::Command::SET_MODE);
    release_radio();
    publish_app_state(0);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void AppFM::on_paint()
{
    status_bar_.draw();
    draw_signal_bar();
    draw_waveform();
    draw_gps();
    draw_controls();
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void AppFM::on_nav(ui::NavKey k)
{
    switch (k) {
        case ui::NavKey::LEFT:
            if (ctrl_list_.focused_idx > 0) --ctrl_list_.focused_idx;
            else ctrl_list_.focused_idx = CTRL_BTN_COUNT - 1;
            for (int i = 0; i < CTRL_BTN_COUNT; ++i)
                ctrl_btns_[i].focused = (i == ctrl_list_.focused_idx);
            break;
        case ui::NavKey::RIGHT:
            if (ctrl_list_.focused_idx < CTRL_BTN_COUNT - 1) ++ctrl_list_.focused_idx;
            else ctrl_list_.focused_idx = 0;
            for (int i = 0; i < CTRL_BTN_COUNT; ++i)
                ctrl_btns_[i].focused = (i == ctrl_list_.focused_idx);
            break;
        case ui::NavKey::SELECT: {
            int sel = ctrl_list_.focused_idx;
            if (sel == 0) {
                if (freq_hz_ > FM_STEP) freq_hz_ -= FM_STEP;
                send_tune();
            } else if (sel == 1) {
                freq_hz_ += FM_STEP;
                send_tune();
            } else if (sel == 2) {
                narrow_ = !narrow_;
                // Update BW immediately
                auto& shm = ipc::shared();
                shm.cmd_bw_hz     = narrow_ ? 12'500u : 200'000u;
                shm.cmd_fm_narrow = narrow_ ? 1u : 0u;
                ipc::send_command(ipc::Command::TUNE);
            } else if (sel == 3) {
                on_exit();
                g_current_app = nullptr;
            }
            update_status();
            break;
        }
        default:
            break;
    }
    draw_controls();
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void AppFM::on_tick()
{
    process_bus_events();

    auto& shm = ipc::shared();

    // Update RSSI
    if (shm.m0_flags & ipc::M0_FLAG_RSSI) {
        shm.m0_flags &= ~ipc::M0_FLAG_RSSI;
        rssi_dbm_x10_ = shm.rssi_dbm_x10;
        update_status();
    }

    // Pull audio samples from the ring buffer
    if (shm.m0_flags & ipc::M0_FLAG_AUDIO) {
        shm.m0_flags &= ~ipc::M0_FLAG_AUDIO;

        uint32_t wr = shm.audio_wr_pos;
        uint32_t rd = shm.audio_rd_pos;

        // Take up to WAVE_SAMPLES from ring buffer, newest first
        uint32_t available = wr - rd;
        if (available > ipc::AUDIO_BUF_SAMPLES)
            available = ipc::AUDIO_BUF_SAMPLES;

        // Copy the last WAVE_SAMPLES into wave_buf_
        uint32_t to_copy = (available > static_cast<uint32_t>(WAVE_SAMPLES))
                             ? static_cast<uint32_t>(WAVE_SAMPLES) : available;
        uint32_t start = wr - to_copy;
        for (uint32_t i = 0; i < to_copy; ++i) {
            uint32_t idx = (start + i) & (ipc::AUDIO_BUF_SAMPLES - 1u);
            wave_buf_[i] = shm.audio_buf[idx];
        }
        // Pad with silence if fewer samples available
        for (uint32_t i = to_copy; i < static_cast<uint32_t>(WAVE_SAMPLES); ++i) {
            wave_buf_[i] = 0;
        }
        shm.audio_rd_pos = wr;
    }
}

// ---------------------------------------------------------------------------
// Bus events
// ---------------------------------------------------------------------------

void AppFM::on_bus_event(const BusEvent& ev)
{
    if (static_cast<EventType>(ev.type) == EventType::GPS_UPDATE) {
        gps_fix_ = (ev.as<GPSUpdatePayload>().fix_quality > 0);
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void AppFM::draw_signal_bar() const
{
    ui::fill_rect(0, SIG_Y, SCREEN_W, SIG_H, ui::colors::DARK);

    // Convert RSSI to percentage for bar: -100 dBm = 0%, -20 dBm = 100%
    int rssi_dbm = rssi_dbm_x10_ / 10;
    int pct = ((rssi_dbm + 100) * 100) / 80;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    ui::draw_progress_bar(4, SIG_Y + 8, SCREEN_W - 8, 20, pct,
                          ui::colors::GREEN, ui::colors::DARK_GREEN);

    char buf[32];
    snprintf(buf, sizeof(buf), "RSSI: %d.%d dBm",
             rssi_dbm_x10_ / 10, (rssi_dbm_x10_ < 0 ? -rssi_dbm_x10_ : rssi_dbm_x10_) % 10);
    ui::draw_text(4, SIG_Y + 36, buf, ui::colors::WHITE, ui::colors::DARK);
}

void AppFM::draw_waveform() const
{
    // Clear waveform area
    ui::fill_rect(0, WAVE_Y, SCREEN_W, WAVE_H, ui::colors::BLACK);

    // Draw horizontal centre line
    int centre_y = WAVE_Y + WAVE_H / 2;
    ui::draw_hline(0, centre_y, SCREEN_W, ui::colors::DARK_GREY);

    // Plot oscilloscope: y = centre_y - (sample / 512)
    // Clamp to WAVE_H bounds
    for (int px = 1; px < WAVE_SAMPLES; ++px) {
        int y0 = centre_y - (static_cast<int>(wave_buf_[px - 1]) / 512);
        int y1 = centre_y - (static_cast<int>(wave_buf_[px])     / 512);

        // Clamp
        if (y0 < WAVE_Y)            y0 = WAVE_Y;
        if (y0 >= WAVE_Y + WAVE_H)  y0 = WAVE_Y + WAVE_H - 1;
        if (y1 < WAVE_Y)            y1 = WAVE_Y;
        if (y1 >= WAVE_Y + WAVE_H)  y1 = WAVE_Y + WAVE_H - 1;

        // Draw vertical line segment between y0 and y1 at column px
        int top    = (y0 < y1) ? y0 : y1;
        int bottom = (y0 < y1) ? y1 : y0;
        ui::draw_vline(px, top, bottom - top + 1, ui::colors::GREEN);
    }
}

void AppFM::draw_gps() const
{
    ui::fill_rect(0, GPS_Y, SCREEN_W, GPS_H, ui::colors::DARK);
    if (gps_fix_) {
        int lat_i = cached_lat_  / 1'000'000;
        int lon_i = cached_lon_  / 1'000'000;
        int lat_f = (cached_lat_  < 0 ? -cached_lat_  : cached_lat_)  % 1'000'000 / 100;
        int lon_f = (cached_lon_  < 0 ? -cached_lon_  : cached_lon_) % 1'000'000 / 100;
        char buf[48];
        snprintf(buf, sizeof(buf), "GPS %d.%04d  %d.%04d", lat_i, lat_f, lon_i, lon_f);
        ui::draw_text(4, GPS_Y + (GPS_H - FONT_H) / 2, buf,
                      ui::colors::CYAN, ui::colors::DARK);
    } else {
        ui::draw_text(4, GPS_Y + (GPS_H - FONT_H) / 2, "GPS: no fix",
                      ui::colors::GREY, ui::colors::DARK);
    }
}

void AppFM::draw_controls()
{
    ctrl_list_.draw_all();
}

void AppFM::update_status()
{
    char lbuf[24], rbuf[24];
    snprintf(lbuf, sizeof(lbuf), "%.3f MHz",
             static_cast<double>(freq_hz_) / 1e6);
    snprintf(rbuf, sizeof(rbuf), "%d.%d dBm",
             rssi_dbm_x10_ / 10,
             (rssi_dbm_x10_ < 0 ? -rssi_dbm_x10_ : rssi_dbm_x10_) % 10);
    status_bar_.update(lbuf, "FM RADIO", rbuf);
}

void AppFM::send_tune()
{
    auto& shm = ipc::shared();
    shm.cmd_freq_hz = freq_hz_;
    ipc::send_command(ipc::Command::TUNE);
    publish_app_state(1, freq_hz_);
}

} // namespace sentinel
