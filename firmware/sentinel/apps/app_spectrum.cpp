// SPDX-License-Identifier: MIT
// Project Sentinel - Spectrum analyser implementation

#include "app_spectrum.hpp"
#include "app_home_dashboard.hpp"
#include "../ui/canvas.hpp"
#include "../ui/font.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>
#include <cstdio>

namespace sentinel {

AppSpectrum::AppSpectrum()
    : AppBase(AppID::SPECTRUM, TaskID::APP_TASK)
{
    // Control buttons: [< Freq >]  [< Gain >]  [Home]
    // Widths: 53, 53, 54, 53, 53 = 266 ... we spread across 320px
    // Layout: x=0 w=64, x=64 w=64, x=128 w=64, x=192 w=64, x=256 w=64
    static const char* labels[CTRL_BTN_COUNT] = { "< Frq", "Frq >", "< Gn", "Gn >", "Home" };
    for (int i = 0; i < CTRL_BTN_COUNT; ++i) {
        ctrl_btns_[i].r       = { i * 64, CTRL_Y, 64, CTRL_H };
        ctrl_btns_[i].label   = labels[i];
        ctrl_btns_[i].focused = (i == 0);
        ctrl_btns_[i].pressed = false;
    }
    ctrl_list_.focused_idx = 0;

    memset(wf_buf_, 0, sizeof(wf_buf_));
    memset(cur_bins_, 0, sizeof(cur_bins_));

    update_status();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AppSpectrum::on_enter()
{
    // Request radio at current centre freq, 2 MHz BW, RX mode (0)
    request_radio(centre_freq_hz_, 2'000'000u, 0);
    bool granted = wait_radio_grant(2000);

    if (granted) {
        // Program M0 to SPECTRUM mode
        auto& shm = ipc::shared();
        shm.cmd_mode    = ipc::BasebandMode::SPECTRUM;
        shm.cmd_freq_hz = centre_freq_hz_;
        shm.cmd_bw_hz   = 2'000'000u;
        ipc::send_command(ipc::Command::SET_MODE);
    }

    publish_app_state(1, centre_freq_hz_);
    update_status();
}

void AppSpectrum::on_exit()
{
    // Return M0 to IDLE
    ipc::shared().cmd_mode = ipc::BasebandMode::IDLE;
    ipc::send_command(ipc::Command::SET_MODE);

    release_radio();
    publish_app_state(0);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void AppSpectrum::on_paint()
{
    status_bar_.draw();
    draw_waterfall();
    draw_bar_graph();
    draw_controls();
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void AppSpectrum::on_nav(ui::NavKey k)
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
                // Freq down
                if (centre_freq_hz_ > FREQ_STEP) centre_freq_hz_ -= FREQ_STEP;
                send_tune();
            } else if (sel == 1) {
                // Freq up
                centre_freq_hz_ += FREQ_STEP;
                send_tune();
            } else if (sel == 2) {
                // Gain down
                if (lna_db_ >= 8) lna_db_ = static_cast<int8_t>(lna_db_ - 8);
                auto& shm = ipc::shared();
                shm.cmd_lna_db = lna_db_;
                shm.cmd_vga_db = vga_db_;
                ipc::send_command(ipc::Command::SET_GAIN);
            } else if (sel == 3) {
                // Gain up
                if (lna_db_ <= 32) lna_db_ = static_cast<int8_t>(lna_db_ + 8);
                auto& shm = ipc::shared();
                shm.cmd_lna_db = lna_db_;
                shm.cmd_vga_db = vga_db_;
                ipc::send_command(ipc::Command::SET_GAIN);
            } else if (sel == 4) {
                // Home
                on_exit();
                if (g_current_app != nullptr) {
                    // Return to dashboard — the task loop will re-enter dashboard
                    extern AppBase* g_current_app;
                    // g_current_app is set by the home dashboard itself
                    // We signal back by nulling so task_app reverts to dashboard
                    g_current_app = nullptr;
                }
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

void AppSpectrum::on_tick()
{
    process_bus_events();

    auto& shm = ipc::shared();
    if (shm.m0_flags & ipc::M0_FLAG_SPECTRUM) {
        // Clear flag
        shm.m0_flags &= ~ipc::M0_FLAG_SPECTRUM;

        uint16_t seq = shm.spectrum_seq;
        if (seq != last_spectrum_seq_) {
            last_spectrum_seq_ = seq;
            for (int i = 0; i < 256; ++i)
                cur_bins_[i] = shm.spectrum_db[i];
            cur_bins_valid_ = true;
            advance_waterfall();
        }
    }
}

// ---------------------------------------------------------------------------
// Bus events
// ---------------------------------------------------------------------------

void AppSpectrum::on_bus_event(const BusEvent& /*ev*/) {}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void AppSpectrum::advance_waterfall()
{
    // Stretch 256 bins to 320 pixels via nearest-neighbour
    for (int px = 0; px < SCREEN_W; ++px) {
        int b = (px * 256) / SCREEN_W;
        if (b >= 256) b = 255;
        wf_buf_[wf_head_][px] = cur_bins_[b];
    }
    wf_head_ = (wf_head_ + 1) % WF_ROWS;
}

void AppSpectrum::draw_waterfall() const
{
    for (int row = 0; row < WF_ROWS; ++row) {
        int src      = (wf_head_ + row) % WF_ROWS;
        int screen_y = WF_Y + row;
        ui::draw_waterfall_row(screen_y, wf_buf_[src], SCREEN_W, SCREEN_W);
    }
}

void AppSpectrum::draw_bar_graph() const
{
    ui::fill_rect(0, BAR_Y, SCREEN_W, BAR_H, ui::colors::BLACK);

    if (!cur_bins_valid_) return;

    // Draw one vertical bar per pixel column
    for (int px = 0; px < SCREEN_W; ++px) {
        int b    = (px * 256) / SCREEN_W;
        if (b >= 256) b = 255;
        int8_t db = cur_bins_[b];
        // Map dBFS [-128..0] to bar height [0..BAR_H]
        int bar_h = ((static_cast<int>(db) + 128) * BAR_H) / 128;
        if (bar_h < 0) bar_h = 0;
        if (bar_h > BAR_H) bar_h = BAR_H;

        ui::Color c = ui::heat_color(
            static_cast<uint8_t>((static_cast<int>(db) + 128) * 2));
        ui::draw_vline(px, BAR_Y + BAR_H - bar_h, bar_h, c);
    }
}

void AppSpectrum::draw_controls() const
{
    ctrl_list_.draw_all();
}

void AppSpectrum::send_tune()
{
    auto& shm = ipc::shared();
    shm.cmd_freq_hz = centre_freq_hz_;
    ipc::send_command(ipc::Command::TUNE);
    publish_app_state(1, centre_freq_hz_);
}

void AppSpectrum::update_status()
{
    char lbuf[24], rbuf[24];
    snprintf(lbuf, sizeof(lbuf), "%.3f MHz",
             static_cast<double>(centre_freq_hz_) / 1e6);
    snprintf(rbuf, sizeof(rbuf), "%.1f MHz",
             static_cast<double>(span_hz_) / 1e6);
    status_bar_.update(lbuf, "SPECTRUM", rbuf);
}

} // namespace sentinel
