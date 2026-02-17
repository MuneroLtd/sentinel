// SPDX-License-Identifier: MIT
// Project Sentinel - RF scanner implementation

#include "app_scanner.hpp"
#include "app_home_dashboard.hpp"
#include "../ui/canvas.hpp"
#include "../ui/font.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>
#include <cstdio>

// FatFS for config file reading
#include "ff.h"

namespace sentinel {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

AppScanner::AppScanner()
    : AppBase(AppID::SCANNER, TaskID::APP_TASK)
{
    static const char* labels[CTRL_BTN_COUNT] = {
        "Start", "< Step", "Step >", "Thresh", "Home"
    };
    for (int i = 0; i < CTRL_BTN_COUNT; ++i) {
        ctrl_btns_[i].r       = { i * 64, CTRL_Y, 64, CTRL_H };
        ctrl_btns_[i].label   = labels[i];
        ctrl_btns_[i].focused = (i == 0);
        ctrl_btns_[i].pressed = false;
    }
    ctrl_list_.focused_idx = 0;

    memset(bar_rssi_, 0, sizeof(bar_rssi_));
    memset(detections_, 0, sizeof(detections_));
}

// ---------------------------------------------------------------------------
// Config loader (parse /SENTINEL/config/scan_config.json with sscanf/strstr)
// ---------------------------------------------------------------------------

void AppScanner::load_scan_config()
{
    FIL   f;
    FRESULT r = f_open(&f, "/SENTINEL/config/scan_config.json", FA_READ);
    if (r != FR_OK) return;  // use defaults

    char buf[256]{};
    UINT br = 0;
    f_read(&f, buf, sizeof(buf) - 1, &br);
    f_close(&f);
    buf[br] = '\0';

    // Parse fields using strstr + sscanf (no JSON library)
    char* p;
    unsigned long v;

    p = strstr(buf, "\"start_hz\"");
    if (p) {
        p = strstr(p, ":");
        if (p && sscanf(p + 1, " %lu", &v) == 1) start_hz_ = static_cast<uint32_t>(v);
    }
    p = strstr(buf, "\"end_hz\"");
    if (p) {
        p = strstr(p, ":");
        if (p && sscanf(p + 1, " %lu", &v) == 1) end_hz_ = static_cast<uint32_t>(v);
    }
    p = strstr(buf, "\"step_hz\"");
    if (p) {
        p = strstr(p, ":");
        if (p && sscanf(p + 1, " %lu", &v) == 1) step_hz_ = static_cast<uint32_t>(v);
    }
    p = strstr(buf, "\"threshold_dbm\"");
    if (p) {
        p = strstr(p, ":");
        if (p) {
            int iv;
            if (sscanf(p + 1, " %d", &iv) == 1)
                threshold_dbm_x10_ = static_cast<int16_t>(iv * 10);
        }
    }

    // Compute number of steps and clamp
    if (step_hz_ == 0) step_hz_ = 25'000u;
    uint32_t steps = (end_hz_ - start_hz_) / step_hz_;
    if (steps > MAX_STEPS) steps = MAX_STEPS;
    bar_count_ = static_cast<int>(steps);
    current_freq_ = start_hz_;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AppScanner::on_enter()
{
    load_scan_config();

    if (bar_count_ == 0) {
        bar_count_ = static_cast<int>((end_hz_ - start_hz_) / step_hz_);
        if (bar_count_ > MAX_STEPS) bar_count_ = MAX_STEPS;
    }
    current_freq_ = start_hz_;

    request_radio(start_hz_, step_hz_, 0);
    bool granted = wait_radio_grant(2000);
    if (granted) {
        auto& shm = ipc::shared();
        shm.cmd_mode    = ipc::BasebandMode::SCANNER;
        shm.cmd_freq_hz = start_hz_;
        shm.cmd_bw_hz   = step_hz_;
        ipc::send_command(ipc::Command::SET_MODE);
    }
    scanning_ = granted;
    ctrl_btns_[0].label = scanning_ ? "Stop" : "Start";

    publish_app_state(1, start_hz_);
    update_status();
}

void AppScanner::on_exit()
{
    scanning_ = false;
    ipc::shared().cmd_mode = ipc::BasebandMode::IDLE;
    ipc::send_command(ipc::Command::SET_MODE);
    release_radio();
    publish_app_state(0);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void AppScanner::on_paint()
{
    status_bar_.draw();
    draw_bars();
    draw_detection_feed();
    draw_controls();
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void AppScanner::on_nav(ui::NavKey k)
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
                // Toggle Start/Stop
                if (scanning_) {
                    scanning_ = false;
                    ipc::shared().cmd_mode = ipc::BasebandMode::IDLE;
                    ipc::send_command(ipc::Command::SET_MODE);
                    ctrl_btns_[0].label = "Start";
                } else {
                    auto& shm = ipc::shared();
                    shm.cmd_mode    = ipc::BasebandMode::SCANNER;
                    shm.cmd_freq_hz = start_hz_;
                    shm.cmd_bw_hz   = step_hz_;
                    ipc::send_command(ipc::Command::SET_MODE);
                    scanning_ = true;
                    ctrl_btns_[0].label = "Stop";
                }
            } else if (sel == 1) {
                // Step down
                if (step_hz_ > 5'000u) step_hz_ -= 5'000u;
                bar_count_ = static_cast<int>((end_hz_ - start_hz_) / step_hz_);
                if (bar_count_ > MAX_STEPS) bar_count_ = MAX_STEPS;
            } else if (sel == 2) {
                // Step up
                step_hz_ += 5'000u;
                bar_count_ = static_cast<int>((end_hz_ - start_hz_) / step_hz_);
                if (bar_count_ > MAX_STEPS) bar_count_ = MAX_STEPS;
            } else if (sel == 3) {
                // Toggle threshold +5 dBm
                threshold_dbm_x10_ = static_cast<int16_t>(threshold_dbm_x10_ + 50);
                if (threshold_dbm_x10_ > -100) threshold_dbm_x10_ = -1200; // wrap to -120 dBm
            } else if (sel == 4) {
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

void AppScanner::on_tick()
{
    process_bus_events();

    if (!scanning_) return;

    auto& shm = ipc::shared();

    // Drain signal ring buffer
    if (shm.m0_flags & ipc::M0_FLAG_SIGNAL) {
        shm.m0_flags &= ~ipc::M0_FLAG_SIGNAL;

        while (shm.signal_rd != shm.signal_wr) {
            uint8_t idx = shm.signal_rd;
            ipc::SignalEvent se = shm.signals[idx];  // copy off volatile
            shm.signal_rd = static_cast<uint8_t>((idx + 1u) & (ipc::SIGNAL_RING_DEPTH - 1u));

            // Map freq to bar index
            if (se.freq_hz >= start_hz_ && se.freq_hz < end_hz_) {
                int bar = static_cast<int>((se.freq_hz - start_hz_) / step_hz_);
                if (bar >= 0 && bar < bar_count_) {
                    bar_rssi_[bar] = se.rssi_dbm_x10;
                }
            }

            // Check threshold
            if (se.rssi_dbm_x10 >= threshold_dbm_x10_) {
                push_detection(se.freq_hz, se.rssi_dbm_x10);
                ++detect_count_;

                // Publish SIGNAL_DETECTED bus event
                BusEvent ev = BusEvent::make(EventType::SIGNAL_DETECTED, task_id_);
                auto& p = ev.as<SignalDetectedPayload>();
                p.frequency_hz  = se.freq_hz;
                p.rssi_dbm      = se.rssi_dbm_x10;
                p.duration_ms   = se.duration_ms;
                p.gps_lat       = cached_lat_;
                p.gps_lon       = cached_lon_;
                EventBus::instance().publish(ev);
            }

            current_freq_ = se.freq_hz;
        }
    }

    update_status();
}

// ---------------------------------------------------------------------------
// Bus events
// ---------------------------------------------------------------------------

void AppScanner::on_bus_event(const BusEvent& /*ev*/) {}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void AppScanner::push_detection(uint32_t freq, int16_t rssi)
{
    detections_[det_head_].freq_hz     = freq;
    detections_[det_head_].rssi_dbm_x10 = rssi;
    detections_[det_head_].time_ms     = xTaskGetTickCount() * portTICK_PERIOD_MS;
    detections_[det_head_].valid       = true;
    det_head_ = (det_head_ + 1) % DET_FEED;
}

void AppScanner::draw_bars() const
{
    ui::fill_rect(0, BAR_Y, SCREEN_W, BAR_H, ui::colors::BLACK);
    if (bar_count_ <= 0) return;

    // Map bar_count_ bars to SCREEN_W pixels
    for (int px = 0; px < SCREEN_W; ++px) {
        int bar = (px * bar_count_) / SCREEN_W;
        if (bar >= bar_count_) bar = bar_count_ - 1;

        int16_t rssi = bar_rssi_[bar];
        // Map dBm x10: -1200..-100 -> 0..BAR_H
        int pct_h = static_cast<int>((static_cast<int>(rssi) + 1200) * BAR_H) / 1100;
        if (pct_h < 0)    pct_h = 0;
        if (pct_h > BAR_H) pct_h = BAR_H;

        // Colour: above threshold = RED, below = GREEN
        ui::Color c = (rssi >= threshold_dbm_x10_) ? ui::colors::RED : ui::colors::DARK_GREEN;

        if (pct_h > 0) {
            ui::draw_vline(px, BAR_Y + BAR_H - pct_h, pct_h, c);
        }
    }

    // Threshold line
    int tline_h = static_cast<int>((static_cast<int>(threshold_dbm_x10_) + 1200) * BAR_H) / 1100;
    if (tline_h >= 0 && tline_h <= BAR_H) {
        ui::draw_hline(0, BAR_Y + BAR_H - tline_h, SCREEN_W, ui::colors::YELLOW);
    }
}

void AppScanner::draw_detection_feed() const
{
    ui::fill_rect(0, FEED_Y, SCREEN_W, FEED_H, ui::colors::DARK);
    int line_h = FEED_H / DET_FEED;

    for (int i = 0; i < DET_FEED; ++i) {
        int idx = ((det_head_ - 1 - i) + DET_FEED) % DET_FEED;
        int y   = FEED_Y + i * line_h;
        if (detections_[idx].valid) {
            char buf[48];
            snprintf(buf, sizeof(buf), "%.3f MHz  %d.%d dBm",
                     static_cast<double>(detections_[idx].freq_hz) / 1e6,
                     detections_[idx].rssi_dbm_x10 / 10,
                     (detections_[idx].rssi_dbm_x10 < 0
                          ? -detections_[idx].rssi_dbm_x10
                          : detections_[idx].rssi_dbm_x10) % 10);
            ui::draw_text(2, y + (line_h - FONT_H) / 2, buf,
                          ui::colors::WHITE, ui::colors::DARK);
        }
    }
}

void AppScanner::draw_controls()
{
    ctrl_list_.draw_all();
}

void AppScanner::update_status()
{
    char lbuf[24], rbuf[24];
    snprintf(lbuf, sizeof(lbuf), "%.3f MHz",
             static_cast<double>(current_freq_) / 1e6);
    snprintf(rbuf, sizeof(rbuf), "%d det", detect_count_);
    status_bar_.update(lbuf, "SCANNER", rbuf);
}

} // namespace sentinel
