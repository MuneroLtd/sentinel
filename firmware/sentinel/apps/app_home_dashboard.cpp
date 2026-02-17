// SPDX-License-Identifier: MIT
// Project Sentinel - Home dashboard implementation

#include "app_home_dashboard.hpp"
#include "app_spectrum.hpp"
#include "app_fm.hpp"
#include "app_adsb.hpp"
#include "app_scanner.hpp"
#include "app_capture.hpp"
#include "../ui/canvas.hpp"
#include "../ui/font.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>
#include <cstdio>

namespace sentinel {

// Global current-app pointer (navigation stack — single depth)
AppBase* g_current_app = nullptr;

// Singleton app instances (statically allocated)
static AppSpectrum  s_app_spectrum;
static AppFM        s_app_fm;
static AppAdsb      s_app_adsb;
static AppScanner   s_app_scanner;
static AppCapture   s_app_capture;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

AppHomeDashboard::AppHomeDashboard()
    : AppBase(AppID::DASHBOARD, TaskID::APP_TASK)
{
    // Initialise button geometry: 5 x 64px wide, 32px tall, row 208
    static const char* labels[BTN_COUNT] = { "Spec", "FM", "ADS-B", "Scan", "Cap" };
    for (int i = 0; i < BTN_COUNT; ++i) {
        btns_[i].r       = { i * 64, BTN_Y, 64, BTN_H };
        btns_[i].label   = labels[i];
        btns_[i].focused = (i == 0);
        btns_[i].pressed = false;
    }
    btn_list_.focused_idx = 0;

    // Initialise status bar
    status_bar_.update("---", "SENTINEL", "---");

    // Clear waterfall and feed
    memset(wf_buf_, 0, sizeof(wf_buf_));
    memset(feed_buf_, 0, sizeof(feed_buf_));
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AppHomeDashboard::on_enter()
{
    publish_app_state(1 /*running*/);
    push_feed("Dashboard started");
}

void AppHomeDashboard::on_exit()
{
    publish_app_state(0 /*stopped*/);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void AppHomeDashboard::on_paint()
{
    status_bar_.draw();
    draw_waterfall();
    draw_feed();
    draw_buttons();
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void AppHomeDashboard::on_nav(ui::NavKey k)
{
    btn_list_.navigate(k);

    int sel = btn_list_.selected_idx();
    if (sel >= 0) {
        btn_list_.clear_pressed();
        on_exit();

        AppBase* next = nullptr;
        switch (sel) {
            case 0: next = &s_app_spectrum; break;
            case 1: next = &s_app_fm;       break;
            case 2: next = &s_app_adsb;     break;
            case 3: next = &s_app_scanner;  break;
            case 4: next = &s_app_capture;  break;
            default: break;
        }
        if (next) {
            g_current_app = next;
            next->on_enter();
        }
    }

    // Redraw buttons to reflect new focus
    draw_buttons();
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void AppHomeDashboard::on_tick()
{
    process_bus_events();

    // Pull latest spectrum from IPC shared memory if a new frame is ready
    auto& shm = ipc::shared();
    uint16_t seq = shm.spectrum_seq;
    if (seq != last_spectrum_seq_) {
        last_spectrum_seq_ = seq;
        // Copy spectrum bins (256) and stretch to 320 pixels
        int8_t local_bins[ipc::SPECTRUM_BINS];
        for (uint32_t i = 0; i < ipc::SPECTRUM_BINS; ++i) {
            local_bins[i] = shm.spectrum_db[i];
        }
        advance_waterfall(local_bins, static_cast<int>(ipc::SPECTRUM_BINS));
    }
}

// ---------------------------------------------------------------------------
// Bus event handler
// ---------------------------------------------------------------------------

void AppHomeDashboard::on_bus_event(const BusEvent& ev)
{
    EventType et = static_cast<EventType>(ev.type);

    switch (et) {
        case EventType::GPS_UPDATE: {
            const auto& gps = ev.as<GPSUpdatePayload>();
            char buf[24];
            // Format: "GPS 12.3456,98.7654" abbreviated
            int lat_i = gps.latitude  / 1000000;
            int lon_i = gps.longitude / 1000000;
            int lat_f = (gps.latitude  < 0 ? -gps.latitude  : gps.latitude)  % 1000000 / 1000;
            int lon_f = (gps.longitude < 0 ? -gps.longitude : gps.longitude) % 1000000 / 1000;
            snprintf(buf, sizeof(buf), "GPS %d.%03d,%d.%03d", lat_i, lat_f, lon_i, lon_f);
            status_bar_.update(nullptr, nullptr, buf);
            break;
        }
        case EventType::SYSTEM_STATUS: {
            const auto& sys = ev.as<SystemStatusPayload>();
            char rbuf[24];
            snprintf(rbuf, sizeof(rbuf), "BAT:%u%%", sys.battery_pct);
            status_bar_.update(nullptr, nullptr, rbuf);
            char fbuf[48];
            snprintf(fbuf, sizeof(fbuf), "SYS cpu:%u%% heap:%u%%",
                     sys.cpu_load_pct, sys.heap_free_pct);
            push_feed(fbuf);
            break;
        }
        case EventType::SIGNAL_DETECTED: {
            const auto& sig = ev.as<SignalDetectedPayload>();
            char fbuf[48];
            snprintf(fbuf, sizeof(fbuf), "SIG %.3f MHz %d dBm",
                     static_cast<double>(sig.frequency_hz) / 1e6,
                     sig.rssi_dbm / 10);
            push_feed(fbuf);
            break;
        }
        case EventType::AIRCRAFT_POSITION: {
            const auto& ac = ev.as<AircraftPositionPayload>();
            char fbuf[48];
            snprintf(fbuf, sizeof(fbuf), "ADSB %02X%02X%02X alt:%u",
                     ac.icao[0], ac.icao[1], ac.icao[2], ac.altitude_ft);
            push_feed(fbuf);
            break;
        }
        case EventType::CAPTURE_START: {
            push_feed("Capture started");
            break;
        }
        case EventType::CAPTURE_STOP: {
            push_feed("Capture stopped");
            break;
        }
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void AppHomeDashboard::push_feed(const char* msg)
{
    strncpy(feed_buf_[feed_head_], msg, sizeof(feed_buf_[0]) - 1);
    feed_buf_[feed_head_][sizeof(feed_buf_[0]) - 1] = '\0';
    feed_head_ = (feed_head_ + 1) % FEED_LINES;
    if (feed_count_ < FEED_LINES) ++feed_count_;
}

void AppHomeDashboard::advance_waterfall(const int8_t* bins, int n_bins)
{
    // Stretch n_bins to WF_COLS via nearest-neighbour
    for (int px = 0; px < WF_COLS; ++px) {
        int b = (px * n_bins) / WF_COLS;
        if (b >= n_bins) b = n_bins - 1;
        wf_buf_[wf_head_][px] = bins[b];
    }
    wf_head_ = (wf_head_ + 1) % WF_ROWS;
}

void AppHomeDashboard::draw_waterfall() const
{
    // Draw WF_ROWS rows, starting from wf_head_ (oldest) to wf_head_-1 (newest)
    for (int row = 0; row < WF_ROWS; ++row) {
        int src = (wf_head_ + row) % WF_ROWS;
        int screen_y = WF_Y + row;
        ui::draw_waterfall_row(screen_y, wf_buf_[src], WF_COLS, SCREEN_W);
    }
}

void AppHomeDashboard::draw_feed() const
{
    // Clear feed area
    ui::fill_rect(0, FEED_Y, SCREEN_W, FEED_H, ui::colors::DARK);
    ui::draw_text(2, FEED_Y, "Activity:", ui::colors::GREY, ui::colors::DARK);

    // Print lines in chronological order (oldest at top)
    for (int i = 0; i < FEED_LINES; ++i) {
        // idx walks backwards from (feed_head_-1) to oldest
        int idx = ((feed_head_ - 1 - i) + FEED_LINES) % FEED_LINES;
        int y   = FEED_Y + 10 + i * FEED_LINE_H;
        // Clear line
        ui::fill_rect(0, y, SCREEN_W, FEED_LINE_H, ui::colors::DARK);
        if (feed_buf_[idx][0] != '\0') {
            ui::draw_text(2, y, feed_buf_[idx], ui::colors::WHITE, ui::colors::DARK);
        }
    }
}

void AppHomeDashboard::draw_buttons() const
{
    btn_list_.draw_all();
}

} // namespace sentinel
