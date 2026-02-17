// SPDX-License-Identifier: MIT
// Project Sentinel - IQ capture implementation

#include "app_capture.hpp"
#include "app_home_dashboard.hpp"
#include "../ui/canvas.hpp"
#include "../ui/font.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>
#include <cstdio>

// FatFS is assumed to be linked in the project
#include "ff.h"

// ---------------------------------------------------------------------------
// IQ buffer access helpers
// We assume task_app feeds an IQ ring buffer starting at
// SHARED_RAM_BASE + sizeof(SharedMemory).  Declare a minimal overlay.
// In a real build these would come from a common header.
// ---------------------------------------------------------------------------
namespace {

struct IQBuf {
    volatile uint32_t wr_pos;  // written by DMA callback (int8 samples)
    volatile uint32_t rd_pos;  // read by M4
    int8_t            buf[sentinel::IQ_BUF_SAMPLES * 2];  // interleaved I, Q
};

// Place IQ buffer immediately after the main SharedMemory struct
inline IQBuf& iq_buf()
{
    static constexpr uint32_t IQ_BASE =
        sentinel::ipc::SHARED_RAM_BASE + 4912u;  // sizeof(SharedMemory) aligned
    return *reinterpret_cast<IQBuf*>(IQ_BASE);
}

} // anonymous namespace

namespace sentinel {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

AppCapture::AppCapture()
    : AppBase(AppID::CAPTURE, TaskID::APP_TASK)
{
    static const char* labels[CTRL_BTN_COUNT] = {
        "REC", "< Frq", "Frq >", "< BW", "BW >", "Home"
    };
    for (int i = 0; i < CTRL_BTN_COUNT; ++i) {
        // 6 buttons across 320px = ~53px each
        ctrl_btns_[i].r       = { i * 53, CTRL_Y, 53, CTRL_H };
        ctrl_btns_[i].label   = labels[i];
        ctrl_btns_[i].focused = (i == 0);
        ctrl_btns_[i].pressed = false;
    }
    ctrl_list_.focused_idx = 0;
    memset(spec_bins_, 0, sizeof(spec_bins_));
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AppCapture::on_enter()
{
    request_radio(freq_hz_, bw_hz_, 0);
    bool granted = wait_radio_grant(2000);
    if (granted) {
        auto& shm = ipc::shared();
        shm.cmd_mode    = ipc::BasebandMode::SPECTRUM;  // use spectrum preview
        shm.cmd_freq_hz = freq_hz_;
        shm.cmd_bw_hz   = bw_hz_;
        ipc::send_command(ipc::Command::SET_MODE);
    }
    publish_app_state(1, freq_hz_);
    update_status();
}

void AppCapture::on_exit()
{
    if (recording_) stop_recording();
    ipc::shared().cmd_mode = ipc::BasebandMode::IDLE;
    ipc::send_command(ipc::Command::SET_MODE);
    release_radio();
    publish_app_state(0);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void AppCapture::on_paint()
{
    status_bar_.draw();
    draw_spectrum();
    draw_capture_status();
    draw_controls();
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void AppCapture::on_nav(ui::NavKey k)
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
                // Toggle REC/STOP
                if (recording_) {
                    stop_recording();
                    ctrl_btns_[0].label = "REC";
                } else {
                    start_recording();
                    ctrl_btns_[0].label = "STOP";
                }
            } else if (sel == 1) {
                if (!recording_ && freq_hz_ > FREQ_STEP) freq_hz_ -= FREQ_STEP;
                auto& shm = ipc::shared();
                shm.cmd_freq_hz = freq_hz_;
                ipc::send_command(ipc::Command::TUNE);
            } else if (sel == 2) {
                if (!recording_) freq_hz_ += FREQ_STEP;
                auto& shm = ipc::shared();
                shm.cmd_freq_hz = freq_hz_;
                ipc::send_command(ipc::Command::TUNE);
            } else if (sel == 3) {
                if (!recording_ && bw_hz_ > BW_STEP) bw_hz_ -= BW_STEP;
            } else if (sel == 4) {
                if (!recording_) bw_hz_ += BW_STEP;
            } else if (sel == 5) {
                if (!recording_) {
                    on_exit();
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

void AppCapture::on_tick()
{
    process_bus_events();

    // Update elapsed time
    if (recording_) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        elapsed_ms_ = now_ms - start_ms_;
        write_iq_chunk();
    }

    // Update spectrum preview
    auto& shm = ipc::shared();
    if (shm.m0_flags & ipc::M0_FLAG_SPECTRUM) {
        shm.m0_flags &= ~ipc::M0_FLAG_SPECTRUM;
        uint16_t seq = shm.spectrum_seq;
        if (seq != last_spec_seq_) {
            last_spec_seq_ = seq;
            for (int i = 0; i < 256; ++i)
                spec_bins_[i] = shm.spectrum_db[i];
        }
    }

    update_status();
}

// ---------------------------------------------------------------------------
// Bus events
// ---------------------------------------------------------------------------

void AppCapture::on_bus_event(const BusEvent& ev)
{
    (void)ev;
}

// ---------------------------------------------------------------------------
// Private: recording control
// ---------------------------------------------------------------------------

void AppCapture::build_filename()
{
    // Filename: capture_433.920_20260217_130000.iq
    // Use GPS lat/lon in name if available; fall back to freq only
    uint32_t freq_mhz  = freq_hz_ / 1'000'000u;
    uint32_t freq_khz  = (freq_hz_ % 1'000'000u) / 1'000u;

    // We don't have RTC access here; use uptime tick as date/time proxy.
    // In a full build, replace with an actual RTC read.
    uint32_t now_s = (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000u;
    uint32_t hh = (now_s / 3600u) % 24u;
    uint32_t mm = (now_s /   60u) % 60u;
    uint32_t ss =  now_s          % 60u;

    snprintf(filename_, sizeof(filename_),
             "capture_%lu.%03lu_%02lu%02lu%02lu.iq",
             static_cast<unsigned long>(freq_mhz),
             static_cast<unsigned long>(freq_khz),
             static_cast<unsigned long>(hh),
             static_cast<unsigned long>(mm),
             static_cast<unsigned long>(ss));
}

void AppCapture::start_recording()
{
    build_filename();

    FRESULT r = f_open(&file_, filename_, FA_CREATE_ALWAYS | FA_WRITE);
    if (r != FR_OK) return;

    file_open_      = true;
    recording_      = true;
    samples_written_ = 0;
    start_ms_       = xTaskGetTickCount() * portTICK_PERIOD_MS;
    elapsed_ms_     = 0;
    last_iq_rd_     = iq_buf().wr_pos;  // start from current head

    // Publish CAPTURE_START event
    BusEvent ev = BusEvent::make(EventType::CAPTURE_START, task_id_);
    auto& p = ev.as<CaptureStartPayload>();
    p.freq_hz        = freq_hz_;
    p.sample_rate_sps = bw_hz_;
    // CRC32 of filename (simple polynomial)
    {
        uint32_t crc = 0xFFFFFFFFu;
        const char* s = filename_;
        while (*s) {
            crc ^= static_cast<uint32_t>(static_cast<uint8_t>(*s++));
            for (int b = 0; b < 8; ++b)
                crc = (crc >> 1) ^ (0xEDB88320u & ~((crc & 1u) - 1u));
        }
        p.filename_hash = crc ^ 0xFFFFFFFFu;
    }
    EventBus::instance().publish(ev);
}

void AppCapture::stop_recording()
{
    recording_ = false;
    if (file_open_) {
        f_close(&file_);
        file_open_ = false;
    }

    // Publish CAPTURE_STOP event
    BusEvent ev = BusEvent::make(EventType::CAPTURE_STOP, task_id_);
    auto& p = ev.as<CaptureStopPayload>();
    p.freq_hz      = freq_hz_;
    p.duration_ms  = elapsed_ms_;
    p.file_size_kb = (samples_written_ * 2u) / 1024u;  // 2 bytes per IQ pair
    EventBus::instance().publish(ev);
}

void AppCapture::write_iq_chunk()
{
    if (!file_open_) return;

    IQBuf& iq = iq_buf();
    uint32_t wr = iq.wr_pos;
    uint32_t rd = last_iq_rd_;

    uint32_t avail_samples = wr - rd;
    if (avail_samples == 0) return;

    // Cap to half the ring buffer to avoid stale wrap-around
    if (avail_samples > IQ_BUF_SAMPLES) avail_samples = IQ_BUF_SAMPLES;

    // Each sample = 2 bytes (I int8, Q int8) interleaved
    // Write in up to 2 contiguous chunks to handle wrap
    uint32_t rd_idx    = (rd * 2u) & ((IQ_BUF_SAMPLES * 2u) - 1u);
    uint32_t bytes_tot = avail_samples * 2u;

    uint32_t to_end = IQ_BUF_SAMPLES * 2u - rd_idx;
    UINT bw = 0;

    if (bytes_tot <= to_end) {
        f_write(&file_, &iq.buf[rd_idx], static_cast<UINT>(bytes_tot), &bw);
    } else {
        f_write(&file_, &iq.buf[rd_idx], static_cast<UINT>(to_end), &bw);
        UINT bw2 = 0;
        f_write(&file_, &iq.buf[0], static_cast<UINT>(bytes_tot - to_end), &bw2);
        bw += bw2;
    }

    samples_written_ += bw / 2u;
    last_iq_rd_ = wr;
}

// ---------------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------------

void AppCapture::draw_spectrum() const
{
    ui::fill_rect(0, SPEC_Y, SCREEN_W, SPEC_H, ui::colors::BLACK);

    // Draw waterfall-style spectrum bar graph (64px tall, 256->320 bins)
    for (int px = 0; px < SCREEN_W; ++px) {
        int b = (px * 256) / SCREEN_W;
        if (b >= 256) b = 255;
        int8_t db = spec_bins_[b];
        int bar_h = ((static_cast<int>(db) + 128) * SPEC_H) / 128;
        if (bar_h < 0) bar_h = 0;
        if (bar_h > SPEC_H) bar_h = SPEC_H;
        if (bar_h > 0) {
            ui::Color c = ui::heat_color(
                static_cast<uint8_t>((static_cast<int>(db) + 128) * 2));
            ui::draw_vline(px, SPEC_Y + SPEC_H - bar_h, bar_h, c);
        }
    }
}

void AppCapture::draw_capture_status() const
{
    ui::fill_rect(0, STATUS_Y, SCREEN_W, STATUS_H, ui::colors::DARK);

    char buf[64];
    int y = STATUS_Y + 4;
    const int line_h = 18;

    // Filename
    snprintf(buf, sizeof(buf), "File: %s", filename_[0] ? filename_ : "(none)");
    ui::draw_text(4, y, buf, ui::colors::WHITE, ui::colors::DARK);
    y += line_h;

    // Elapsed time
    uint32_t secs = elapsed_ms_ / 1000u;
    snprintf(buf, sizeof(buf), "Time: %02lu:%02lu",
             static_cast<unsigned long>(secs / 60u),
             static_cast<unsigned long>(secs % 60u));
    ui::draw_text(4, y, buf, recording_ ? ui::colors::RED : ui::colors::GREY,
                  ui::colors::DARK);
    y += line_h;

    // Samples written
    snprintf(buf, sizeof(buf), "Samples: %lu", static_cast<unsigned long>(samples_written_));
    ui::draw_text(4, y, buf, ui::colors::CYAN, ui::colors::DARK);
    y += line_h;

    // Estimated file size
    uint32_t size_kb = (samples_written_ * 2u) / 1024u;
    snprintf(buf, sizeof(buf), "Size: %lu KB", static_cast<unsigned long>(size_kb));
    ui::draw_text(4, y, buf, ui::colors::YELLOW, ui::colors::DARK);
    y += line_h;

    // Frequency / BW
    snprintf(buf, sizeof(buf), "Freq: %.3f MHz  BW: %.3f MHz",
             static_cast<double>(freq_hz_) / 1e6,
             static_cast<double>(bw_hz_)   / 1e6);
    ui::draw_text(4, y, buf, ui::colors::WHITE, ui::colors::DARK);

    // Recording indicator
    if (recording_) {
        ui::fill_rect(SCREEN_W - 12, STATUS_Y + 4, 8, 8, ui::colors::RED);
        ui::draw_text(SCREEN_W - 40, STATUS_Y + 4, "REC", ui::colors::RED, ui::colors::DARK);
    }
}

void AppCapture::draw_controls()
{
    ctrl_list_.draw_all();
}

void AppCapture::update_status()
{
    char lbuf[24], rbuf[24];
    snprintf(lbuf, sizeof(lbuf), "%.3f MHz",
             static_cast<double>(freq_hz_) / 1e6);
    uint32_t size_kb = (samples_written_ * 2u) / 1024u;
    snprintf(rbuf, sizeof(rbuf), "%lu KB", static_cast<unsigned long>(size_kb));
    status_bar_.update(lbuf, "CAPTURE", rbuf);
}

} // namespace sentinel
