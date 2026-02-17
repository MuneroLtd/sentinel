// SPDX-License-Identifier: MIT
// Project Sentinel - IQ capture application
//
// Layout (320x240):
//   Row   0-15  : StatusBar (freq | "CAPTURE" | file size KB)
//   Row  16-79  : Spectrum preview (64px tall)
//   Row  80-175 : Capture status (filename, elapsed, samples, est. size)
//   Row 208-239 : [REC/STOP] [< Freq >] [< BW >] [Home]
#pragma once

#include "sentinel_app_base.hpp"
#include "../ui/widgets.hpp"
#include <cstdint>

// FatFS
#include "ff.h"

namespace sentinel {

// IQ capture uses a DMA-fed ring buffer in shared memory.
// We add an IQ_BUF layout overlay here since the actual field is task-fed.
// Assumed fields in shared memory (task_app feeds this from DMA callback):
//   volatile uint32_t iq_wr_pos;  // samples written (each sample = 2 bytes: I,Q int8)
//   int8_t            iq_buf[IQ_BUF_SIZE];  // ring buffer
static constexpr uint32_t IQ_BUF_SAMPLES = 4096u;  // int8 I/Q pairs

class AppCapture : public AppBase {
public:
    AppCapture();

    void on_enter() override;
    void on_exit()  override;
    void on_paint() override;
    void on_nav(ui::NavKey k) override;
    void on_tick()  override;

protected:
    void on_bus_event(const BusEvent& ev) override;

private:
    static constexpr int SPEC_Y    = 16;
    static constexpr int SPEC_H    = 64;
    static constexpr int STATUS_Y  = 80;
    static constexpr int STATUS_H  = 128;
    static constexpr int CTRL_Y    = 208;
    static constexpr int CTRL_H    = 32;
    static constexpr int SCREEN_W  = 320;

    // Capture parameters
    uint32_t freq_hz_{433'920'000u};
    uint32_t bw_hz_{2'000'000u};
    static constexpr uint32_t FREQ_STEP = 100'000u;
    static constexpr uint32_t BW_STEP   = 500'000u;

    // Capture state
    bool     recording_{false};
    uint32_t samples_written_{0};
    uint32_t start_ms_{0};
    uint32_t elapsed_ms_{0};
    uint32_t last_iq_rd_{0};
    char     filename_[64]{};

    // FatFS file handle
    FIL      file_{};
    bool     file_open_{false};

    // Spectrum preview (from IPC shared memory)
    int8_t   spec_bins_[256]{};
    uint16_t last_spec_seq_{0};

    // Status bar
    ui::StatusBar status_bar_{};

    // Control buttons: [REC/STOP] [< Freq] [Freq >] [< BW] [BW >] [Home]
    static constexpr int CTRL_BTN_COUNT = 6;
    ui::Button     ctrl_btns_[CTRL_BTN_COUNT]{};
    ui::ButtonList ctrl_list_{ctrl_btns_, CTRL_BTN_COUNT, 0};

    void start_recording();
    void stop_recording();
    void build_filename();
    void write_iq_chunk();

    void draw_spectrum() const;
    void draw_capture_status() const;
    void draw_controls();
    void update_status();
};

} // namespace sentinel
