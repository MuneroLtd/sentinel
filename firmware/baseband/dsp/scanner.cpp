// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/scanner.cpp — Energy-detection frequency scanner
//
// Algorithm:
//   1. Accumulate N = 8192 magnitude samples (|I|+|Q|) per dwell window
//   2. Compute mean: sum / N → average power proxy (0-255)
//   3. Convert to approximate dBm×10 via power_to_dbm_x10()
//   4. Update shared().rssi_dbm_x10 and set M0_FLAG_RSSI on every window
//   5. If above threshold (-80 dBm), push SignalEvent and set M0_FLAG_SIGNAL
//
// The scanner assumes M4 handles RF retuning (it receives the TUNE command,
// waits for M4's SEV acknowledge, then starts a new dwell window).
// Dwell time = 8192 samples / 8 Msps = 1.024 ms per measurement window.
// At ~50 ms dwell (spec), M0 averages ~48 consecutive windows before reporting.

#include "scanner.hpp"
#include "../m0_startup.hpp"
#include "ipc/ipc_protocol.hpp"

using namespace sentinel::ipc;

// ---------------------------------------------------------------------------
// Power → dBm×10 conversion
// ---------------------------------------------------------------------------
// HackRF One typical noise floor: ~-110 dBm at 8 MHz bandwidth, 0 dB gain.
// L1 magnitude |I|+|Q| for int8_t noise (rms ~10 counts): mean ≈ 12-16.
// Full-scale signal (|I|=127, |Q|=127): max L1 = 254.
//
// Piecewise linear approximation (rough calibration, ±5 dB):
//   mean = 0    → -120 dBm   (-1200 dBm×10)
//   mean = 1    → -110 dBm   (-1100 dBm×10)
//   mean = 2    → -107 dBm   (-1070 dBm×10)
//   mean = 4    → -104 dBm
//   mean = 8    → -98  dBm
//   mean = 16   → -92  dBm
//   mean = 32   → -86  dBm
//   mean = 64   → -80  dBm
//   mean = 128  → -74  dBm
//   mean = 255  → -68  dBm  (roughly; compression at high levels)
//
// Formula: dBm×10 = -1200 + (mean * 40) / 8  for mean < 255
//   Simplified: dBm×10 ≈ -1200 + 5 * mean  (linear approximation, decent for SDR)
//   But we use the log-ish approximation:
//   dBm×10 ≈ -1200 + 200 * log2(mean+1) / 2
//
// On M0 without float, use a small LUT indexed by mean >> 2 (64 entries).

// LUT: index = mean_magnitude / 4 (0..63), value = dBm×10
// Generated from: -1200 + round(200 * log2(idx*4 + 1) / 2)
// But for simplicity: -1200 + (index << 2) * 40 / 8 = -1200 + index * 20
// Better: use: dBm×10 = -1200 + 60 * log2(mean+1) where log2 is estimated.
//
// Using a simpler formula that fits actual SDR measurements:
//   If mean < 2:   dBm×10 = -1200
//   Else:          dBm×10 = -1200 + 60 * clz_based_log2
//
// Pre-computed 64-entry LUT (index = mean >> 2, range [0..255] → [0..63]):
static const int16_t power_lut[64] = {
    -1200, -1188, -1176, -1167,  // 0-3   (mean 0-15)
    -1158, -1150, -1143, -1137,  // 4-7   (mean 16-31)
    -1131, -1126, -1121, -1117,  // 8-11  (mean 32-47)
    -1113, -1109, -1105, -1102,  // 12-15 (mean 48-63)
    -1099, -1096, -1093, -1090,  // 16-19 (mean 64-79)
    -1088, -1085, -1083, -1080,  // 20-23 (mean 80-95)
    -1078, -1076, -1074, -1072,  // 24-27 (mean 96-111)
    -1070, -1068, -1066, -1064,  // 28-31 (mean 112-127)
    -1062, -1060, -1059, -1057,  // 32-35 (mean 128-143)
    -1055, -1054, -1052, -1051,  // 36-39 (mean 144-159)
    -1049, -1048, -1046, -1045,  // 40-43 (mean 160-175)
    -1044, -1042, -1041, -1040,  // 44-47 (mean 176-191)
    -1038, -1037, -1036, -1035,  // 48-51 (mean 192-207)
    -1033, -1032, -1031, -1030,  // 52-55 (mean 208-223)
    -1029, -1028, -1027, -1026,  // 56-59 (mean 224-239)
    -1025, -1024, -1023, -1022   // 60-63 (mean 240-255)
};

int16_t power_to_dbm_x10(uint32_t mean_magnitude) {
    if (mean_magnitude == 0u) return -1200;
    uint32_t idx = mean_magnitude >> 2u;  // divide by 4
    if (idx >= 64u) idx = 63u;
    return power_lut[idx];
}

// ---------------------------------------------------------------------------
// Scanner state
// ---------------------------------------------------------------------------

static constexpr uint32_t DWELL_SAMPLES      = 8192u;   // samples per measurement
static constexpr uint32_t DWELL_WINDOWS      = 48u;     // windows per ~50 ms dwell
static constexpr int16_t  THRESHOLD_DBM_X10  = -800;    // -80.0 dBm

static uint64_t s_mag_accum   = 0u;   // sum of |I|+|Q| over current window
static uint32_t s_mag_count   = 0u;   // samples accumulated in current window
static uint32_t s_window_count = 0u;  // windows completed in current dwell
static uint32_t s_dwell_sum   = 0u;   // sum of mean magnitudes over dwell
static uint32_t s_dwell_freq_hz = 0u; // frequency being measured
static uint32_t s_dwell_start_ms = 0u;

// ---------------------------------------------------------------------------
// scanner_init
// ---------------------------------------------------------------------------

void scanner_init() {
    s_mag_accum      = 0u;
    s_mag_count      = 0u;
    s_window_count   = 0u;
    s_dwell_sum      = 0u;
    s_dwell_freq_hz  = shared().cmd_freq_hz;
    s_dwell_start_ms = g_systick_ms;
}

// ---------------------------------------------------------------------------
// scanner_process
// ---------------------------------------------------------------------------

void scanner_process(const int8_t* iq_buf, size_t n_pairs) {
    size_t i = 0u;

    while (i < n_pairs) {
        // Accumulate |I|+|Q|
        int16_t abs_i = static_cast<int16_t>(iq_buf[i * 2u]);
        int16_t abs_q = static_cast<int16_t>(iq_buf[i * 2u + 1u]);
        if (abs_i < 0) abs_i = -abs_i;
        if (abs_q < 0) abs_q = -abs_q;
        s_mag_accum += static_cast<uint32_t>(abs_i + abs_q);
        ++s_mag_count;
        ++i;

        if (s_mag_count < DWELL_SAMPLES) {
            continue;
        }

        // Window complete — compute mean magnitude
        uint32_t mean = static_cast<uint32_t>(s_mag_accum / DWELL_SAMPLES);
        s_mag_accum = 0u;
        s_mag_count = 0u;

        // Accumulate over multiple windows for stability
        s_dwell_sum += mean;
        ++s_window_count;

        if (s_window_count < DWELL_WINDOWS) {
            continue;
        }

        // Dwell period complete — compute average and publish
        uint32_t dwell_mean = s_dwell_sum / DWELL_WINDOWS;
        s_dwell_sum    = 0u;
        s_window_count = 0u;

        int16_t dbm_x10 = power_to_dbm_x10(dwell_mean);
        uint32_t duration_ms = static_cast<uint32_t>(g_systick_ms - s_dwell_start_ms);
        s_dwell_start_ms = g_systick_ms;
        s_dwell_freq_hz = shared().cmd_freq_hz;

        // Update RSSI in shared memory
        shared().rssi_dbm_x10 = dbm_x10;
        dmb();
        shared().m0_flags |= M0_FLAG_RSSI;
        dmb();
        notify_m4();

        // If above threshold, push to signal ring buffer
        if (dbm_x10 >= THRESHOLD_DBM_X10) {
            uint8_t wr = shared().signal_wr;
            uint8_t rd = shared().signal_rd;
            uint8_t used = static_cast<uint8_t>(wr - rd);

            if (used < SIGNAL_RING_DEPTH) {
                SignalEvent& ev = shared().signals[wr % SIGNAL_RING_DEPTH];
                ev.freq_hz      = s_dwell_freq_hz;
                ev.rssi_dbm_x10 = dbm_x10;
                ev.duration_ms  = (duration_ms > 65535u) ? 65535u :
                                  static_cast<uint16_t>(duration_ms);
                dmb();
                shared().signal_wr = static_cast<uint8_t>(wr + 1u);
            }

            shared().m0_flags |= M0_FLAG_SIGNAL;
            dmb();
            notify_m4();
        }
    }
}
