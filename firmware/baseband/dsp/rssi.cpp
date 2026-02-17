// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/rssi.cpp — Continuous RSSI measurement
//
// Collects |I|+|Q| magnitudes continuously, and every 10 ms (or when
// 8192 samples have been accumulated, whichever comes later) publishes
// an updated RSSI reading to the shared memory.
//
// This module is called from every active DSP path via rssi_process(),
// and also from rssi_tick() in the main loop for IDLE mode.

#include "rssi.hpp"
#include "../m0_startup.hpp"
#include "ipc/ipc_protocol.hpp"

using namespace sentinel::ipc;

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

static constexpr uint32_t RSSI_WINDOW_SAMPLES = 8192u;  // samples per measurement
static constexpr uint32_t RSSI_MIN_INTERVAL_MS = 10u;   // minimum update interval

static uint64_t s_mag_accum    = 0u;   // running sum of |I|+|Q|
static uint32_t s_sample_count = 0u;   // samples accumulated so far
static uint32_t s_last_ms      = 0u;   // timestamp of last published update

// ---------------------------------------------------------------------------
// dBm×10 conversion — same LUT logic as scanner.cpp
// ---------------------------------------------------------------------------
// Mean |I|+|Q| over 8192 samples → approximate dBm×10.
// Index = mean >> 2 (0..63), same table as in scanner.

static const int16_t rssi_power_lut[64] = {
    -1200, -1188, -1176, -1167,
    -1158, -1150, -1143, -1137,
    -1131, -1126, -1121, -1117,
    -1113, -1109, -1105, -1102,
    -1099, -1096, -1093, -1090,
    -1088, -1085, -1083, -1080,
    -1078, -1076, -1074, -1072,
    -1070, -1068, -1066, -1064,
    -1062, -1060, -1059, -1057,
    -1055, -1054, -1052, -1051,
    -1049, -1048, -1046, -1045,
    -1044, -1042, -1041, -1040,
    -1038, -1037, -1036, -1035,
    -1033, -1032, -1031, -1030,
    -1029, -1028, -1027, -1026,
    -1025, -1024, -1023, -1022
};

static inline int16_t mean_to_dbm_x10(uint32_t mean_mag) {
    if (mean_mag == 0u) return -1200;
    uint32_t idx = mean_mag >> 2u;
    if (idx >= 64u) idx = 63u;
    return rssi_power_lut[idx];
}

// ---------------------------------------------------------------------------
// rssi_init
// ---------------------------------------------------------------------------

void rssi_init() {
    s_mag_accum    = 0u;
    s_sample_count = 0u;
    s_last_ms      = g_systick_ms;
}

// ---------------------------------------------------------------------------
// Internal: publish current accumulator to shared memory
// ---------------------------------------------------------------------------

static void rssi_publish() {
    if (s_sample_count == 0u) return;

    uint32_t mean = static_cast<uint32_t>(s_mag_accum / s_sample_count);
    int16_t dbm_x10 = mean_to_dbm_x10(mean);

    shared().rssi_dbm_x10 = dbm_x10;
    dmb();
    shared().rssi_seq = static_cast<uint16_t>(shared().rssi_seq + 1u);
    dmb();
    shared().m0_flags |= M0_FLAG_RSSI;
    dmb();
    notify_m4();

    // Reset accumulator
    s_mag_accum    = 0u;
    s_sample_count = 0u;
    s_last_ms      = g_systick_ms;
}

// ---------------------------------------------------------------------------
// rssi_process — called per DMA buffer in all active modes
// ---------------------------------------------------------------------------

void rssi_process(const int8_t* iq_buf, size_t n_pairs) {
    // Accumulate |I|+|Q| for all pairs in this buffer
    for (size_t k = 0u; k < n_pairs; ++k) {
        int16_t abs_i = static_cast<int16_t>(iq_buf[k * 2u]);
        int16_t abs_q = static_cast<int16_t>(iq_buf[k * 2u + 1u]);
        if (abs_i < 0) abs_i = -abs_i;
        if (abs_q < 0) abs_q = -abs_q;
        s_mag_accum += static_cast<uint32_t>(abs_i + abs_q);
    }
    s_sample_count += static_cast<uint32_t>(n_pairs);

    // Publish when we have enough samples AND minimum time has elapsed
    bool enough_samples = (s_sample_count >= RSSI_WINDOW_SAMPLES);
    bool enough_time    = ((g_systick_ms - s_last_ms) >= RSSI_MIN_INTERVAL_MS);

    if (enough_samples && enough_time) {
        rssi_publish();
    }
}

// ---------------------------------------------------------------------------
// rssi_tick — called from main loop in IDLE mode
// ---------------------------------------------------------------------------
// In IDLE mode there are no IQ samples, so we publish a stale RSSI
// (which will reflect whatever was last measured) every 10 ms as a heartbeat.

void rssi_tick() {
    uint32_t now_ms = g_systick_ms;
    if ((now_ms - s_last_ms) >= RSSI_MIN_INTERVAL_MS) {
        // Publish whatever we have, even if accumulator is partial
        if (s_sample_count > 0u) {
            rssi_publish();
        } else {
            // No samples at all — publish floor value
            shared().rssi_dbm_x10 = -1200;
            dmb();
            shared().rssi_seq = static_cast<uint16_t>(shared().rssi_seq + 1u);
            dmb();
            shared().m0_flags |= M0_FLAG_RSSI;
            dmb();
            notify_m4();
            s_last_ms = now_ms;
        }
    }
}
