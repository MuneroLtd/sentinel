// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/adsb.cpp — ADS-B 1090ES preamble detection and frame decoder
//
// Timing reference (8 Msps = 8 samples per µs):
//
// ADS-B preamble (16 µs total = 128 samples at 8 Msps):
//   Pulse 1:  bits  0..7   HIGH  (1 µs)
//   Gap:      bits  8..15  LOW   (1 µs)
//   Pulse 2:  bits 16..23  HIGH  (1 µs)
//   Gap:      bits 24..55  LOW   (3.5 µs)  [spec: 24..55 = 32 samples = 4 µs gap to pulse 3]
//   Pulse 3:  bits 56..63  HIGH  (1 µs)
//   Gap:      bits 64..71  LOW   (1 µs)
//   Pulse 4:  bits 72..79  HIGH  (1 µs)
//   Guard:    bits 80..127 LOW   (6 µs guard + data start)
//
// Data: 112 bits, 1 bit per µs (= 8 samples per bit)
// Each bit is PPM encoded over 2 × 0.5 µs chips (4 samples each):
//   Bit = 1: first chip HIGH, second chip LOW
//   Bit = 0: first chip LOW, second chip HIGH
//
// Total frame: 128 (preamble) + 896 (112 bits × 8) = 1024 samples at 8 Msps

#include "adsb.hpp"
#include "../m0_startup.hpp"
#include "ipc/ipc_protocol.hpp"

using namespace sentinel::ipc;

// ---------------------------------------------------------------------------
// Magnitude helper
// ---------------------------------------------------------------------------

uint8_t adsb_get_magnitude(int8_t i, int8_t q) {
    int16_t ai = (i < 0) ? static_cast<int16_t>(-i) : static_cast<int16_t>(i);
    int16_t aq = (q < 0) ? static_cast<int16_t>(-q) : static_cast<int16_t>(q);
    int16_t sum = ai + aq;
    return (sum > 255) ? 255u : static_cast<uint8_t>(sum);
}

// ---------------------------------------------------------------------------
// CRC24 — polynomial 0xFFF409 (ICAO Annex 10 Mode-S CRC)
// ---------------------------------------------------------------------------
// The CRC covers bits 1..88 of the message; the last 24 bits (3 bytes) are
// the parity/CRC field.  For a received frame the CRC of all 14 bytes should
// be zero if error-free.
//
// Implementation: bit-by-bit, no table (saves 1KB of ROM on M0).
// 112-bit message = 14 bytes.  CRC is applied to all 14 bytes as a stream;
// the result should be zero for a valid frame.

static constexpr uint32_t CRC24_POLY = 0xFFF409u;

bool adsb_check_crc(const uint8_t* frame, size_t len) {
    uint32_t crc = 0u;

    for (size_t byte = 0u; byte < len; ++byte) {
        crc ^= static_cast<uint32_t>(frame[byte]) << 16u;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if (crc & 0x800000u) {
                crc = (crc << 1u) ^ CRC24_POLY;
            } else {
                crc <<= 1u;
            }
        }
    }

    // For a valid frame with CRC appended, remainder should be zero.
    return (crc & 0xFFFFFFu) == 0u;
}

// ---------------------------------------------------------------------------
// Preamble correlator
// ---------------------------------------------------------------------------
// Checks whether a 128-sample window starting at `mag[0]` matches the
// ADS-B preamble pattern.
//
// Preamble structure at 8 Msps (8 samples per µs):
//   HIGH: samples  0.. 7  (pulse 1)
//   LOW:  samples  8..15
//   HIGH: samples 16..23  (pulse 2)
//   LOW:  samples 24..55  (3.5 µs gap — actually spec says gap from P2 end to P3
//                          start is 3.5 µs: 28 samples LOW)
//          Wait, re-checking the spec:
//          P1 at 0 µs, P2 at 2 µs, P3 at 7 µs, P4 at 8 µs
//          (all 0.5 µs wide = 4 samples at 8 Msps)
//   So preamble at 8 Msps (0.5 µs pulses, 4 samples each):
//     0- 3: HIGH (P1 first half)
//     4- 7: LOW  (P1 second half — spec 0.5+0.5 per bit chip)
//
//   Using a simplified correlator: check that 4 specific windows are HIGH
//   and the gaps between them are LOW.  Use a threshold = mean of HIGH peaks.
//
// Simplified preamble template at 8 Msps (8 samples per µs, 0.5 µs chip):
//   The preamble has 4 pulses, each 0.5 µs wide (4 samples at 8 Msps):
//     P1 start:  0 µs → sample  0
//     P2 start:  1 µs → sample  8
//     P3 start:  3.5 µs → sample 28  [note: spec ambiguity; use 56 at 8/µs = 7µs]
//     P4 start:  4.5 µs → sample 36  [or 64 at 7µs interpretation]
//
// Per ADS-B MOPS (RTCA DO-260B):
//   Preamble timing measured from first edge:
//   P1: 0 µs,  P2: 1 µs,  P3: 3.5 µs,  P4: 4.5 µs
//   Each pulse = 0.5 µs wide.
//   At 8 Msps: 1 µs = 8 samples, 0.5 µs = 4 samples.
//   P1: samples 0-3
//   P2: samples 8-11
//   P3: samples 28-31
//   P4: samples 36-39
//   Data starts at 8 µs = sample 64.
//   Total preamble = 8 µs = 64 samples. Data = 112 bits × 8 = 896 samples.
//   Total = 64 + 896 = 960 samples minimum.

static constexpr uint16_t PREAMBLE_SAMPLES = 64u;
static constexpr uint16_t DATA_BITS        = 112u;
static constexpr uint16_t SAMPLES_PER_BIT  = 8u;
static constexpr uint16_t DATA_SAMPLES     = DATA_BITS * SAMPLES_PER_BIT;
static constexpr uint16_t FRAME_SAMPLES    = PREAMBLE_SAMPLES + DATA_SAMPLES; // 64 + 896 = 960

// Pulse sample ranges (4 samples each at 8 Msps, 0.5 µs pulses)
struct PulseRange { uint8_t start; uint8_t len; };
static constexpr PulseRange PULSES[4] = {
    {  0u, 4u },  // P1: 0.0 µs
    {  8u, 4u },  // P2: 1.0 µs
    { 28u, 4u },  // P3: 3.5 µs
    { 36u, 4u }   // P4: 4.5 µs
};

// Sum of magnitude samples in a range
static inline uint16_t mag_sum(const uint8_t* mag, uint8_t start, uint8_t len) {
    uint16_t s = 0u;
    for (uint8_t i = 0u; i < len; ++i) {
        s += mag[start + i];
    }
    return s;
}

// Returns true if the 64-sample window at mag[] looks like a valid preamble.
// Also returns the signal threshold in *threshold (average pulse magnitude / 2).
static bool check_preamble(const uint8_t* mag, uint8_t* threshold) {
    // Sum pulse regions
    uint16_t pulse_sum = 0u;
    for (uint8_t p = 0u; p < 4u; ++p) {
        pulse_sum += mag_sum(mag, PULSES[p].start, PULSES[p].len);
    }
    // Average pulse magnitude (16 samples total across 4 pulses)
    uint8_t pulse_avg = static_cast<uint8_t>(pulse_sum >> 4u);  // / 16
    if (pulse_avg < 8u) {
        return false;  // Too weak — noise floor
    }
    *threshold = pulse_avg >> 1u;  // threshold = half average pulse height

    // Verify gap regions are LOW (below threshold)
    // Gaps at: 4-7, 12-27, 32-35, 40-63
    const struct { uint8_t s; uint8_t l; } gaps[] = {
        {  4u,  4u },   // gap after P1
        { 12u, 16u },   // gap between P2 and P3
        { 32u,  4u },   // gap between P3 and P4
        { 40u, 24u }    // guard interval before data
    };

    for (uint8_t g = 0u; g < 4u; ++g) {
        uint16_t gap_sum = mag_sum(mag, gaps[g].s, gaps[g].l);
        uint16_t gap_avg = gap_sum / gaps[g].l;
        if (gap_avg > static_cast<uint16_t>(*threshold)) {
            return false;  // Gap is too high — not a valid preamble
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Bit slicer — decode one PPM bit from 8 samples
// ---------------------------------------------------------------------------
// Each bit occupies 8 samples (1 µs at 8 Msps), divided into two 4-sample chips:
//   Chip 1: samples 0-3  (first half)
//   Chip 2: samples 4-7  (second half)
// Bit = 1 if chip 1 > chip 2, Bit = 0 if chip 2 > chip 1.

static uint8_t slice_bit(const uint8_t* mag, uint16_t offset) {
    uint16_t chip1 = 0u, chip2 = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        chip1 += mag[offset + i];
        chip2 += mag[offset + 4u + i];
    }
    return (chip1 > chip2) ? 1u : 0u;
}

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

// Magnitude buffer: we build this from the IQ buffer on each call.
// Size = DMA_BUF_SAMPLES = 2048 (enough for the full DMA buffer).
static uint8_t s_mag_buf[2048u];

// Carry-over buffer: when a frame spans two DMA buffers, we keep the tail
// of the previous buffer so we don't miss frames.
static constexpr uint16_t CARRY_SAMPLES = FRAME_SAMPLES;  // 960 samples worst case
static uint8_t s_carry_buf[CARRY_SAMPLES];
static uint16_t s_carry_len = 0u;

// Timestamp (ms) at the start of each DMA buffer
static uint32_t s_buf_timestamp_ms = 0u;

// ---------------------------------------------------------------------------
// adsb_init
// ---------------------------------------------------------------------------

void adsb_init() {
    s_carry_len = 0u;
    for (uint16_t i = 0u; i < CARRY_SAMPLES; ++i) s_carry_buf[i] = 0u;
    shared().adsb_wr = 0u;
    shared().adsb_rd = 0u;
}

// ---------------------------------------------------------------------------
// Frame extraction from a linear magnitude buffer
// ---------------------------------------------------------------------------

static void process_magnitude(const uint8_t* mag, uint16_t mag_len, uint32_t base_ms) {
    // Scan magnitude stream for preambles
    uint16_t i = 0u;
    while (i + FRAME_SAMPLES <= mag_len) {
        uint8_t threshold = 0u;

        if (!check_preamble(&mag[i], &threshold)) {
            ++i;
            continue;
        }

        // Preamble found at offset i.
        // Decode 112 data bits starting at i + PREAMBLE_SAMPLES
        uint8_t frame[14] = {};
        uint16_t data_start = static_cast<uint16_t>(i + PREAMBLE_SAMPLES);

        for (uint16_t bit = 0u; bit < DATA_BITS; ++bit) {
            uint16_t bit_offset = static_cast<uint16_t>(data_start + bit * SAMPLES_PER_BIT);
            uint8_t b = slice_bit(mag, bit_offset);
            frame[bit >> 3u] = static_cast<uint8_t>(frame[bit >> 3u] | (b << (7u - (bit & 7u))));
        }

        // Validate CRC24
        if (!adsb_check_crc(frame, 14u)) {
            // CRC failed — advance by 1 and keep searching
            ++i;
            continue;
        }

        // Compute approximate RSSI for this frame from preamble level
        // threshold = pulse_avg / 2.  pulse_avg is in [0, 255] (L1 magnitude).
        // Map to dBm×10: rough formula dBm ≈ -120 + threshold (very approximate)
        int16_t rssi_x10 = static_cast<int16_t>(-1200 + static_cast<int16_t>(threshold) * 10);

        // Write to ring buffer if there is space
        uint8_t wr = shared().adsb_wr;
        uint8_t rd = shared().adsb_rd;
        uint8_t used = static_cast<uint8_t>(wr - rd);

        if (used < ADSB_RING_DEPTH) {
            AdsbFrame& dst = shared().adsb_frames[wr % ADSB_RING_DEPTH];
            for (uint8_t b = 0u; b < 14u; ++b) {
                dst.data[b] = frame[b];
            }
            dst.timestamp_ms = base_ms;
            dst.rssi_dbm_x10 = rssi_x10;
            dmb();
            shared().adsb_wr = static_cast<uint8_t>(wr + 1u);
        }

        // Notify M4
        shared().m0_flags |= M0_FLAG_ADSB;
        dmb();
        notify_m4();

        // Skip past this frame to avoid re-detecting within it
        i = static_cast<uint16_t>(i + FRAME_SAMPLES);
    }

    // Save tail for carry-over (in case a frame spans this buffer and the next)
    uint16_t tail = (mag_len > CARRY_SAMPLES) ?
                    static_cast<uint16_t>(mag_len - CARRY_SAMPLES) : 0u;
    uint16_t new_carry = static_cast<uint16_t>(mag_len - tail);
    for (uint16_t j = 0u; j < new_carry; ++j) {
        s_carry_buf[j] = mag[tail + j];
    }
    s_carry_len = new_carry;
}

// ---------------------------------------------------------------------------
// adsb_process
// ---------------------------------------------------------------------------

void adsb_process(const int8_t* iq_buf, size_t n_pairs) {
    uint32_t ts_ms = g_systick_ms;

    // Build magnitude buffer from IQ pairs
    size_t n = (n_pairs > 2048u) ? 2048u : n_pairs;
    for (size_t k = 0u; k < n; ++k) {
        s_mag_buf[k] = adsb_get_magnitude(iq_buf[k * 2u], iq_buf[k * 2u + 1u]);
    }

    // Process carry-over + new data together if we have carry samples
    if (s_carry_len > 0u && s_carry_len + n <= sizeof(s_mag_buf)) {
        // Shift carry to beginning of mag buffer; append new data
        // We use a scratch approach: process [carry | new_data]
        // Since mag_buf might not fit both, only do this if it fits
        uint16_t total = static_cast<uint16_t>(s_carry_len + n);
        // Build combined buffer in place (move new data right to make room for carry)
        // We process carry+new together only if total <= 2048
        if (total <= 2048u) {
            // Move new data right by s_carry_len positions
            for (int32_t j = static_cast<int32_t>(total) - 1;
                 j >= static_cast<int32_t>(s_carry_len); --j) {
                s_mag_buf[static_cast<uint32_t>(j)] =
                    s_mag_buf[static_cast<uint32_t>(j) - s_carry_len];
            }
            // Copy carry to front
            for (uint16_t j = 0u; j < s_carry_len; ++j) {
                s_mag_buf[j] = s_carry_buf[j];
            }
            s_carry_len = 0u;
            process_magnitude(s_mag_buf, total, ts_ms);
            return;
        }
    }

    // Normal path: process this buffer directly
    s_carry_len = 0u;
    process_magnitude(s_mag_buf, static_cast<uint16_t>(n), ts_ms);
}
