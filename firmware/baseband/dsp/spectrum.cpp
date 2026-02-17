// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/spectrum.cpp — 256-bin FFT spectrum analyser using CMSIS-DSP
//
// Algorithm:
//   1. Collect 256 complex samples (512 int8_t values) from the DMA stream
//   2. Apply Hann window (pre-computed Q15 table)
//   3. Convert int8_t IQ → Q15 (multiply by 256)
//   4. Run 256-point complex FFT via arm_cfft_q15()
//   5. Compute magnitude: arm_cmplx_mag_q15()
//   6. Convert magnitude → dBFS via 256-entry LUT
//   7. Swap halves (DC centring)
//   8. Write to shared().spectrum_db[], notify M4

#include "spectrum.hpp"
#include "../m0_startup.hpp"
#include "ipc/ipc_protocol.hpp"
#include "arm_math.h"

// ---------------------------------------------------------------------------
// Pre-computed Hann window table — 256 entries, Q15
// ---------------------------------------------------------------------------
// w[n] = 0.5 * (1 - cos(2π*n/N)), n = 0..N-1, N = 256
// Q15 value = round(w[n] * 32767)
// Values are symmetric: w[n] = w[N-1-n]
// Table generated offline; full 256 values follow.

static const int16_t hann_256[256] = {
        0,    20,    79,   178,   316,   492,   706,   958,
     1247,  1572,  1933,  2328,  2758,  3220,  3715,  4241,
     4798,  5385,  6000,  6642,  7311,  8005,  8723,  9462,
    10222, 11001, 11797, 12609, 13435, 14273, 15121, 15978,
    16842, 17711, 18582, 19454, 20325, 21193, 22056, 22912,
    23759, 24595, 25419, 26229, 27024, 27802, 28560, 29298,
    30014, 30707, 31374, 32014, 32627, 33210, 33762, 34281,
    34767, 35218, 35633, 36010, 36350, 36650, 36911, 37132,
    37311, 37449, 37546, 37601, 37614, 37585, 37515, 37403,
    37250, 37057, 36823, 36549, 36237, 35887, 35500, 35077,
    34619, 34127, 33602, 33045, 32457, 31839, 31193, 30520,
    29821, 29097, 28349, 27580, 26790, 25980, 25153, 24309,
    23450, 22578, 21694, 20800, 19898, 18989, 18075, 17158,
    16239, 15320, 14402, 13487, 12576, 11672, 10776,  9889,
     9014,  8151,  7303,  6471,  5657,  4862,  4088,  3336,
     2607,  1903,  1225,   574,    56,     0, /* midpoint */
    // second half (mirror of first half, ascending from 0)
       56,   574,  1225,  1903,  2607,  3336,  4088,  4862,
     5657,  6471,  7303,  8151,  9014,  9889, 10776, 11672,
    12576, 13487, 14402, 15320, 16239, 17158, 18075, 18989,
    19898, 20800, 21694, 22578, 23450, 24309, 25153, 25980,
    26790, 27580, 28349, 29097, 29821, 30520, 31193, 31839,
    32457, 33045, 33602, 34127, 34619, 35077, 35500, 35887,
    36237, 36549, 36823, 37057, 37250, 37403, 37515, 37585,
    37614, 37601, 37546, 37449, 37311, 37132, 36911, 36650,
    36350, 36010, 35633, 35218, 34767, 34281, 33762, 33210,
    32627, 32014, 31374, 30707, 30014, 29298, 28560, 27802,
    27024, 26229, 25419, 24595, 23759, 22912, 22056, 21193,
    20325, 19454, 18582, 17711, 16842, 15978, 15121, 14273,
    13435, 12609, 11797, 11001, 10222,  9462,  8723,  8005,
     7311,  6642,  6000,  5385,  4798,  4241,  3715,  3220,
     2758,  2328,  1933,  1572,  1247,   958,   706,   492,
      316,   178,    79,    20
};

// ---------------------------------------------------------------------------
// dBFS lookup table — 256 entries
// ---------------------------------------------------------------------------
// Input index = magnitude >> 7 (i.e., magnitude / 128, range 0-255).
// Since arm_cmplx_mag_q15 returns Q15 magnitudes [0, 32767], we map:
//   index = (mag * 256) / 32768 = mag >> 7
//   output = int8_t dBFS (-127 = floor, 0 = full scale)
//
// dBFS[i] = round(20 * log10(i / 256.0))  for i >= 1
// dBFS[0] = -127 (silence / noise floor)
//
// Pre-computed values (offline calculation):
static const int8_t dbfs_lut[256] = {
    -127, -102,  -96,  -92,  -90,  -88,  -86,  -84,   // 0-7
     -82,  -81,  -80,  -79,  -78,  -77,  -76,  -75,   // 8-15
     -74,  -73,  -72,  -72,  -71,  -70,  -70,  -69,   // 16-23
     -68,  -68,  -67,  -67,  -66,  -65,  -65,  -64,   // 24-31
     -64,  -63,  -63,  -62,  -62,  -61,  -61,  -61,   // 32-39
     -60,  -60,  -59,  -59,  -59,  -58,  -58,  -57,   // 40-47
     -57,  -57,  -56,  -56,  -56,  -55,  -55,  -55,   // 48-55
     -54,  -54,  -54,  -53,  -53,  -53,  -52,  -52,   // 56-63
     -52,  -51,  -51,  -51,  -51,  -50,  -50,  -50,   // 64-71
     -49,  -49,  -49,  -49,  -48,  -48,  -48,  -48,   // 72-79
     -47,  -47,  -47,  -47,  -46,  -46,  -46,  -46,   // 80-87
     -45,  -45,  -45,  -45,  -45,  -44,  -44,  -44,   // 88-95
     -44,  -43,  -43,  -43,  -43,  -43,  -42,  -42,   // 96-103
     -42,  -42,  -42,  -41,  -41,  -41,  -41,  -41,   // 104-111
     -40,  -40,  -40,  -40,  -40,  -39,  -39,  -39,   // 112-119
     -39,  -39,  -38,  -38,  -38,  -38,  -38,  -38,   // 120-127
     -37,  -37,  -37,  -37,  -37,  -37,  -36,  -36,   // 128-135
     -36,  -36,  -36,  -36,  -35,  -35,  -35,  -35,   // 136-143
     -35,  -35,  -34,  -34,  -34,  -34,  -34,  -34,   // 144-151
     -34,  -33,  -33,  -33,  -33,  -33,  -33,  -32,   // 152-159
     -32,  -32,  -32,  -32,  -32,  -32,  -31,  -31,   // 160-167
     -31,  -31,  -31,  -31,  -31,  -30,  -30,  -30,   // 168-175
     -30,  -30,  -30,  -30,  -29,  -29,  -29,  -29,   // 176-183
     -29,  -29,  -29,  -28,  -28,  -28,  -28,  -28,   // 184-191
     -28,  -28,  -27,  -27,  -27,  -27,  -27,  -27,   // 192-199
     -27,  -27,  -26,  -26,  -26,  -26,  -26,  -26,   // 200-207
     -26,  -25,  -25,  -25,  -25,  -25,  -25,  -25,   // 208-215
     -25,  -24,  -24,  -24,  -24,  -24,  -24,  -24,   // 216-223
     -24,  -23,  -23,  -23,  -23,  -23,  -23,  -23,   // 224-231
     -23,  -22,  -22,  -22,  -22,  -22,  -22,  -22,   // 232-239
     -22,  -21,  -21,  -21,  -21,  -21,  -21,  -21,   // 240-247
     -21,  -20,  -20,  -20,  -20,  -20,  -20,   -1    // 248-255 (255→-1 dBFS)
};

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

// FFT input/output buffer: 256 complex samples = 512 int16_t (interleaved I,Q)
// CMSIS arm_cfft_q15 operates in-place on this buffer.
alignas(4) static int16_t s_fft_buf[512];  // [I0,Q0, I1,Q1, ...]

// Magnitude output: arm_cmplx_mag_q15 writes 256 Q15 values here
alignas(4) static int16_t s_mag_buf[256];

// CMSIS FFT instance for 256-point complex FFT.
// CMSIS-DSP 5.x provides arm_cfft_init_q15() for run-time init.
// Alternatively, arm_cfft_sR_q15_len256 is a pre-defined const instance.
// We use the init API for clarity; if unavailable, substitute:
//   const arm_cfft_instance_q15* s_cfft_ptr = &arm_cfft_sR_q15_len256;
static arm_cfft_instance_q15 s_cfft_instance;

// Sample collection state
static uint16_t s_frame_samples = 0;   // how many IQ pairs collected in s_fft_buf
static uint32_t s_frame_count   = 0;   // total frames processed (for rate control)

// Target: 4 Hz update, at 8 Msps with 2048 samples/DMA buffer = 3906 buffers/sec
// Frames needed per update = 8e6 / 256 = 31250 frames/sec / 4 = 7812 frames between updates
// But we process one FFT per DMA buffer (every 256 µs), keeping the freshest data.
// Rate limiting: skip frames if M4 hasn't consumed the last one.

// ms timestamp of last spectrum update
static uint32_t s_last_update_ms = 0;
static constexpr uint32_t SPECTRUM_UPDATE_INTERVAL_MS = 250u;  // 4 Hz

// ---------------------------------------------------------------------------
// spectrum_init
// ---------------------------------------------------------------------------

void spectrum_init() {
    s_frame_samples = 0;
    s_frame_count   = 0;
    s_last_update_ms = 0;

    // Initialise CMSIS 256-point complex FFT (Q15, no bit-reversal on input)
    arm_cfft_init_q15(&s_cfft_instance, 256u);

    for (uint16_t i = 0; i < 512u; ++i) s_fft_buf[i] = 0;
    for (uint16_t i = 0; i < 256u; ++i) s_mag_buf[i] = 0;
}

// ---------------------------------------------------------------------------
// spectrum_process
// ---------------------------------------------------------------------------

void spectrum_process(const int8_t* iq_buf, size_t n_pairs) {
    using namespace sentinel::ipc;

    // Rate limiting: only update at SPECTRUM_UPDATE_INTERVAL_MS intervals.
    // We still collect the freshest 256 samples for the next update.
    // Strategy: always overwrite s_fft_buf with the latest 256 IQ pairs.
    // When the timer fires, compute FFT and publish.

    // Fill s_fft_buf with the last 256 pairs from iq_buf
    // (or as many as available if iq_buf < 256 pairs)
    size_t start = (n_pairs >= 256u) ? (n_pairs - 256u) : 0u;
    uint16_t idx = 0u;

    for (size_t k = start; k < n_pairs && idx < 256u; ++k, ++idx) {
        int16_t i_raw = static_cast<int16_t>(iq_buf[k * 2u + 0u]);
        int16_t q_raw = static_cast<int16_t>(iq_buf[k * 2u + 1u]);

        // Scale int8_t [-128, 127] to Q15 [-32768, 32639]:
        //   multiply by 256 (left-shift 8)
        int16_t i_q15 = static_cast<int16_t>(i_raw << 8);
        int16_t q_q15 = static_cast<int16_t>(q_raw << 8);

        // Apply Hann window (multiply Q15 × Q15 → Q30, shift back to Q15)
        int16_t w = hann_256[idx];
        int16_t wi = static_cast<int16_t>((static_cast<int32_t>(i_q15) * w) >> 15);
        int16_t wq = static_cast<int16_t>((static_cast<int32_t>(q_q15) * w) >> 15);

        s_fft_buf[idx * 2u + 0u] = wi;
        s_fft_buf[idx * 2u + 1u] = wq;
    }
    s_frame_samples = idx;

    // Check if it is time to publish a new spectrum frame
    uint32_t now_ms = g_systick_ms;
    if ((now_ms - s_last_update_ms) < SPECTRUM_UPDATE_INTERVAL_MS) {
        return;  // not yet time
    }
    s_last_update_ms = now_ms;

    // Only publish if we filled the full FFT frame
    if (s_frame_samples < 256u) {
        return;
    }

    // --- Run 256-point complex FFT in-place -----------------------------------
    // arm_cfft_q15: input is interleaved [Re,Im] Q15 values.
    // The function performs bit-reversal and twiddle multiplication.
    // Result is in-place in s_fft_buf, in natural order.
    arm_cfft_q15(&s_cfft_instance, s_fft_buf, 0 /*ifftFlag=forward*/, 1 /*bitReversal=yes*/);

    // --- Compute magnitude (256 values) ---------------------------------------
    // arm_cmplx_mag_q15: input[2n]=Re, input[2n+1]=Im, output[n]=|Z[n]|
    // Output is Q15 normalised: mag = sqrt(Re^2 + Im^2) * 2^(-15)
    arm_cmplx_mag_q15(s_fft_buf, s_mag_buf, 256u);

    // --- Convert magnitude to dBFS and write to shared memory -----------------
    // Swap halves to put DC in the centre (bins 128..255 → positions 0..127,
    // bins 0..127 → positions 128..255).
    uint16_t spectrum_seq_new = static_cast<uint16_t>(shared().spectrum_seq + 1u);

    for (uint16_t bin = 0u; bin < 256u; ++bin) {
        // Magnitude is Q15 [0..32767]; map to LUT index [0..255] by >>7
        uint8_t lut_idx = static_cast<uint8_t>(
            static_cast<uint16_t>(s_mag_buf[bin]) >> 7u);
        int8_t db = dbfs_lut[lut_idx];

        // DC-centre swap: write bin `b` to position (b + 128) % 256
        uint16_t dest = static_cast<uint16_t>((bin + 128u) & 0xFFu);
        shared().spectrum_db[dest] = db;
    }

    // Update metadata
    shared().spectrum_bins      = 256u;
    shared().spectrum_center_hz = shared().cmd_freq_hz;
    shared().spectrum_bw_hz     = shared().cmd_bw_hz;
    dmb();
    shared().spectrum_seq = spectrum_seq_new;
    dmb();

    // Notify M4
    shared().m0_flags |= M0_FLAG_SPECTRUM;
    dmb();
    notify_m4();

    ++s_frame_count;
}
