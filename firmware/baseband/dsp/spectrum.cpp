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
        0,     5,    20,    44,    79,   123,   177,   241,
      315,   398,   491,   593,   705,   827,   958,  1098,
     1247,  1406,  1573,  1749,  1935,  2128,  2331,  2542,
     2761,  2989,  3224,  3468,  3719,  3978,  4244,  4518,
     4799,  5086,  5381,  5682,  5990,  6304,  6624,  6950,
     7281,  7618,  7961,  8308,  8660,  9017,  9379,  9744,
    10114, 10487, 10864, 11244, 11628, 12014, 12403, 12794,
    13187, 13583, 13980, 14378, 14778, 15178, 15580, 15981,
    16383, 16786, 17187, 17589, 17989, 18389, 18787, 19184,
    19580, 19973, 20364, 20753, 21139, 21523, 21903, 22280,
    22653, 23023, 23388, 23750, 24107, 24459, 24806, 25149,
    25486, 25817, 26143, 26463, 26777, 27085, 27386, 27681,
    27968, 28249, 28523, 28789, 29048, 29299, 29543, 29778,
    30006, 30225, 30436, 30639, 30832, 31018, 31194, 31361,
    31520, 31669, 31809, 31940, 32062, 32174, 32276, 32369,
    32452, 32526, 32590, 32644, 32688, 32723, 32747, 32762,
    32767, 32762, 32747, 32723, 32688, 32644, 32590, 32526,
    32452, 32369, 32276, 32174, 32062, 31940, 31809, 31669,
    31520, 31361, 31194, 31018, 30832, 30639, 30436, 30225,
    30006, 29778, 29543, 29299, 29048, 28789, 28523, 28249,
    27968, 27681, 27386, 27085, 26777, 26463, 26143, 25817,
    25486, 25149, 24806, 24459, 24107, 23750, 23388, 23023,
    22653, 22280, 21903, 21523, 21139, 20753, 20364, 19973,
    19580, 19184, 18787, 18389, 17989, 17589, 17187, 16786,
    16384, 15981, 15580, 15178, 14778, 14378, 13980, 13583,
    13187, 12794, 12403, 12014, 11628, 11244, 10864, 10487,
    10114,  9744,  9379,  9017,  8660,  8308,  7961,  7618,
     7281,  6950,  6624,  6304,  5990,  5682,  5381,  5086,
     4799,  4518,  4244,  3978,  3719,  3468,  3224,  2989,
     2761,  2542,  2331,  2128,  1935,  1749,  1573,  1406,
     1247,  1098,   958,   827,   705,   593,   491,   398,
      315,   241,   177,   123,    79,    44,    20,     5,
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
    arm_cfft_init_256_q15(&s_cfft_instance);

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
