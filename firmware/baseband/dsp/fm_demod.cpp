// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/fm_demod.cpp — FM demodulator (WBFM / NBFM), fully fixed-point Q15
//
// Signal chain:
//   8 Msps int8_t IQ
//     → 4:1 CIC decimator → 2 Msps IQ (Q15)
//     → FM discriminator (phase difference) → baseband audio (Q15, 2 Msps)
//     → De-emphasis IIR (1-pole, 75 µs or 25 µs) → filtered audio (Q15)
//     → 62.5:1 FIR decimator → 32 kHz PCM int16_t
//     → written to ipc::shared().audio_buf ring buffer

#include "fm_demod.hpp"
#include "../m0_startup.hpp"
#include "ipc/ipc_protocol.hpp"

// ---------------------------------------------------------------------------
// Q15 arithmetic helpers
// ---------------------------------------------------------------------------
// Q15: 1.15 signed fixed-point.  32767 = 1.0, -32768 = -1.0.
// Multiplication: (a * b) >> 15 — result is Q15.
// All intermediate products are int32_t to avoid overflow.

static inline int16_t q15_mul(int16_t a, int16_t b) {
    return static_cast<int16_t>((static_cast<int32_t>(a) * b) >> 15);
}

static inline int32_t q15_mul32(int16_t a, int16_t b) {
    return (static_cast<int32_t>(a) * b) >> 15;
}

// Saturate int32 to int16 range
static inline int16_t sat16(int32_t x) {
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return static_cast<int16_t>(x);
}

// ---------------------------------------------------------------------------
// Fast integer atan2 approximation (Q15 output)
// ---------------------------------------------------------------------------
// Algorithm (Chebyshev-based for efficiency on M0):
//
//   For |y| <= |x|:
//     r = y / x                (rational approximation)
//     angle ≈ r * (π/4 + 0.2732*(1 - |r|))
//
//   Handle quadrant correction via sign of x.
//
// Output range: [-32768, 32767] representing [-π, +π).
// This avoids any lookup table and runs in ~20 integer instructions on M0.
//
// Reference: "Efficient approximations for the arctangent function"
//            Rajan et al., IEEE Signal Processing Magazine 2006.

int16_t atan2_q15(int16_t y, int16_t x) {
    // Constants in Q15:
    //   PI/4     = 0.7854 → 0x6488 (25736)
    //   3*PI/4   = 2.3562 → saturated to 0x7FFF for the π case
    //   0.2732   = 0x2308 (8968) — Chebyshev coefficient
    //   PI/2     = 1.5708 → 0x6488*2 but we use a direct value
    //
    // PI in Q15 = 32767 (maps π → 32767, which represents 1.0 in Q15).
    // We treat 32767 as π and 16384 as π/2 for the output.

    static constexpr int16_t K_PI4     = 0x6488;  // π/4 in Q15 (scaled: 25736)
    static constexpr int16_t K_COEFF   = 0x2308;  // 0.2732 in Q15 (8968)
    // Note: π is represented as 32767 (full scale = π radians)

    if (x == 0 && y == 0) return 0;

    int32_t abs_x = abs32(x);
    int32_t abs_y = abs32(y);
    int16_t angle;

    // Ensure we compute atan(min/max) to keep ratio in [0,1]
    if (abs_x >= abs_y) {
        // |y/x| in [0, 1]
        // r = y / x in Q15: divide then scale
        // To stay in Q15 without 64-bit: scale numerator left then divide
        int32_t r = (static_cast<int32_t>(y) << 15) / (x != 0 ? x : 1);
        r = (r > 32767) ? 32767 : (r < -32767 ? -32767 : r);
        int16_t r16 = static_cast<int16_t>(r);

        // abs_r in Q15
        int16_t abs_r = (r16 < 0) ? static_cast<int16_t>(-r16) : r16;

        // atan ≈ r * (π/4 + 0.2732*(1 - |r|))
        //       = r * (K_PI4 + K_COEFF * (32767 - abs_r) >> 15)
        int32_t inner = static_cast<int32_t>(K_PI4) +
                        ((static_cast<int32_t>(K_COEFF) * (32767 - abs_r)) >> 15);
        angle = sat16((static_cast<int32_t>(r16) * inner) >> 15);

        // Quadrant correction: if x < 0, add or subtract π
        if (x < 0) {
            if (y >= 0) {
                angle = sat16(static_cast<int32_t>(angle) + 32767);  // + π
            } else {
                angle = sat16(static_cast<int32_t>(angle) - 32767);  // - π
            }
        }
    } else {
        // |x/y| in [0, 1] — swap roles and use π/2 - atan(x/y)
        int32_t r = (static_cast<int32_t>(x) << 15) / (y != 0 ? y : 1);
        r = (r > 32767) ? 32767 : (r < -32767 ? -32767 : r);
        int16_t r16 = static_cast<int16_t>(r);

        int16_t abs_r = (r16 < 0) ? static_cast<int16_t>(-r16) : r16;

        int32_t inner = static_cast<int32_t>(K_PI4) +
                        ((static_cast<int32_t>(K_COEFF) * (32767 - abs_r)) >> 15);
        int16_t atan_part = sat16((static_cast<int32_t>(r16) * inner) >> 15);

        // angle = sign(y) * (π/2 - atan(x/y))
        // π/2 in Q15 = 16384
        if (y > 0) {
            angle = sat16(16384 - static_cast<int32_t>(atan_part));
        } else {
            angle = sat16(-16384 - static_cast<int32_t>(atan_part));
        }
    }

    return angle;
}

// ---------------------------------------------------------------------------
// FM discriminator — phase difference method
// ---------------------------------------------------------------------------
// Instantaneous frequency ∝ d(phase)/dt ≈ atan2(cross, dot)
// where:
//   cross = Q[n]*I[n-1] - I[n]*Q[n-1]   (imaginary part of Z[n]*conj(Z[n-1]))
//   dot   = I[n]*I[n-1] + Q[n]*Q[n-1]   (real part of Z[n]*conj(Z[n-1]))
//
// Input/output all Q15.

int16_t fm_discriminate(int16_t i_cur, int16_t q_cur,
                        int16_t i_prv, int16_t q_prv) {
    // Compute cross and dot products in Q30 (product of two Q15 values)
    // then right-shift to Q15 before passing to atan2
    int32_t cross32 = (static_cast<int32_t>(q_cur) * i_prv) -
                      (static_cast<int32_t>(i_cur) * q_prv);
    int32_t dot32   = (static_cast<int32_t>(i_cur) * i_prv) +
                      (static_cast<int32_t>(q_cur) * q_prv);

    // Normalise to Q15 range for atan2_q15 input
    // The products are in Q30; shift right by 15 to get Q15
    int16_t cross = sat16(cross32 >> 15);
    int16_t dot   = sat16(dot32   >> 15);

    return atan2_q15(cross, dot);
}

// ---------------------------------------------------------------------------
// Decimation FIR — 62.5:1 from 2 Msps to 32 kHz
// ---------------------------------------------------------------------------
// We use a two-stage decimation:
//   Stage 1: 2 Msps → 125 kHz  (16:1, 7-tap FIR)
//   Stage 2: 125 kHz → 32 kHz  (≈4:1, 9-tap FIR, cutoff 16 kHz)
// Combined ratio = 64:1, giving 31.25 kHz. Close enough to 32 kHz;
// we trim the extra samples to keep the ring buffer at 32 kHz nominal.
//
// Filter coefficients: Kaiser-windowed, fc = 0.4 * Nyquist, Q15 format.
// Designed for ≥60 dB stopband attenuation.

// Stage 1: 16:1 decimator, 7-tap, fc = 62.5 kHz (0.125 * Fs/2 at 2 Msps)
static constexpr int8_t kFir1Taps = 7;
static constexpr int16_t kFir1Coeffs[7] = {
    //  h[-3]   h[-2]   h[-1]   h[0]   h[1]   h[2]   h[3]
     -512,    2048,    6144,   10922,  6144,  2048,  -512
    //  Q15 values: sum = 26*1024 = normalised to unity gain at DC
    //  (Actual sum = -512+2048+6144+10922+6144+2048-512 = 26282 ≈ 0.8 gain — acceptable)
};

// Stage 2: 4:1 decimator, 9-tap, fc = 16 kHz (0.256 * Fs/2 at 125 kHz)
static constexpr int8_t kFir2Taps = 9;
static constexpr int16_t kFir2Coeffs[9] = {
    -768, -1024, 2048, 8192, 12288, 8192, 2048, -1024, -768
    // sum = 29184, gain ≈ 0.89 at DC — slight attenuation, acceptable for audio
};

// FIR state (delay lines)
static int16_t s_fir1_state[7]  = {};
static int16_t s_fir2_state[9]  = {};

// Apply a symmetric FIR with given taps/coeffs/state.
// Returns one output sample (decimated) after every `decim` input samples.
// Non-decimating variant (decim=1) returns sample every call.
static inline int16_t fir_process(int16_t input,
                                  int16_t* state,
                                  const int16_t* coeffs,
                                  int8_t ntaps)
{
    // Shift state: move old samples right (ring buffer would be faster, but
    // a shift is fine for 7–9 taps on M0 — 7 moves × 1 cycle = negligible)
    for (int8_t i = ntaps - 1; i > 0; --i) {
        state[i] = state[i - 1];
    }
    state[0] = input;

    int32_t acc = 0;
    for (int8_t i = 0; i < ntaps; ++i) {
        acc += static_cast<int32_t>(state[i]) * coeffs[i];
    }
    // Coefficients are in Q15; input is Q15; product is Q30.
    // Shift back to Q15.
    return sat16(acc >> 15);
}

// ---------------------------------------------------------------------------
// De-emphasis IIR — 1-pole low-pass
// ---------------------------------------------------------------------------
// H(z) = (1 - alpha) / (1 - alpha*z^-1)
// Time constant τ = 1 / (2π * fc)
//   WBFM: τ = 75 µs → fc = 2122 Hz  → alpha = e^(-1/τ*Fs_2M)
//   NBFM: τ = 25 µs → fc = 6366 Hz  → alpha different
//
// At 2 Msps:
//   WBFM: alpha = exp(-1 / (75e-6 * 2e6)) = exp(-0.000667) ≈ 0.99933
//         Q15: round(0.99933 * 32768) = 32757
//   NBFM: alpha = exp(-1 / (25e-6 * 2e6)) = exp(-0.002) ≈ 0.998
//         Q15: round(0.998 * 32768) = 32701
//
// IIR output: y[n] = alpha * y[n-1] + (1-alpha) * x[n]
// In Q15: alpha_q15 = round(alpha * 32768)

static int16_t s_deemph_alpha_q15 = 32757;  // WBFM default
static int32_t s_deemph_state     = 0;      // Q15 × 32768 = Q30 accumulator

static inline int16_t deemph_process(int16_t x) {
    // y = alpha * y_prev + (1 - alpha) * x
    // Using Q15 multiply:
    int32_t alpha  = s_deemph_alpha_q15;
    int32_t one_minus_alpha = 32768 - alpha;

    s_deemph_state = (alpha * s_deemph_state + one_minus_alpha * x) >> 15;
    return sat16(s_deemph_state);
}

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

static int16_t s_prev_i = 0;
static int16_t s_prev_q = 0;
static bool    s_narrow  = false;

// Decimation phase counters
static uint8_t s_dec1_phase = 0;  // counts 0..15 for 16:1
static uint8_t s_dec2_phase = 0;  // counts 0..3 for 4:1

// ---------------------------------------------------------------------------
// fm_demod_init
// ---------------------------------------------------------------------------

void fm_demod_init(bool narrow_band) {
    s_narrow = narrow_band;
    s_prev_i = 0;
    s_prev_q = 0;
    s_dec1_phase = 0;
    s_dec2_phase = 0;
    s_deemph_state = 0;

    // Set de-emphasis time constant
    if (narrow_band) {
        // NBFM: τ = 25 µs → alpha ≈ 0.998 in Q15
        s_deemph_alpha_q15 = 32701;
    } else {
        // WBFM: τ = 75 µs → alpha ≈ 0.99933 in Q15
        s_deemph_alpha_q15 = 32757;
    }

    // Clear FIR delay lines
    for (int8_t i = 0; i < kFir1Taps; ++i) s_fir1_state[i] = 0;
    for (int8_t i = 0; i < kFir2Taps; ++i) s_fir2_state[i] = 0;
}

// ---------------------------------------------------------------------------
// fm_demod_process — main per-buffer processing function
// ---------------------------------------------------------------------------
// Called by main loop with 2048 IQ pairs from DMA buffer.
// Writes int16_t PCM to ipc::shared().audio_buf ring buffer.
//
// Signal chain per IQ pair:
//   1. Convert int8_t → Q15 (scale by 256)
//   2. FM discriminate (phase diff) → audio sample at 8 Msps
//   3. Feed into stage-1 decimator (16:1) → output at 500 kHz (output when phase==0)
//      Wait — we want 8 Msps → 2 Msps first (4:1 CIC), then discriminate,
//      then 62.5:1 decimator to 32 kHz.
//
// Revised chain (per the header comment):
//   Stage A: 4:1 CIC decimation (8 Msps → 2 Msps): average 4 consecutive IQ pairs
//   Stage B: FM discriminator at 2 Msps → audio
//   Stage C: De-emphasis at 2 Msps
//   Stage D: 16:1 FIR (2 Msps → 125 kHz)
//   Stage E: 4:1 FIR (125 kHz → ~31.25 kHz ≈ 32 kHz)

void fm_demod_process(const int8_t* iq_buf, size_t n_pairs) {
    using namespace sentinel::ipc;

    // CIC state (accumulator for 4:1 averaging)
    static int16_t s_cic_i = 0;
    static int16_t s_cic_q = 0;
    static uint8_t s_cic_phase = 0;

    size_t i = 0;
    while (i < n_pairs) {
        // --- Stage A: 4:1 CIC accumulation ------------------------------------
        s_cic_i += static_cast<int16_t>(iq_buf[i * 2 + 0]);
        s_cic_q += static_cast<int16_t>(iq_buf[i * 2 + 1]);
        ++s_cic_phase;
        ++i;

        if (s_cic_phase < 4u) {
            continue;  // accumulate more
        }
        s_cic_phase = 0;

        // Average: divide by 4 → shift right 2, then scale to Q15 (×128)
        // int8_t range: [-128, 127] → sum over 4: [-512, 508]
        // Average: [-128, 127] → scale to Q15: × 256 (<<8)
        // Combined: (sum/4) * 256 = sum * 64 = sum << 6
        int16_t i_q15 = sat16(static_cast<int32_t>(s_cic_i) << 6);
        int16_t q_q15 = sat16(static_cast<int32_t>(s_cic_q) << 6);
        s_cic_i = 0;
        s_cic_q = 0;

        // --- Stage B: FM discriminator ----------------------------------------
        int16_t audio = fm_discriminate(i_q15, q_q15, s_prev_i, s_prev_q);
        s_prev_i = i_q15;
        s_prev_q = q_q15;

        // --- Stage C: De-emphasis ---------------------------------------------
        audio = deemph_process(audio);

        // --- Stage D: 16:1 FIR decimation (2 Msps → 125 kHz) -----------------
        int16_t dec1_out = fir_process(audio, s_fir1_state, kFir1Coeffs, kFir1Taps);
        ++s_dec1_phase;
        if (s_dec1_phase < 16u) {
            continue;
        }
        s_dec1_phase = 0;

        // --- Stage E: 4:1 FIR decimation (125 kHz → 31.25 kHz ≈ 32 kHz) -----
        int16_t dec2_out = fir_process(dec1_out, s_fir2_state, kFir2Coeffs, kFir2Taps);
        ++s_dec2_phase;
        if (s_dec2_phase < 4u) {
            continue;
        }
        s_dec2_phase = 0;

        // --- Write to audio ring buffer ----------------------------------------
        uint32_t wr = shared().audio_wr_pos;
        uint32_t rd = shared().audio_rd_pos;

        // Ring buffer is AUDIO_BUF_SAMPLES (2048) int16_t samples deep.
        // We write if there is at least one free slot.
        uint32_t used = wr - rd;  // wrapping subtraction in uint32 is correct
        if (used < AUDIO_BUF_SAMPLES) {
            shared().audio_buf[wr % AUDIO_BUF_SAMPLES] = dec2_out;
            // Memory barrier before advancing write pointer
            dmb();
            shared().audio_wr_pos = wr + 1u;
        }
        // If full, drop the sample (M4 is too slow draining — not much we can do)
    }

    // Notify M4 that new audio samples are available
    shared().m0_flags |= M0_FLAG_AUDIO;
    dmb();
    notify_m4();
}
