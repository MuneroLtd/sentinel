// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/fm_demod.hpp — FM demodulator (WBFM 200 kHz / NBFM 12.5 kHz)
//
// Input:  int8_t IQ pairs at 8 Msps (from DMA buffer)
// Output: int16_t PCM audio at 32 kHz written to ipc::shared().audio_buf
//
// All DSP is Q15 fixed-point (no float on hot paths).

#pragma once

#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Initialise (or reinitialise) the FM demodulator.
/// @param narrow_band  false = WBFM (200 kHz deviation, 75 µs de-emphasis)
///                     true  = NBFM (12.5 kHz deviation, 25 µs de-emphasis)
void fm_demod_init(bool narrow_band);

/// Process one DMA buffer of IQ samples.
/// Writes demodulated PCM audio to ipc::shared().audio_buf.
/// @param iq_buf   Pointer to interleaved int8_t I,Q pairs
/// @param n_pairs  Number of IQ pairs (DMA_BUF_SAMPLES = 2048)
void fm_demod_process(const int8_t* iq_buf, size_t n_pairs);

// ---------------------------------------------------------------------------
// Internal helpers (exposed for unit testing)
// ---------------------------------------------------------------------------

/// Fast integer atan2 approximation.
/// Returns result in Q15 format scaled to [-32768, 32767] representing [-π, π).
/// Accuracy: ±0.5° (Chebyshev polynomial approximation).
/// @param y  Q15 imaginary component
/// @param x  Q15 real component
int16_t atan2_q15(int16_t y, int16_t x);

/// FM discriminator: compute instantaneous frequency (phase derivative).
/// Uses phase-difference method: atan2(Q[n]*I[n-1] - I[n]*Q[n-1],
///                                    I[n]*I[n-1] + Q[n]*Q[n-1])
/// @param i_cur  Current I sample (Q15)
/// @param q_cur  Current Q sample (Q15)
/// @param i_prv  Previous I sample (Q15)
/// @param q_prv  Previous Q sample (Q15)
/// @return  Phase difference in Q15 (proportional to instantaneous frequency)
int16_t fm_discriminate(int16_t i_cur, int16_t q_cur,
                        int16_t i_prv, int16_t q_prv);
