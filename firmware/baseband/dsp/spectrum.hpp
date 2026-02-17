// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/spectrum.hpp — 256-bin FFT spectrum analyser
//
// Input:  int8_t IQ pairs at 8 Msps (from DMA buffer)
// Output: 256 int8_t dBFS values written to ipc::shared().spectrum_db[]
//         Update rate: ~4 Hz (every 250 ms at 8 Msps / 2048 samples per DMA buffer)

#pragma once

#include <cstdint>
#include <cstddef>

/// Initialise the spectrum analyser (clear state, reset frame counter).
void spectrum_init();

/// Process one DMA buffer of IQ samples.
/// Accumulates samples until a full 256-point FFT frame is ready, then
/// computes the FFT, magnitude, dBFS conversion, and writes results to
/// ipc::shared().spectrum_db[].
/// @param iq_buf   Pointer to interleaved int8_t I,Q pairs (8 Msps)
/// @param n_pairs  Number of IQ pairs (DMA_BUF_SAMPLES = 2048)
void spectrum_process(const int8_t* iq_buf, size_t n_pairs);
