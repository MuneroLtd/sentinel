// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/scanner.hpp — Energy-detection frequency scanner
//
// The scanner sweeps frequencies under M4 direction:
//   M4 writes cmd_freq_hz and sends Command::TUNE
//   M0 tunes RF (via IPC-mediated approach: updates shared state and
//   notifies M4 which performs the actual SPI tuning, then M0 dwells
//   for ~50 ms measuring RMS power)
//   M0 writes RSSI to shared().rssi_dbm_x10 and pushes SignalEvent to
//   shared().signals[] if above threshold.

#pragma once

#include <cstdint>
#include <cstddef>

/// Initialise scanner state (clear accumulators, reset dwell timer).
void scanner_init();

/// Process one DMA buffer of IQ samples.
/// Accumulates power over the dwell period; when 8192 samples have been
/// collected, computes mean magnitude, converts to dBm, and publishes result.
/// @param iq_buf   Pointer to interleaved int8_t I,Q pairs
/// @param n_pairs  Number of IQ pairs (DMA_BUF_SAMPLES = 2048)
void scanner_process(const int8_t* iq_buf, size_t n_pairs);

/// Convert average magnitude (|I|+|Q|, range 0-255) to dBm×10.
/// Uses a simple piecewise-linear approximation calibrated for HackRF One.
/// @param mean_magnitude  Average L1 magnitude over the measurement window
/// @return dBm×10 (e.g., -800 = -80.0 dBm)
int16_t power_to_dbm_x10(uint32_t mean_magnitude);
