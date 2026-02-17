// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/adsb.hpp — ADS-B 1090ES receiver and frame decoder
//
// Receives Mode-S / 1090ES Extended Squitter messages.
// Signal: OOK with PPM encoding at 1 Mbps on 1090 MHz.
// Requires 8 Msps sample rate for accurate preamble detection (8 samples/µs).
//
// Output: AdsbFrame structures written to ipc::shared().adsb_frames[] ring buffer.

#pragma once

#include <cstdint>
#include <cstddef>

/// Initialise the ADS-B decoder (clear state and ring buffer).
void adsb_init();

/// Process one DMA buffer of IQ samples at 8 Msps.
/// Searches for ADS-B preambles, decodes frames, validates CRC24, and
/// writes valid frames to ipc::shared().adsb_frames[].
/// @param iq_buf   Pointer to interleaved int8_t I,Q pairs
/// @param n_pairs  Number of IQ pairs (DMA_BUF_SAMPLES = 2048)
void adsb_process(const int8_t* iq_buf, size_t n_pairs);

/// CRC24 frame check using polynomial 0xFFF409.
/// @param frame  Pointer to frame bytes (up to 14 bytes for long squitter)
/// @param len    Number of bytes to check (should be 14 for DF17/18/19)
/// @return true if CRC passes (last 3 bytes are the CRC residue)
bool adsb_check_crc(const uint8_t* frame, size_t len);

/// Fast magnitude approximation: |I| + |Q| (L1 norm), clamped to [0, 255].
/// Avoids sqrt; good enough for preamble detection and RSSI estimation.
/// @param i  I sample (int8_t, range -128..127)
/// @param q  Q sample (int8_t, range -128..127)
/// @return   Approximate magnitude [0, 255]
uint8_t adsb_get_magnitude(int8_t i, int8_t q);
