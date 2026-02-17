// SPDX-License-Identifier: MIT
// Project Sentinel — RFFC5072 Wideband Mixer/Synthesizer Driver
//
// RFFC5072: SPI wideband mixer + fractional-N synthesizer.
// LO frequency range: 85 MHz – 4200 MHz.
// RF conversion range: 30 MHz – 6000 MHz.
//
// SPI format: 3-wire, two 16-bit words per transaction.
//   Word 0: [7:0] = command byte (bit7=R/W, bits[6:0]=register address)
//           [15:8] = unused (send 0)
//   Word 1: [15:0] = register data
// CS active-low.
//
// In HackRF, RFFC5072 reference clock = 40 MHz (from Si5351C CLK0).
// IF output feeds MAX2837 operating at ~2600 MHz.
// Frequency plan for RX:
//   IF  = 2600 MHz (fixed MAX2837 centre)
//   LO  = RF + IF  (for low-side injection: LO > RF → image below RF)
//       = RF + 2600 MHz   when RF < 2300 MHz
//
// Reference: RFFC5071/5072 datasheet (Qorvo), HackRF rffc5071.c

#pragma once
#include <cstdint>
#include <cstdbool>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialize the RFFC5072.
//   - Resets register state to known defaults.
//   - Configures for RX mode with LO powered up.
//   - Sets RFFC5072 to standby (LO disabled) until tune is called.
// Returns true on success.
bool rffc5072_init();

// Set the LO output frequency.
//   freq_hz: desired LO frequency in Hz (85 MHz – 4200 MHz).
// Computes and writes the N, fractional, and LO-divider registers.
// Returns true on success (PLL lock not verified here — use lock-detect GPIO).
bool rffc5072_set_lo_freq(uint64_t freq_hz);

// Configure signal path for receive (MIX_SEL for RX, correct port routing).
void rffc5072_rx_mode();

// Configure signal path for transmit.
void rffc5072_tx_mode();

// Put RFFC5072 into low-power standby (PLL off, outputs disabled).
void rffc5072_standby();

// Re-enable after standby (restores last configuration).
void rffc5072_enable();
