// SPDX-License-Identifier: MIT
// Project Sentinel — Si5351C Clock Generator Driver
//
// Si5351C: I2C clock synthesizer at address 0x60.
// Generates reference clocks for RFFC5072 (CLK0 ~40 MHz) and
// MAX2837 (CLK1 = 26 MHz), with CLK2 spare.
//
// Reference: Skyworks AN619, Si5351A/B/C datasheet.
// Integer-only arithmetic throughout.

#pragma once
#include <cstdint>
#include <cstdbool>

// ---------------------------------------------------------------------------
// Clock output indices
// ---------------------------------------------------------------------------
static constexpr uint8_t SI5351_CLK0 = 0u;  // RFFC5072 reference (~40 MHz)
static constexpr uint8_t SI5351_CLK1 = 1u;  // MAX2837 reference  (26 MHz)
static constexpr uint8_t SI5351_CLK2 = 2u;  // Spare / audio

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialize the Si5351C.
//   - Verifies the device is present on I2C.
//   - Configures PLLA and PLLB from the 25 MHz XTAL reference.
//   - Sets default frequencies for CLK0 (40 MHz) and CLK1 (26 MHz).
//   - Disables all other outputs.
// Returns true on success.
bool si5351_init();

// Set the output frequency of a single CLK output (0–2).
//   freq_hz : desired frequency in Hz (500 kHz – 200 MHz).
// The function selects a PLL (PLLA for CLK0/CLK2, PLLB for CLK1),
// computes PLL and MultiSynth dividers, and writes the registers.
// Returns true on success.
bool si5351_set_freq(uint8_t clk_output, uint32_t freq_hz);

// Enable or disable a CLK output driver.
//   clk_output : 0, 1, or 2
//   enable     : true = powered-up, false = powered-down
void si5351_enable(uint8_t clk_output, bool enable);
