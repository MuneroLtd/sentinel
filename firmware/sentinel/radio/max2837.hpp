// SPDX-License-Identifier: MIT
// Project Sentinel — MAX2837 RF Transceiver Driver
//
// MAX2837: Direct-conversion zero-IF RF transceiver, 2.3–2.7 GHz.
// Provides LNA, VGA, LPF, VCO/PLL, and IQ baseband outputs/inputs.
//
// SPI format: single 16-bit word per transaction.
//   Bit 15    : R/W (1=read, 0=write)
//   Bits[14:10]: 5-bit register address (0–31)
//   Bits[9:0] : 10-bit register data
//
// Operating frequency: 2.3 GHz – 2.7 GHz (VCO direct).
//   For HackRF RX at 2600 MHz IF (when RFFC5072 performs first downconversion).
//
// Reference: MAX2837 datasheet (Analog Devices/Maxim), HackRF max2837.c.

#pragma once
#include <cstdint>
#include <cstdbool>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialize MAX2837: power up, set default register state, enable IQ outputs.
// Returns true on success.
bool max2837_init();

// Tune the internal VCO to freq_hz (must be 2300–2700 MHz range).
// Configures LOGEN band switch, integer N, fractional dividers.
// Returns true on success.
bool max2837_set_freq(uint32_t freq_hz);

// Set LNA gain. Valid values: 0, 8, 16, 24, 32, 40 (dB).
// Returns true if gain value is valid.
bool max2837_set_lna_gain(uint8_t gain_db);

// Set baseband VGA gain. Range: 0–62 dB in 2 dB steps.
// Returns true if gain is valid (even, 0–62).
bool max2837_set_vga_gain(uint8_t gain_db);

// Enable RX signal path (LNA + IQ ADC outputs active).
void max2837_rx_mode();

// Enable TX signal path (IQ DAC inputs active, LNA disabled).
void max2837_tx_mode();

// Power down all active sections. SPI register state is preserved.
void max2837_standby();
