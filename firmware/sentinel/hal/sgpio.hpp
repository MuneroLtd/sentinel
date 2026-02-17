// SPDX-License-Identifier: MIT
// Project Sentinel — SGPIO HAL Driver
//
// Configures the LPC4320 SGPIO peripheral for 8-bit parallel IQ capture
// from the HackRF XC2C64A CPLD.
//
// Data path: MAX5864 ADC → XC2C64A CPLD → SGPIO parallel bus → GPDMA → memory
//
// The CPLD provides:
//   - 8 data bits on SGPIO0–SGPIO7 (parallel byte)
//   - External clock on SGPIO8 at the sample rate
//
// SGPIO captures 8 bits in parallel into 32-bit shadow registers.
// When a 32-bit word is complete (4 bytes shifted in), the shadow register
// swaps and a DMA request fires to move the data to SRAM.

#pragma once
#include <cstdint>

// Configure SGPIO for 8-bit parallel capture mode.
// Must be called after clocks_init() has enabled the SGPIO branch clock
// and after SGPIO pins have been muxed via SCU.
void sgpio_init();

// Configure SGPIO specifically for HackRF baseband IQ capture.
// Sets up slice concatenation for 8-bit parallel mode with external clock.
void sgpio_configure_for_baseband();

// Start SGPIO capture (enable configured slices).
// Call after sgpio_init() and before M0 release.
void sgpio_start();

// Stop SGPIO capture (disable all slices).
void sgpio_stop();

// Return true if SGPIO slices are currently enabled (capturing).
bool sgpio_is_running();
