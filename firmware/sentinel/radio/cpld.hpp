// SPDX-License-Identifier: MIT
// Project Sentinel — CPLD (Xilinx XC2C64A) Interface Driver
//
// The XC2C64A CPLD on HackRF One controls:
//   - RF antenna port switching (ANT_BIAS, ANT_SEL)
//   - RX/TX path switching (SKY13317/SKY13351 RF switches)
//   - LNA bypass control
//   - SGPIO → MAX5864 data routing
//
// Programming:
//   The CPLD is configured at boot via JTAG bit-banging from the LPC4320.
//   JTAG pins are GPIO-driven (bit-bang); the XSVF bitstream is embedded
//   in the firmware image.  The XSVF execution follows the Xilinx XAPP058
//   state machine (TAP controller states).
//
// Runtime control:
//   After programming, the CPLD exposes a small set of control bits via
//   SGPIO or dedicated control lines.  In HackRF, the CPLD sgpio_if design
//   takes direction from the LPC4320 SGPIO peripheral.  For Project Sentinel
//   we provide the JTAG programming layer and a thin runtime API.
//
// References:
//   - HackRF firmware/common/cpld_jtag.c (greatscottgadgets/hackrf)
//   - Xilinx XAPP503 (SVF/XSVF format)
//   - XC2C64A datasheet

#pragma once
#include <cstdint>
#include <cstdbool>

// ---------------------------------------------------------------------------
// CPLD JTAG programming
// ---------------------------------------------------------------------------

// Program the CPLD from an XSVF bitstream buffer.
//   data   : pointer to XSVF byte stream (embedded in firmware)
//   length : number of bytes in the buffer
// Bit-bangs the JTAG TAP state machine via GPIO.
// Returns true if programming completed without error.
bool cpld_program(const uint8_t* data, uint32_t length);

// ---------------------------------------------------------------------------
// Runtime RF switch control (post-programming)
// ---------------------------------------------------------------------------

// Initialize CPLD interface: configure JTAG GPIOs, program the CPLD with
// the embedded bitstream, and set the initial RF path to RX.
// Returns true on success.
bool cpld_init();

// Configure RF switch network for receive:
//   - Enable ANT port
//   - Route antenna → LNA → mixer
//   - Disable TX PA path
void cpld_set_rx();

// Configure RF switch network for transmit:
//   - Enable TX PA path
//   - Route DAC → PA → ANT port
//   - Gate LNA input
void cpld_set_tx();

// Disable all RF paths (safe state during reconfiguration).
void cpld_rf_off();
