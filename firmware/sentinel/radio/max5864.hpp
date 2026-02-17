// SPDX-License-Identifier: MIT
// Project Sentinel — MAX5864 Baseband ADC/DAC Driver
//
// MAX5864: Ultra-low-power baseband ADC/DAC for HackRF One.
//   - Dual 8-bit RX ADCs + dual 10-bit TX DACs.
//   - SPI 3-wire control interface (single byte command).
//   - IQ sample stream flows directly via LPC4320 SGPIO peripheral with DMA.
//     This driver only manages the control/mode register.
//
// SPI format: single byte transferred to the device selects operating mode.
//   The device has no readable registers.
//
// Mode byte values (Table 3, MAX5864 datasheet):
//   0x00 = Shutdown   (REF off, CLK off, ADC off, DAC off)
//   0x01 = Idle       (REF on, CLK on, ADC off, DAC off)
//   0x02 = RX         (REF on, CLK on, ADC on, DAC off)  ← default RX
//   0x03 = TX         (REF on, CLK on, ADC off, DAC on)
//   0x04 = Transceiver(REF on, CLK on, ADC on, DAC on)   ← simultaneous FDD
//   0x05 = Standby    (REF on, CLK off, ADC off, DAC off)
//
// Reference: MAX5864 datasheet (Analog Devices/Maxim).

#pragma once
#include <cstdint>
#include <cstdbool>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialize MAX5864: configure GPIO CS, put device into RX mode.
// Returns true on success.
bool max5864_init();

// Switch to ADC-active receive mode (ADCs on, DACs off).
void max5864_rx_mode();

// Switch to DAC-active transmit mode (DACs on, ADCs off).
void max5864_tx_mode();

// Transceiver mode (ADCs and DACs simultaneously active — FDD).
void max5864_xcvr_mode();

// Idle mode: reference and clock active, converters off.
void max5864_idle();

// Standby: reference on, clock off, converters off (fastest wakeup).
void max5864_standby();

// Full shutdown: all circuits off, minimum power.
void max5864_shutdown();
