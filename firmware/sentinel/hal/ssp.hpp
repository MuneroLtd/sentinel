// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 SSP (SPI) Master Driver
//
// Supports SSP0 (bus=0) and SSP1 (bus=1).
// Chip-select is managed externally via GPIO — not driven here.
// All transfers are blocking (polling on FIFO/busy flags).

#pragma once
#include <cstdint>
#include <cstddef>

// Initialize an SSP bus as SPI master.
//   bus       : 0 = SSP0, 1 = SSP1
//   clock_hz  : desired SCK frequency in Hz (rounded down to nearest achievable)
//   cpol      : clock polarity (0 or 1)
//   cpha      : clock phase    (0 or 1)
// Note: caller must have already configured SCU pin-mux for the SSP pins.
void ssp_init(uint8_t bus, uint32_t clock_hz, uint8_t cpol, uint8_t cpha);

// Transfer one byte (full-duplex).  Returns the received byte.
uint8_t ssp_transfer(uint8_t bus, uint8_t byte);

// Write 'len' bytes from 'data'. Received bytes are discarded.
void ssp_write(uint8_t bus, const uint8_t* data, size_t len);

// Read 'len' bytes into 'data'. 0xFF is clocked out as MOSI.
void ssp_read(uint8_t bus, uint8_t* data, size_t len);

// Full-duplex transfer: transmit from 'tx', receive into 'rx', both 'len' bytes.
void ssp_write_read(uint8_t bus, const uint8_t* tx, uint8_t* rx, size_t len);

// Write a 16-bit word (for ILI9341 which uses 8-bit transfers but back-to-back).
// Equivalent to two ssp_transfer calls but with optimised FIFO usage.
void ssp_write16(uint8_t bus, uint16_t word);
