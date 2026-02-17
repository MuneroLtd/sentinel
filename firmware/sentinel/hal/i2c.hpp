// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 I2C Master Driver
//
// Supports I2C0 (bus=0, on APB1) and I2C1 (bus=1, on APB3).
// All transfers are interrupt-free (polling state machine on STAT register).
// 7-bit addressing only.

#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdbool>

// Initialize an I2C bus.
//   bus      : 0 = I2C0, 1 = I2C1
//   freq_hz  : 100000 or 400000 (other values are clamped to nearest supported)
// Returns true on success.
bool i2c_init(uint8_t bus, uint32_t freq_hz);

// Write 'len' bytes from 'data' to slave at 7-bit address 'addr7'.
// Returns true if slave ACKed address and all data bytes.
bool i2c_write(uint8_t bus, uint8_t addr7, const uint8_t* data, size_t len);

// Read 'len' bytes from slave at 7-bit address 'addr7' into 'data'.
// Returns true if slave ACKed address.
bool i2c_read(uint8_t bus, uint8_t addr7, uint8_t* data, size_t len);

// Combined write-then-repeated-start-read transaction.
//   Writes wr_len bytes from wr, then issues repeated START and reads rd_len bytes into rd.
// Returns true if all phases succeeded.
bool i2c_write_read(uint8_t bus, uint8_t addr7,
                    const uint8_t* wr, size_t wr_len,
                    uint8_t*       rd, size_t rd_len);
