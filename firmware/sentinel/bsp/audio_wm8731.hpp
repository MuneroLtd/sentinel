// SPDX-License-Identifier: MIT
// Project Sentinel — WM8731 Audio Codec Driver
//
// The WM8731 is configured as I2S master, 32 kHz, 16-bit stereo (mono duplicated).
// Control registers are written via I2C0 (address 0x1A, 9-bit register protocol).
// Audio sample data flows via the LPC4320 I2S0 peripheral TX FIFO.
//
// Typical usage:
//   audio_init();              // called once after i2c_init(0, 100000)
//   audio_play_samples(buf, n); // called whenever PCM samples are available
//
// WM8731 register write protocol:
//   7-bit register address (bits 15:9) | 9-bit data (bits 8:0)
//   Sent as two bytes over I2C: {addr[6:0]<<1 | data[8], data[7:0]}

#pragma once
#include <cstdint>
#include <cstddef>

// Initialise WM8731 via I2C and configure LPC4320 I2S0 peripheral.
// Call after clocks_init() and i2c_init(0, 100000).
// Returns true on success (I2C ACKed all register writes).
bool audio_init();

// Write 'count' 16-bit signed mono samples to the I2S TX FIFO.
// Each sample is duplicated to both left and right channels.
// Blocks until all samples have been loaded into the FIFO.
void audio_play_samples(const int16_t* samples, size_t count);

// Set headphone output volume.
//   vol : 0..127  (0 = mute, 127 = +6 dB; recommended 100 for normal listening)
void audio_set_volume(uint8_t vol);

// Mute or unmute the DAC output.
void audio_set_mute(bool mute);

// --- Legacy alias kept for compatibility with other sentinel source files ---
inline bool audio_wm8731_init() { return audio_init(); }
inline void audio_feed_samples(const int16_t* s, size_t n) { audio_play_samples(s, n); }
