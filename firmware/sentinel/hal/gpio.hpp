// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 GPIO Driver
// Thin register-level GPIO and pin-mux helpers.
// SCU pin-mux must be configured before GPIO functions are meaningful.

#pragma once
#include <cstdint>
#include <cstdbool>

// ---------------------------------------------------------------------------
// SCU pin-mux configuration
// ---------------------------------------------------------------------------

// Set the function and electrical mode of an SCU pin.
//   group : SCU group number (0..F hex / 0..15 decimal for the main groups)
//   pin   : pin number within group (0..31)
//   func  : function select (0..7), selects which peripheral drives/reads the pin
//   mode  : pull resistor + input buffer flags, OR of SCU_MODE_* constants
void scu_set_pinmode(uint8_t group, uint8_t pin, uint8_t func, uint32_t mode);

// ---------------------------------------------------------------------------
// GPIO direction and value
// ---------------------------------------------------------------------------

// Set pin direction: output = true, input = false.
void gpio_set_dir(uint8_t port, uint8_t pin, bool output);

// Drive a GPIO output high (high=true) or low (high=false).
// Behaviour is undefined if the pin is configured as input.
void gpio_write(uint8_t port, uint8_t pin, bool high);

// Read the current logic level of a GPIO pin (works for both input and output).
bool gpio_read(uint8_t port, uint8_t pin);

// Toggle the output state of a GPIO pin.
void gpio_toggle(uint8_t port, uint8_t pin);

// ---------------------------------------------------------------------------
// Convenience: configure and assert/deassert a chip-select (active-low) pin.
// ---------------------------------------------------------------------------
inline void gpio_cs_high(uint8_t port, uint8_t pin) { gpio_write(port, pin, true);  }
inline void gpio_cs_low (uint8_t port, uint8_t pin) { gpio_write(port, pin, false); }
