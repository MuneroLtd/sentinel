// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 GPIO Driver Implementation

#include "gpio.hpp"
#include "lpc4320_regs.hpp"

// ---------------------------------------------------------------------------
// SCU pin-mux
// ---------------------------------------------------------------------------

void scu_set_pinmode(uint8_t group, uint8_t pin, uint8_t func, uint32_t mode) {
    volatile uint32_t* reg = scu_sfs(group, pin);
    // Clear function and mode bits, then apply new values.
    // Function is bits [2:0]; mode flags are combined in 'mode'.
    *reg = (func & 0x07u) | mode;
}

// ---------------------------------------------------------------------------
// GPIO direction
// ---------------------------------------------------------------------------

void gpio_set_dir(uint8_t port, uint8_t pin, bool output) {
    const uint32_t mask = (1u << pin);
    if (output) {
        LPC_GPIO->DIR[port] |= mask;
    } else {
        LPC_GPIO->DIR[port] &= ~mask;
    }
}

// ---------------------------------------------------------------------------
// GPIO write
// ---------------------------------------------------------------------------

void gpio_write(uint8_t port, uint8_t pin, bool high) {
    const uint32_t mask = (1u << pin);
    if (high) {
        LPC_GPIO->SET[port] = mask;
    } else {
        LPC_GPIO->CLR[port] = mask;
    }
}

// ---------------------------------------------------------------------------
// GPIO read
// ---------------------------------------------------------------------------

bool gpio_read(uint8_t port, uint8_t pin) {
    return (LPC_GPIO->PIN[port] & (1u << pin)) != 0u;
}

// ---------------------------------------------------------------------------
// GPIO toggle
// ---------------------------------------------------------------------------

void gpio_toggle(uint8_t port, uint8_t pin) {
    LPC_GPIO->NOT[port] = (1u << pin);
}
