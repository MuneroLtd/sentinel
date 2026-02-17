// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 Clock Initialization
// Sets up PLL1 for 204 MHz M4 core and enables peripheral branch clocks.
// Call clocks_init() once at the very beginning of sentinel_main().

#pragma once
#include <cstdint>

// Initialize all clocks.
// After return: M4 core running at 204 MHz from PLL1, all used peripherals clocked.
void clocks_init();

// Returns the current M4 core clock frequency (Hz).
// Always returns 204000000 after clocks_init().
uint32_t clocks_get_cpu_hz();

// Returns the peripheral APB clock frequency (Hz) — same as CPU clock / 2 = 102 MHz.
uint32_t clocks_get_pclk_hz();
