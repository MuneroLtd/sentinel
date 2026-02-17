// SPDX-License-Identifier: MIT
// Project Sentinel — SGPIO HAL Driver Implementation
//
// Configures SGPIO for 8-bit parallel IQ capture from the HackRF CPLD.
//
// Architecture (per HackRF firmware / LPC43xx UM10503):
//   - 8 slices (A, I, E, J, C, K, F, L) capture SGPIO0–SGPIO7 in parallel
//   - External clock from SGPIO8 (provided by CPLD at sample rate)
//   - 32-bit shadow register exchange triggers interrupt for M0 to read data
//
// IMPORTANT: GPDMA does NOT support peripheral-to-memory for SGPIO on LPC43xx.
// Data movement is handled by M0 via SGPIO exchange interrupts.
//
// Reference: hackrf/firmware/common/sgpio.c

#include "sgpio.hpp"
#include "lpc4320_regs.hpp"

// The 8 slices used for 8-bit parallel capture, in hardware order.
// These correspond to SGPIO pins 0–7 data lines.
// Slice mapping: A=0, I=8, E=4, J=9, C=2, K=10, F=5, L=11
static constexpr uint8_t k_data_slices[] = {
    SGPIO_SLICE_A, SGPIO_SLICE_I, SGPIO_SLICE_E, SGPIO_SLICE_J,
    SGPIO_SLICE_C, SGPIO_SLICE_K, SGPIO_SLICE_F, SGPIO_SLICE_L,
};
static constexpr uint32_t k_num_data_slices = 8;

// Bitmask of all data slices for CTRL_ENABLED/CTRL_DISABLED
static constexpr uint32_t k_data_slice_mask =
    (1u << SGPIO_SLICE_A) | (1u << SGPIO_SLICE_I) |
    (1u << SGPIO_SLICE_E) | (1u << SGPIO_SLICE_J) |
    (1u << SGPIO_SLICE_C) | (1u << SGPIO_SLICE_K) |
    (1u << SGPIO_SLICE_F) | (1u << SGPIO_SLICE_L);

// ---------------------------------------------------------------------------
// sgpio_configure_for_baseband
// ---------------------------------------------------------------------------
void sgpio_configure_for_baseband() {
    SGPIO_Type* sgpio = LPC_SGPIO;

    // Disable all slices before configuration
    sgpio->CTRL_DISABLED = 0xFFFFu;

    // Clear all interrupt enables and status
    for (int i = 0; i < 3; i++) {
        sgpio->CLR_EN[i] = 0xFFFFu;
        sgpio->CLR_STATUS[i] = 0xFFFFu;
    }

    // Configure each of the 8 data slices for parallel capture
    for (uint32_t i = 0; i < k_num_data_slices; i++) {
        uint8_t s = k_data_slices[i];

        // OUT_MUX_CFG: not used for input capture, set to 0
        sgpio->OUT_MUX_CFG[s] = 0;

        // SGPIO_MUX_CFG:
        //   bit [0] = 1: external clock enable
        //   bits [3:1] = 0: CLK_SOURCE_PIN_MODE = SGPIO8 (pin 8)
        //   bits [5:4] = 0: CLK_SOURCE_SLICE_MODE = slice A
        //   bits [8:6] = 0: QUALIFIER_MODE = enable
        //   bit [11] = 0: QUALIFIER_PIN_ID = 0
        //   bits [13:12] = 0: QUALIFIER_SLICE_MODE = 0
        //   bit [14] = 1: CONCAT_ENABLE
        //   bit [15] = 1: CONCAT_ORDER (8-slice)
        sgpio->SGPIO_MUX_CFG[s] =
            (1u << 0)  |  // EXT_CLK_ENABLE
            (0u << 1)  |  // CLK_SOURCE_PIN_MODE = SGPIO8
            (0u << 4)  |  // CLK_SOURCE_SLICE_MODE = A
            (0u << 6)  |  // QUALIFIER_MODE = enable
            (0u << 9)  |  // QUALIFIER_PIN_ID
            (0u << 12) |  // QUALIFIER_SLICE_MODE
            (1u << 14) |  // CONCAT_ENABLE
            (1u << 15);   // CONCAT_ORDER = 8 slices

        // SLICE_MUX_CFG:
        //   bit [0] = 0: MATCH_MODE off
        //   bit [1] = 0: CLK_CAPTURE_MODE = edge
        //   bit [2] = 1: CLKGEN_MODE = external (from pin)
        //   bit [3] = 0: INV_OUT_CLK = no invert
        //   bit [4] = 0: DATA_CAPTURE_MODE = rising edge
        //   bits [7:6] = 3: PARALLEL_MODE = 8-bit (byte)
        //   bit [8] = 0: INV_QUALIFIER = no invert
        sgpio->SLICE_MUX_CFG[s] =
            (0u << 0)  |  // MATCH_MODE off
            (0u << 1)  |  // CLK_CAPTURE_MODE = edge
            (1u << 2)  |  // CLKGEN_MODE = external
            (0u << 3)  |  // INV_OUT_CLK = no
            (0u << 4)  |  // DATA_CAPTURE_MODE = rising
            (3u << 6)  |  // PARALLEL_MODE = 8-bit
            (0u << 8);    // INV_QUALIFIER = no

        // PRESET: not used for external clock mode
        sgpio->PRESET[s] = 0;

        // POS: shift count for 32-bit exchange
        // POS_RESET [15:8] = 0x1F (31 = 32 bits - 1)
        // POS [7:0] = 0x1F (start at 31)
        sgpio->POS[s] = (0x1Fu << 8) | 0x1Fu;

        // Clear data registers
        sgpio->REG[s] = 0;
        sgpio->REG_SS[s] = 0;
    }

    // Enable exchange interrupt for slice A only (fires once per 32-bit word).
    // All 8 slices are concatenated so they all exchange simultaneously.
    // Exchange interrupt = interrupt set 0.
    sgpio->SET_EN[0] = (1u << SGPIO_SLICE_A);
}

// ---------------------------------------------------------------------------
// sgpio_init
// ---------------------------------------------------------------------------
void sgpio_init() {
    sgpio_configure_for_baseband();
}

// ---------------------------------------------------------------------------
// sgpio_start
// ---------------------------------------------------------------------------
void sgpio_start() {
    LPC_SGPIO->CTRL_ENABLED = k_data_slice_mask;
}

// ---------------------------------------------------------------------------
// sgpio_stop
// ---------------------------------------------------------------------------
void sgpio_stop() {
    LPC_SGPIO->CTRL_DISABLED = k_data_slice_mask;
}

// ---------------------------------------------------------------------------
// sgpio_is_running
// ---------------------------------------------------------------------------
bool sgpio_is_running() {
    // Check if any data slice is enabled by reading CTRL_ENABLED
    // CTRL_ENABLED is write-only; check GPIO_OENREG as proxy or just
    // read a counter — if COUNT is changing, slices are active.
    // For simplicity, track state with a static flag.
    // (CTRL_ENABLED register is write-only per UM10503)
    return (LPC_SGPIO->COUNT[SGPIO_SLICE_A] != 0);
}
