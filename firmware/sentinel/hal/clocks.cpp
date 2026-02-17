// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 Clock Initialization
//
// IMPORTANT: We execute from SPIFI flash (XIP at 0x00000000).
// The LPC4320 boot ROM has already configured PLL1 and the clock tree:
//
//   PLL1:           IRC × 24 = 288 MHz
//   IDIVB:          PLL1 / 9 = 32 MHz  → BASE_SPIFI_CLK
//   IDIVC:          PLL1 / 3 = 96 MHz  → BASE_M4_CLK
//
// We MUST NOT power-down or reconfigure PLL1 while SPIFI is sourced from it,
// or the SPIFI clock dies and the CPU can't fetch the next instruction.
//
// Strategy: leave PLL1 and the M4/SPIFI clocks alone. Just set up peripheral
// base clocks (SSP, UART, etc.) and enable CCU branch clocks. This gives us
// a stable 96 MHz M4 with working SPIFI from the boot ROM configuration.
//
// We optionally boost SPIFI to 96 MHz (PLL1/3 via IDIVB) once peripheral
// clocks are set up, since the W25Q80BV supports up to 104 MHz.

#include "clocks.hpp"
#include "lpc4320_regs.hpp"

// ---------------------------------------------------------------------------
// Local constants
// ---------------------------------------------------------------------------

// Boot ROM leaves M4 on IDIVC = PLL1/3 = 96 MHz
static constexpr uint32_t CPU_FREQ_HZ  = 96000000u;
static constexpr uint32_t PCLK_FREQ_HZ = CPU_FREQ_HZ;  // APB runs at CPU speed on LPC43xx

// ---------------------------------------------------------------------------
// Simple busy-wait loop
// ---------------------------------------------------------------------------
static void busy_wait(volatile uint32_t count) {
    while (count--) {
        __asm volatile("nop");
    }
}

// ---------------------------------------------------------------------------
// clocks_init
// ---------------------------------------------------------------------------
void clocks_init() {
    // -------- Step 1: Peripheral base clocks ----------------------------------
    // Source peripheral clocks from IDIVC (PLL1/3 = 96 MHz), same as M4 core.
    // This keeps all peripheral baud/clock calculations consistent with
    // clocks_get_pclk_hz() = 96 MHz.
    //
    // We do NOT touch BASE_M4_CLK or BASE_SPIFI_CLK — the boot ROM set
    // those up correctly and we're executing from SPIFI.

    // APB1 and APB3
    *CGU_BASE_APB1_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IDIVC;
    *CGU_BASE_APB3_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IDIVC;

    // SSP0 and SSP1 (shared SPI bus for LCD, RF ICs)
    *CGU_BASE_SSP0_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IDIVC;
    *CGU_BASE_SSP1_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IDIVC;

    // UART0 (debug console)
    *CGU_BASE_UART0_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IDIVC;

    // SGPIO peripheral clock (for baseband IQ capture)
    *CGU_BASE_PERIPH_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IDIVC;

    busy_wait(100);

    // -------- Step 2: Enable CCU branch clocks --------------------------------
    // Each branch: set RUN | AUTO bits.

    // GPIO (CCU1)
    *CCU1_CLK_M4_GPIO_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // SSP0 (CCU2 APB0)
    *CCU2_CLK_APB0_SSP0_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // SSP1 (CCU2 APB2)
    *CCU2_CLK_APB2_SSP1_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // UART0 (CCU2 APB0)
    *CCU2_CLK_APB0_UART0_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // I2C0 (CCU1 APB1)
    *CCU1_CLK_APB1_I2C0_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // I2C1 (CCU2 APB1)
    *CCU2_CLK_APB1_I2C1_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // SGPIO branch clock (CCU1)
    *CCU1_CLK_PERIPH_SGPIO_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // -------- Step 3: SPIFI clock left at boot ROM default ---------------------
    // Boot ROM runs SPIFI at 32 MHz (IDIVB = PLL1/9) with standard SPI read.
    // Boosting requires reconfiguring SPIFI controller for fast/quad read mode
    // with correct dummy cycles. Leave at 32 MHz for stability; optimize later.

    busy_wait(200);
}

// ---------------------------------------------------------------------------
uint32_t clocks_get_cpu_hz() {
    return CPU_FREQ_HZ;
}

uint32_t clocks_get_pclk_hz() {
    return PCLK_FREQ_HZ;
}
