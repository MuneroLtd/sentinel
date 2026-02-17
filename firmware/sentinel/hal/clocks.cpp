// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 Clock Initialization
//
// Target: PLL1 locked to 204 MHz using 12 MHz IRC oscillator.
//
// PLL1 output formula (direct mode, FBSEL=1):
//   Fout = Fin * (MSEL + 1)
//   Fin  = IRC = 12 MHz
//   MSEL = 16  → Fout = 12 * 17 = 204 MHz
//
// Sequence (per UM10503 §12.4.2):
//  1. Switch BASE_M4_CLK to safe IRC
//  2. Configure PLL1 (still powered down / not yet locked)
//  3. Power up PLL1 and wait for lock
//  4. Switch BASE_M4_CLK to PLL1
//  5. Enable branch clocks for each peripheral

#include "clocks.hpp"
#include "lpc4320_regs.hpp"

// ---------------------------------------------------------------------------
// Local constants
// ---------------------------------------------------------------------------

static constexpr uint32_t IRC_FREQ_HZ  = 12000000u;
static constexpr uint32_t CPU_FREQ_HZ  = 204000000u;
static constexpr uint32_t PCLK_FREQ_HZ = CPU_FREQ_HZ / 2u;  // peripheral APB divider = 2

// PLL1 CTRL value for 204 MHz direct mode:
//   DIRECT = 1 (bypass output divider)
//   FBSEL  = 1 (feedback from output, not from output divider)
//   MSEL   = 16 → bits [31:24] = 0x10
//   PSEL   = 0  → bits [13:12] = 0 (P=1, only matters if DIRECT=0)
//   NSEL   = 0  → bits [23:22] = 0 (N=1 pre-divider)
//   CLK SRC = IRC = 0x01 → bits [28:24] in BASE_CLK, not in CTRL (src lives in BASE_CLK)
//
// PLL1_CTRL:
//   [0]    PD     = 0  (powered up)
//   [1]    BYPASS = 0  (output is PLL)
//   [6]    FBSEL  = 1
//   [7]    DIRECT = 1
//   [13:12] PSEL  = 0
//   [23:22] NSEL  = 0
//   [31:24] MSEL  = 16 (0x10)
static constexpr uint32_t PLL1_CTRL_204MHZ =
    CGU_PLL1_CTRL_FBSEL |   // bit 6
    CGU_PLL1_CTRL_DIRECT |  // bit 7
    (0x10u << 24);           // MSEL = 16

// ---------------------------------------------------------------------------
// Simple busy-wait loop (~1 µs per iteration at slow clocks — generous enough)
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
    // -------- Step 1: Switch M4 base clock to safe IRC -----------------------
    // Make sure we're on IRC before touching PLL1.
    // BASE_M4_CLK: AUTOBLOCK=1, CLK_SRC = IRC (0x01)
    *CGU_BASE_M4_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_IRC;
    busy_wait(1000);

    // -------- Step 2: Configure and power up PLL1 ----------------------------
    // First write with PD=1 (powered down) to configure safely
    *CGU_PLL1_CTRL = PLL1_CTRL_204MHZ | CGU_PLL1_CTRL_PD;
    busy_wait(100);

    // Now power up PLL1 (clear PD bit)
    *CGU_PLL1_CTRL = PLL1_CTRL_204MHZ;

    // -------- Step 3: Wait for PLL1 lock -------------------------------------
    // Poll STAT[0] (LOCK bit). Should lock within ~300 µs at 12 MHz IRC.
    uint32_t timeout = 100000u;
    while ((*CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK) == 0u) {
        if (--timeout == 0u) {
            // PLL failed to lock — stay on IRC (safe fallback)
            return;
        }
    }

    // -------- Step 4: Switch BASE_M4_CLK to PLL1 ----------------------------
    // AUTOBLOCK ensures glitch-free transition.
    *CGU_BASE_M4_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;
    busy_wait(100);

    // -------- Step 5: Set up BASE_PERIPH_CLK (for peripheral clocking) ------
    // Use PLL1 divided by IDIVB = 2 → 102 MHz for peripheral base.
    // IDIVB_CTRL: source = PLL1, divisor = 2-1 = 1 (bits [5:2])
    *CGU_IDIVB_CTRL = CGU_BASE_CLK_AUTOBLOCK | (0x1u << 2) | CGU_CLK_SRC_PLL1;

    // APB1 and APB3 clocked from PLL1 directly (many peripherals have internal /2)
    *CGU_BASE_APB1_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;
    *CGU_BASE_APB3_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;

    // SSP clocks — from PLL1
    *CGU_BASE_SSP0_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;
    *CGU_BASE_SSP1_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;

    // UART0 clock — from PLL1
    *CGU_BASE_UART0_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;

    // -------- Step 6: Enable CCU branch clocks -------------------------------
    // Each branch: set RUN | AUTO bits so the clock is always running.

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

    // -------- Step 7: SGPIO peripheral clock (for baseband IQ capture) --------
    // BASE_PERIPH_CLK drives SGPIO — source from PLL1 (204 MHz)
    *CGU_BASE_PERIPH_CLK = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;

    // SGPIO branch clock (CCU1)
    *CCU1_CLK_PERIPH_SGPIO_CFG = CCU_CLK_RUN | CCU_CLK_AUTO;

    // Short settling wait
    busy_wait(500);
}

// ---------------------------------------------------------------------------
uint32_t clocks_get_cpu_hz() {
    return CPU_FREQ_HZ;
}

uint32_t clocks_get_pclk_hz() {
    return PCLK_FREQ_HZ;
}
