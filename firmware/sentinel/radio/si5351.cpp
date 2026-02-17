// SPDX-License-Identifier: MIT
// Project Sentinel — Si5351C Clock Generator Driver Implementation
//
// Reference: Skyworks AN619 "Manually Generating an Si5351 Register Map"
// Si5351A/B/C datasheet Rev 1.0
//
// PLL architecture:
//   XTAL = 25 MHz input on XA/XB
//   PLLA  → CLK0 (RFFC5072 ref, 40 MHz) and CLK2 (spare)
//   PLLB  → CLK1 (MAX2837 ref, 26 MHz)
//
// Multisynth divider formula (AN619 Section 4):
//   fout = fvco / (a + b/c)
//   P1 = 128*a + floor(128*b/c) - 512
//   P2 = 128*b - c*floor(128*b/c)
//   P3 = c
//
// Integer-only arithmetic. No float/double.

#include "si5351.hpp"
#include "hal/i2c.hpp"
#include "bsp/portapack_pins.hpp"

// ---------------------------------------------------------------------------
// Si5351C register addresses (datasheet Table 1)
// ---------------------------------------------------------------------------
static constexpr uint8_t REG_DEVICE_STATUS   = 0u;
static constexpr uint8_t REG_INT_STATUS_STICKY = 1u;
static constexpr uint8_t REG_INT_STATUS_MASK = 2u;
static constexpr uint8_t REG_OUTPUT_ENABLE   = 3u;   // CLK output enable (active-low bits)
static constexpr uint8_t REG_OEB_PIN_ENABLE  = 9u;
static constexpr uint8_t REG_PLL_INPUT_SRC   = 15u;

// CLK control registers: 16 = CLK0, 17 = CLK1, 18 = CLK2
static constexpr uint8_t REG_CLK_BASE        = 16u;

// MultiSynth divider parameter registers (AN619 Table 9)
// Each MS occupies 8 bytes starting at:
//   MS0 = 42, MS1 = 50, MS2 = 58
static constexpr uint8_t REG_MS0_BASE        = 42u;
static constexpr uint8_t REG_MS1_BASE        = 50u;
static constexpr uint8_t REG_MS2_BASE        = 58u;

// PLL (feedback multisynth) parameter registers
//   MSNA = 26 (PLLA feedback), MSNB = 34 (PLLB feedback)
static constexpr uint8_t REG_MSNA_BASE       = 26u;
static constexpr uint8_t REG_MSNB_BASE       = 34u;

static constexpr uint8_t REG_SS_EN           = 149u; // Spread spectrum off
static constexpr uint8_t REG_CLK0_PHOFF     = 165u;
static constexpr uint8_t REG_XTAL_CL        = 183u;  // Crystal load capacitance
static constexpr uint8_t REG_PLL_RESET      = 177u;  // PLL soft-reset

// ---------------------------------------------------------------------------
// CLK control register bit definitions (REG_CLK_BASE + clk)
// ---------------------------------------------------------------------------
static constexpr uint8_t CLK_PDN           = (1u << 7); // Power down
static constexpr uint8_t CLK_INT_MODE      = (1u << 6); // Integer mode (MSx_INT)
static constexpr uint8_t CLK_PLL_SRC_B     = (1u << 5); // 0=PLLA, 1=PLLB
static constexpr uint8_t CLK_INV           = (1u << 4); // Invert output
// CLK_SRC[1:0] = bits[3:2]: 00=XTAL, 01=CLKIN, 10=VC XO, 11=MultiSynth n
static constexpr uint8_t CLK_SRC_MS        = (3u << 2); // Source = own MS divider
// CLK_IDRV[1:0] = bits[1:0]: output drive strength
static constexpr uint8_t CLK_IDRV_8MA      = 3u;

// PLL reset bits
static constexpr uint8_t PLLB_RST          = (1u << 7);
static constexpr uint8_t PLLA_RST          = (1u << 5);

// ---------------------------------------------------------------------------
// Crystal load capacitance bits (REG_XTAL_CL bits[7:6])
// 00 = reserved, 01 = 6pF, 10 = 8pF, 11 = 10pF
// PortaPack uses a 25 MHz 10pF load crystal.
// ---------------------------------------------------------------------------
static constexpr uint8_t XTAL_CL_10PF      = (3u << 6);

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr uint32_t XTAL_FREQ_HZ     = 25000000u;  // 25 MHz crystal
static constexpr uint32_t VCO_MIN_HZ       = 600000000u; // 600 MHz
static constexpr uint32_t VCO_MAX_HZ       = 900000000u; // 900 MHz

// Target VCO frequencies
// PLLA targets a multiple of 40 MHz in the VCO range → 800 MHz (800/40=20 even integer)
static constexpr uint32_t PLLA_VCO_HZ      = 800000000u; // 800 MHz for CLK0/CLK2
// PLLB targets a multiple of 26 MHz → 832 MHz (832/26=32 even integer)
static constexpr uint32_t PLLB_VCO_HZ      = 832000000u; // 832 MHz for CLK1

// ---------------------------------------------------------------------------
// Helper: single register write
// ---------------------------------------------------------------------------
static bool si_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write(SI5351_I2C_BUS, SI5351_I2C_ADDR, buf, 2u);
}

// ---------------------------------------------------------------------------
// Helper: write 8 consecutive bytes starting at reg_base
// ---------------------------------------------------------------------------
static bool si_write_regs(uint8_t reg_base, const uint8_t* data, uint8_t count) {
    // We need to prefix the register address
    uint8_t buf[9]; // max 1 addr + 8 data
    if (count > 8u) return false;
    buf[0] = reg_base;
    for (uint8_t i = 0; i < count; i++) buf[i + 1u] = data[i];
    return i2c_write(SI5351_I2C_BUS, SI5351_I2C_ADDR, buf, (size_t)(count + 1u));
}

// ---------------------------------------------------------------------------
// Compute P1, P2, P3 from a+b/c (AN619 eq. 1–3)
// All integer arithmetic.
// P1 = 128*a + floor(128*b/c) - 512
// P2 = 128*b - c * floor(128*b/c)
// P3 = c
// ---------------------------------------------------------------------------
struct MsParams {
    uint32_t p1;
    uint32_t p2;
    uint32_t p3;
};

// Compute MultiSynth params for integer divide ratio 'a' with no fractional part.
// (b=0, c=1 → P1 = 128*a - 512, P2 = 0, P3 = 1)
static MsParams ms_params_integer(uint32_t a) {
    MsParams p;
    p.p1 = 128u * a - 512u;
    p.p2 = 0u;
    p.p3 = 1u;
    return p;
}

// Compute MultiSynth params for fractional ratio a + b/c.
// b must be < c.  c must not be zero.
static MsParams ms_params_frac(uint32_t a, uint32_t b, uint32_t c) {
    MsParams p;
    uint32_t floor_128b_c = (128u * b) / c;
    p.p1 = 128u * a + floor_128b_c - 512u;
    p.p2 = 128u * b - c * floor_128b_c;
    p.p3 = c;
    return p;
}

// ---------------------------------------------------------------------------
// Write 8 bytes of MultiSynth/PLL feedback params to the register block.
// Format (AN619 Table 9, bytes 0-7):
//   byte0[7:0]  = P3[15:8]
//   byte1[7:0]  = P3[7:0]
//   byte2[1:0]  = P1[17:16]   (and R divider in bits[6:4] — 0 here = /1)
//   byte3[7:0]  = P1[15:8]
//   byte4[7:0]  = P1[7:0]
//   byte5[7:4]  = P3[19:16]   byte5[3:0] = P2[19:16]
//   byte6[7:0]  = P2[15:8]
//   byte7[7:0]  = P2[7:0]
// ---------------------------------------------------------------------------
static bool si_write_ms_params(uint8_t reg_base, const MsParams& p) {
    uint8_t buf[8];
    buf[0] = (uint8_t)((p.p3 >> 8u) & 0xFFu);
    buf[1] = (uint8_t)( p.p3        & 0xFFu);
    buf[2] = (uint8_t)((p.p1 >> 16u) & 0x03u); // bits 17:16 of P1; R_DIV=0 (/1)
    buf[3] = (uint8_t)((p.p1 >>  8u) & 0xFFu);
    buf[4] = (uint8_t)( p.p1         & 0xFFu);
    buf[5] = (uint8_t)(((p.p3 >> 12u) & 0xF0u) | ((p.p2 >> 16u) & 0x0Fu));
    buf[6] = (uint8_t)((p.p2 >>  8u) & 0xFFu);
    buf[7] = (uint8_t)( p.p2         & 0xFFu);
    return si_write_regs(reg_base, buf, 8u);
}

// ---------------------------------------------------------------------------
// Configure a PLL (PLLA or PLLB) to the given VCO frequency.
//   vco_hz must be in [600 MHz, 900 MHz].
//   Uses XTAL_FREQ_HZ as reference.
//   Feedback multiplier = vco_hz / xtal_hz  (integer or fractional a+b/c)
// ---------------------------------------------------------------------------
static bool si_config_pll(uint8_t reg_base, uint32_t vco_hz) {
    // a = vco_hz / xtal_hz  (integer part)
    // b/c = (vco_hz % xtal_hz) / xtal_hz, expressed as b=rem, c=xtal
    uint32_t a = vco_hz / XTAL_FREQ_HZ;
    uint32_t b = vco_hz % XTAL_FREQ_HZ;
    uint32_t c = XTAL_FREQ_HZ; // denominator = Fxtal for best resolution

    // Reduce b/c by GCD to keep values in range
    // Simple Euclidean GCD (integer only)
    {
        uint32_t u = b, v = c;
        while (v) { uint32_t t = v; v = u % v; u = t; }
        if (u > 1u) { b /= u; c /= u; }
    }

    MsParams p;
    if (b == 0u) {
        p = ms_params_integer(a);
    } else {
        p = ms_params_frac(a, b, c);
    }
    return si_write_ms_params(reg_base, p);
}

// ---------------------------------------------------------------------------
// si5351_init
// ---------------------------------------------------------------------------
bool si5351_init() {
    // 1. Wait for SYS_INIT (bit7 of REG_DEVICE_STATUS) to clear
    //    (device ready). Poll up to ~100 ms worth of iterations.
    {
        uint8_t status = 0u;
        uint8_t reg = REG_DEVICE_STATUS;
        for (uint32_t i = 0u; i < 1000u; i++) {
            if (!i2c_write_read(SI5351_I2C_BUS, SI5351_I2C_ADDR,
                                &reg, 1u, &status, 1u)) {
                return false; // I2C error — device not present
            }
            if ((status & 0x80u) == 0u) break; // SYS_INIT cleared
        }
    }

    // 2. Disable all outputs while configuring
    if (!si_write_reg(REG_OUTPUT_ENABLE, 0xFFu)) return false; // all CLKs disabled

    // 3. Power down all CLK outputs
    for (uint8_t i = 0u; i < 8u; i++) {
        if (!si_write_reg((uint8_t)(REG_CLK_BASE + i), CLK_PDN)) return false;
    }

    // 4. Set crystal load capacitance to 10 pF
    if (!si_write_reg(REG_XTAL_CL, XTAL_CL_10PF)) return false;

    // 5. Set PLL input source register (both PLLs from XTAL)
    if (!si_write_reg(REG_PLL_INPUT_SRC, 0x00u)) return false;

    // 6. Configure PLLA → 800 MHz (for CLK0 = 40 MHz, CLK2 = spare)
    if (!si_config_pll(REG_MSNA_BASE, PLLA_VCO_HZ)) return false;

    // 7. Configure PLLB → 832 MHz (for CLK1 = 26 MHz)
    if (!si_config_pll(REG_MSNB_BASE, PLLB_VCO_HZ)) return false;

    // 8. Soft-reset both PLLs to synchronise
    if (!si_write_reg(REG_PLL_RESET, PLLA_RST | PLLB_RST)) return false;

    // 9. Configure CLK0 = 40 MHz  (PLLA, 800/40 = 20, integer mode)
    //    MultiSynth0: divide by 20
    {
        MsParams p = ms_params_integer(20u); // 800 MHz / 20 = 40 MHz
        if (!si_write_ms_params(REG_MS0_BASE, p)) return false;
    }
    // CLK0 control: source = MS0, PLLA, 8 mA drive, integer mode
    if (!si_write_reg(REG_CLK_BASE + SI5351_CLK0,
                      CLK_INT_MODE | CLK_SRC_MS | CLK_IDRV_8MA)) return false;

    // 10. Configure CLK1 = 26 MHz  (PLLB, 832/32 = 26, integer mode)
    //     MultiSynth1: divide by 32
    {
        MsParams p = ms_params_integer(32u); // 832 MHz / 32 = 26 MHz
        if (!si_write_ms_params(REG_MS1_BASE, p)) return false;
    }
    // CLK1 control: source = MS1, PLLB (CLK_PLL_SRC_B), 8 mA drive, integer mode
    if (!si_write_reg(REG_CLK_BASE + SI5351_CLK1,
                      CLK_INT_MODE | CLK_PLL_SRC_B | CLK_SRC_MS | CLK_IDRV_8MA)) return false;

    // 11. CLK2 off (powered down, will be enabled on request)
    if (!si_write_reg(REG_CLK_BASE + SI5351_CLK2, CLK_PDN)) return false;

    // 12. Enable CLK0 and CLK1; keep others disabled
    //     REG_OUTPUT_ENABLE: bit = 0 → output enabled, bit = 1 → disabled
    //     So 0b11111100 = only CLK0 and CLK1 enabled
    if (!si_write_reg(REG_OUTPUT_ENABLE, 0xFCu)) return false;

    return true;
}

// ---------------------------------------------------------------------------
// si5351_set_freq
// ---------------------------------------------------------------------------
bool si5351_set_freq(uint8_t clk_output, uint32_t freq_hz) {
    if (clk_output > 2u) return false;
    if (freq_hz < 500000u || freq_hz > 200000000u) return false;

    // Determine which PLL to use:
    //   CLK0/CLK2 → PLLA, CLK1 → PLLB
    // We retune the PLL to a VCO that is a clean multiple or near-integer
    // multiple of the desired frequency, staying within [600, 900] MHz.
    //
    // Strategy: pick VCO = freq_hz * integer_divisor, where divisor is
    // chosen so VCO falls in [600 MHz, 900 MHz].  Prefer even divisors for
    // integer mode.  If no exact integer VCO exists in range, use fractional.

    bool use_pllb = (clk_output == SI5351_CLK1);
    uint8_t pll_reg = use_pllb ? REG_MSNB_BASE : REG_MSNA_BASE;

    uint8_t ms_reg_base;
    switch (clk_output) {
        case SI5351_CLK0: ms_reg_base = REG_MS0_BASE; break;
        case SI5351_CLK1: ms_reg_base = REG_MS1_BASE; break;
        default:          ms_reg_base = REG_MS2_BASE; break;
    }

    // Find the MultiSynth divisor 'div' such that VCO = freq_hz * div
    // is in [VCO_MIN, VCO_MAX].
    // div range: VCO_MIN/freq_hz .. VCO_MAX/freq_hz
    uint32_t div_min = VCO_MIN_HZ / freq_hz;
    uint32_t div_max = VCO_MAX_HZ / freq_hz;
    if (div_min < 6u)  div_min = 6u;   // Minimum MultiSynth divisor
    if (div_max > 900u) div_max = 900u; // Maximum MultiSynth divisor (fractional mode)

    if (div_min > div_max) return false;

    // Prefer even divisors for integer mode / low jitter
    uint32_t ms_div = div_min;
    // Round up to nearest even if odd
    if (ms_div & 1u) ms_div++;
    if (ms_div > div_max) ms_div = div_min; // fall back to any value

    // VCO = freq_hz * ms_div
    uint32_t vco_hz = freq_hz * ms_div;

    // Clamp to valid VCO range
    if (vco_hz < VCO_MIN_HZ) vco_hz = VCO_MIN_HZ;
    if (vco_hz > VCO_MAX_HZ) vco_hz = VCO_MAX_HZ;

    // Recalculate actual ms_div from VCO (may be fractional now)
    // ms_a = integer part, ms_b/ms_c = fractional part
    uint32_t ms_a = vco_hz / freq_hz;
    uint32_t ms_rem = vco_hz % freq_hz;

    // Configure PLL
    if (!si_config_pll(pll_reg, vco_hz)) return false;

    // Configure MultiSynth divider
    MsParams ms_p;
    if (ms_rem == 0u) {
        ms_p = ms_params_integer(ms_a);
    } else {
        // Use c = freq_hz as denominator for best precision
        // Reduce by GCD
        uint32_t g_u = ms_rem, g_v = freq_hz;
        while (g_v) { uint32_t t = g_v; g_v = g_u % g_v; g_u = t; }
        ms_p = ms_params_frac(ms_a, ms_rem / g_u, freq_hz / g_u);
    }
    if (!si_write_ms_params(ms_reg_base, ms_p)) return false;

    // Update CLK control register
    uint8_t clk_ctrl = CLK_SRC_MS | CLK_IDRV_8MA;
    if (use_pllb) clk_ctrl |= CLK_PLL_SRC_B;
    if (ms_rem == 0u && (ms_a & 1u) == 0u) clk_ctrl |= CLK_INT_MODE;
    if (!si_write_reg((uint8_t)(REG_CLK_BASE + clk_output), clk_ctrl)) return false;

    // Reset the relevant PLL to synchronise
    uint8_t rst = use_pllb ? PLLB_RST : PLLA_RST;
    if (!si_write_reg(REG_PLL_RESET, rst)) return false;

    return true;
}

// ---------------------------------------------------------------------------
// si5351_enable
// ---------------------------------------------------------------------------
void si5351_enable(uint8_t clk_output, bool enable) {
    if (clk_output > 7u) return;

    // Read current enable register
    uint8_t reg = REG_OUTPUT_ENABLE;
    uint8_t oe = 0xFFu; // default: all disabled
    i2c_write_read(SI5351_I2C_BUS, SI5351_I2C_ADDR, &reg, 1u, &oe, 1u);

    uint8_t mask = (uint8_t)(1u << clk_output);
    if (enable) {
        oe &= (uint8_t)(~mask); // clear bit → output enabled
        // Also clear CLK_PDN bit in the CLK control register
        uint8_t ctrl_reg_addr = (uint8_t)(REG_CLK_BASE + clk_output);
        uint8_t ctrl = 0u;
        i2c_write_read(SI5351_I2C_BUS, SI5351_I2C_ADDR, &ctrl_reg_addr, 1u, &ctrl, 1u);
        ctrl &= (uint8_t)(~CLK_PDN);
        si_write_reg(ctrl_reg_addr, ctrl);
    } else {
        oe |= mask; // set bit → output disabled
        // Also set CLK_PDN in CLK control register
        uint8_t ctrl_reg_addr = (uint8_t)(REG_CLK_BASE + clk_output);
        uint8_t ctrl = 0u;
        i2c_write_read(SI5351_I2C_BUS, SI5351_I2C_ADDR, &ctrl_reg_addr, 1u, &ctrl, 1u);
        ctrl |= CLK_PDN;
        si_write_reg(ctrl_reg_addr, ctrl);
    }
    si_write_reg(REG_OUTPUT_ENABLE, oe);
}
