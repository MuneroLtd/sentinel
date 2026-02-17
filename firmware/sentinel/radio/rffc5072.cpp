// SPDX-License-Identifier: MIT
// Project Sentinel — RFFC5072 Wideband Mixer/Synthesizer Driver Implementation
//
// Reference: RFFC5071/5072 datasheet (Qorvo), Skyworks application note.
// Ported algorithm from greatscottgadgets/hackrf firmware/common/rffc5071.c
// (original by Mike Ossmann, BSD 2-clause — logic rewritten in C++ for
// Project Sentinel with integer-only arithmetic, no dependency on HackRF HAL).
//
// SPI protocol:
//   Two 16-bit transfers per operation.
//   Transfer 1 (command): {0x00|addr, 0x00}  for write
//                         {0x80|addr, 0x00}  for read
//   Transfer 2 (data):    16-bit register value
//   CS held low across both transfers.

#include "rffc5072.hpp"
#include "hal/ssp.hpp"
#include "hal/gpio.hpp"
#include "bsp/portapack_pins.hpp"

// ---------------------------------------------------------------------------
// SPI bus assignment (shared SSP0 with LCD/MAX2837; CS distinguishes devices)
// ---------------------------------------------------------------------------
static constexpr uint8_t RFFC_SSP_BUS = 0u; // SSP0

// ---------------------------------------------------------------------------
// RFFC5072 Register map (addresses 0x00 – 0x1E, 31 registers, 16 bits each)
// Names follow RFFC5071 datasheet and HackRF firmware conventions.
// ---------------------------------------------------------------------------
static constexpr uint8_t RFFC_REG_LF       = 0x00u; // Loop filter
static constexpr uint8_t RFFC_REG_XO       = 0x01u; // Crystal oscillator / ref
static constexpr uint8_t RFFC_REG_CAL_TIME = 0x02u; // Calibration time
static constexpr uint8_t RFFC_REG_VCO_CTRL = 0x03u; // VCO control
static constexpr uint8_t RFFC_REG_CT_CAL1  = 0x04u; // CT calibration 1
static constexpr uint8_t RFFC_REG_CT_CAL2  = 0x05u; // CT calibration 2
static constexpr uint8_t RFFC_REG_PLL_CAL1 = 0x06u; // PLL calibration 1
static constexpr uint8_t RFFC_REG_PLL_CAL2 = 0x07u; // PLL calibration 2
static constexpr uint8_t RFFC_REG_VCO_AUTO = 0x08u; // VCO auto-cal
static constexpr uint8_t RFFC_REG_PLL_CTRL = 0x09u; // PLL control
static constexpr uint8_t RFFC_REG_LODIV    = 0x0Au; // LO divider / output
static constexpr uint8_t RFFC_REG_FUNCTION = 0x0Bu; // Function register
static constexpr uint8_t RFFC_REG_RESERVED_0C = 0x0Cu;
static constexpr uint8_t RFFC_REG_DEVID    = 0x0Du; // Device ID (read-only)
static constexpr uint8_t RFFC_REG_RESERVED_0E = 0x0Eu;
static constexpr uint8_t RFFC_REG_RESERVED_0F = 0x0Fu;
static constexpr uint8_t RFFC_REG_P2N_FRAC = 0x10u; // N fraction MSB (P2NMSB)
static constexpr uint8_t RFFC_REG_P2N      = 0x11u; // N integer / LODIV / prescaler
static constexpr uint8_t RFFC_REG_P2NLSB   = 0x12u; // N fraction LSB (P2NLSB bits[7:0] in bits[15:8])
static constexpr uint8_t RFFC_REG_P2DBL    = 0x13u; // Doubler control
static constexpr uint8_t RFFC_REG_P2VCO    = 0x14u; // VCO band
static constexpr uint8_t RFFC_REG_P2LOVAL  = 0x15u; // LO output amplitude
static constexpr uint8_t RFFC_REG_P2TEST1  = 0x16u;
static constexpr uint8_t RFFC_REG_P2TEST2  = 0x17u;
static constexpr uint8_t RFFC_REG_BIAS     = 0x18u; // Bias control
static constexpr uint8_t RFFC_REG_MIXCONF  = 0x19u; // Mixer configuration
static constexpr uint8_t RFFC_REG_RESERVED_1A = 0x1Au;
static constexpr uint8_t RFFC_REG_RESERVED_1B = 0x1Bu;
static constexpr uint8_t RFFC_REG_RESERVED_1C = 0x1Cu;
static constexpr uint8_t RFFC_REG_RESERVED_1D = 0x1Du;
static constexpr uint8_t RFFC_REG_RESERVED_1E = 0x1Eu;

static constexpr uint8_t RFFC_NUM_REGS     = 31u;

// ---------------------------------------------------------------------------
// Default register values from HackRF rffc5071.c (verified against datasheet)
// These are the known-good startup values for path-2 operation in HackRF.
// ---------------------------------------------------------------------------
static const uint16_t rffc5072_regs_default[RFFC_NUM_REGS] = {
    0xfffbu, /* 00 LF         */
    0x4064u, /* 01 XO         */
    0x9055u, /* 02 CAL_TIME   */
    0x2d02u, /* 03 VCO_CTRL   */
    0xacbcu, /* 04 CT_CAL1    */
    0xfde8u, /* 05 CT_CAL2    */
    0x00fau, /* 06 PLL_CAL1   */
    0x0350u, /* 07 PLL_CAL2   */
    0xc9e6u, /* 08 VCO_AUTO   */
    0x0000u, /* 09 PLL_CTRL   */
    0x1231u, /* 0A LODIV      */
    0x0000u, /* 0B FUNCTION   */
    0x0000u, /* 0C reserved   */
    0x0000u, /* 0D DEVID      */
    0x0000u, /* 0E reserved   */
    0x0000u, /* 0F reserved   */
    0x0000u, /* 10 P2NMSB     */
    0x0000u, /* 11 P2N        */
    0x0000u, /* 12 P2NLSB     */
    0x0000u, /* 13 P2DBL      */
    0x0000u, /* 14 P2VCO      */
    0x0000u, /* 15 P2LOVAL    */
    0x0000u, /* 16 P2TEST1    */
    0x0000u, /* 17 P2TEST2    */
    0x0000u, /* 18 BIAS       */
    0x0000u, /* 19 MIXCONF    */
    0x0000u, /* 1A reserved   */
    0x0000u, /* 1B reserved   */
    0x0000u, /* 1C reserved   */
    0x0000u, /* 1D reserved   */
    0x0000u, /* 1E reserved   */
};

// Shadow register array (local copy, written to device on commit)
static uint16_t rffc_regs[RFFC_NUM_REGS];
static uint32_t rffc_dirty; // bitmask of registers needing commit

// ---------------------------------------------------------------------------
// Register bit field helpers
// ---------------------------------------------------------------------------

// FUNCTION register (0x0B) bits
static constexpr uint16_t FUNC_ENBL   = (1u << 0);  // Path 2 enable
static constexpr uint16_t FUNC_STANDBY= (1u << 1);  // Standby
static constexpr uint16_t FUNC_RXMIX  = (1u << 4);  // 0=TX, 1=RX mixer select
static constexpr uint16_t FUNC_BYPASS = (1u << 5);  // Bypass path

// PLL_CTRL register (0x09) bits
static constexpr uint16_t PLLCTRL_CPL_SHIFT = 0u;
static constexpr uint16_t PLLCTRL_CPL_MASK  = 0x0003u;

// P2N register (0x11) bit fields (as used in HackRF firmware)
// bits [6:0]  = P2N (integer N divider)
// bits [9:7]  = P2LODIV (logarithmic LO divider selector 0-5)
// bits [11:10]= P2PRESC (prescaler: 01 = /4, 10 = /8)
static constexpr uint16_t P2N_SHIFT    = 0u;
static constexpr uint16_t P2N_MASK     = 0x007Fu;
static constexpr uint16_t P2LODIV_SHIFT= 7u;
static constexpr uint16_t P2LODIV_MASK = (0x07u << 7);
static constexpr uint16_t P2PRESC_SHIFT= 10u;
static constexpr uint16_t P2PRESC_MASK = (0x03u << 10);

// P2NMSB register (0x10): [15:0] = upper 16 bits of fractional N
// P2NLSB register (0x12): [15:8] = lower 8 bits of fractional N

// ---------------------------------------------------------------------------
// Constants for frequency calculation
// ---------------------------------------------------------------------------
static constexpr uint64_t REF_FREQ_HZ  = 40000000ULL;  // 40 MHz reference
static constexpr uint64_t LO_MAX_HZ    = 5400000000ULL; // VCO max 5.4 GHz
static constexpr uint64_t LO_MIN_HZ    = 85000000ULL;   // Minimum LO output

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

static void rffc_cs_assert()   { gpio_cs_low (PORT_RFFC_CS, PIN_RFFC_CS); }
static void rffc_cs_deassert() { gpio_cs_high(PORT_RFFC_CS, PIN_RFFC_CS); }

// Write a 16-bit register value to the device.
// Two 16-bit SPI words: [cmd_byte, 0x00] then [data_hi, data_lo]
static void rffc_write(uint8_t reg, uint16_t val) {
    rffc_cs_assert();
    // Word 1: command (write = 0x00 | addr)
    ssp_write16(RFFC_SSP_BUS, (uint16_t)(reg & 0x7Fu));
    // Word 2: data
    ssp_write16(RFFC_SSP_BUS, val);
    rffc_cs_deassert();
    // Update shadow
    if (reg < RFFC_NUM_REGS) {
        rffc_regs[reg] = val;
        rffc_dirty &= ~(1u << reg);
    }
}

// Read a 16-bit register value from the device.
static uint16_t rffc_read(uint8_t reg) {
    rffc_cs_assert();
    // Word 1: command (read = 0x80 | addr)
    ssp_write16(RFFC_SSP_BUS, (uint16_t)(0x80u | (reg & 0x7Fu)));
    // Word 2: clock out to receive data
    uint16_t result = 0u;
    uint8_t buf[2] = { 0xFFu, 0xFFu };
    uint8_t rx[2]  = { 0u, 0u };
    ssp_write_read(RFFC_SSP_BUS, buf, rx, 2u);
    result = (uint16_t)(((uint16_t)rx[0] << 8u) | rx[1]);
    rffc_cs_deassert();
    return result;
}

// Mark a register as dirty (needs commit)
static inline void rffc_mark_dirty(uint8_t reg) {
    if (reg < RFFC_NUM_REGS) rffc_dirty |= (1u << reg);
}

// Set a field within shadow register and mark dirty
static void rffc_set_field(uint8_t reg, uint16_t mask, uint16_t shift, uint16_t val) {
    rffc_regs[reg] = (uint16_t)((rffc_regs[reg] & ~mask) | ((val << shift) & mask));
    rffc_mark_dirty(reg);
}

// Commit all dirty registers to the device
static void rffc_commit() {
    for (uint8_t i = 0u; i < RFFC_NUM_REGS; i++) {
        if (rffc_dirty & (1u << i)) {
            rffc_write(i, rffc_regs[i]);
        }
    }
    rffc_dirty = 0u;
}

// ---------------------------------------------------------------------------
// rffc5072_init
// ---------------------------------------------------------------------------
bool rffc5072_init() {
    // Configure GPIO CS as output, deasserted
    gpio_set_dir(PORT_RFFC_CS, PIN_RFFC_CS, true);
    rffc_cs_deassert();

    // Load shadow registers with defaults
    for (uint8_t i = 0u; i < RFFC_NUM_REGS; i++) {
        rffc_regs[i] = rffc5072_regs_default[i];
    }
    rffc_dirty = 0u;

    // Write all registers to device
    for (uint8_t i = 0u; i < RFFC_NUM_REGS; i++) {
        rffc_write(i, rffc_regs[i]);
    }

    // Enable path 2, select RX mode, disable standby
    uint16_t func = rffc_regs[RFFC_REG_FUNCTION];
    func |=  FUNC_ENBL;
    func |=  FUNC_RXMIX;
    func &= (uint16_t)(~FUNC_STANDBY);
    rffc_write(RFFC_REG_FUNCTION, func);

    return true;
}

// ---------------------------------------------------------------------------
// rffc5072_set_lo_freq
//
// Algorithm (from HackRF rffc5071.c rffc5071_config_synth):
//   1. Select n_lo: smallest integer so that fvco = lo_hz << n_lo ≤ LO_MAX
//      (n_lo in 0..5, each step halves the effective frequency via LO divider)
//   2. Select charge pump prescaler based on fvco:
//      fvco > 3200 MHz → fbkdivlog=2, CPL=3
//      otherwise       → fbkdivlog=1, CPL=2
//   3. Compute 24-bit fixed-point fractional N:
//      tmp_n = (fvco << (24 - fbkdivlog)) / REF_FREQ
//      Round to nearest multiple of 2^d where d = (24-fbkdivlog+n_lo) - 6
//      N integer = tmp_n >> 24
//      P2NMSB    = (tmp_n >> 8) & 0xFFFF
//      P2NLSB    = tmp_n & 0xFF  (only if step granularity fine enough)
// ---------------------------------------------------------------------------
bool rffc5072_set_lo_freq(uint64_t freq_hz) {
    if (freq_hz < LO_MIN_HZ || freq_hz > 4200000000ULL) return false;

    // Step 1: find n_lo so VCO is in range
    uint8_t  n_lo  = 0u;
    uint64_t fvco  = freq_hz;
    uint64_t x     = LO_MAX_HZ >> 1u; // 2700 MHz

    while ((x >= freq_hz) && (n_lo < 5u)) {
        n_lo++;
        x >>= 1u;
    }
    fvco = freq_hz << n_lo;

    // Step 2: charge pump / prescaler
    uint8_t fbkdivlog;
    uint16_t cpl;
    if (fvco > 3200000000ULL) {
        fbkdivlog = 2u;
        cpl       = 3u;
    } else {
        fbkdivlog = 1u;
        cpl       = 2u;
    }

    // Step 3: fractional-N calculation
    // tmp_n = (fvco << (24 - fbkdivlog)) / REF_FREQ
    // Use 64-bit arithmetic to avoid overflow.
    const uint8_t shift_up = (uint8_t)(24u - fbkdivlog);
    uint64_t tmp_n = (fvco << shift_up) / REF_FREQ_HZ;

    // Round to nearest achievable step (s=6 gives ~625 kHz steps)
    const uint8_t s = 6u;
    const uint8_t d = (uint8_t)((24u - fbkdivlog + n_lo) - s);
    if (d > 0u) {
        uint64_t round_bit = (uint64_t)1u << (d - 1u);
        tmp_n = ((tmp_n + round_bit) >> d) << d;
    }

    uint8_t  p2n    = (uint8_t)(tmp_n >> 24u);
    uint16_t p2nmsb = (uint16_t)((tmp_n >> 8u) & 0xFFFFu);
    uint8_t  p2nlsb = (uint8_t)(tmp_n & 0xFFu);

    // Write PLL control (charge pump)
    rffc_set_field(RFFC_REG_PLL_CTRL, PLLCTRL_CPL_MASK, PLLCTRL_CPL_SHIFT, cpl);

    // Write P2N register: N integer | LODIV | PRESC
    {
        uint16_t val = 0u;
        val |= (uint16_t)((p2n & 0x7Fu) << P2N_SHIFT);
        val |= (uint16_t)((n_lo & 0x07u) << P2LODIV_SHIFT);
        val |= (uint16_t)((fbkdivlog & 0x03u) << P2PRESC_SHIFT);
        rffc_regs[RFFC_REG_P2N] = val;
        rffc_mark_dirty(RFFC_REG_P2N);
    }

    // Write P2NMSB (upper 16 bits of fractional N)
    rffc_regs[RFFC_REG_P2N_FRAC] = p2nmsb;
    rffc_mark_dirty(RFFC_REG_P2N_FRAC);

    // Write P2NLSB only when step size is fine enough (s > 14 would mean d<0,
    // but with s=6 we always have d≥0; only write LSB if d < 8 meaning the
    // 8 LSBs contribute to frequency resolution).
    if (d < 8u) {
        rffc_regs[RFFC_REG_P2NLSB] = (uint16_t)(p2nlsb << 8u);
        rffc_mark_dirty(RFFC_REG_P2NLSB);
    }

    rffc_commit();
    return true;
}

// ---------------------------------------------------------------------------
// rffc5072_rx_mode
// ---------------------------------------------------------------------------
void rffc5072_rx_mode() {
    uint16_t func = rffc_regs[RFFC_REG_FUNCTION];
    func |=  FUNC_RXMIX;  // RX mixer selected
    func &= (uint16_t)(~FUNC_STANDBY);
    func |=  FUNC_ENBL;
    rffc_write(RFFC_REG_FUNCTION, func);
}

// ---------------------------------------------------------------------------
// rffc5072_tx_mode
// ---------------------------------------------------------------------------
void rffc5072_tx_mode() {
    uint16_t func = rffc_regs[RFFC_REG_FUNCTION];
    func &= (uint16_t)(~FUNC_RXMIX);  // TX mixer selected
    func &= (uint16_t)(~FUNC_STANDBY);
    func |=  FUNC_ENBL;
    rffc_write(RFFC_REG_FUNCTION, func);
}

// ---------------------------------------------------------------------------
// rffc5072_standby
// ---------------------------------------------------------------------------
void rffc5072_standby() {
    uint16_t func = rffc_regs[RFFC_REG_FUNCTION];
    func |= FUNC_STANDBY;
    func &= (uint16_t)(~FUNC_ENBL);
    rffc_write(RFFC_REG_FUNCTION, func);
}

// ---------------------------------------------------------------------------
// rffc5072_enable
// ---------------------------------------------------------------------------
void rffc5072_enable() {
    uint16_t func = rffc_regs[RFFC_REG_FUNCTION];
    func &= (uint16_t)(~FUNC_STANDBY);
    func |= FUNC_ENBL;
    rffc_write(RFFC_REG_FUNCTION, func);
}
