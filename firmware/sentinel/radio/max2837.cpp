// SPDX-License-Identifier: MIT
// Project Sentinel — MAX2837 RF Transceiver Driver Implementation
//
// Reference: MAX2837 datasheet (Analog Devices), HackRF max2837.c
// (original by Mike Ossmann, BSD 2-clause — algorithm rewritten in C++
// for Project Sentinel, integer-only arithmetic).
//
// SPI format: 16-bit word
//   bit15 = R/W (0=write, 1=read)
//   bits[14:10] = 5-bit address
//   bits[9:0]   = 10-bit data
//
// VCO frequency calculation (from datasheet and HackRF firmware):
//   Assume 40 MHz PLL reference.
//   Divide ratio = F * (4/3) / 40_000_000 = F / 30_000_000
//   div_int  = F / 30_000_000
//   div_frac = (F % 30_000_000) scaled to 20-bit fractional (iterated bit-by-bit)
//   SYN_INT  = div_int                    → register 19
//   SYN_FRAC_HI = div_frac >> 10          → register 18
//   SYN_FRAC_LO = div_frac & 0x3FF        → register 17 (triggers VCO auto-select)

#include "max2837.hpp"
#include "hal/ssp.hpp"
#include "hal/gpio.hpp"
#include "bsp/portapack_pins.hpp"

// ---------------------------------------------------------------------------
// Register count
// ---------------------------------------------------------------------------
static constexpr uint8_t MAX2837_NUM_REGS = 32u;

// ---------------------------------------------------------------------------
// Default register values (from HackRF firmware max2837.c)
// These match the MAX2837 datasheet power-on state with HackRF-specific tweaks.
// ---------------------------------------------------------------------------
static const uint16_t max2837_regs_default[MAX2837_NUM_REGS] = {
    0x150u, /* 0:  RXRF_CFG_1       */
    0x002u, /* 1:  RXRF_CFG_2       */
    0x1f4u, /* 2:  LPF_CFG_1        */
    0x1b9u, /* 3:  LPF_CFG_2        */
    0x00au, /* 4:  LPF_VGA_CFG_1    */
    0x080u, /* 5:  LPF_VGA_CFG_2    */
    0x006u, /* 6:  RSSI_CFG_1       */
    0x000u, /* 7:  RSSI_CFG_2       */
    0x080u, /* 8:  RXBBFTune         */
    0x018u, /* 9:  TXLPF             */
    0x058u, /* 10: TXVGA_CFG_1      */
    0x016u, /* 11: TXVGA_CFG_2      */
    0x24fu, /* 12: LOGEN_TRIM_1     */
    0x150u, /* 13: LOGEN_TRIM_2     */
    0x1c5u, /* 14: LOGEN_TRIM_3     */
    0x081u, /* 15: VCO_CFG_1        */
    0x01cu, /* 16: VCO_CFG_2        */
    0x155u, /* 17: SYN_FRAC_LO      */
    0x155u, /* 18: SYN_FRAC_HI      */
    0x153u, /* 19: SYN_INT          */
    0x241u, /* 20: SYN_CFG_1        */
    0x02du, /* 21: SYN_CFG_2 — CP common mode enable bit[0] must be set */
    0x1a9u, /* 22: VAS_CFG_1        */
    0x24fu, /* 23: VAS_CFG_2        */
    0x180u, /* 24: LPBK_CFG_1       */
    0x100u, /* 25: LPBK_CFG_2       */
    0x3cau, /* 26: LPBK_CFG_3       */
    0x3e3u, /* 27: LPBK_CFG_4       */
    0x0c0u, /* 28: LPBK_CFG_5       */
    0x3f0u, /* 29: TXVGA_GAIN       */
    0x080u, /* 30: SPI_DEBUG        */
    0x000u, /* 31: BANDSEL          */
};

// Shadow register array
static uint16_t max2837_regs[MAX2837_NUM_REGS];
static uint32_t max2837_dirty; // bitmask

// ---------------------------------------------------------------------------
// Named register addresses (from MAX2837 datasheet Table 2)
// ---------------------------------------------------------------------------
static constexpr uint8_t RXRF_CFG_1    =  0u;
static constexpr uint8_t RXRF_CFG_2    =  1u;
static constexpr uint8_t LPF_CFG_1     =  2u;
static constexpr uint8_t LPF_CFG_2     =  3u;
static constexpr uint8_t LPF_VGA_1     =  4u;
static constexpr uint8_t LPF_VGA_2     =  5u;
// ...
static constexpr uint8_t RXBBF_TUNE    =  8u;
static constexpr uint8_t SYN_FRAC_LO  = 17u;
static constexpr uint8_t SYN_FRAC_HI  = 18u;
static constexpr uint8_t SYN_INT      = 19u;
static constexpr uint8_t SYN_CFG_1    = 20u;
static constexpr uint8_t SYN_CFG_2    = 21u;
static constexpr uint8_t VAS_CFG_1    = 22u;
static constexpr uint8_t TXVGA_GAIN   = 29u;
static constexpr uint8_t BANDSEL      = 31u;

// ---------------------------------------------------------------------------
// Key bit field positions (from datasheet)
// ---------------------------------------------------------------------------
// RXRF_CFG_1 (reg 0):
//   bits[8:7] = LNAband:  0=2.3-2.4 GHz, 1=2.4-2.5 GHz (not used in HackRF path)
//                         Actually: 0=2.4GHz band, 1=2.6GHz band
//   bits[6:5] = LOGEN_BSW: band switch
//   bit[4]    = LNAgain_SPI_EN: 1 = gain controlled by SPI
//   bits[3:1] = LNAgain: 000=max(40dB), 001=-8dB, 010=-16dB, 011=-24dB,
//                        100=-32dB, 101=-40dB
static constexpr uint16_t LOGEN_BSW_2_3 = 0u; // 2.3 GHz band
static constexpr uint16_t LOGEN_BSW_2_4 = 1u; // 2.4 GHz band
static constexpr uint16_t LOGEN_BSW_2_5 = 2u; // 2.5 GHz band
static constexpr uint16_t LOGEN_BSW_2_6 = 3u; // 2.6 GHz band

static constexpr uint16_t LNAband_2_4   = 0u;
static constexpr uint16_t LNAband_2_6   = 1u;

static constexpr uint16_t LNAgain_MAX   = 0u;  // 40 dB
static constexpr uint16_t LNAgain_M8    = 1u;  // 32 dB
static constexpr uint16_t LNAgain_M16   = 2u;  // 24 dB
static constexpr uint16_t LNAgain_M24   = 3u;  // 16 dB
static constexpr uint16_t LNAgain_M32   = 4u;  //  8 dB
static constexpr uint16_t LNAgain_M40   = 5u;  //  0 dB

// LPF_VGA_1 (reg 4): bits[4:0] = VGA gain field (31 = 0dB, 0 = 62dB)
// LPF_VGA_2 (reg 5): contains additional VGA control
// RXRF_CFG_1 (reg 0): LOGEN_BSW in bits[6:5], LNAband in bit[8], LNAgain_SPI_EN in bit[4], LNAgain in bits[3:1]

// ---------------------------------------------------------------------------
// Low-level SPI
// ---------------------------------------------------------------------------

static void max2837_cs_assert()   { gpio_cs_low (PORT_MAX2837_CS, PIN_MAX2837_CS); }
static void max2837_cs_deassert() { gpio_cs_high(PORT_MAX2837_CS, PIN_MAX2837_CS); }

static void max2837_write(uint8_t reg, uint16_t val) {
    // 16-bit word: bit15=0(write), bits[14:10]=addr, bits[9:0]=data
    uint16_t word = (uint16_t)(((uint16_t)(reg & 0x1Fu) << 10u) | (val & 0x3FFu));
    max2837_cs_assert();
    ssp_write16(MAX2837_SSP_BUS, word);
    max2837_cs_deassert();
    if (reg < MAX2837_NUM_REGS) {
        max2837_regs[reg] = val & 0x3FFu;
        max2837_dirty &= ~(1u << reg);
    }
}

static uint16_t max2837_read(uint8_t reg) {
    // 16-bit word: bit15=1(read), bits[14:10]=addr, bits[9:0]=0
    uint16_t word = (uint16_t)((1u << 15u) | ((uint16_t)(reg & 0x1Fu) << 10u));
    max2837_cs_assert();
    // Write address word, read back data word in same transaction
    uint8_t tx[2] = { (uint8_t)(word >> 8u), (uint8_t)(word & 0xFFu) };
    uint8_t rx[2] = { 0u, 0u };
    ssp_write_read(MAX2837_SSP_BUS, tx, rx, 2u);
    max2837_cs_deassert();
    return (uint16_t)(((uint16_t)(rx[0] & 0x03u) << 8u) | rx[1]);
}

static inline void max2837_set_field(uint8_t reg, uint16_t mask, uint16_t shift, uint16_t val) {
    max2837_regs[reg] = (uint16_t)((max2837_regs[reg] & ~mask) | ((val << shift) & mask));
    max2837_dirty |= (1u << reg);
}

static void max2837_commit(uint8_t reg) {
    if (reg < MAX2837_NUM_REGS) {
        max2837_write(reg, max2837_regs[reg]);
    }
}

static void max2837_commit_all() {
    for (uint8_t i = 0u; i < MAX2837_NUM_REGS; i++) {
        max2837_write(i, max2837_regs[i]);
    }
    max2837_dirty = 0u;
}

// ---------------------------------------------------------------------------
// Field set helpers (mirrors HackRF set_MAX2837_xxx macros)
// ---------------------------------------------------------------------------

static void set_LOGEN_BSW(uint16_t val) {
    // bits[6:5] of reg 0
    max2837_set_field(RXRF_CFG_1, (0x03u << 5u), 5u, val);
}

static void set_LNAband(uint16_t val) {
    // bit[8] of reg 0
    max2837_set_field(RXRF_CFG_1, (0x01u << 8u), 8u, val);
}

static void set_LNAgain_SPI_EN(uint16_t en) {
    // bit[4] of reg 0
    max2837_set_field(RXRF_CFG_1, (0x01u << 4u), 4u, en);
}

static void set_LNAgain(uint16_t val) {
    // bits[3:1] of reg 0
    max2837_set_field(RXRF_CFG_1, (0x07u << 1u), 1u, val);
}

static void set_VGA(uint16_t val) {
    // bits[4:0] of reg 4
    max2837_set_field(LPF_VGA_1, 0x1Fu, 0u, val);
}

static void set_VGA_SPI_EN(uint16_t en) {
    // bit[9] of reg 5 (VGAgain_SPI_EN)
    max2837_set_field(LPF_VGA_2, (0x01u << 9u), 9u, en);
}

static void set_SYN_INT(uint16_t val) {
    // bits[6:0] of reg 19
    max2837_set_field(SYN_INT, 0x7Fu, 0u, val);
}

static void set_SYN_FRAC_HI(uint16_t val) {
    // bits[9:0] of reg 18
    max2837_set_field(SYN_FRAC_HI, 0x3FFu, 0u, val);
}

static void set_SYN_FRAC_LO(uint16_t val) {
    // bits[9:0] of reg 17
    max2837_set_field(SYN_FRAC_LO, 0x3FFu, 0u, val);
}

// ---------------------------------------------------------------------------
// max2837_init
// ---------------------------------------------------------------------------
bool max2837_init() {
    // Configure CS
    gpio_set_dir(PORT_MAX2837_CS, PIN_MAX2837_CS, true);
    max2837_cs_deassert();

    // Load defaults into shadow registers
    for (uint8_t i = 0u; i < MAX2837_NUM_REGS; i++) {
        max2837_regs[i] = max2837_regs_default[i] & 0x3FFu;
    }
    max2837_dirty = 0u;

    // Write all registers
    max2837_commit_all();

    // Enable LNA gain control via SPI
    set_LNAgain_SPI_EN(1u);
    // Set default LNA gain to maximum (40 dB)
    set_LNAgain(LNAgain_MAX);
    max2837_commit(RXRF_CFG_1);

    // Enable VGA gain control via SPI and set a sensible default (24 dB)
    set_VGA_SPI_EN(1u);
    max2837_commit(LPF_VGA_2);
    set_VGA(31u - (24u >> 1u)); // VGA field = 31 - gain_db/2
    max2837_commit(LPF_VGA_1);

    return true;
}

// ---------------------------------------------------------------------------
// max2837_set_freq
//
// VCO divide ratio: F / 30_000_000 (integer + 20-bit fraction)
// Based on: 40 MHz PLL ref, internal x4/3 multiplier → effective ref = 30 MHz
// ---------------------------------------------------------------------------
bool max2837_set_freq(uint32_t freq_hz) {
    // Accept frequencies outside datasheet range for HackRF flexibility
    // (mixer images allow some use outside 2.3–2.7 GHz)
    uint8_t  band;
    uint8_t  lna_band;

    if (freq_hz < 2400000000u) {
        band     = LOGEN_BSW_2_3;
        lna_band = LNAband_2_4;
    } else if (freq_hz < 2500000000u) {
        band     = LOGEN_BSW_2_4;
        lna_band = LNAband_2_4;
    } else if (freq_hz < 2600000000u) {
        band     = LOGEN_BSW_2_5;
        lna_band = LNAband_2_6;
    } else {
        band     = LOGEN_BSW_2_6;
        lna_band = LNAband_2_6;
    }

    // Round to nearest achievable frequency
    // div = F / 30_000_000
    //   Add half a step for rounding: step = 30_000_000 >> 20 ≈ 28.6 Hz
    //   Rounding add = 30_000_000 >> 21
    freq_hz += (30000000u >> 21u);

    uint32_t div_int  = freq_hz / 30000000u;
    uint32_t div_rem  = freq_hz % 30000000u;

    // Compute 20-bit fractional part by iterative bit extraction
    uint32_t div_frac = 0u;
    uint32_t div_cmp  = 30000000u;
    for (int i = 0; i < 20; i++) {
        div_frac <<= 1u;
        div_rem  <<= 1u;
        if (div_rem >= div_cmp) {
            div_frac |= 1u;
            div_rem  -= div_cmp;
        }
    }

    // Band select
    set_LOGEN_BSW(band);
    set_LNAband(lna_band);

    // Write INT and FRAC_HI first, then FRAC_LO (triggers VCO auto-select)
    set_SYN_INT((uint16_t)(div_int & 0x7Fu));
    set_SYN_FRAC_HI((uint16_t)((div_frac >> 10u) & 0x3FFu));
    // Commit INT, FRAC_HI, and band regs before FRAC_LO
    max2837_commit(RXRF_CFG_1);
    max2837_commit(SYN_INT);
    max2837_commit(SYN_FRAC_HI);

    // Now commit FRAC_LO — this triggers VCO auto-select
    set_SYN_FRAC_LO((uint16_t)(div_frac & 0x3FFu));
    max2837_commit(SYN_FRAC_LO);

    return true;
}

// ---------------------------------------------------------------------------
// max2837_set_lna_gain
// ---------------------------------------------------------------------------
bool max2837_set_lna_gain(uint8_t gain_db) {
    uint16_t val;
    switch (gain_db) {
        case 40u: val = LNAgain_MAX; break;
        case 32u: val = LNAgain_M8;  break;
        case 24u: val = LNAgain_M16; break;
        case 16u: val = LNAgain_M24; break;
        case  8u: val = LNAgain_M32; break;
        case  0u: val = LNAgain_M40; break;
        default:  return false;
    }
    set_LNAgain(val);
    max2837_commit(RXRF_CFG_1);
    return true;
}

// ---------------------------------------------------------------------------
// max2837_set_vga_gain
// ---------------------------------------------------------------------------
bool max2837_set_vga_gain(uint8_t gain_db) {
    if ((gain_db & 1u) || gain_db > 62u) return false;
    // VGA field = 31 - gain_db/2
    set_VGA((uint16_t)(31u - (gain_db >> 1u)));
    max2837_commit(LPF_VGA_1);
    return true;
}

// ---------------------------------------------------------------------------
// max2837_rx_mode
// ---------------------------------------------------------------------------
void max2837_rx_mode() {
    // Ensure LNA SPI control enabled, IQ outputs active
    // In RX mode the LPF connects to ADC path
    // Register 0 bit[0] = RX_ENABLE (active mode)
    // In HackRF firmware, RX is the default state after init.
    set_LNAgain_SPI_EN(1u);
    max2837_commit(RXRF_CFG_1);
}

// ---------------------------------------------------------------------------
// max2837_tx_mode
// ---------------------------------------------------------------------------
void max2837_tx_mode() {
    // TX VGA gain and TX path enable
    // Register 20 (SYN_CFG_1) has TX enable bits in some variants.
    // For MAX2837 in HackRF, TX is controlled via the VGA gain register path.
    // Set CP common mode enable (reg 21 bit[0]) as required by HackRF note.
    max2837_regs[SYN_CFG_2] |= 0x001u; // bit[0] = CP common mode enable
    max2837_commit(SYN_CFG_2);
}

// ---------------------------------------------------------------------------
// max2837_standby
// ---------------------------------------------------------------------------
void max2837_standby() {
    // Power down the chip by disabling outputs.
    // Write 0 to register 0 to disable RX RF — preserve VCO state.
    // In HackRF, TX off = LNA off + VGA min.
    set_LNAgain(LNAgain_M40); // 0 dB LNA
    set_VGA(31u);              // 0 dB VGA
    max2837_commit(RXRF_CFG_1);
    max2837_commit(LPF_VGA_1);
}
