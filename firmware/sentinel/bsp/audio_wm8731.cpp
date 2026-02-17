// SPDX-License-Identifier: MIT
// Project Sentinel — WM8731 Audio Codec Driver Implementation
//
// WM8731 register layout (all registers 9-bit data, 7-bit address):
//   R0  Left line input volume
//   R1  Right line input volume
//   R2  Left headphone output volume
//   R3  Right headphone output volume
//   R4  Analogue audio path control
//   R5  Digital audio path control
//   R6  Power down control
//   R7  Digital audio interface format
//   R8  Sampling control
//   R9  Active control
//   R15 Reset
//
// I2C write format: 2 bytes = {(addr<<1)|(data>>8), data[7:0]}

#include "audio_wm8731.hpp"
#include "portapack_pins.hpp"
#include "../hal/i2c.hpp"
#include "../hal/lpc4320_regs.hpp"
#include "../hal/gpio.hpp"
#include "../hal/clocks.hpp"

// ---------------------------------------------------------------------------
// WM8731 register address constants
// ---------------------------------------------------------------------------
static constexpr uint8_t WM8731_REG_LINVOL   = 0x00;
static constexpr uint8_t WM8731_REG_RINVOL   = 0x01;
static constexpr uint8_t WM8731_REG_LHPVOL   = 0x02;
static constexpr uint8_t WM8731_REG_RHPVOL   = 0x03;
static constexpr uint8_t WM8731_REG_AAPCTRL  = 0x04;  // Analogue path control
static constexpr uint8_t WM8731_REG_DAPCTRL  = 0x05;  // Digital path control
static constexpr uint8_t WM8731_REG_PDCTRL   = 0x06;  // Power down control
static constexpr uint8_t WM8731_REG_DAIFMT   = 0x07;  // Digital audio interface format
static constexpr uint8_t WM8731_REG_SRATE    = 0x08;  // Sampling rate control
static constexpr uint8_t WM8731_REG_ACTIVE   = 0x09;  // Active control
static constexpr uint8_t WM8731_REG_RESET    = 0x0F;  // Reset

// Power down register bits
static constexpr uint16_t WM8731_PD_LINEIN   = (1u << 0);
static constexpr uint16_t WM8731_PD_MIC      = (1u << 1);
static constexpr uint16_t WM8731_PD_ADC      = (1u << 2);
static constexpr uint16_t WM8731_PD_DAC      = (1u << 3);
static constexpr uint16_t WM8731_PD_OUT      = (1u << 4);
static constexpr uint16_t WM8731_PD_OSC      = (1u << 5);
static constexpr uint16_t WM8731_PD_CLKOUT   = (1u << 6);
static constexpr uint16_t WM8731_PD_POWEROFF = (1u << 7);

// Digital interface format bits (R7)
// [1:0] FORMAT: 00=right-justified, 01=left-justified, 10=I2S, 11=DSP
// [3:2] IWL: 00=16bit, 01=20bit, 10=24bit, 11=32bit
// [4]   LRP: 0=LRCLK low=left, 1=LRCLK high=left (for I2S polarity)
// [5]   LRSWAP
// [6]   MS: 0=slave, 1=master
// [7]   BCLKINV
static constexpr uint16_t WM8731_FMT_I2S     = 0x02u;  // I2S format
static constexpr uint16_t WM8731_FMT_16BIT   = 0x00u;  // 16-bit IWL
static constexpr uint16_t WM8731_FMT_MASTER  = (1u << 6);

// Sampling control (R8): USB=0 (normal), BOSR, SR[3:0], CLKIDIV2, CLKODIV2
// For 32 kHz from 12 MHz MCLK: MCLK/384 mode
// USB mode=0, BOSR=1 (384fs), SR=0110 (32 kHz)
static constexpr uint16_t WM8731_SRATE_32KHZ = 0x18u;  // SR=0110, BOSR=1, USB=0 → 32 kHz @ 12 MHz

// ---------------------------------------------------------------------------
// Write one WM8731 register over I2C
// reg: 7-bit register address
// val: 9-bit data value
// ---------------------------------------------------------------------------
static bool wm8731_write_reg(uint8_t reg, uint16_t val) {
    // I2C frame: 2 bytes = {(reg << 1) | (val >> 8), val & 0xFF}
    uint8_t buf[2] = {
        static_cast<uint8_t>((static_cast<uint16_t>(reg) << 1) | ((val >> 8) & 0x01u)),
        static_cast<uint8_t>(val & 0xFFu)
    };
    return i2c_write(AUDIO_I2C_BUS, AUDIO_WM8731_ADDR, buf, 2);
}

// ---------------------------------------------------------------------------
// Configure I2S0 peripheral on LPC4320
// I2S master, 16-bit, 32 kHz, MCLK from Si5351C (12 MHz on SGPIO8/CLK2)
//
// I2S0 TX bit rate calculation:
//   TXRATE: X/Y fractional rate = MCLK_out / MCLK_in
//   For BCLK = 2 * 32 * 16 = 1.024 MHz from 12 MHz:
//     Use TXBITRATE = (MCLK/BCLK) - 1 = (12000000/1024000) - 1 ≈ 11
//   LPC43xx I2S bitrate divider: BCLK = MCLK / (2 * (X + 1))
//   For 32 kHz, 16-bit stereo: BCLK = 32000 * 32 = 1.024 MHz
//   TXBITRATE divider = 12000000 / (2 * 1024000) - 1 = 5.something ≈ 5
// ---------------------------------------------------------------------------
static void i2s0_init() {
    // Configure SCU mux for I2S0 pins
    scu_set_pinmode(I2S_SCK_SCU_GRP, I2S_SCK_SCU_PIN, I2S_SCK_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_FAST);
    scu_set_pinmode(I2S_WS_SCU_GRP,  I2S_WS_SCU_PIN,  I2S_WS_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_FAST);
    scu_set_pinmode(I2S_SDA_SCU_GRP, I2S_SDA_SCU_PIN, I2S_SDA_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_FAST);

    // Enable I2S0 clock via CCU1 (APB1 clock source is already PLL1)
    // CCU1 branch for I2S0: offset 0x200 in CCU1
    volatile uint32_t* ccu1_i2s = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x200u);
    *ccu1_i2s = CCU_CLK_RUN | CCU_CLK_AUTO;

    // Set I2S0 base clock source to PLL1 in CGU
    volatile uint32_t* cgu_i2s0 = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x090u);
    *cgu_i2s0 = CGU_BASE_CLK_AUTOBLOCK | CGU_CLK_SRC_PLL1;

    I2S_Type* i2s = LPC_I2S0;

    // Reset I2S TX: set STOP and RESET bits
    i2s->DAO = I2S_DAO_STOP | I2S_DAO_RESET;

    // DAO configuration:
    //   WORDWIDTH = 16 bit
    //   MONO = 0 (stereo — left/right)
    //   STOP = 0 (enabled)
    //   RESET = 0
    //   WS_SEL = 0 (I2S master — LPC4320 generates WS)
    //   WS_HALFPERIOD = 15 (for 16-bit = 16 - 1 = 15 BCLK cycles per half)
    //   MUTE = 0
    i2s->DAO = I2S_DAO_WORDWIDTH_16 |
               (0u << 5) |   // WS_SEL = master
               (15u << 6);   // WS_HALFPERIOD = 15 (bits 14:6)

    // TX bit rate: MCLK / (BCLK_prescale * 2)
    // BCLK = 32000 * 32 = 1024000 Hz
    // MCLK = 204 MHz (PLL1, but I2S has its own rate dividers)
    // Use TXRATE fractional: for simplicity set X=1, Y=1 (no fraction, use TXBITRATE)
    i2s->TXRATE    = (1u << 8) | 1u;  // Y=1, X=1 (no fractional adjustment)

    // TXBITRATE: BCLK = MCLK / (2 * (TXBITRATE + 1))
    // MCLK here is the I2S peripheral clock which we set from PLL1 / 204 MHz.
    // Target BCLK = 1.024 MHz
    // TXBITRATE + 1 = 204000000 / (2 * 1024000) ≈ 99.6 → use 99
    // Resulting BCLK = 204000000 / (2*100) = 1.02 MHz (close enough for WM8731)
    i2s->TXBITRATE = 99u;

    // TXMODE: use internal clock (bit 0 = 0: use TXBITRATE), no MCLK output
    i2s->TXMODE    = 0u;

    // Enable I2S TX (clear STOP)
    i2s->DAO &= ~I2S_DAO_STOP;
}

// ---------------------------------------------------------------------------
// audio_init
// ---------------------------------------------------------------------------
bool audio_init() {
    bool ok = true;

    // Reset WM8731
    ok &= wm8731_write_reg(WM8731_REG_RESET, 0x000u);

    // Power down everything except DAC and headphone out
    // Disable: line in, mic, ADC, OSC, CLKOUT (save power)
    // Keep:    DAC, OUT, core
    ok &= wm8731_write_reg(WM8731_REG_PDCTRL,
        WM8731_PD_LINEIN |
        WM8731_PD_MIC    |
        WM8731_PD_ADC    |
        WM8731_PD_OSC    |
        WM8731_PD_CLKOUT);

    // Headphone volume: 100 (0dB) on both channels, simultaneous load
    // Bits [6:0] = volume (79 = 0dB, 127 = +6dB, 48 = -73dB)
    // Bit  [7]   = LRHPBOTH (simultaneous update)
    static constexpr uint16_t HP_VOL = 100u;
    ok &= wm8731_write_reg(WM8731_REG_LHPVOL, HP_VOL | (1u << 8));  // LRHPBOTH
    ok &= wm8731_write_reg(WM8731_REG_RHPVOL, HP_VOL | (1u << 8));

    // Analogue path: DAC selected, output = line-out + headphone
    // MICBOOST=0, MUTEMIC=1, INSEL=1(mic), BYPASS=0, DACSEL=1, SIDETONE=0
    ok &= wm8731_write_reg(WM8731_REG_AAPCTRL, 0x012u); // DACSEL | MUTEMIC

    // Digital path: no HPF, no de-emphasis, no soft mute
    ok &= wm8731_write_reg(WM8731_REG_DAPCTRL, 0x000u);

    // Digital audio interface: I2S mode, 16-bit, master
    ok &= wm8731_write_reg(WM8731_REG_DAIFMT,
        WM8731_FMT_I2S | WM8731_FMT_16BIT | WM8731_FMT_MASTER);

    // Sampling rate: 32 kHz from 12 MHz MCLK (normal mode, 384fs)
    ok &= wm8731_write_reg(WM8731_REG_SRATE, WM8731_SRATE_32KHZ);

    // Activate codec (start clocking audio)
    ok &= wm8731_write_reg(WM8731_REG_ACTIVE, 0x001u);

    // Configure LPC4320 I2S0 peripheral
    i2s0_init();

    return ok;
}

// ---------------------------------------------------------------------------
// audio_play_samples — push PCM samples into I2S TX FIFO
// Duplicates mono sample to left/right channels (stereo frame).
// ---------------------------------------------------------------------------
void audio_play_samples(const int16_t* samples, size_t count) {
    if (!samples || count == 0) return;

    I2S_Type* i2s = LPC_I2S0;

    for (size_t i = 0; i < count; i++) {
        int16_t s = samples[i];

        // Pack left+right into a 32-bit I2S frame (16-bit per channel)
        // Upper 16 bits = left, lower 16 bits = right
        uint32_t frame = (static_cast<uint32_t>(static_cast<uint16_t>(s)) << 16) |
                          static_cast<uint32_t>(static_cast<uint16_t>(s));

        // Wait for TX FIFO to have space (STATE register: TX_LEVEL bits [11:8])
        // TX FIFO depth = 8 words; we wait until there is at least 1 free slot.
        while (((i2s->STATE >> 16) & 0xFu) >= 7u) {
            // FIFO nearly full — spin
        }

        i2s->TXFIFO = frame;
    }
}

// ---------------------------------------------------------------------------
// audio_set_volume
// ---------------------------------------------------------------------------
void audio_set_volume(uint8_t vol) {
    // WM8731 headphone volume range 0–127 (0 = mute threshold below)
    uint16_t v = vol & 0x7Fu;
    wm8731_write_reg(WM8731_REG_LHPVOL, v | (1u << 8));  // LRHPBOTH
    wm8731_write_reg(WM8731_REG_RHPVOL, v | (1u << 8));
}

// ---------------------------------------------------------------------------
// audio_set_mute
// ---------------------------------------------------------------------------
void audio_set_mute(bool mute) {
    // Digital audio path control: bit 3 = DACMU (mute)
    uint16_t reg = mute ? (1u << 3) : 0u;
    wm8731_write_reg(WM8731_REG_DAPCTRL, reg);
}
