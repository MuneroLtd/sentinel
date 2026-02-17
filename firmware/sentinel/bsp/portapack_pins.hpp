// SPDX-License-Identifier: MIT
// Project Sentinel — PortaPack H4M + HackRF One Pin Assignments
//
// SCU notation: scu_sfs(group, pin) maps to SCU_BASE + 0x80*group + 4*pin
// GPIO notation: GPIO port and bit — LPC4320 has ports 0..7
//
// Pin assignments verified against:
//  - portapack-mayhem/mayhem-firmware board.cpp (SCU pin_mux array)
//  - greatscottgadgets/hackrf firmware/common/portapack.c
//  - NXP UM10503 LPC43xx User Manual pin description tables
//
// The H4M (compared to H1/H2) replaces the parallel 8080 LCD bus (through
// the CPLD) with a direct SPI ILI9341 connection on SSP1.  The CPLD is still
// present for SGPIO sample routing.
//
// Bus assignment summary:
//   SSP1 = RF ICs (MAX2837, MAX5864) and LCD ILI9341
//   SSP0 = SD card and RFFC5072 mixer
//   I2C0 = Si5351C clock gen + WM8731 audio codec
//   I2C1 = ESP32 bridge (H4M feature)
//   UART0 = Debug serial

#pragma once
#include <cstdint>

// ===========================================================================
// SSP1 — shared by LCD (ILI9341) and RF transceivers (MAX2837, MAX5864)
// ===========================================================================
static constexpr uint8_t SSP1_BUS           = 1;

// P1_19 → SSP1_SCK  (SCU func 1)
static constexpr uint8_t SSP1_SCK_SCU_GRP   = 1;
static constexpr uint8_t SSP1_SCK_SCU_PIN   = 19;
static constexpr uint8_t SSP1_SCK_SCU_FUNC  = 1;

// P1_4  → SSP1_MOSI (SCU func 5)
static constexpr uint8_t SSP1_MOSI_SCU_GRP  = 1;
static constexpr uint8_t SSP1_MOSI_SCU_PIN  = 4;
static constexpr uint8_t SSP1_MOSI_SCU_FUNC = 5;

// P1_3  → SSP1_MISO (SCU func 5)
static constexpr uint8_t SSP1_MISO_SCU_GRP  = 1;
static constexpr uint8_t SSP1_MISO_SCU_PIN  = 3;
static constexpr uint8_t SSP1_MISO_SCU_FUNC = 5;

// ===========================================================================
// SSP0 — SD card (SPI mode) and RFFC5072
// ===========================================================================
static constexpr uint8_t SSP0_BUS           = 0;

// P3_3  → SSP0_SCK  (SCU func 2)
static constexpr uint8_t SSP0_SCK_SCU_GRP   = 3;
static constexpr uint8_t SSP0_SCK_SCU_PIN   = 3;
static constexpr uint8_t SSP0_SCK_SCU_FUNC  = 2;

// P3_7  → SSP0_MOSI (SCU func 2)
static constexpr uint8_t SSP0_MOSI_SCU_GRP  = 3;
static constexpr uint8_t SSP0_MOSI_SCU_PIN  = 7;
static constexpr uint8_t SSP0_MOSI_SCU_FUNC = 2;

// P3_6  → SSP0_MISO (SCU func 2)
static constexpr uint8_t SSP0_MISO_SCU_GRP  = 3;
static constexpr uint8_t SSP0_MISO_SCU_PIN  = 6;
static constexpr uint8_t SSP0_MISO_SCU_FUNC = 2;

// ===========================================================================
// LCD ILI9341 — SPI via SSP1 (H4M direct SPI mode)
// ===========================================================================
static constexpr uint8_t LCD_SSP_BUS        = SSP1_BUS;

// LCD CS: P1_20 → GPIO0[15]  (active-low, SCU func 0 = GPIO)
static constexpr uint8_t LCD_CS_SCU_GRP     = 1;
static constexpr uint8_t LCD_CS_SCU_PIN     = 20;
static constexpr uint8_t LCD_CS_SCU_FUNC    = 0;
static constexpr uint8_t LCD_CS_GPIO_PORT   = 0;
static constexpr uint8_t LCD_CS_GPIO_PIN    = 15;

// LCD DC (Data/nCommand): P6_4 → GPIO3[3]  (high = data, low = command)
// Note: P6_4 is shared with RFFC5072 SDATA on H1/H2; H4M dedicates it to LCD_DC.
static constexpr uint8_t LCD_DC_SCU_GRP     = 6;
static constexpr uint8_t LCD_DC_SCU_PIN     = 4;
static constexpr uint8_t LCD_DC_SCU_FUNC    = 0;
static constexpr uint8_t LCD_DC_GPIO_PORT   = 3;
static constexpr uint8_t LCD_DC_GPIO_PIN    = 3;

// LCD RST (active-low reset): P2_8 → GPIO5[7]
static constexpr uint8_t LCD_RST_SCU_GRP    = 2;
static constexpr uint8_t LCD_RST_SCU_PIN    = 8;
static constexpr uint8_t LCD_RST_SCU_FUNC   = 0;
static constexpr uint8_t LCD_RST_GPIO_PORT  = 5;
static constexpr uint8_t LCD_RST_GPIO_PIN   = 7;

// ===========================================================================
// SD Card — SPI mode via SSP0
// ===========================================================================
static constexpr uint8_t SD_SSP_BUS         = SSP0_BUS;

// SD CS: P1_6 → GPIO0[6]  (active-low)
static constexpr uint8_t SD_CS_SCU_GRP      = 1;
static constexpr uint8_t SD_CS_SCU_PIN      = 6;
static constexpr uint8_t SD_CS_SCU_FUNC     = 0;
static constexpr uint8_t SD_CS_GPIO_PORT    = 0;
static constexpr uint8_t SD_CS_GPIO_PIN     = 6;

// ===========================================================================
// RFFC5072 Mixer — SSP0 bus, dedicated LE (latch enable / CS)
// ===========================================================================
// RFFC LE: P5_7 → GPIO2[7]  (active-low strobe to latch SPI word)
static constexpr uint8_t RFFC_SSP_BUS       = SSP0_BUS;
static constexpr uint8_t RFFC_LE_SCU_GRP    = 5;
static constexpr uint8_t RFFC_LE_SCU_PIN    = 7;
static constexpr uint8_t RFFC_LE_SCU_FUNC   = 0;
static constexpr uint8_t RFFC_LE_GPIO_PORT  = 2;
static constexpr uint8_t RFFC_LE_GPIO_PIN   = 7;
// Convenience aliases used by rffc5072.cpp
static constexpr uint8_t PORT_RFFC_CS       = RFFC_LE_GPIO_PORT;
static constexpr uint8_t PIN_RFFC_CS        = RFFC_LE_GPIO_PIN;

// ===========================================================================
// MAX2837 Transceiver — SSP1 bus, CS_XCVR
// ===========================================================================
static constexpr uint8_t MAX2837_SSP_BUS    = SSP1_BUS;

// MAX2837 CS (CS_XCVR): separate GPIO from LCD_CS on H4M
// P1_20 usage: on H1/H2 this is CS_XCVR; H4M provides a dedicated GPIO.
// Using P4_0 → GPIO2[0] as the MAX2837 CS on H4M (confirmed by H4M board files).
static constexpr uint8_t MAX2837_CS_SCU_GRP = 4;
static constexpr uint8_t MAX2837_CS_SCU_PIN = 0;
static constexpr uint8_t MAX2837_CS_SCU_FUNC= 0;
static constexpr uint8_t MAX2837_CS_GPIO_PORT= 2;
static constexpr uint8_t MAX2837_CS_GPIO_PIN = 0;
// Convenience aliases used by max2837.cpp
static constexpr uint8_t PORT_MAX2837_CS    = MAX2837_CS_GPIO_PORT;
static constexpr uint8_t PIN_MAX2837_CS     = MAX2837_CS_GPIO_PIN;

// MAX5864 ADC/DAC — SSP1 bus (shared with MAX2837 and LCD), dedicated CS
// CS (CS_AD): P5_7 → GPIO2[7]  (shared with RFFC LE on H1/H2)
// H4M: uses P5_6 → GPIO2[6]
static constexpr uint8_t MAX5864_SSP_BUS    = SSP1_BUS;
static constexpr uint8_t MAX5864_CS_SCU_GRP = 5;
static constexpr uint8_t MAX5864_CS_SCU_PIN = 6;
static constexpr uint8_t MAX5864_CS_SCU_FUNC= 0;
static constexpr uint8_t MAX5864_CS_GPIO_PORT= 2;
static constexpr uint8_t MAX5864_CS_GPIO_PIN = 6;
// Convenience aliases used by max5864.cpp
static constexpr uint8_t PORT_MAX5864_CS     = MAX5864_CS_GPIO_PORT;
static constexpr uint8_t PIN_MAX5864_CS      = MAX5864_CS_GPIO_PIN;

// ===========================================================================
// I2C0 (APB1) — Si5351C clock generator + WM8731 audio codec
// I2C0 uses the special fast-mode I2C pins (SFSI2C0 register).
// No standard SCU group pins needed — handled via dedicated I2C register.
// ===========================================================================
static constexpr uint8_t I2C0_BUS           = 0;
static constexpr uint8_t SI5351_I2C_BUS     = I2C0_BUS;
static constexpr uint8_t SI5351_I2C_ADDR    = 0x60;  // 7-bit (ADD=GND)
static constexpr uint8_t AUDIO_I2C_BUS      = I2C0_BUS;
static constexpr uint8_t AUDIO_WM8731_ADDR  = 0x1A;  // 7-bit (CSB=0)

// SFSI2C0 register for fast-mode I2C0 pins (SCU_BASE + 0xC84)
// Bit 3:   ZIF (glitch filter 3 ns)
// Bit 6,7: EZI, EHS (input buffer enable, high-speed I2C mode)
static constexpr uint32_t SCU_SFSI2C0_OFFSET = 0xC84u;

// ===========================================================================
// I2C1 (APB3) — ESP32 bridge (H4M exclusive)
// ===========================================================================
static constexpr uint8_t I2C1_BUS           = 1;
static constexpr uint8_t ESP32_I2C_BUS      = I2C1_BUS;
static constexpr uint8_t ESP32_I2C_ADDR     = 0x42;  // 7-bit

// I2C1 SDA: P2_3 → SCU func 1 (I2C1_SDA)
static constexpr uint8_t I2C1_SDA_SCU_GRP   = 2;
static constexpr uint8_t I2C1_SDA_SCU_PIN   = 3;
static constexpr uint8_t I2C1_SDA_SCU_FUNC  = 1;

// I2C1 SCL: P2_4 → SCU func 1 (I2C1_SCL)
static constexpr uint8_t I2C1_SCL_SCU_GRP   = 2;
static constexpr uint8_t I2C1_SCL_SCU_PIN   = 4;
static constexpr uint8_t I2C1_SCL_SCU_FUNC  = 1;

// ===========================================================================
// WM8731 Audio Codec — I2S data path (I2S0 on APB1)
// I2C control on I2C0 above.
// ===========================================================================
// I2S0_TX_SCK (BCLK): P3_0 → SCU func 2
static constexpr uint8_t I2S_SCK_SCU_GRP    = 3;
static constexpr uint8_t I2S_SCK_SCU_PIN    = 0;
static constexpr uint8_t I2S_SCK_SCU_FUNC   = 2;

// I2S0_TX_WS (LRCLK): P3_1 → SCU func 2
static constexpr uint8_t I2S_WS_SCU_GRP     = 3;
static constexpr uint8_t I2S_WS_SCU_PIN     = 1;
static constexpr uint8_t I2S_WS_SCU_FUNC    = 2;

// I2S0_TX_SDA (DATA): P3_2 → SCU func 0
static constexpr uint8_t I2S_SDA_SCU_GRP    = 3;
static constexpr uint8_t I2S_SDA_SCU_PIN    = 2;
static constexpr uint8_t I2S_SDA_SCU_FUNC   = 0;

// WM8731 MCLK: provided by Si5351C CLK0 output.
// On HackRF SGPIO8/CLK2 (P4_7 func 6) routes 12 MHz to WM8731 MCLK pin.
static constexpr uint32_t SI5351_XTAL_HZ     = 25000000u;
static constexpr uint32_t WM8731_MCLK_HZ     = 12000000u;
static constexpr uint32_t AUDIO_SAMPLE_RATE  = 32000u;

// ===========================================================================
// Debug UART0 — USART0 on APB0
// Available on PortaPack expansion header / test pads.
// ===========================================================================
// UART0_TXD: P2_0 → SCU func 1
static constexpr uint8_t UART0_TX_SCU_GRP   = 2;
static constexpr uint8_t UART0_TX_SCU_PIN   = 0;
static constexpr uint8_t UART0_TX_SCU_FUNC  = 1;

// UART0_RXD: P2_1 → SCU func 1
static constexpr uint8_t UART0_RX_SCU_GRP   = 2;
static constexpr uint8_t UART0_RX_SCU_PIN   = 1;
static constexpr uint8_t UART0_RX_SCU_FUNC  = 1;

// ===========================================================================
// Navigation input (H4M)
// H4M uses a touchscreen + rotary encoder routed through the EPM240 CPLD —
// NOT direct GPIO pins.  A proper nav driver will be added when the EPM240
// CPLD data-bus driver is implemented.  The old P0_0–P0_4 assignments were
// incorrect: those pins are SGPIO data lines on HackRF.
// ===========================================================================

// ===========================================================================
// SGPIO data bus (8-bit parallel from XC2C64A CPLD to LPC4320)
// Pin → SGPIO function (SCU func number from UM10503 Table 169)
//
// Data path: MAX5864 ADC → XC2C64A CPLD → SGPIO parallel bus → GPDMA → memory
// The CPLD provides an external clock on SGPIO8 and 8 data bits on SGPIO0–7.
// ===========================================================================
// SGPIO0: P0_0 → SCU func 3
static constexpr uint8_t SGPIO0_SCU_GRP   = 0;
static constexpr uint8_t SGPIO0_SCU_PIN   = 0;
static constexpr uint8_t SGPIO0_SCU_FUNC  = 3;

// SGPIO1: P0_1 → SCU func 3
static constexpr uint8_t SGPIO1_SCU_GRP   = 0;
static constexpr uint8_t SGPIO1_SCU_PIN   = 1;
static constexpr uint8_t SGPIO1_SCU_FUNC  = 3;

// SGPIO2: P1_15 → SCU func 2
static constexpr uint8_t SGPIO2_SCU_GRP   = 1;
static constexpr uint8_t SGPIO2_SCU_PIN   = 15;
static constexpr uint8_t SGPIO2_SCU_FUNC  = 2;

// SGPIO3: P1_16 → SCU func 2
static constexpr uint8_t SGPIO3_SCU_GRP   = 1;
static constexpr uint8_t SGPIO3_SCU_PIN   = 16;
static constexpr uint8_t SGPIO3_SCU_FUNC  = 2;

// SGPIO4: P1_0 → SCU func 6
static constexpr uint8_t SGPIO4_SCU_GRP   = 1;
static constexpr uint8_t SGPIO4_SCU_PIN   = 0;
static constexpr uint8_t SGPIO4_SCU_FUNC  = 6;

// SGPIO5: P1_17 → SCU func 6
static constexpr uint8_t SGPIO5_SCU_GRP   = 1;
static constexpr uint8_t SGPIO5_SCU_PIN   = 17;
static constexpr uint8_t SGPIO5_SCU_FUNC  = 6;

// SGPIO6: P1_1 → SCU func 3
static constexpr uint8_t SGPIO6_SCU_GRP   = 1;
static constexpr uint8_t SGPIO6_SCU_PIN   = 1;
static constexpr uint8_t SGPIO6_SCU_FUNC  = 3;

// SGPIO7: P1_2 → SCU func 3
static constexpr uint8_t SGPIO7_SCU_GRP   = 1;
static constexpr uint8_t SGPIO7_SCU_PIN   = 2;
static constexpr uint8_t SGPIO7_SCU_FUNC  = 3;

// SGPIO8 (clock from CPLD): dedicated CLK2 pin
// CLK2 is a special clock input pin, not a regular P-group pin.
// Its SCU register is at SCU_BASE + 0xC08 (not addressable via scu_sfs).
static constexpr uint32_t SCU_SFSCLK2_OFFSET = 0xC08u;

// ===========================================================================
// CPLD JTAG interface (for CPLD programming / bitstream update)
// ===========================================================================
// TMS: P6_5 → GPIO3[4]
static constexpr uint8_t CPLD_TMS_SCU_GRP  = 6;
static constexpr uint8_t CPLD_TMS_SCU_PIN  = 5;
static constexpr uint8_t CPLD_TMS_GPIO_PORT= 3;
static constexpr uint8_t CPLD_TMS_GPIO_PIN = 4;

// TCK: P6_1 → GPIO3[0]
static constexpr uint8_t CPLD_TCK_SCU_GRP  = 6;
static constexpr uint8_t CPLD_TCK_SCU_PIN  = 1;
static constexpr uint8_t CPLD_TCK_GPIO_PORT= 3;
static constexpr uint8_t CPLD_TCK_GPIO_PIN = 0;

// TDI: P6_2 → GPIO3[1]
static constexpr uint8_t CPLD_TDI_SCU_GRP  = 6;
static constexpr uint8_t CPLD_TDI_SCU_PIN  = 2;
static constexpr uint8_t CPLD_TDI_GPIO_PORT= 3;
static constexpr uint8_t CPLD_TDI_GPIO_PIN = 1;

// TDO: P1_5 (P9_5 alt) → GPIO1[8]
static constexpr uint8_t CPLD_TDO_SCU_GRP  = 1;
static constexpr uint8_t CPLD_TDO_SCU_PIN  = 5;
static constexpr uint8_t CPLD_TDO_GPIO_PORT= 1;
static constexpr uint8_t CPLD_TDO_GPIO_PIN = 8;

// Convenience aliases used by cpld.cpp
static constexpr uint8_t PORT_CPLD_TMS     = CPLD_TMS_GPIO_PORT;
static constexpr uint8_t PIN_CPLD_TMS      = CPLD_TMS_GPIO_PIN;
static constexpr uint8_t PORT_CPLD_TCK     = CPLD_TCK_GPIO_PORT;
static constexpr uint8_t PIN_CPLD_TCK      = CPLD_TCK_GPIO_PIN;
static constexpr uint8_t PORT_CPLD_TDI     = CPLD_TDI_GPIO_PORT;
static constexpr uint8_t PIN_CPLD_TDI      = CPLD_TDI_GPIO_PIN;
static constexpr uint8_t PORT_CPLD_TDO     = CPLD_TDO_GPIO_PORT;
static constexpr uint8_t PIN_CPLD_TDO      = CPLD_TDO_GPIO_PIN;
