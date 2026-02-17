// SPDX-License-Identifier: MIT
// Project Sentinel — PortaPack H4M + HackRF One Pin Assignments
//
// SCU notation: scu_sfs(group, pin) maps to SCU_BASE + 0x80*group + 4*pin
// GPIO notation: GPIO port and bit — LPC4320 has ports 0..7
//
// Pin assignments verified against:
//  - portapack-mayhem/mayhem-firmware board.cpp (SCU pin_mux array)
//  - portapack-mayhem/mayhem-firmware portapack_hal.hpp (GPIO assignments)
//  - greatscottgadgets/hackrf firmware/common/portapack.c
//  - NXP UM10503 LPC43xx User Manual pin description tables
//
// The H4M uses an 8-bit parallel 8080-I bus routed through the EPM240 CPLD
// for LCD communication — identical to the original PortaPack H1/H2 design.
// The CPLD translates two 8-bit bus cycles into a single 16-bit LCD write.
//
// Bus assignment summary:
//   LCD     = 8-bit parallel via CPLD (GPIO3[8:15] data, GPIO1/5 control)
//   SSP1    = RF ICs (MAX2837, MAX5864)
//   SSP0    = SD card and RFFC5072 mixer
//   I2C0    = Si5351C clock gen + WM8731 audio codec
//   UART0   = Debug serial (shares P2_0/P2_1 with CPLD — see note below)
//
// NOTE: P2_0 and P2_1 are shared between UART0 and CPLD control signals
// (IO_STBX, ADDR).  UART0 works during early boot; lcd_init() reconfigures
// those pins for the LCD parallel bus, after which UART output ceases.
// P2_3 and P2_4 are LCD_TE and LCD_RDX (not I2C1 as previously assumed).

#pragma once
#include <cstdint>

// ===========================================================================
// SSP1 — RF transceivers (MAX2837, MAX5864)
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
// LCD ILI9341 — 8-bit parallel 8080-I bus via EPM240 CPLD
// ===========================================================================

// --- Control signals ---

// LCD_WRX: P2_9 → GPIO1[10] (SCU func 0) — Write strobe (active low)
static constexpr uint8_t LCD_WRX_SCU_GRP    = 2;
static constexpr uint8_t LCD_WRX_SCU_PIN    = 9;
static constexpr uint8_t LCD_WRX_SCU_FUNC   = 0;
static constexpr uint8_t LCD_WRX_GPIO_PORT  = 1;
static constexpr uint8_t LCD_WRX_GPIO_PIN   = 10;

// LCD_RDX: P2_4 → GPIO5[4] (SCU func 4) — Read strobe (active low)
static constexpr uint8_t LCD_RDX_SCU_GRP    = 2;
static constexpr uint8_t LCD_RDX_SCU_PIN    = 4;
static constexpr uint8_t LCD_RDX_SCU_FUNC   = 4;
static constexpr uint8_t LCD_RDX_GPIO_PORT  = 5;
static constexpr uint8_t LCD_RDX_GPIO_PIN   = 4;

// ADDR: P2_1 → GPIO5[1] (SCU func 4) — 0=command, 1=data (D/CX)
// NOTE: Shared with UART0_RXD — reconfigured by lcd_init()
static constexpr uint8_t LCD_ADDR_SCU_GRP   = 2;
static constexpr uint8_t LCD_ADDR_SCU_PIN   = 1;
static constexpr uint8_t LCD_ADDR_SCU_FUNC  = 4;
static constexpr uint8_t LCD_ADDR_GPIO_PORT = 5;
static constexpr uint8_t LCD_ADDR_GPIO_PIN  = 1;

// DIR: P2_13 → GPIO1[13] (SCU func 0) — 0=MCU writes, 1=CPLD drives
static constexpr uint8_t LCD_DIR_SCU_GRP    = 2;
static constexpr uint8_t LCD_DIR_SCU_PIN    = 13;
static constexpr uint8_t LCD_DIR_SCU_FUNC   = 0;
static constexpr uint8_t LCD_DIR_GPIO_PORT  = 1;
static constexpr uint8_t LCD_DIR_GPIO_PIN   = 13;

// IO_STBX: P2_0 → GPIO5[0] (SCU func 4) — CPLD IO register strobe
// NOTE: Shared with UART0_TXD — reconfigured by lcd_init()
static constexpr uint8_t LCD_STBX_SCU_GRP   = 2;
static constexpr uint8_t LCD_STBX_SCU_PIN   = 0;
static constexpr uint8_t LCD_STBX_SCU_FUNC  = 4;
static constexpr uint8_t LCD_STBX_GPIO_PORT = 5;
static constexpr uint8_t LCD_STBX_GPIO_PIN  = 0;

// LCD_TE: P2_3 → GPIO5[3] (SCU func 4) — Tearing effect (input from LCD)
static constexpr uint8_t LCD_TE_SCU_GRP     = 2;
static constexpr uint8_t LCD_TE_SCU_PIN     = 3;
static constexpr uint8_t LCD_TE_SCU_FUNC    = 4;
static constexpr uint8_t LCD_TE_GPIO_PORT   = 5;
static constexpr uint8_t LCD_TE_GPIO_PIN    = 3;

// --- 8-bit data bus: GPIO3[8:15] via P7_0..P7_7, all SCU func 0 ---
static constexpr uint8_t LCD_DATA_GPIO_PORT = 3;
static constexpr uint8_t LCD_DATA_SHIFT     = 8;    // data bits start at GPIO3 bit 8
static constexpr uint32_t LCD_DATA_MASK     = 0xFFu << LCD_DATA_SHIFT; // bits [15:8]

// Individual data bus pin SCU definitions (P7_0..P7_7 → GPIO3[8..15])
static constexpr uint8_t LCD_D0_SCU_GRP     = 7;
static constexpr uint8_t LCD_D0_SCU_PIN     = 0;    // → GPIO3[8]
static constexpr uint8_t LCD_D1_SCU_PIN     = 1;    // → GPIO3[9]
static constexpr uint8_t LCD_D2_SCU_PIN     = 2;    // → GPIO3[10]
static constexpr uint8_t LCD_D3_SCU_PIN     = 3;    // → GPIO3[11]
static constexpr uint8_t LCD_D4_SCU_PIN     = 4;    // → GPIO3[12]
static constexpr uint8_t LCD_D5_SCU_PIN     = 5;    // → GPIO3[13]
static constexpr uint8_t LCD_D6_SCU_PIN     = 6;    // → GPIO3[14]
static constexpr uint8_t LCD_D7_SCU_PIN     = 7;    // → GPIO3[15]
static constexpr uint8_t LCD_DATA_SCU_FUNC  = 0;    // All P7_x use func 0 for GPIO

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

// MAX5864 ADC/DAC — SSP1 bus (shared with MAX2837), dedicated CS
// CS (CS_AD): P5_6 → GPIO2[6]
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
// I2C1 (APB3) — ESP32 bridge (H4M feature)
// WARNING: I2C1 SDA/SCL (P2_3/P2_4) conflict with LCD_TE and LCD_RDX.
// I2C1 can only be used before lcd_init() or if a future mux scheme is added.
// ===========================================================================
static constexpr uint8_t I2C1_BUS           = 1;
static constexpr uint8_t ESP32_I2C_BUS      = I2C1_BUS;
static constexpr uint8_t ESP32_I2C_ADDR     = 0x42;  // 7-bit

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
// WARNING: P2_0 and P2_1 are shared with CPLD control signals (IO_STBX,
// ADDR).  UART works during early boot but lcd_init() reconfigures these
// pins for the LCD parallel bus, after which UART output ceases.
// ===========================================================================
// UART0_TXD: P2_0 → SCU func 1  (shared with LCD_STBX)
static constexpr uint8_t UART0_TX_SCU_GRP   = 2;
static constexpr uint8_t UART0_TX_SCU_PIN   = 0;
static constexpr uint8_t UART0_TX_SCU_FUNC  = 1;

// UART0_RXD: P2_1 → SCU func 1  (shared with LCD_ADDR)
static constexpr uint8_t UART0_RX_SCU_GRP   = 2;
static constexpr uint8_t UART0_RX_SCU_PIN   = 1;
static constexpr uint8_t UART0_RX_SCU_FUNC  = 1;

// ===========================================================================
// HackRF One LEDs (active high, accent LEDs on the HackRF PCB)
// These are directly on the HackRF board, not on the PortaPack.
// Useful for early-boot heartbeat before the LCD is initialised.
// ===========================================================================
// LED1: P4_1 → GPIO2[1]
static constexpr uint8_t LED1_SCU_GRP      = 4;
static constexpr uint8_t LED1_SCU_PIN      = 1;
static constexpr uint8_t LED1_SCU_FUNC     = 0;  // GPIO
static constexpr uint8_t LED1_GPIO_PORT    = 2;
static constexpr uint8_t LED1_GPIO_PIN     = 1;

// LED2: P4_2 → GPIO2[2]
static constexpr uint8_t LED2_SCU_GRP      = 4;
static constexpr uint8_t LED2_SCU_PIN      = 2;
static constexpr uint8_t LED2_SCU_FUNC     = 0;  // GPIO
static constexpr uint8_t LED2_GPIO_PORT    = 2;
static constexpr uint8_t LED2_GPIO_PIN     = 2;

// LED3: P6_12 → GPIO2[8]
static constexpr uint8_t LED3_SCU_GRP      = 6;
static constexpr uint8_t LED3_SCU_PIN      = 12;
static constexpr uint8_t LED3_SCU_FUNC     = 0;  // GPIO
static constexpr uint8_t LED3_GPIO_PORT    = 2;
static constexpr uint8_t LED3_GPIO_PIN     = 8;

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
// TMS: P1_8 → GPIO1[1] (PortaPack CPLD, NOT HackRF CPLD)
static constexpr uint8_t CPLD_TMS_SCU_GRP  = 1;
static constexpr uint8_t CPLD_TMS_SCU_PIN  = 8;
static constexpr uint8_t CPLD_TMS_GPIO_PORT= 1;
static constexpr uint8_t CPLD_TMS_GPIO_PIN = 1;

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
