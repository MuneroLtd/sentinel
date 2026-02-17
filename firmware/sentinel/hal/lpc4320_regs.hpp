// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 Peripheral Register Definitions
// Based on NXP UM10503 LPC43xx User Manual Rev 1.9
//
// All peripheral bases are compile-time constexpr pointers.
// Peripheral structs are packed and declared volatile for correct HW access.
// No dependency on any external SDK.

#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// Helper: typed peripheral pointer from base address
// ---------------------------------------------------------------------------
template<typename T>
static inline T* reg_ptr(uint32_t base) {
    return reinterpret_cast<T*>(base);
}

// ---------------------------------------------------------------------------
// CGU — Clock Generation Unit  (UM10503 §12)
// Base: 0x40050000
// ---------------------------------------------------------------------------
struct CGU_Type {
    uint32_t FREQ_MON;         // 0x014  Frequency monitor
    uint32_t XTAL_OSC_CTRL;    // 0x018  Crystal oscillator control
    uint32_t PLL0USB_STAT;     // 0x01C  PLL0 USB status
    uint32_t PLL0USB_CTRL;     // 0x020  PLL0 USB control
    uint32_t PLL0USB_MDIV;     // 0x024  PLL0 USB M-divider
    uint32_t PLL0USB_NP_DIV;   // 0x028  PLL0 USB N/P-divider
    uint32_t PLL0AUDIO_STAT;   // 0x02C  PLL0 audio status
    uint32_t PLL0AUDIO_CTRL;   // 0x030  PLL0 audio control
    uint32_t PLL0AUDIO_MDIV;   // 0x034  PLL0 audio M-divider
    uint32_t PLL0AUDIO_NP_DIV; // 0x038  PLL0 audio N/P-divider
    uint32_t PLL0AUDIO_FRAC;   // 0x03C  PLL0 audio fractional
    uint32_t PLL1_STAT;        // 0x040  PLL1 status
    uint32_t PLL1_CTRL;        // 0x044  PLL1 control
    uint32_t IDIVA_CTRL;       // 0x048  Integer divider A
    uint32_t IDIVB_CTRL;       // 0x04C  Integer divider B
    uint32_t IDIVC_CTRL;       // 0x050  Integer divider C
    uint32_t IDIVD_CTRL;       // 0x054  Integer divider D
    uint32_t IDIVE_CTRL;       // 0x058  Integer divider E
    uint32_t BASE_SAFE_CLK;    // 0x05C  Safe base clock
    uint32_t BASE_USB0_CLK;    // 0x060  USB0 base clock
    uint32_t BASE_PERIPH_CLK;  // 0x064  Peripheral base clock
    uint32_t BASE_USB1_CLK;    // 0x068  USB1 base clock
    uint32_t BASE_M4_CLK;      // 0x06C  M4 base clock (also M0 when divided)
    uint32_t BASE_SPIFI_CLK;   // 0x070  SPIFI base clock
    uint32_t BASE_SPI_CLK;     // 0x074  SPI base clock
    uint32_t BASE_PHY_RX_CLK;  // 0x078
    uint32_t BASE_PHY_TX_CLK;  // 0x07C
    uint32_t BASE_APB1_CLK;    // 0x080  APB1 base clock (I2C0, I2S0)
    uint32_t BASE_APB3_CLK;    // 0x084  APB3 base clock (I2C1, QEI, etc.)
    uint32_t BASE_LCD_CLK;     // 0x088
    uint32_t BASE_ADCHS_CLK;   // 0x08C
    uint32_t BASE_SDIO_CLK;    // 0x090
    uint32_t BASE_SSP0_CLK;    // 0x094  SSP0 base clock
    uint32_t BASE_SSP1_CLK;    // 0x098  SSP1 base clock
    uint32_t BASE_UART0_CLK;   // 0x09C  UART0 base clock
    uint32_t BASE_UART1_CLK;   // 0x0A0
    uint32_t BASE_UART2_CLK;   // 0x0A4
    uint32_t BASE_UART3_CLK;   // 0x0A8
    uint32_t BASE_OUT_CLK;     // 0x0AC
    uint32_t _pad[4];
    uint32_t BASE_APLL_CLK;    // 0x0C0
    uint32_t BASE_CGU_OUT0_CLK;// 0x0C4
    uint32_t BASE_CGU_OUT1_CLK;// 0x0C8
};
// CGU struct is offset-indexed: actual registers start at 0x40050014
// We'll access via CGU_BASE + offsets directly for simplicity

static constexpr uint32_t CGU_BASE = 0x40050000u;

// CGU register addresses (absolute)
static inline volatile uint32_t* CGU_XTAL_OSC_CTRL   = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x018u);
static inline volatile uint32_t* CGU_PLL1_STAT        = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x040u);
static inline volatile uint32_t* CGU_PLL1_CTRL        = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x044u);
static inline volatile uint32_t* CGU_IDIVA_CTRL       = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x048u);
static inline volatile uint32_t* CGU_IDIVB_CTRL       = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x04Cu);
static inline volatile uint32_t* CGU_IDIVC_CTRL       = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x050u);
static inline volatile uint32_t* CGU_IDIVD_CTRL       = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x054u);
static inline volatile uint32_t* CGU_IDIVE_CTRL       = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x058u);
static inline volatile uint32_t* CGU_BASE_M4_CLK      = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x06Cu);
static inline volatile uint32_t* CGU_BASE_APB1_CLK    = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x080u);
static inline volatile uint32_t* CGU_BASE_APB3_CLK    = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x084u);
static inline volatile uint32_t* CGU_BASE_SSP0_CLK    = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x094u);
static inline volatile uint32_t* CGU_BASE_SSP1_CLK    = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x098u);
static inline volatile uint32_t* CGU_BASE_UART0_CLK   = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x09Cu);
static inline volatile uint32_t* CGU_BASE_PERIPH_CLK  = reinterpret_cast<volatile uint32_t*>(CGU_BASE + 0x064u);

// CGU_PLL1_CTRL bit fields
static constexpr uint32_t CGU_PLL1_CTRL_PD       = (1u <<  0);  // Power down
static constexpr uint32_t CGU_PLL1_CTRL_BYPASS   = (1u <<  1);  // Bypass (output = input)
static constexpr uint32_t CGU_PLL1_CTRL_FBSEL    = (1u <<  6);  // Feedback select
static constexpr uint32_t CGU_PLL1_CTRL_DIRECT   = (1u <<  7);  // Direct output (bypass output divider)
static constexpr uint32_t CGU_PLL1_CTRL_CLKEN    = (1u << 11);  // Block clock output
// PSEL [13:12], NSEL [23:22], MSEL [31:24]
static constexpr uint32_t CGU_PLL1_STAT_LOCK     = (1u <<  0);  // PLL locked

// CGU BASE_CLK: bits [28:24] = clock source
static constexpr uint32_t CGU_BASE_CLK_PD        = (1u <<  0);  // Power down base clock
static constexpr uint32_t CGU_BASE_CLK_AUTOBLOCK = (1u << 11);  // Auto-block during freq change
// Clock source values for bits [28:24]
static constexpr uint32_t CGU_CLK_SRC_IRC        = (0x01u << 24); // 12 MHz IRC
static constexpr uint32_t CGU_CLK_SRC_XTAL       = (0x06u << 24); // Crystal osc
static constexpr uint32_t CGU_CLK_SRC_PLL1       = (0x09u << 24); // PLL1 output

// ---------------------------------------------------------------------------
// CCU — Clock Control Unit  (UM10503 §13)
// CCU1 Base: 0x40051000  — core/M4 bus clocks
// CCU2 Base: 0x40052000  — peripheral clocks (SSP, UART, I2C, etc.)
// ---------------------------------------------------------------------------
static constexpr uint32_t CCU1_BASE = 0x40051000u;
static constexpr uint32_t CCU2_BASE = 0x40052000u;

// CCU branch clock register: bit 0 = RUN, bit 1 = AUTO, bit 2 = WAKEUP
static constexpr uint32_t CCU_CLK_RUN     = (1u << 0);
static constexpr uint32_t CCU_CLK_AUTO    = (1u << 1);
static constexpr uint32_t CCU_CLK_WAKEUP  = (1u << 2);

// CCU1 branch registers (absolute addresses)
static inline volatile uint32_t* CCU1_CLK_M4_CORE_CFG  = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x000u);
static inline volatile uint32_t* CCU1_CLK_M4_BUS_CFG   = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x000u);
static inline volatile uint32_t* CCU1_CLK_M4_BUS_STAT  = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x004u);
static inline volatile uint32_t* CCU1_CLK_M4_GPIO_CFG  = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x030u);
static inline volatile uint32_t* CCU1_CLK_M4_GPIO_STAT = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x034u);

// CCU2 branch registers (absolute addresses) — per UM10503 Table 75
static inline volatile uint32_t* CCU2_CLK_APB2_SSP1_CFG  = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x200u);
static inline volatile uint32_t* CCU2_CLK_APB2_SSP1_STAT = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x204u);
static inline volatile uint32_t* CCU2_CLK_APB0_SSP0_CFG  = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x700u);
static inline volatile uint32_t* CCU2_CLK_APB0_SSP0_STAT = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x704u);
static inline volatile uint32_t* CCU2_CLK_APB2_UART1_CFG = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x100u);
static inline volatile uint32_t* CCU2_CLK_APB0_UART0_CFG = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x600u);
static inline volatile uint32_t* CCU2_CLK_APB0_UART0_STAT= reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x604u);
static inline volatile uint32_t* CCU2_CLK_APB1_I2C1_CFG  = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x500u);
static inline volatile uint32_t* CCU2_CLK_APB1_I2C1_STAT = reinterpret_cast<volatile uint32_t*>(CCU2_BASE + 0x504u);
static inline volatile uint32_t* CCU1_CLK_APB1_I2C0_CFG  = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x208u);
static inline volatile uint32_t* CCU1_CLK_APB1_I2C0_STAT = reinterpret_cast<volatile uint32_t*>(CCU1_BASE + 0x20Cu);

// ---------------------------------------------------------------------------
// SCU — System Control Unit for pin muxing  (UM10503 §16)
// Base: 0x40086000
// Each SFS register: bits [2:0] = function, [4:3] = mode (pull-up/down),
//   bit 6 = input buffer enable (IBUF), bit 7 = slew rate (ZIF)
// ---------------------------------------------------------------------------
static constexpr uint32_t SCU_BASE = 0x40086000u;

// SFS register address: SCU_BASE + 0x80*group + 4*pin
inline volatile uint32_t* scu_sfs(uint8_t group, uint8_t pin) {
    return reinterpret_cast<volatile uint32_t*>(
        SCU_BASE + (static_cast<uint32_t>(group) * 0x80u) + (static_cast<uint32_t>(pin) * 4u));
}

// SCU SFS mode bits
static constexpr uint32_t SCU_MODE_INACT     = (0x0u << 3);  // No pull
static constexpr uint32_t SCU_MODE_PULLDOWN  = (0x1u << 3);  // Pull-down
static constexpr uint32_t SCU_MODE_PULLUP    = (0x2u << 3);  // Pull-up (default reset)
static constexpr uint32_t SCU_MODE_REPEATER  = (0x3u << 3);  // Repeater
static constexpr uint32_t SCU_MODE_IBUF      = (1u   << 6);  // Input buffer enable
static constexpr uint32_t SCU_MODE_ZIF_DIS   = (1u   << 7);  // Disable glitch filter
static constexpr uint32_t SCU_MODE_FAST      = (1u   << 5);  // Fast slew rate (for high-speed)

// ---------------------------------------------------------------------------
// GPIO — General-Purpose I/O  (UM10503 §17)
// Base: 0x400F4000
// LPC4320 has ports 0..7, each up to 32 pins.
// ---------------------------------------------------------------------------
struct GPIO_PORT_Type {
    uint8_t  B[256];          // 0x000-0x0FF  Byte pin registers (B[port][pin])
    uint32_t _pad0[960];
    uint32_t W[256];          // 0x1000-0x13FF Word pin registers
    uint32_t _pad1[768];
    uint32_t DIR[8];          // 0x2000  Direction registers
    uint32_t _pad2[24];
    uint32_t MASK[8];         // 0x2080  Mask registers
    uint32_t _pad3[24];
    uint32_t PIN[8];          // 0x2100  Port pin value
    uint32_t _pad4[24];
    uint32_t MPIN[8];         // 0x2180  Masked port value
    uint32_t _pad5[24];
    uint32_t SET[8];          // 0x2200  Atomic set
    uint32_t _pad6[24];
    uint32_t CLR[8];          // 0x2280  Atomic clear
    uint32_t _pad7[24];
    uint32_t NOT[8];          // 0x2300  Toggle
};

static constexpr uint32_t GPIO_PORT_BASE = 0x400F4000u;
static inline GPIO_PORT_Type* LPC_GPIO = reinterpret_cast<GPIO_PORT_Type*>(GPIO_PORT_BASE);

// ---------------------------------------------------------------------------
// SSP — Synchronous Serial Port  (SPI compatible)  (UM10503 §40)
// SSP0 Base: 0x40083000
// SSP1 Base: 0x400C5000
// ---------------------------------------------------------------------------
struct SSP_Type {
    volatile uint32_t CR0;    // 0x000  Control 0: data size, frame, CPOL, CPHA, SCR
    volatile uint32_t CR1;    // 0x004  Control 1: LBM, SSE, MS, SOD
    volatile uint32_t DR;     // 0x008  Data register
    volatile uint32_t SR;     // 0x00C  Status: TFE, TNF, RNE, RFF, BSY
    volatile uint32_t CPSR;   // 0x010  Clock prescale (must be even, 2..254)
    volatile uint32_t IMSC;   // 0x014  Interrupt mask set/clear
    volatile uint32_t RIS;    // 0x018  Raw interrupt status
    volatile uint32_t MIS;    // 0x01C  Masked interrupt status
    volatile uint32_t ICR;    // 0x020  Interrupt clear register
    volatile uint32_t DMACR;  // 0x024  DMA control
};

static constexpr uint32_t SSP0_BASE = 0x40083000u;
static constexpr uint32_t SSP1_BASE = 0x400C5000u;
static inline SSP_Type* LPC_SSP0 = reinterpret_cast<SSP_Type*>(SSP0_BASE);
static inline SSP_Type* LPC_SSP1 = reinterpret_cast<SSP_Type*>(SSP1_BASE);

// SSP CR0 fields
static constexpr uint32_t SSP_CR0_DSS(uint8_t bits) { return (static_cast<uint32_t>(bits) - 1u) & 0xFu; } // bits = 4..16
static constexpr uint32_t SSP_CR0_FRF_SPI   = (0u << 4);   // SPI frame format
static constexpr uint32_t SSP_CR0_FRF_TI    = (1u << 4);   // TI SSF
static constexpr uint32_t SSP_CR0_FRF_MW    = (2u << 4);   // Microwire
static constexpr uint32_t SSP_CR0_CPOL      = (1u << 6);   // Clock polarity
static constexpr uint32_t SSP_CR0_CPHA      = (1u << 7);   // Clock phase
// SCR [15:8] = serial clock rate

// SSP CR1 fields
static constexpr uint32_t SSP_CR1_LBM  = (1u << 0);  // Loopback mode
static constexpr uint32_t SSP_CR1_SSE  = (1u << 1);  // SSP enable
static constexpr uint32_t SSP_CR1_MS   = (1u << 2);  // Master(0) or Slave(1)
static constexpr uint32_t SSP_CR1_SOD  = (1u << 3);  // Slave output disable

// SSP SR fields
static constexpr uint32_t SSP_SR_TFE   = (1u << 0);  // TX FIFO empty
static constexpr uint32_t SSP_SR_TNF   = (1u << 1);  // TX FIFO not full
static constexpr uint32_t SSP_SR_RNE   = (1u << 2);  // RX FIFO not empty
static constexpr uint32_t SSP_SR_RFF   = (1u << 3);  // RX FIFO full
static constexpr uint32_t SSP_SR_BSY   = (1u << 4);  // Busy

// ---------------------------------------------------------------------------
// I2C — I2C Controller  (UM10503 §41)
// I2C0 Base: 0x400A1000  (on APB1)
// I2C1 Base: 0x400E0000  (on APB3)
// ---------------------------------------------------------------------------
struct I2C_Type {
    volatile uint32_t CONSET;  // 0x000  Control Set: AA, SI, STO, STA, I2EN
    volatile uint32_t STAT;    // 0x004  Status code
    volatile uint32_t DAT;     // 0x008  Data register
    volatile uint32_t ADR0;    // 0x00C  Slave address 0
    volatile uint32_t SCLH;    // 0x010  SCL high time
    volatile uint32_t SCLL;    // 0x014  SCL low time
    volatile uint32_t CONCLR;  // 0x018  Control Clear: AAC, SIC, STAC, I2ENC
};

static constexpr uint32_t I2C0_BASE = 0x400A1000u;
static constexpr uint32_t I2C1_BASE = 0x400E0000u;
static inline I2C_Type* LPC_I2C0 = reinterpret_cast<I2C_Type*>(I2C0_BASE);
static inline I2C_Type* LPC_I2C1 = reinterpret_cast<I2C_Type*>(I2C1_BASE);

// I2C CONSET/CONCLR bits
static constexpr uint32_t I2C_AA   = (1u << 2);  // Assert acknowledge
static constexpr uint32_t I2C_SI   = (1u << 3);  // Interrupt flag
static constexpr uint32_t I2C_STO  = (1u << 4);  // STOP
static constexpr uint32_t I2C_STA  = (1u << 5);  // START
static constexpr uint32_t I2C_I2EN = (1u << 6);  // I2C enable

// I2C status codes (STAT register)
static constexpr uint32_t I2C_STAT_START       = 0x08u;
static constexpr uint32_t I2C_STAT_REP_START   = 0x10u;
static constexpr uint32_t I2C_STAT_SLA_W_ACK   = 0x18u;
static constexpr uint32_t I2C_STAT_SLA_W_NACK  = 0x20u;
static constexpr uint32_t I2C_STAT_DAT_W_ACK   = 0x28u;
static constexpr uint32_t I2C_STAT_DAT_W_NACK  = 0x30u;
static constexpr uint32_t I2C_STAT_SLA_R_ACK   = 0x40u;
static constexpr uint32_t I2C_STAT_SLA_R_NACK  = 0x48u;
static constexpr uint32_t I2C_STAT_DAT_R_ACK   = 0x50u;
static constexpr uint32_t I2C_STAT_DAT_R_NACK  = 0x58u;

// ---------------------------------------------------------------------------
// USART — Universal Asynchronous Receiver Transmitter  (UM10503 §34)
// USART0 Base: 0x40081000
// ---------------------------------------------------------------------------
struct UART_Type {
    union {
        volatile uint32_t RBR;  // 0x000  Receive Buffer (DLAB=0, read)
        volatile uint32_t THR;  // 0x000  Transmit Holding (DLAB=0, write)
        volatile uint32_t DLL;  // 0x000  Divisor Latch LSB (DLAB=1)
    };
    union {
        volatile uint32_t DLM;  // 0x004  Divisor Latch MSB (DLAB=1)
        volatile uint32_t IER;  // 0x004  Interrupt Enable (DLAB=0)
    };
    union {
        volatile uint32_t IIR;  // 0x008  Interrupt ID (read)
        volatile uint32_t FCR;  // 0x008  FIFO Control (write)
    };
    volatile uint32_t LCR;      // 0x00C  Line Control
    volatile uint32_t MCR;      // 0x010  Modem Control
    volatile uint32_t LSR;      // 0x014  Line Status
    volatile uint32_t MSR;      // 0x018  Modem Status
    volatile uint32_t SCR;      // 0x01C  Scratch
    volatile uint32_t ACR;      // 0x020  Auto-baud control
    volatile uint32_t ICR;      // 0x024  IrDA control
    volatile uint32_t FDR;      // 0x028  Fractional divider
    volatile uint32_t OSR;      // 0x02C  Oversample selection
    uint32_t          _pad[5];
    volatile uint32_t TER;      // 0x044  Transmit enable
};

static constexpr uint32_t USART0_BASE = 0x40081000u;
static inline UART_Type* LPC_USART0 = reinterpret_cast<UART_Type*>(USART0_BASE);

// UART LCR bits
static constexpr uint32_t UART_LCR_WORD_5BIT  = (0u << 0);
static constexpr uint32_t UART_LCR_WORD_6BIT  = (1u << 0);
static constexpr uint32_t UART_LCR_WORD_7BIT  = (2u << 0);
static constexpr uint32_t UART_LCR_WORD_8BIT  = (3u << 0);
static constexpr uint32_t UART_LCR_STOP2      = (1u << 2);  // 2 stop bits
static constexpr uint32_t UART_LCR_PARITY_EN  = (1u << 3);
static constexpr uint32_t UART_LCR_DLAB       = (1u << 7);  // Divisor latch access

// UART LSR bits
static constexpr uint32_t UART_LSR_RDR    = (1u << 0);  // Receiver data ready
static constexpr uint32_t UART_LSR_THRE   = (1u << 5);  // Transmitter holding empty
static constexpr uint32_t UART_LSR_TEMT   = (1u << 6);  // Transmitter empty

// UART FCR bits
static constexpr uint32_t UART_FCR_FIFO_EN  = (1u << 0);
static constexpr uint32_t UART_FCR_RX_RST   = (1u << 1);
static constexpr uint32_t UART_FCR_TX_RST   = (1u << 2);

// ---------------------------------------------------------------------------
// CREG — Configuration Registers  (UM10503 §4.5)
// Base: 0x40043000
// Controls M0 subsystem reset and start address.
// ---------------------------------------------------------------------------
struct CREG_Type {
    uint32_t _reserved[52];
    volatile uint32_t M0APPTXEVENT;   // 0x0D0  M0APP TXEVENT register
    uint32_t _reserved2[11];
    volatile uint32_t M0APPMEMMAP;    // 0x100  M0APP memory remap (start address)
    uint32_t _reserved3;
    volatile uint32_t M0SUBTXEVENT;   // 0x108  M0SUB TXEVENT
    uint32_t _reserved4[5];
    volatile uint32_t M0SUBMEMMAP;    // 0x120  M0SUB memory remap
    volatile uint32_t CREG5;          // 0x124
    volatile uint32_t DMAMUX;         // 0x128
    volatile uint32_t FLASHCFGA;      // 0x12C
    volatile uint32_t FLASHCFGB;      // 0x130
    volatile uint32_t ETBCFG;         // 0x134
    volatile uint32_t CREG6;          // 0x138  EMC clock + M0APPRSTCTL
};
// For clarity, use absolute addresses based on UM10503 Table 10
static constexpr uint32_t CREG_BASE          = 0x40043000u;
static inline volatile uint32_t* CREG_M0APPMEMMAP  = reinterpret_cast<volatile uint32_t*>(CREG_BASE + 0x100u);
static inline volatile uint32_t* CREG_M0APPRSTCTL  = reinterpret_cast<volatile uint32_t*>(CREG_BASE + 0x104u);  // M0APP reset control
static inline volatile uint32_t* CREG_M0TXEVENT    = reinterpret_cast<volatile uint32_t*>(CREG_BASE + 0x01C4u); // per ipc_protocol.hpp
// Simpler alias used in main.cpp:
static inline volatile uint32_t* CREG_M0SUB_RST    = reinterpret_cast<volatile uint32_t*>(CREG_BASE + 0x104u);

// RGU — Reset Generation Unit  (UM10503 §5)
// Used to release / assert M0APP core reset
static constexpr uint32_t RGU_BASE  = 0x40053000u;
static inline volatile uint32_t* RGU_RESET_CTRL1    = reinterpret_cast<volatile uint32_t*>(RGU_BASE + 0x104u);
static constexpr uint32_t RGU_M0APP_RST                = (1u << 24); // bit 24 of RESET_CTRL1

// ---------------------------------------------------------------------------
// GPDMA — General Purpose DMA Controller  (UM10503 §§26-27)
// Base: 0x40002000
// ---------------------------------------------------------------------------
struct DMA_CH_Type {
    volatile uint32_t SRCADDR;
    volatile uint32_t DESTADDR;
    volatile uint32_t LLI;
    volatile uint32_t CONTROL;
    volatile uint32_t CONFIG;
    uint32_t          _pad[3];
};

struct GPDMA_Type {
    volatile uint32_t INTSTAT;        // 0x000
    volatile uint32_t INTTCSTAT;      // 0x004
    volatile uint32_t INTTCCLEAR;     // 0x008
    volatile uint32_t INTERRSTAT;     // 0x00C
    volatile uint32_t INTERRCLR;      // 0x010
    volatile uint32_t RAWINTTCSTAT;   // 0x014
    volatile uint32_t RAWINTERRSTAT;  // 0x018
    volatile uint32_t ENBLDCHNS;      // 0x01C
    volatile uint32_t SOFTBREQ;       // 0x020
    volatile uint32_t SOFTSREQ;       // 0x024
    volatile uint32_t SOFTLBREQ;      // 0x028
    volatile uint32_t SOFTLSREQ;      // 0x02C
    volatile uint32_t CONFIG;         // 0x030
    volatile uint32_t SYNC;           // 0x034
    uint32_t          _pad[50];
    DMA_CH_Type       CH[8];          // 0x100, 8 channels
};

static constexpr uint32_t GPDMA_BASE = 0x40002000u;
static inline GPDMA_Type* LPC_GPDMA  = reinterpret_cast<GPDMA_Type*>(GPDMA_BASE);

// GPDMA CONFIG
static constexpr uint32_t GPDMA_CFG_EN = (1u << 0);

// ---------------------------------------------------------------------------
// I2S — Inter-IC Sound  (UM10503 §42)
// I2S0 Base: 0x400A2000  (on APB1)
// ---------------------------------------------------------------------------
struct I2S_Type {
    volatile uint32_t DAO;     // 0x000  Digital Audio Output
    volatile uint32_t DAI;     // 0x004  Digital Audio Input
    volatile uint32_t TXFIFO;  // 0x008  TX FIFO
    volatile uint32_t RXFIFO;  // 0x00C  RX FIFO
    volatile uint32_t STATE;   // 0x010  Status: IRQ, DMAREQ1, DMAREQ2, RX/TX levels
    volatile uint32_t DMA1;    // 0x014  DMA1 config
    volatile uint32_t DMA2;    // 0x018  DMA2 config
    volatile uint32_t IRQ;     // 0x01C  IRQ config
    volatile uint32_t TXRATE;  // 0x020  TX bit rate
    volatile uint32_t RXRATE;  // 0x024  RX bit rate
    volatile uint32_t TXBITRATE; // 0x028  TX bit rate divider
    volatile uint32_t RXBITRATE; // 0x02C  RX bit rate divider
    volatile uint32_t TXMODE;  // 0x030  TX mode
    volatile uint32_t RXMODE;  // 0x034  RX mode
};

static constexpr uint32_t I2S0_BASE = 0x400A2000u;
static inline I2S_Type* LPC_I2S0 = reinterpret_cast<I2S_Type*>(I2S0_BASE);

// I2S DAO / DAI bit fields
static constexpr uint32_t I2S_DAO_WORDWIDTH_8    = (0u << 0);
static constexpr uint32_t I2S_DAO_WORDWIDTH_16   = (1u << 0);
static constexpr uint32_t I2S_DAO_WORDWIDTH_32   = (3u << 0);
static constexpr uint32_t I2S_DAO_MONO           = (1u << 2);
static constexpr uint32_t I2S_DAO_STOP           = (1u << 3);
static constexpr uint32_t I2S_DAO_RESET          = (1u << 4);
static constexpr uint32_t I2S_DAO_WS_SEL         = (1u << 5);   // WS = master (0) or slave (1)
static constexpr uint32_t I2S_DAO_MUTE           = (1u << 15);
