// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// m0_startup.hpp — M0 core definitions, NVIC helpers, timing, and IPC sync macros
//
// Target: NXP LPC4320 Cortex-M0 sub-core, 204 MHz
// NOTE: Cortex-M0 has NO FPU, NO DWT, NO BASEPRI register.
// Timing is SysTick-based only.

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// M0 SRAM memory map
// ---------------------------------------------------------------------------

// M0 code+data executes from LPC4320 SRAM0 (first 6 KB of 8 KB)
// The top 2 KB of this 8 KB region is reserved for the M0 stack.
static constexpr uint32_t M0_SRAM_BASE   = 0x10080000u;
static constexpr uint32_t M0_SRAM_SIZE   = 8192u;          // 8 KB total
static constexpr uint32_t M0_STACK_TOP   = M0_SRAM_BASE + M0_SRAM_SIZE;  // 0x10082000
static constexpr uint32_t M0_STACK_SIZE  = 2048u;          // 2 KB stack
static constexpr uint32_t M0_CODE_BASE   = M0_SRAM_BASE;   // 0x10080000
static constexpr uint32_t M0_CODE_SIZE   = M0_SRAM_SIZE - M0_STACK_SIZE; // 6 KB

// ---------------------------------------------------------------------------
// LPC4320 peripheral base addresses (used by M0 without OS)
// ---------------------------------------------------------------------------

static constexpr uint32_t SSP0_BASE      = 0x40083000u;
static constexpr uint32_t SSP1_BASE      = 0x400C5000u;
static constexpr uint32_t GPDMA_BASE     = 0x40002000u;
static constexpr uint32_t SYSTICK_BASE   = 0xE000E010u;    // ARM SysTick (both cores)
static constexpr uint32_t NVIC_BASE      = 0xE000E100u;    // M0 NVIC
static constexpr uint32_t SCB_BASE       = 0xE000ED00u;    // M0 System Control Block

// GPDMA channel registers (channel 0 = RX ping, channel 1 = RX pong)
static constexpr uint32_t GPDMA_CH0_BASE = GPDMA_BASE + 0x100u;
static constexpr uint32_t GPDMA_CH1_BASE = GPDMA_BASE + 0x120u;

// ---------------------------------------------------------------------------
// Cortex-M0 SysTick registers
// ---------------------------------------------------------------------------

struct SysTickRegs {
    volatile uint32_t CSR;    // Control and Status Register
    volatile uint32_t RVR;    // Reload Value Register
    volatile uint32_t CVR;    // Current Value Register
    volatile uint32_t CALIB;  // Calibration Value Register
};

static inline SysTickRegs& SysTick() {
    return *reinterpret_cast<SysTickRegs*>(SYSTICK_BASE);
}

// SysTick CSR bits
static constexpr uint32_t SYSTICK_CSR_ENABLE    = (1u << 0);
static constexpr uint32_t SYSTICK_CSR_TICKINT   = (1u << 1);  // enable SysTick exception
static constexpr uint32_t SYSTICK_CSR_CLKSOURCE = (1u << 2);  // 1 = processor clock
static constexpr uint32_t SYSTICK_CSR_COUNTFLAG = (1u << 16); // set on wrap

// ---------------------------------------------------------------------------
// Cortex-M0 NVIC registers
// ---------------------------------------------------------------------------
// Cortex-M0 NVIC supports up to 32 external interrupts.
// Priority is 2-bit (4 levels: 0=highest, 3=lowest).
// IMPORTANT: M0 has NO BASEPRI — use PRIMASK only (global IRQ enable/disable).

struct NVICRegs {
    volatile uint32_t ISER;      // Interrupt Set-Enable Register     (offset 0x000)
    uint32_t          _r0[31];
    volatile uint32_t ICER;      // Interrupt Clear-Enable Register   (offset 0x080)
    uint32_t          _r1[31];
    volatile uint32_t ISPR;      // Interrupt Set-Pending Register    (offset 0x100)
    uint32_t          _r2[31];
    volatile uint32_t ICPR;      // Interrupt Clear-Pending Register  (offset 0x180)
    uint32_t          _r3[31];
    uint32_t          _r4[64];   // reserved (IABR not in M0)
    volatile uint32_t IPR[8];    // Interrupt Priority Registers       (offset 0x300)
};

static inline NVICRegs& NVIC() {
    return *reinterpret_cast<NVICRegs*>(NVIC_BASE);
}

// LPC4320 M0 IRQ numbers (subset used by baseband firmware)
// LPC4320 UM10503 Table 65 — M0 sub-core interrupt table
static constexpr uint32_t IRQ_GPDMA  = 2u;   // GPDMA combined interrupt → DMA TC/Error
static constexpr uint32_t IRQ_SSP0   = 12u;  // SSP0 (not used — DMA mode)
static constexpr uint32_t IRQ_TIMER3 = 20u;  // TIMER3 (spare timer for scanner dwell)

// NVIC helpers — Cortex-M0 only has a single ISER/ICER 32-bit word
static inline void nvic_enable_irq(uint32_t irq) {
    NVIC().ISER = (1u << irq);
}

static inline void nvic_disable_irq(uint32_t irq) {
    NVIC().ICER = (1u << irq);
}

static inline void nvic_clear_pending(uint32_t irq) {
    NVIC().ICPR = (1u << irq);
}

// Set 2-bit priority for IRQ n.  IPR[n/4] holds 4 × 8-bit priority fields;
// only the top 2 bits of each 8-bit field are implemented on Cortex-M0.
static inline void nvic_set_priority(uint32_t irq, uint32_t priority) {
    uint32_t reg   = irq / 4u;
    uint32_t shift = (irq % 4u) * 8u;
    uint32_t val   = NVIC().IPR[reg];
    val &= ~(0xFFu << shift);
    val |=  ((priority & 0x3u) << 6u) << shift;  // top 2 bits of byte
    NVIC().IPR[reg] = val;
}

// Global interrupt enable/disable (Cortex-M0: only PRIMASK available)
static inline void irq_enable()  { __asm volatile("cpsie i" ::: "memory"); }
static inline void irq_disable() { __asm volatile("cpsid i" ::: "memory"); }

// ---------------------------------------------------------------------------
// Cortex-M0 Vector Table (positions in SRAM at M0_CODE_BASE)
// ---------------------------------------------------------------------------
// The M0 boots from SRAM. M4 sets CREG_M0APPMEMMAP before releasing M0 reset.
// Vector table layout (must be at word 0 of M0 image):

struct M0VectorTable {
    uint32_t  initial_sp;           // [0]  Initial stack pointer
    uint32_t  reset_handler;        // [1]  Reset / entry point
    uint32_t  nmi_handler;          // [2]
    uint32_t  hardfault_handler;    // [3]
    uint32_t  _reserved[8];         // [4..11]
    uint32_t  svcall_handler;       // [11] — actually [11] in ARMv6-M
    uint32_t  _reserved2[2];        // [12..13]
    uint32_t  pendsv_handler;       // [14]
    uint32_t  systick_handler;      // [15]
    // External interrupts [16..47] — 32 IRQs
    uint32_t  irq_handlers[32];
};

// ---------------------------------------------------------------------------
// WFE / SEV — IPC synchronisation
// ---------------------------------------------------------------------------
// M0 waits for M4 to signal via CREG_M0TXEVENT (generates RXEV on M0 core).
// M0 signals M4 by executing SEV (RXEV on M4).
// After WFE returns, the caller must check flags in SharedMemory.

static inline void wfe() { __asm volatile("wfe" ::: "memory"); }
static inline void sev() { __asm volatile("sev" ::: "memory"); }

// Memory barrier — use DMB before/after writes to shared memory on M0
// (M0 does not implement DSB/ISB as separate instructions in all compilers,
// but the GCC intrinsic maps to dmb/dsb/isb via __asm)
static inline void dmb() { __asm volatile("dmb" ::: "memory"); }
static inline void dsb() { __asm volatile("dsb" ::: "memory"); }
static inline void isb() { __asm volatile("isb" ::: "memory"); }

// ---------------------------------------------------------------------------
// SysTick — 1 ms timebase (NO DWT on Cortex-M0)
// ---------------------------------------------------------------------------
// Call systick_init() once from m0_main().
// systick_get_ms() returns monotonic millisecond counter (wraps at ~49 days).
// systick_delay_ms(n) is a blocking spin delay.

extern volatile uint32_t g_systick_ms;  // incremented by SysTick_Handler

static inline void systick_init(uint32_t cpu_hz) {
    // Reload value = cpu_hz / 1000 - 1 for 1ms tick
    SysTick().RVR = (cpu_hz / 1000u) - 1u;
    SysTick().CVR = 0u;
    SysTick().CSR = SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE;
}

static inline uint32_t systick_get_ms() {
    return g_systick_ms;
}

static inline void systick_delay_ms(uint32_t ms) {
    uint32_t start = systick_get_ms();
    while ((systick_get_ms() - start) < ms) {
        wfe();
    }
}

// ---------------------------------------------------------------------------
// GPDMA — ping-pong DMA for SSP0 IQ sample capture
// ---------------------------------------------------------------------------
// Two DMA channels (0=ping, 1=pong) in linked-list mode.
// Each fills one half-buffer of 2048 int8_t pairs = 4096 bytes.
// When a channel fires its TC interrupt, the ISR flips g_dma_buf_ready.

static constexpr uint32_t DMA_BUF_SAMPLES = 2048u;   // IQ pairs per buffer
static constexpr uint32_t DMA_BUF_BYTES   = DMA_BUF_SAMPLES * 2u; // 4096 bytes

// DMA buffer pair — placed in SRAM (not shared memory) to avoid contention
alignas(4) extern int8_t g_dma_buf[2][DMA_BUF_BYTES];

// Which buffer is ready for DSP processing (0 or 1), or -1 if none
extern volatile int32_t  g_dma_buf_ready;

// GPDMA register layout (LPC4320 UM10503 §4.7)
struct GpdmaChannelRegs {
    volatile uint32_t SRCADDR;    // Source address
    volatile uint32_t DESTADDR;   // Destination address
    volatile uint32_t LLI;        // Linked List Item pointer
    volatile uint32_t CONTROL;    // Control register
    volatile uint32_t CONFIG;     // Configuration register
};

struct GpdmaRegs {
    volatile uint32_t    INTSTAT;        // +0x000 Interrupt status
    volatile uint32_t    INTTCSTAT;      // +0x004 TC interrupt status
    volatile uint32_t    INTTCCLEAR;     // +0x008 TC interrupt clear
    volatile uint32_t    INTERRSTAT;     // +0x00C Error interrupt status
    volatile uint32_t    INTERRCLR;      // +0x010 Error interrupt clear
    volatile uint32_t    RAWINTTCSTAT;   // +0x014 Raw TC interrupt
    volatile uint32_t    RAWINTERRSTAT;  // +0x018 Raw error interrupt
    volatile uint32_t    ENBLDCHNS;      // +0x01C Enabled channels
    volatile uint32_t    SOFTBREQ;       // +0x020
    volatile uint32_t    SOFTSREQ;       // +0x024
    volatile uint32_t    SOFTLBREQ;      // +0x028
    volatile uint32_t    SOFTLSREQ;      // +0x02C
    volatile uint32_t    CONFIG;         // +0x030 DMA config (enable)
    volatile uint32_t    SYNC;           // +0x034
    uint32_t             _reserved[50];
    GpdmaChannelRegs     CH[8];          // +0x100 Channels 0-7
};

static inline GpdmaRegs& GPDMA() {
    return *reinterpret_cast<GpdmaRegs*>(GPDMA_BASE);
}

// DMA CONTROL register fields (relevant bits)
static constexpr uint32_t DMA_CTRL_TRANSFERSIZE_SHIFT = 0u;
static constexpr uint32_t DMA_CTRL_SWIDTH_BYTE        = (0u << 18);  // 8-bit src
static constexpr uint32_t DMA_CTRL_DWIDTH_BYTE        = (0u << 21);  // 8-bit dst
static constexpr uint32_t DMA_CTRL_DINC               = (1u << 27);  // dest increment
static constexpr uint32_t DMA_CTRL_I                  = (1u << 31);  // TC IRQ enable

// DMA CONFIG register fields
static constexpr uint32_t DMA_CFG_E                   = (1u << 0);   // channel enable
static constexpr uint32_t DMA_CFG_SRCPERIPHERAL_SHIFT = 1u;
static constexpr uint32_t DMA_CFG_FLOWCTRL_P2M_DMA    = (2u << 11); // periph→mem, DMA ctrl
static constexpr uint32_t DMA_CFG_IE                  = (1u << 14); // IRQ error enable
static constexpr uint32_t DMA_CFG_ITC                 = (1u << 15); // IRQ TC enable

// LPC4320 GPDMA peripheral request source for SSP0 RX = 2
static constexpr uint32_t DMA_PERIPH_SSP0_RX          = 2u;

// ---------------------------------------------------------------------------
// SSP0 (synchronous serial) registers — used for IQ capture from MAX5864 ADC
// ---------------------------------------------------------------------------

struct SspRegs {
    volatile uint32_t CR0;    // Control Register 0
    volatile uint32_t CR1;    // Control Register 1
    volatile uint32_t DR;     // Data Register
    volatile uint32_t SR;     // Status Register
    volatile uint32_t CPSR;   // Clock Prescale Register
    volatile uint32_t IMSC;   // Interrupt Mask Set/Clear
    volatile uint32_t RIS;    // Raw Interrupt Status
    volatile uint32_t MIS;    // Masked Interrupt Status
    volatile uint32_t ICR;    // Interrupt Clear Register
    volatile uint32_t DMACR;  // DMA Control Register
};

static inline SspRegs& SSP0() {
    return *reinterpret_cast<SspRegs*>(SSP0_BASE);
}

// SSP CR1 bits
static constexpr uint32_t SSP_CR1_SSE = (1u << 1);  // SSP enable
static constexpr uint32_t SSP_CR1_MS  = (1u << 2);  // 0=master, 1=slave
// SSP DMACR bits
static constexpr uint32_t SSP_DMACR_RXDMAE = (1u << 0);  // Enable Rx DMA

// ---------------------------------------------------------------------------
// CPU frequency constant
// ---------------------------------------------------------------------------

static constexpr uint32_t CPU_HZ = 204'000'000u;  // 204 MHz

// ---------------------------------------------------------------------------
// Utility: count leading zeros (CLZ) — available on Cortex-M0 via GCC
// ---------------------------------------------------------------------------

static inline uint32_t clz(uint32_t x) {
    return static_cast<uint32_t>(__builtin_clz(x));
}

// ---------------------------------------------------------------------------
// Utility: absolute value for int16_t / int32_t (avoids libm)
// ---------------------------------------------------------------------------

static inline int16_t abs16(int16_t x) { return (x < 0) ? -x : x; }
static inline int32_t abs32(int32_t x) { return (x < 0) ? -x : x; }
