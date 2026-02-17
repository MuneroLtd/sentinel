// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// main_m0.cpp — M0 entry point, DMA ping-pong setup, and main dispatch loop
//
// Target: LPC4320 Cortex-M0 sub-core at 204 MHz (no FPU, no DWT)
// Entry point: m0_main() — called from reset vector after minimal CRT init.

#include "m0_startup.hpp"
#include "ipc/ipc_protocol.hpp"
#include "dsp/fm_demod.hpp"
#include "dsp/spectrum.hpp"
#include "dsp/adsb.hpp"
#include "dsp/scanner.hpp"
#include "dsp/rssi.hpp"

// ---------------------------------------------------------------------------
// Global state definitions (declared extern in m0_startup.hpp)
// ---------------------------------------------------------------------------

volatile uint32_t g_systick_ms  = 0u;
volatile int32_t  g_dma_buf_ready = -1;   // -1 = none; 0 or 1 = buffer index

// DMA ping-pong buffers: two × 4096 bytes of raw int8_t IQ data.
// Placed in ETB_SRAM (0x20000000) via the .dma_buffers linker section so
// they don't eat into the limited 8 KB M0_SRAM.  GPDMA can access ETB SRAM.
alignas(4) __attribute__((section(".dma_buffers")))
int8_t g_dma_buf[2][DMA_BUF_BYTES];

// ---------------------------------------------------------------------------
// SysTick ISR — 1 ms timebase
// ---------------------------------------------------------------------------

extern "C" void SysTick_Handler(void) {
    ++g_systick_ms;
}

// ---------------------------------------------------------------------------
// GPDMA ISR — fires on terminal count (buffer full) for channel 0 or 1
// ---------------------------------------------------------------------------
// The two DMA channels run in a self-reloading linked-list ping-pong scheme:
// channel 0's LLI points to channel 1's descriptor, and vice-versa.
// When a channel fires TC, we record which buffer is ready and wake the main loop.

// LLI descriptor (must be word-aligned in SRAM; DMA engine reads it directly)
struct DmaLli {
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t lli;      // pointer to next descriptor, or 0 for end
    uint32_t control;
};

alignas(4) static DmaLli s_dma_lli[2];  // LLI for ping and pong channels

extern "C" void DMA_IRQHandler(void) {
    uint32_t tc = GPDMA().INTTCSTAT;
    GPDMA().INTTCCLEAR = tc;  // acknowledge TC interrupts

    if (tc & (1u << 0u)) {
        // Channel 0 (ping buffer) completed
        g_dma_buf_ready = 0;
    } else if (tc & (1u << 1u)) {
        // Channel 1 (pong buffer) completed
        g_dma_buf_ready = 1;
    }

    // Clear any error flags
    GPDMA().INTERRCLR = GPDMA().INTERRSTAT;

    // Wake main loop via SEV (M0 WFE will return)
    sev();
}

// ---------------------------------------------------------------------------
// DMA initialisation — SSP0 RX → ping-pong SRAM buffers
// ---------------------------------------------------------------------------
// SSP0 is already configured by M4 as SPI slave (IQ capture from MAX5864).
// M0 owns the DMA channels and services the TC interrupt.

static void dma_init(void) {
    // Enable GPDMA controller
    GPDMA().CONFIG = 1u;  // enable bit

    // Build LLI for ping channel (ch0) → when done, ch1 starts (pong)
    // and for pong channel (ch1) → when done, ch0 starts (ping)
    //
    // CONTROL word:
    //   TransferSize = DMA_BUF_BYTES (4096)
    //   SBSize = 4 (source burst = 4 bytes)
    //   DBSize = 4 (dest burst = 4 bytes)
    //   Swidth = byte (8-bit)
    //   Dwidth = byte (8-bit)
    //   DI = 1 (destination increment)
    //   I = 1 (TC interrupt enable)

    const uint32_t ctrl_word =
        (DMA_BUF_BYTES << DMA_CTRL_TRANSFERSIZE_SHIFT) |
        (1u << 12) |  // SBSize = 4 (001)
        (1u << 15) |  // DBSize = 4 (001)
        DMA_CTRL_SWIDTH_BYTE |
        DMA_CTRL_DWIDTH_BYTE |
        DMA_CTRL_DINC |
        DMA_CTRL_I;

    // SSP0 RX FIFO address
    const uint32_t ssp0_dr = SSP0_BASE + 0x008u;

    // Ping (buf 0): source = SSP0 DR, dest = g_dma_buf[0], next = pong LLI
    s_dma_lli[0].srcaddr = ssp0_dr;
    s_dma_lli[0].dstaddr = reinterpret_cast<uint32_t>(&g_dma_buf[0][0]);
    s_dma_lli[0].lli     = reinterpret_cast<uint32_t>(&s_dma_lli[1]);
    s_dma_lli[0].control = ctrl_word;

    // Pong (buf 1): source = SSP0 DR, dest = g_dma_buf[1], next = ping LLI
    s_dma_lli[1].srcaddr = ssp0_dr;
    s_dma_lli[1].dstaddr = reinterpret_cast<uint32_t>(&g_dma_buf[1][0]);
    s_dma_lli[1].lli     = reinterpret_cast<uint32_t>(&s_dma_lli[0]);
    s_dma_lli[1].control = ctrl_word;

    // Program channel 0 (ping) and start it
    GPDMA().CH[0].SRCADDR  = ssp0_dr;
    GPDMA().CH[0].DESTADDR = reinterpret_cast<uint32_t>(&g_dma_buf[0][0]);
    GPDMA().CH[0].LLI      = reinterpret_cast<uint32_t>(&s_dma_lli[1]);
    GPDMA().CH[0].CONTROL  = ctrl_word;
    GPDMA().CH[0].CONFIG   =
        DMA_CFG_E |
        (DMA_PERIPH_SSP0_RX << DMA_CFG_SRCPERIPHERAL_SHIFT) |
        DMA_CFG_FLOWCTRL_P2M_DMA |
        DMA_CFG_IE | DMA_CFG_ITC;

    // Channel 1 (pong) is chained from LLI — starts automatically when ch0 fires

    // Enable SSP0 RX DMA
    SSP0().DMACR |= SSP_DMACR_RXDMAE;

    // Enable GPDMA interrupt in M0 NVIC (priority 0 = highest)
    nvic_clear_pending(IRQ_GPDMA);
    nvic_set_priority(IRQ_GPDMA, 0u);
    nvic_enable_irq(IRQ_GPDMA);
}

// ---------------------------------------------------------------------------
// Command handler — called when M4 has written a new command
// ---------------------------------------------------------------------------

static void handle_command(void) {
    using namespace sentinel::ipc;

    // Read command atomically
    Command cmd = shared().cmd;
    shared().cmd = Command::NONE;
    shared().m4_flags &= ~M4_FLAG_CMD;
    dmb();

    switch (cmd) {
        case Command::SET_MODE: {
            // Update operating mode and reinitialise DSP pipeline.
            // BasebandMode is stored in cmd_mode (the IPC struct has no separate
            // `mode` field — M0 always reads cmd_mode for the current mode).
            BasebandMode new_mode = shared().cmd_mode;
            switch (new_mode) {
                case BasebandMode::FM:
                    fm_demod_init(shared().cmd_fm_narrow != 0u);
                    break;
                case BasebandMode::SPECTRUM:
                    spectrum_init();
                    break;
                case BasebandMode::ADSB:
                    adsb_init();
                    break;
                case BasebandMode::SCANNER:
                    scanner_init();
                    break;
                default:
                    break;
            }
            rssi_init();  // RSSI runs in all modes
            break;
        }

        case Command::TUNE:
            // Frequency change only — pipeline continues, DSP state is kept
            // Radio tuning is handled by M4's RF control; M0 just tracks the
            // centre frequency for spectrum metadata.
            shared().spectrum_center_hz = shared().cmd_freq_hz;
            break;

        case Command::SET_GAIN:
            // Gain changes are applied by M4 directly via RF registers.
            // M0 acknowledges but takes no DSP action.
            break;

        case Command::SHUTDOWN:
            // Halt all pipelines; M0 will WFE until next SET_MODE
            // (handled by BasebandMode::IDLE in the main loop default case)
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// M0 entry point
// ---------------------------------------------------------------------------
// Called from reset vector.  M4 has already:
//   1. Configured clocks (M0 inherits M4's 204 MHz PLL)
//   2. Set CREG_M0APPMEMMAP = M0_SRAM_BASE
//   3. Released M0 from reset
// M0 must NOT touch CCU/CGU/RGU — those are M4's domain.

extern "C" void m0_main(void) {
    // --- 1. Initialise SysTick for 1ms timebase (no DWT on M0) ---------------
    systick_init(CPU_HZ);

    // --- 2. Initialise RSSI (always active) -----------------------------------
    rssi_init();

    // --- 3. Set up DMA ping-pong for SSP0 IQ capture -------------------------
    dma_init();

    // --- 4. Global interrupt enable -------------------------------------------
    irq_enable();

    // --- 5. Signal M4 that M0 is alive and ready ------------------------------
    {
        using namespace sentinel::ipc;
        shared().m0_flags |= M0_FLAG_RSSI;  // dummy flag to wake M4 watchdog
        dmb();
        notify_m4();
    }

    // --- 6. Main dispatch loop ------------------------------------------------
    while (true) {
        // Wait for any event: DMA TC, SysTick, or M4 command via CREG_M0TXEVENT
        wfe();

        // --- Check M4 command flags -------------------------------------------
        if (sentinel::ipc::shared().m4_flags & sentinel::ipc::M4_FLAG_CMD) {
            handle_command();
        }

        // --- Consume ready DMA buffer -------------------------------------------
        int32_t buf_idx;
        {
            irq_disable();
            buf_idx = g_dma_buf_ready;
            g_dma_buf_ready = -1;
            irq_enable();
        }

        if (buf_idx < 0) {
            // No new DMA data — check RSSI tick and loop
            rssi_tick();
            continue;
        }

        const int8_t* iq = g_dma_buf[static_cast<uint32_t>(buf_idx)];
        const size_t  n  = DMA_BUF_SAMPLES;  // IQ pairs

        // --- RSSI measurement (every mode) ------------------------------------
        rssi_process(iq, n);

        // --- Active DSP pipeline ----------------------------------------------
        using sentinel::ipc::BasebandMode;
        BasebandMode mode = sentinel::ipc::shared().cmd_mode;

        switch (mode) {
            case BasebandMode::FM:
                fm_demod_process(iq, n);
                break;

            case BasebandMode::SPECTRUM:
                spectrum_process(iq, n);
                break;

            case BasebandMode::ADSB:
                adsb_process(iq, n);
                break;

            case BasebandMode::SCANNER:
                scanner_process(iq, n);
                break;

            case BasebandMode::IDLE:
            default:
                // Nothing to do; power is saved by WFE
                break;
        }
    }
}
