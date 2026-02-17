// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// main_m0.cpp — M0 entry point, SGPIO capture, and main dispatch loop
//
// Target: LPC4320 Cortex-M0 sub-core at 204 MHz (no FPU, no DWT)
// Entry point: m0_main() — called from reset vector after minimal CRT init.
//
// Data path: SGPIO parallel bus (8-bit from CPLD) → exchange interrupt →
//            M0 ISR reads shadow registers → ping-pong buffers → DSP
//
// NOTE: GPDMA does NOT support peripheral-to-memory for SGPIO on LPC43xx.
// Data movement is handled by M0 via SGPIO exchange interrupts.

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

// Ping-pong buffers: two × 4096 bytes of raw int8_t IQ data.
// Placed in ETB_SRAM (0x20000000) via the .dma_buffers linker section so
// they don't eat into the limited 8 KB M0_SRAM.
alignas(4) __attribute__((section(".dma_buffers")))
int8_t g_dma_buf[2][DMA_BUF_BYTES];

// ---------------------------------------------------------------------------
// SGPIO capture state
// ---------------------------------------------------------------------------

// Current write position within the active buffer
static volatile uint32_t s_buf_write_pos = 0;
// Which buffer is currently being filled (0 or 1)
static volatile uint32_t s_buf_active = 0;

// Pointers to SGPIO registers (cached for ISR speed)
static volatile uint32_t* const s_sgpio_reg_ss =
    reinterpret_cast<volatile uint32_t*>(SGPIO_BASE + SGPIO_REG_SS_OFFSET);
static volatile uint32_t* const s_sgpio_clr_status0 =
    reinterpret_cast<volatile uint32_t*>(SGPIO_BASE + SGPIO_CLR_STATUS0_OFFSET);
static volatile uint32_t* const s_sgpio_status0 =
    reinterpret_cast<volatile uint32_t*>(SGPIO_BASE + SGPIO_STATUS0_OFFSET);

// ---------------------------------------------------------------------------
// SysTick ISR — 1 ms timebase
// ---------------------------------------------------------------------------

extern "C" void SysTick_Handler(void) {
    ++g_systick_ms;
}

// ---------------------------------------------------------------------------
// SGPIO exchange interrupt handler
// ---------------------------------------------------------------------------
// Fires every time the SGPIO shadow registers swap (every 32 bits = 4 bytes
// per slice). With 8 slices in parallel mode, each exchange gives us one
// 32-bit word from each slice. In 8-bit parallel mode, each word contains
// 4 samples (bytes). We read slice A's shadow register which contains the
// interleaved 8-bit data packed by the SGPIO concatenation logic.
//
// Each exchange provides 4 bytes (one 32-bit word from the concatenated
// 8-bit parallel input). We accumulate these into the ping-pong buffers.

extern "C" void SGPIO_IRQHandler(void) {
    // Clear exchange interrupt for slice A
    *s_sgpio_clr_status0 = (1u << SGPIO_SLICE_A);

    // Read the 32-bit shadow register from slice A.
    // In 8-bit parallel + concatenated mode, slice A's REG_SS contains
    // the packed result of all 8 input bits shifted together.
    const uint32_t data = s_sgpio_reg_ss[SGPIO_SLICE_A];

    // Write 4 bytes into the active buffer
    const uint32_t pos = s_buf_write_pos;
    int8_t* dst = &g_dma_buf[s_buf_active][pos];

    // Store as 32-bit word (4 IQ bytes)
    *reinterpret_cast<uint32_t*>(dst) = data;

    const uint32_t new_pos = pos + 4u;

    if (new_pos >= DMA_BUF_BYTES) {
        // Buffer is full — signal it to main loop and swap
        g_dma_buf_ready = static_cast<int32_t>(s_buf_active);
        s_buf_active ^= 1u;
        s_buf_write_pos = 0;
        sev();  // Wake main loop
    } else {
        s_buf_write_pos = new_pos;
    }
}

// ---------------------------------------------------------------------------
// SGPIO capture initialisation
// ---------------------------------------------------------------------------
// M4 has already configured SGPIO slices and pins. M0 just enables the
// SGPIO exchange interrupt so it can read data as it arrives.

static void sgpio_capture_init(void) {
    // Reset buffer state
    s_buf_write_pos = 0;
    s_buf_active = 0;
    g_dma_buf_ready = -1;

    // Clear any pending SGPIO exchange interrupt status
    *s_sgpio_clr_status0 = 0xFFFFu;

    // Enable SGPIO interrupt in M0 NVIC (priority 0 = highest)
    nvic_clear_pending(IRQ_SGPIO);
    nvic_set_priority(IRQ_SGPIO, 0u);
    nvic_enable_irq(IRQ_SGPIO);
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
            shared().spectrum_center_hz = shared().cmd_freq_hz;
            break;

        case Command::SET_GAIN:
            break;

        case Command::SHUTDOWN:
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
//   2. Configured SGPIO slices and pins
//   3. Started SGPIO capture (slices enabled)
//   4. Set CREG_M0APPMEMMAP = M0_SRAM_BASE
//   5. Released M0 from reset
// M0 must NOT touch CCU/CGU/RGU — those are M4's domain.

extern "C" void m0_main(void) {
    // --- 1. Initialise SysTick for 1ms timebase (no DWT on M0) ---------------
    systick_init(CPU_HZ);

    // --- 2. Initialise RSSI (always active) -----------------------------------
    rssi_init();

    // --- 3. Set up SGPIO exchange interrupt for IQ capture --------------------
    sgpio_capture_init();

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
        // Wait for any event: SGPIO exchange, SysTick, or M4 command
        wfe();

        // --- Check M4 command flags -------------------------------------------
        if (sentinel::ipc::shared().m4_flags & sentinel::ipc::M4_FLAG_CMD) {
            handle_command();
        }

        // --- Consume ready buffer ---------------------------------------------
        int32_t buf_idx;
        {
            irq_disable();
            buf_idx = g_dma_buf_ready;
            g_dma_buf_ready = -1;
            irq_enable();
        }

        if (buf_idx < 0) {
            // No new data — check RSSI tick and loop
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
                break;
        }
    }
}
