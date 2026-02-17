#pragma once
// SPDX-License-Identifier: MIT
// Project Sentinel - M4 ↔ M0 Shared-Memory IPC Protocol
//
// Both cores access the same AHB SRAM bank.  The M4 signals the M0 by
// writing to the CREG M0TXEVENT register (generates RXEV on M0).
// The M0 signals the M4 by executing a SEV instruction (RXEV on M4) after
// setting a flag in m0_flags.
//
// All fields written by M0 are read by M4 and vice-versa; use volatile
// for every cross-core field.

#include <cstdint>

namespace sentinel::ipc {

// ---------------------------------------------------------------------------
// Physical addresses
// ---------------------------------------------------------------------------

// Shared AHB SRAM window (accessible by both M4 and M0 via the AHB matrix).
// Located in LPC4320 SRAM1 bank starting at 0x10080000; we use the upper 8KB.
static constexpr uint32_t SHARED_RAM_BASE = 0x10088000u;
static constexpr uint32_t SHARED_RAM_SIZE = 8192u;        // 8 KB

// CREG register: M4 writes 1 to assert RXEV (event) on M0 core.
// LPC4320 UM10503, Section 4.5 (CREG_M0TXEVENT).
static constexpr uint32_t CREG_M0TXEVENT = 0x40043000u + 0x01C4u;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

// Baseband operating mode — controls which DSP pipeline M0 runs.
enum class BasebandMode : uint8_t {
    IDLE     = 0,
    FM       = 1,   // Wideband or narrowband FM audio demodulation
    SPECTRUM = 2,   // FFT spectrum analysis (256 bins)
    ADSB     = 3,   // 1090ES ADS-B receive and decode
    SCANNER  = 4,   // Swept energy-detection scanner
};

// Commands written by M4 into cmd field; M0 reads and clears to NONE.
enum class Command : uint32_t {
    NONE       = 0,
    SET_MODE   = 1,  // Change mode + frequency; uses cmd_mode/cmd_freq_hz/cmd_bw_hz
    TUNE       = 2,  // Change frequency only (same mode); uses cmd_freq_hz
    SET_GAIN   = 3,  // Set RF gain; uses cmd_lna_db / cmd_vga_db
    SHUTDOWN   = 0xFF,
};

// ---------------------------------------------------------------------------
// Payload structures
// ---------------------------------------------------------------------------

// ADS-B frame: 112-bit ES message + reception metadata. 20 bytes.
struct __attribute__((packed)) AdsbFrame {
    uint8_t  data[14];        // Raw 112-bit frame, MSB first
    uint32_t timestamp_ms;    // xTaskGetTickCount() ms at reception
    int16_t  rssi_dbm_x10;   // Signal strength (dBm × 10)
};
static_assert(sizeof(AdsbFrame) == 20);

// Signal detection event from the SCANNER mode. 8 bytes.
struct __attribute__((packed)) SignalEvent {
    uint32_t freq_hz;         // Centre frequency of detected signal
    int16_t  rssi_dbm_x10;   // Peak RSSI (dBm × 10)
    uint16_t duration_ms;     // Dwell time the signal was above threshold
};
static_assert(sizeof(SignalEvent) == 8);

// ---------------------------------------------------------------------------
// Ring buffer depths and buffer sizes
// ---------------------------------------------------------------------------

static constexpr uint32_t SPECTRUM_BINS      = 256;
static constexpr uint32_t ADSB_RING_DEPTH    = 16;
static constexpr uint32_t SIGNAL_RING_DEPTH  = 16;
static constexpr uint32_t AUDIO_BUF_SAMPLES  = 2048;  // int16 mono @ 32 kHz = 64 ms

// ---------------------------------------------------------------------------
// Shared memory layout
// ---------------------------------------------------------------------------

struct __attribute__((packed, aligned(4))) SharedMemory {

    // ---- M4 → M0 command block (64 bytes) ---------------------------------
    volatile Command      cmd;            // M4 writes, M0 reads and clears to NONE
    volatile BasebandMode cmd_mode;       // Target mode for SET_MODE
    volatile uint8_t      cmd_fm_narrow;  // 0 = WBFM (200 kHz), 1 = NBFM (12.5 kHz)
    uint8_t               _p0;
    volatile uint32_t     cmd_freq_hz;    // Centre frequency (Hz)
    volatile uint32_t     cmd_bw_hz;      // Baseband bandwidth (Hz)
    volatile int8_t       cmd_lna_db;     // LNA gain: 0,8,16,24,32,40 dB
    volatile int8_t       cmd_vga_db;     // VGA gain: 0–62 dB (2 dB steps)
    uint8_t               _p1[42];
    // 4+1+1+1+1+4+4+1+1+42 = 60... let me pad to 64
    uint8_t               _p1b[4];
    // Total cmd block: 64 bytes

    // ---- M0 → M4 RSSI (8 bytes) -------------------------------------------
    volatile int16_t      rssi_dbm_x10;  // Current RSSI
    volatile uint16_t     rssi_seq;       // Incremented by M0 on each update
    uint8_t               _p2[4];

    // ---- M0 → M4 Spectrum FFT (272 bytes) ---------------------------------
    volatile uint16_t     spectrum_seq;         // Incremented by M0 per frame
    volatile uint16_t     spectrum_bins;        // Actual bin count filled
    volatile uint32_t     spectrum_center_hz;
    volatile uint32_t     spectrum_bw_hz;
    uint8_t               _p3[4];
    volatile int8_t       spectrum_db[SPECTRUM_BINS];  // dBFS per bin (signed)
    // 2+2+4+4+4+256 = 272 bytes

    // ---- M0 → M4 ADS-B ring buffer (324 bytes) ----------------------------
    volatile uint8_t      adsb_wr;        // Write index (M0 advances)
    volatile uint8_t      adsb_rd;        // Read index (M4 advances)
    uint8_t               _p4[2];
    AdsbFrame             adsb_frames[ADSB_RING_DEPTH];  // 16 × 20 = 320 bytes
    // 4+320 = 324 bytes

    // ---- M0 → M4 Signal detection ring buffer (132 bytes) -----------------
    volatile uint8_t      signal_wr;
    volatile uint8_t      signal_rd;
    uint8_t               _p5[2];
    SignalEvent           signals[SIGNAL_RING_DEPTH];    // 16 × 8 = 128 bytes
    // 4+128 = 132 bytes

    // ---- M0 → M4 FM audio PCM ring buffer (4104 bytes) --------------------
    volatile uint32_t     audio_wr_pos;   // Sample count written by M0
    volatile uint32_t     audio_rd_pos;   // Sample count read by M4
    int16_t               audio_buf[AUDIO_BUF_SAMPLES];  // 2048 × 2 = 4096 bytes
    // 4+4+4096 = 4104 bytes

    // ---- Cross-core event flags (8 bytes) ---------------------------------
    volatile uint32_t     m4_flags;  // M4 sets bits, M0 clears
    volatile uint32_t     m0_flags;  // M0 sets bits, M4 clears

    // Total: 64+8+272+324+132+4104+8 = 4912 bytes — fits in 8 KB
};

// Flag bit definitions
static constexpr uint32_t M4_FLAG_CMD        = (1u << 0);  // New command pending
static constexpr uint32_t M0_FLAG_SPECTRUM   = (1u << 0);  // New spectrum frame
static constexpr uint32_t M0_FLAG_ADSB       = (1u << 1);  // New ADS-B frame(s)
static constexpr uint32_t M0_FLAG_SIGNAL     = (1u << 2);  // Signal detection event(s)
static constexpr uint32_t M0_FLAG_AUDIO      = (1u << 3);  // Audio samples available
static constexpr uint32_t M0_FLAG_RSSI       = (1u << 4);  // RSSI updated

// ---------------------------------------------------------------------------
// Accessor and notification helpers
// ---------------------------------------------------------------------------

inline SharedMemory& shared() {
    return *reinterpret_cast<SharedMemory*>(SHARED_RAM_BASE);
}

// M4: assert RXEV on M0 (wakes M0 from WFE).
inline void notify_m0() {
    *reinterpret_cast<volatile uint32_t*>(CREG_M0TXEVENT) = 1u;
}

// M4: send a command to M0, then notify it.
inline void send_command(Command c) {
    shared().cmd      = c;
    shared().m4_flags |= M4_FLAG_CMD;
    notify_m0();
}

// M0: assert RXEV on M4 (SEV instruction wakes M4 from WFE).
// Call this after setting bits in m0_flags.
inline void notify_m4() {
    __asm volatile("sev" ::: "memory");
}

} // namespace sentinel::ipc
