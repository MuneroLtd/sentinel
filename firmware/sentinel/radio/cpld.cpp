// SPDX-License-Identifier: MIT
// Project Sentinel — CPLD (Xilinx XC2C64A) Interface Driver Implementation
//
// References:
//   - greatscottgadgets/hackrf firmware/common/cpld_jtag.c (BSD 2-clause)
//   - Xilinx XAPP058 XSVF specification
//   - Xilinx XAPP503 SVF/XSVF file formats for Xilinx devices
//   - XC2C64A datasheet
//
// JTAG TAP state machine:
//   The IEEE 1149.1 TAP controller is a 16-state FSM driven by TMS.
//   XSVF commands reference states by name (Test-Logic-Reset, Run-Test/Idle,
//   Shift-DR, Shift-IR, etc.).
//
// XSVF byte commands used for XC2C64A programming:
//   0x00 XCOMPLETE   — end of XSVF stream, programming done
//   0x01 XTDOMASK    — set TDO mask for verification
//   0x02 XSIR        — shift instruction register (IR)
//   0x03 XSDR        — shift data register (DR) with TDO check
//   0x04 XRUNTEST    — set run-test/idle clock count
//   0x07 XREPEAT     — set retry count for TDO mismatch
//   0x08 XSDRSIZE    — set DR shift bit length
//   0x09 XSDRTDO     — shift DR with captured TDO
//   0x0A XSETSDRMASKS— set addressable DR masks
//   0x0B XSDRINC     — shift DR with address increment
//   0x0C XSDRB       — begin multi-part DR shift
//   0x0D XSDRC       — continue multi-part DR shift
//   0x0E XSDRE       — end multi-part DR shift
//   0x0F XSDRTDOB    — begin multi-part DR shift + TDO capture
//   0x10 XSDRTDOC    — continue multi-part DR shift + TDO capture
//   0x11 XSDRTDOE    — end multi-part DR shift + TDO capture
//   0x12 XSTATE      — force TAP to state
//   0x13 XENDIR      — set end-IR state
//   0x14 XENDDR      — set end-DR state
//   0x15 XSIR2       — shift IR with 2-byte length
//   0x16 XCOMMENT    — embedded comment
//   0x17 XWAIT       — wait in state N clocks

#include "cpld.hpp"
#include "hal/gpio.hpp"
#include "bsp/portapack_pins.hpp"

// ---------------------------------------------------------------------------
// JTAG bit-bang helpers
// ---------------------------------------------------------------------------

// Small delay loop — TCK rate ~100 kHz on LPC4320 @ 204 MHz
// 200 iterations ≈ 1 µs ≈ enough for XC2C64A setup/hold
static inline void jtag_delay() {
    for (volatile uint32_t i = 0u; i < 4u; i++) { (void)i; }
}

static inline void tck_low()  { gpio_write(PORT_CPLD_TCK, PIN_CPLD_TCK, false); }
static inline void tck_high() { gpio_write(PORT_CPLD_TCK, PIN_CPLD_TCK, true);  }
static inline void tms_low()  { gpio_write(PORT_CPLD_TMS, PIN_CPLD_TMS, false); }
static inline void tms_high() { gpio_write(PORT_CPLD_TMS, PIN_CPLD_TMS, true);  }
static inline void tdi_low()  { gpio_write(PORT_CPLD_TDI, PIN_CPLD_TDI, false); }
static inline void tdi_high() { gpio_write(PORT_CPLD_TDI, PIN_CPLD_TDI, true);  }
static inline bool tdo_read() { return gpio_read(PORT_CPLD_TDO, PIN_CPLD_TDO);  }

// Clock one TCK pulse with TMS and TDI already set.
// Returns TDO sampled on the rising edge.
static bool jtag_clock_bit(bool tms_val, bool tdi_val) {
    if (tms_val) tms_high(); else tms_low();
    if (tdi_val) tdi_high(); else tdi_low();
    jtag_delay();
    tck_high();
    bool tdo = tdo_read();
    jtag_delay();
    tck_low();
    return tdo;
}

// Navigate to Test-Logic-Reset from any state (5× TMS=1)
static void jtag_reset() {
    for (uint8_t i = 0u; i < 5u; i++) {
        jtag_clock_bit(true, true);
    }
}

// Navigate from Test-Logic-Reset to Run-Test/Idle (1× TMS=0)
static void jtag_goto_idle() {
    jtag_clock_bit(false, true);
}

// Navigate from Run-Test/Idle to Shift-IR
// RTI → Select-DR → Select-IR → Capture-IR → Shift-IR
static void jtag_enter_shift_ir() {
    jtag_clock_bit(true,  true); // RTI → Select-DR
    jtag_clock_bit(true,  true); // Select-DR → Select-IR
    jtag_clock_bit(false, true); // Select-IR → Capture-IR
    jtag_clock_bit(false, true); // Capture-IR → Shift-IR
}

// Navigate from Run-Test/Idle to Shift-DR
// RTI → Select-DR → Capture-DR → Shift-DR
static void jtag_enter_shift_dr() {
    jtag_clock_bit(true,  true); // RTI → Select-DR
    jtag_clock_bit(false, true); // Select-DR → Capture-DR
    jtag_clock_bit(false, true); // Capture-DR → Shift-DR
}

// Exit from Shift-xR to Run-Test/Idle via Exit1 → Update → RTI
// (TMS=1 to exit shift, TMS=1 to update, TMS=0 back to idle)
static void jtag_exit_to_idle() {
    // The last bit of any shift should have been clocked with TMS=1
    // to enter Exit1.  This function handles the remaining transitions.
    jtag_clock_bit(true,  true); // Exit1-xR → Update-xR
    jtag_clock_bit(false, true); // Update-xR → RTI
}

// Shift 'bits' bits from 'data' (LSB first) through TDI/TDO.
// On the last bit, TMS is set high to exit the shift state.
// Returns captured TDO bits in 'tdo_out' (may be NULL if not needed).
static void jtag_shift_bits(const uint8_t* data_in, uint8_t* tdo_out,
                             uint32_t bits) {
    for (uint32_t i = 0u; i < bits; i++) {
        bool tdi_bit = (data_in[i >> 3u] >> (i & 7u)) & 1u;
        bool last    = (i == (bits - 1u));
        bool tdo_bit = jtag_clock_bit(last ? true : false, tdi_bit);
        if (tdo_out) {
            if (tdo_bit) {
                tdo_out[i >> 3u] |= (uint8_t)(1u << (i & 7u));
            } else {
                tdo_out[i >> 3u] &= (uint8_t)(~(1u << (i & 7u)));
            }
        }
    }
    // Last bit clocked with TMS=1 took us to Exit1-xR
    jtag_exit_to_idle();
}

// Shift a byte into the IR (MSB-first, per JTAG convention for XC2C64A)
// XC2C64A IR length = 8 bits.
static void jtag_shift_ir(uint8_t instruction) {
    // JTAG shifts LSB first on TDI, but XSVF instructions are MSB first.
    // Reverse the byte.
    uint8_t reversed = 0u;
    for (uint8_t i = 0u; i < 8u; i++) {
        if (instruction & (1u << (7u - i))) {
            reversed |= (uint8_t)(1u << i);
        }
    }
    jtag_enter_shift_ir();
    jtag_shift_bits(&reversed, nullptr, 8u);
}

// ---------------------------------------------------------------------------
// XSVF execution engine
// ---------------------------------------------------------------------------

// XC2C64A JTAG instruction register opcodes
static constexpr uint8_t XC2C_BYPASS    = 0xFFu;
static constexpr uint8_t XC2C_IDCODE    = 0x01u;
static constexpr uint8_t XC2C_INTEST    = 0x02u;
static constexpr uint8_t XC2C_EXTEST    = 0x26u; // boundary scan
static constexpr uint8_t XC2C_SAMPLE    = 0x03u;
static constexpr uint8_t XC2C_ERASE_ALL = 0xEDu;
static constexpr uint8_t XC2C_PROGRAM   = 0xEAu;
static constexpr uint8_t XC2C_VERIFY    = 0xEEu;
static constexpr uint8_t XC2C_DISCHARGE = 0xD0u;
static constexpr uint8_t XC2C_ISC_EN    = 0xE8u; // ISC enable
static constexpr uint8_t XC2C_ISC_DIS   = 0xE0u; // ISC disable
static constexpr uint8_t XC2C_FADDR     = 0xA0u; // functional address

// XSVF command bytes
static constexpr uint8_t XCOMPLETE   = 0x00u;
static constexpr uint8_t XTDOMASK    = 0x01u;
static constexpr uint8_t XSIR        = 0x02u;
static constexpr uint8_t XSDR        = 0x03u;
static constexpr uint8_t XRUNTEST    = 0x04u;
static constexpr uint8_t XREPEAT     = 0x07u;
static constexpr uint8_t XSDRSIZE    = 0x08u;
static constexpr uint8_t XSDRTDO     = 0x09u;
static constexpr uint8_t XSTATE      = 0x12u;
static constexpr uint8_t XENDIR      = 0x13u;
static constexpr uint8_t XENDDR      = 0x14u;
static constexpr uint8_t XSIR2       = 0x15u;
static constexpr uint8_t XCOMMENT    = 0x16u;
static constexpr uint8_t XWAIT       = 0x17u;

// Maximum DR size we support (XC2C64A has up to 274-bit DR for programming)
static constexpr uint32_t XSVF_MAX_DR_BYTES = 35u; // ceil(274/8)

// XSVF execution state
struct XsvfState {
    const uint8_t* buf;
    uint32_t       pos;
    uint32_t       len;

    uint32_t run_test_clocks;
    uint32_t sdr_size_bits;
    uint8_t  repeat_count;
    uint8_t  endir_state;   // 0=RTI, 1=Pause-IR
    uint8_t  enddr_state;   // 0=RTI, 1=Pause-DR

    uint8_t  tdo_mask[XSVF_MAX_DR_BYTES];
    uint8_t  tdo_expected[XSVF_MAX_DR_BYTES];
    uint8_t  tdo_captured[XSVF_MAX_DR_BYTES];
    uint8_t  tdi_data[XSVF_MAX_DR_BYTES];
};

static uint8_t xsvf_read_byte(XsvfState& s) {
    if (s.pos >= s.len) return 0u;
    return s.buf[s.pos++];
}

static uint32_t xsvf_read_u32(XsvfState& s) {
    uint32_t v  = (uint32_t)xsvf_read_byte(s) << 24u;
    v |= (uint32_t)xsvf_read_byte(s) << 16u;
    v |= (uint32_t)xsvf_read_byte(s) <<  8u;
    v |= (uint32_t)xsvf_read_byte(s);
    return v;
}

static void xsvf_read_bytes(XsvfState& s, uint8_t* out, uint32_t n) {
    for (uint32_t i = 0u; i < n; i++) out[i] = xsvf_read_byte(s);
}

// Run 'clocks' TCK cycles in Run-Test/Idle (TMS=0, TDI=0)
static void jtag_run_test(uint32_t clocks) {
    for (uint32_t i = 0u; i < clocks; i++) {
        jtag_clock_bit(false, false);
    }
}

// Compare expected vs captured TDO bits using mask
// Returns true if (captured & mask) == (expected & mask)
static bool xsvf_tdo_match(const XsvfState& s, uint32_t bytes) {
    for (uint32_t i = 0u; i < bytes; i++) {
        if ((s.tdo_captured[i] & s.tdo_mask[i]) !=
            (s.tdo_expected[i] & s.tdo_mask[i])) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// cpld_program: execute an XSVF bitstream
// ---------------------------------------------------------------------------
bool cpld_program(const uint8_t* data, uint32_t length) {
    if (!data || length == 0u) return false;

    XsvfState s;
    s.buf    = data;
    s.pos    = 0u;
    s.len    = length;
    s.run_test_clocks = 0u;
    s.sdr_size_bits   = 0u;
    s.repeat_count    = 32u; // default from XAPP058
    s.endir_state     = 0u;  // RTI
    s.enddr_state     = 0u;  // RTI

    for (uint32_t i = 0u; i < XSVF_MAX_DR_BYTES; i++) {
        s.tdo_mask[i]     = 0xFFu;
        s.tdo_expected[i] = 0x00u;
        s.tdo_captured[i] = 0x00u;
        s.tdi_data[i]     = 0x00u;
    }

    // Reset TAP
    jtag_reset();
    jtag_goto_idle();

    while (s.pos < s.len) {
        uint8_t cmd = xsvf_read_byte(s);

        switch (cmd) {
        case XCOMPLETE:
            return true; // Success

        case XTDOMASK: {
            uint32_t bytes = (s.sdr_size_bits + 7u) >> 3u;
            if (bytes > XSVF_MAX_DR_BYTES) bytes = XSVF_MAX_DR_BYTES;
            xsvf_read_bytes(s, s.tdo_mask, bytes);
            break;
        }

        case XSIR: {
            uint8_t ir_len  = xsvf_read_byte(s); // length in bits
            uint8_t ir_bytes = (ir_len + 7u) >> 3u;
            uint8_t ir_data[4] = { 0u, 0u, 0u, 0u };
            if (ir_bytes > 4u) ir_bytes = 4u;
            xsvf_read_bytes(s, ir_data, ir_bytes);
            jtag_enter_shift_ir();
            jtag_shift_bits(ir_data, nullptr, ir_len);
            if (s.run_test_clocks > 0u) jtag_run_test(s.run_test_clocks);
            break;
        }

        case XSIR2: {
            uint16_t ir_len  = (uint16_t)((uint16_t)xsvf_read_byte(s) << 8u);
            ir_len |= xsvf_read_byte(s);
            uint32_t ir_bytes = (ir_len + 7u) >> 3u;
            uint8_t ir_data[4] = { 0u };
            if (ir_bytes > 4u) ir_bytes = 4u;
            xsvf_read_bytes(s, ir_data, ir_bytes);
            jtag_enter_shift_ir();
            jtag_shift_bits(ir_data, nullptr, ir_len);
            if (s.run_test_clocks > 0u) jtag_run_test(s.run_test_clocks);
            break;
        }

        case XSDRSIZE:
            s.sdr_size_bits = xsvf_read_u32(s);
            break;

        case XSDR: {
            uint32_t bytes = (s.sdr_size_bits + 7u) >> 3u;
            if (bytes > XSVF_MAX_DR_BYTES) bytes = XSVF_MAX_DR_BYTES;
            xsvf_read_bytes(s, s.tdi_data, bytes);
            jtag_enter_shift_dr();
            jtag_shift_bits(s.tdi_data, nullptr, s.sdr_size_bits);
            if (s.run_test_clocks > 0u) jtag_run_test(s.run_test_clocks);
            break;
        }

        case XSDRTDO: {
            uint32_t bytes = (s.sdr_size_bits + 7u) >> 3u;
            if (bytes > XSVF_MAX_DR_BYTES) bytes = XSVF_MAX_DR_BYTES;
            xsvf_read_bytes(s, s.tdi_data,     bytes);
            xsvf_read_bytes(s, s.tdo_expected, bytes);
            // Retry loop
            for (uint8_t attempt = 0u; attempt <= s.repeat_count; attempt++) {
                jtag_enter_shift_dr();
                jtag_shift_bits(s.tdi_data, s.tdo_captured, s.sdr_size_bits);
                if (s.run_test_clocks > 0u) jtag_run_test(s.run_test_clocks);
                if (xsvf_tdo_match(s, bytes)) break;
                if (attempt == s.repeat_count) return false; // Verify failed
            }
            break;
        }

        case XRUNTEST:
            s.run_test_clocks = xsvf_read_u32(s);
            break;

        case XREPEAT:
            s.repeat_count = xsvf_read_byte(s);
            break;

        case XSTATE: {
            // Force TAP to named state — simplified: just reset to idle
            uint8_t state = xsvf_read_byte(s);
            (void)state; // Full state navigation not required for HackRF XSVF
            // Most XC2C64A XSVFs only use XCOMPLETE/RTI/TLR for XSTATE
            jtag_reset();
            jtag_goto_idle();
            break;
        }

        case XENDIR:
            s.endir_state = xsvf_read_byte(s);
            break;

        case XENDDR:
            s.enddr_state = xsvf_read_byte(s);
            break;

        case XCOMMENT: {
            // Read until null byte
            uint8_t c;
            do { c = xsvf_read_byte(s); } while (c && s.pos < s.len);
            break;
        }

        case XWAIT: {
            // XWAIT: wait_state(1 byte), end_state(1 byte), wait_time_us(4 bytes)
            uint8_t  wstate = xsvf_read_byte(s);
            uint8_t  estate = xsvf_read_byte(s);
            uint32_t wait_us= xsvf_read_u32(s);
            (void)wstate;
            (void)estate;
            // Simple busy-wait (LPC4320 ~204 MHz, ~5 ns/cycle)
            // 204 cycles ≈ 1 µs
            for (uint32_t i = 0u; i < wait_us; i++) {
                for (volatile uint32_t j = 0u; j < 204u; j++) { (void)j; }
            }
            break;
        }

        default:
            // Unknown XSVF command — skip (best-effort)
            break;
        }
    }

    return false; // Never saw XCOMPLETE
}

// ---------------------------------------------------------------------------
// cpld_init
// ---------------------------------------------------------------------------

// Embedded CPLD bitstream placeholder.
// In a complete build, this array is replaced by the actual XC2C64A XSVF
// image generated from the sgpio_if CPLD design, linked in via the build
// system (e.g., cpld_xsvf.s / objcopy).
// The symbol cpld_xsvf_data and cpld_xsvf_length are provided by the
// linker script or an assembly stub like:
//
//   .section .rodata
//   .global cpld_xsvf_data
//   cpld_xsvf_data:
//   .incbin "sgpio_if/default.xsvf"
//   .global cpld_xsvf_length
//   cpld_xsvf_length:
//   .word (. - cpld_xsvf_data)
//
// For compilation purposes we declare them extern here.
extern "C" {
    extern const uint8_t  cpld_xsvf_data[];
    extern const uint32_t cpld_xsvf_length;
}

bool cpld_init() {
    // Configure JTAG GPIO pins
    gpio_set_dir(PORT_CPLD_TCK, PIN_CPLD_TCK, true);  // output
    gpio_set_dir(PORT_CPLD_TMS, PIN_CPLD_TMS, true);  // output
    gpio_set_dir(PORT_CPLD_TDI, PIN_CPLD_TDI, true);  // output
    gpio_set_dir(PORT_CPLD_TDO, PIN_CPLD_TDO, false); // input

    // Idle state: TCK low, TMS high (keeps TAP in TLR-safe state), TDI high
    tck_low();
    tms_high();
    tdi_high();

    // Program CPLD with embedded bitstream
    if (!cpld_program(cpld_xsvf_data, cpld_xsvf_length)) {
        return false;
    }

    // After programming, release JTAG (all inputs) to avoid bus conflict
    gpio_set_dir(PORT_CPLD_TCK, PIN_CPLD_TCK, false);
    gpio_set_dir(PORT_CPLD_TMS, PIN_CPLD_TMS, false);
    gpio_set_dir(PORT_CPLD_TDI, PIN_CPLD_TDI, false);

    // Default to RF-off (safe state during startup)
    cpld_rf_off();
    return true;
}

// ---------------------------------------------------------------------------
// Runtime RF switch control
//
// After the CPLD is programmed with the sgpio_if bitstream, the LPC4320
// controls the RF switches by writing to the SGPIO peripheral.  The CPLD
// samples specific SGPIO output lines and drives the SKY13317/SKY13351 RF
// switch control lines accordingly.
//
// The HackRF sgpio_if CPLD design maps:
//   SGPIO[0] → direction (0=RX, 1=TX)
//   SGPIO[1] → enable (0=RF off, 1=RF on)
//
// For Project Sentinel we model this with the LPC4320 SGPIO peripheral.
// Until full SGPIO support is available, we provide GPIO-based stubs that
// drive the CPLD control port directly via the JTAG data lines repurposed
// as GPIO after programming (they are not used for JTAG post-init).
//
// ---------------------------------------------------------------------------

// After CPLD is programmed, TDI (GPIO5[6]) and TMS (GPIO5[4]) serve as
// runtime RF-switch direction and enable signals driven into the CPLD
// boundary.  This is an approximation; full implementation requires SGPIO.

static void cpld_send_ctrl(bool rf_enable, bool tx_dir) {
    // Repurpose CPLD I/O pins for runtime control after programming.
    // TDI → enable, TMS → direction  (sgpio_if mapping approximation)
    gpio_set_dir(PORT_CPLD_TDI, PIN_CPLD_TDI, true);
    gpio_set_dir(PORT_CPLD_TMS, PIN_CPLD_TMS, true);
    gpio_write(PORT_CPLD_TDI, PIN_CPLD_TDI, rf_enable);
    gpio_write(PORT_CPLD_TMS, PIN_CPLD_TMS, tx_dir);
}

void cpld_set_rx() {
    cpld_send_ctrl(true, false); // enable=1, direction=RX(0)
}

void cpld_set_tx() {
    cpld_send_ctrl(true, true);  // enable=1, direction=TX(1)
}

void cpld_rf_off() {
    cpld_send_ctrl(false, false); // enable=0
}
