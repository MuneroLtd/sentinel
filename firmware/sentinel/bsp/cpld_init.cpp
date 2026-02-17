// SPDX-License-Identifier: MIT
// Project Sentinel — PortaPack CPLD JTAG Programmer
//
// Bit-bang JTAG implementation for Altera MAX V (EPM240 / AG256SL100).
// Ported from portapack-mayhem cpld_max5.cpp / jtag.cpp / jtag_target_gpio.hpp.

#include "cpld_init.hpp"
#include "portapack_pins.hpp"
#include "../hal/gpio.hpp"
#include "../hal/lpc4320_regs.hpp"
#include "../hal/uart.hpp"

// ---------------------------------------------------------------------------
// JTAG GPIO helpers
// ---------------------------------------------------------------------------
static inline void jtag_tck(bool v) {
    if (v) LPC_GPIO->SET[PORT_CPLD_TCK] = (1u << PIN_CPLD_TCK);
    else   LPC_GPIO->CLR[PORT_CPLD_TCK] = (1u << PIN_CPLD_TCK);
}
static inline void jtag_tms(bool v) {
    if (v) LPC_GPIO->SET[PORT_CPLD_TMS] = (1u << PIN_CPLD_TMS);
    else   LPC_GPIO->CLR[PORT_CPLD_TMS] = (1u << PIN_CPLD_TMS);
}
static inline void jtag_tdi(bool v) {
    if (v) LPC_GPIO->SET[PORT_CPLD_TDI] = (1u << PIN_CPLD_TDI);
    else   LPC_GPIO->CLR[PORT_CPLD_TDI] = (1u << PIN_CPLD_TDI);
}
static inline bool jtag_tdo() {
    return (LPC_GPIO->PIN[PORT_CPLD_TDO] >> PIN_CPLD_TDO) & 1u;
}

// One JTAG clock cycle: set TMS + TDI, pulse TCK, return TDO (sampled before rising edge)
static inline bool jtag_clock(bool tms_val, bool tdi_val) {
    bool tdo = jtag_tdo();
    jtag_tms(tms_val);
    jtag_tdi(tdi_val);
    __asm volatile("nop"); __asm volatile("nop"); __asm volatile("nop");
    __asm volatile("nop"); __asm volatile("nop"); __asm volatile("nop");
    jtag_tck(true);
    __asm volatile("nop"); __asm volatile("nop"); __asm volatile("nop");
    __asm volatile("nop"); __asm volatile("nop"); __asm volatile("nop");
    jtag_tck(false);
    return tdo;
}

// ---------------------------------------------------------------------------
// JTAG state machine navigation
// ---------------------------------------------------------------------------
static void jtag_reset() {
    // 8 clocks with TMS=1 → Test-Logic-Reset
    for (int i = 0; i < 8; i++) jtag_clock(true, true);
}

static void jtag_run_test_idle() {
    jtag_clock(false, true);  // TMS=0 → Run-Test/Idle
}

static void jtag_runtest_tck(uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        jtag_clock(false, true);
    }
}

// Shift data through IR or DR (LSB-first). Returns shifted-out value.
// state_path: true=IR (Select-DR → Select-IR), false=DR (Select-DR)
static uint32_t jtag_shift(bool ir, uint32_t bits, uint32_t value) {
    // Navigate to Shift-IR or Shift-DR
    jtag_clock(true, true);     // → Select-DR-Scan
    if (ir) jtag_clock(true, true);  // → Select-IR-Scan
    jtag_clock(false, true);    // → Capture
    jtag_clock(false, true);    // → Shift

    uint32_t result = 0;
    for (uint32_t i = 0; i < bits; i++) {
        bool is_last = (i == bits - 1);
        bool tdo = jtag_clock(is_last, value & 1);  // Last bit: TMS=1 → Exit1
        value >>= 1;
        if (tdo) result |= (1u << i);
    }

    // Exit1 → Update → Run-Test/Idle
    jtag_clock(true, true);     // → Update
    jtag_clock(false, true);    // → Run-Test/Idle
    return result;
}

static uint32_t shift_ir(uint32_t bits, uint32_t value) {
    return jtag_shift(true, bits, value);
}

static uint32_t shift_dr(uint32_t bits, uint32_t value) {
    return jtag_shift(false, bits, value);
}

// ---------------------------------------------------------------------------
// MAX V CPLD instruction codes (10-bit IR)
// ---------------------------------------------------------------------------
static constexpr uint32_t IR_BITS         = 10;
static constexpr uint32_t IR_BYPASS       = 0x3FF;
static constexpr uint32_t IR_SAMPLE       = 0x005;
static constexpr uint32_t IR_IDCODE       = 0x006;
static constexpr uint32_t IR_ISC_ENABLE   = 0x2CC;
static constexpr uint32_t IR_ISC_DISABLE  = 0x201;
static constexpr uint32_t IR_ISC_PROGRAM  = 0x2F4;
static constexpr uint32_t IR_ISC_ERASE    = 0x2F2;
static constexpr uint32_t IR_ISC_ADDR     = 0x203;
static constexpr uint32_t IR_ISC_READ     = 0x205;
static constexpr uint32_t IR_CLAMP        = 0x00A;

// Expected IDCODE for AG256SL100 / MAX V
static constexpr uint32_t EXPECTED_IDCODE = 0x02814A3D;

// Expected silicon ID (5 × 16-bit words at sector 0x0089)
static constexpr uint16_t EXPECTED_SILICON_ID[5] = {
    0x8232, 0x2AA2, 0x4A82, 0x8C0C, 0x0000
};

// ---------------------------------------------------------------------------
// CPLD operations
// ---------------------------------------------------------------------------
static void sector_select(uint16_t id) {
    shift_ir(IR_BITS, IR_ISC_ADDR);
    jtag_runtest_tck(93);
    shift_dr(13, id);
}

static void cpld_enable() {
    shift_ir(IR_BITS, IR_ISC_ENABLE);
    jtag_runtest_tck(18003);
}

static void cpld_disable() {
    shift_ir(IR_BITS, IR_ISC_DISABLE);
    jtag_runtest_tck(18003);
}

static void cpld_bypass() {
    shift_ir(IR_BITS, IR_BYPASS);
    jtag_runtest_tck(18003);
}

static void cpld_sample() {
    shift_ir(IR_BITS, IR_SAMPLE);
    jtag_runtest_tck(93);
    for (int i = 0; i < 80; i++) shift_dr(3, 0x7);
}

static void cpld_clamp() {
    shift_ir(IR_BITS, IR_CLAMP);
    jtag_runtest_tck(93);
}

static bool cpld_idcode_ok() {
    shift_ir(IR_BITS, IR_IDCODE);
    uint32_t id = shift_dr(32, 0xFFFFFFFF);
    uart_printf("[CPLD] IDCODE: 0x%08X (expect 0x%08X)\r\n", id, EXPECTED_IDCODE);
    return id == EXPECTED_IDCODE;
}

static bool cpld_silicon_id_ok() {
    sector_select(0x0089);
    shift_ir(IR_BITS, IR_ISC_READ);
    jtag_runtest_tck(93);

    bool ok = true;
    for (int i = 0; i < 5; i++) {
        uint16_t val = static_cast<uint16_t>(shift_dr(16, 0xFFFF));
        if (val != EXPECTED_SILICON_ID[i]) ok = false;
    }
    uart_printf("[CPLD] Silicon ID: %s\r\n", ok ? "OK" : "MISMATCH");
    return ok;
}

static void erase_sector(uint16_t id) {
    sector_select(id);
    shift_ir(IR_BITS, IR_ISC_ERASE);
    jtag_runtest_tck(9000003);  // ~500ms erase pulse
}

static void bulk_erase() {
    uart_puts("[CPLD] Erasing...\r\n");
    erase_sector(0x0011);  // CFM config
    erase_sector(0x0001);  // UFM
    erase_sector(0x0000);  // CFM main
}

static void program_block(uint16_t sector, const uint16_t* data, size_t count) {
    sector_select(sector);
    shift_ir(IR_BITS, IR_ISC_PROGRAM);
    jtag_runtest_tck(93);

    for (size_t i = 0; i < count; i++) {
        shift_dr(16, data[i]);
        jtag_runtest_tck(1800);  // 100μs program pulse
    }
}

static bool verify_block(uint16_t sector, const uint16_t* data, size_t count) {
    sector_select(sector);
    shift_ir(IR_BITS, IR_ISC_READ);
    jtag_runtest_tck(93);

    bool ok = true;
    for (size_t i = 0; i < count; i++) {
        uint16_t val = static_cast<uint16_t>(shift_dr(16, 0xFFFF));
        uint16_t expect = data[i];

        // Block 0 word 0: mask bit 10 (device-controlled "ISP done" flag)
        if (sector == 0x0000 && i == 0) {
            if ((val & 0xFBFF) != (expect & 0xFBFF)) ok = false;
        } else {
            if (val != expect) ok = false;
        }
    }
    return ok;
}

static bool verify_all(const CpldConfig& cfg) {
    bool b0 = verify_block(0x0000, cfg.block_0, CPLD_BLOCK_0_SIZE);
    bool b1 = verify_block(0x0001, cfg.block_1, CPLD_BLOCK_1_SIZE);
    uart_printf("[CPLD] Verify: block0=%s block1=%s\r\n",
        b0 ? "OK" : "FAIL", b1 ? "OK" : "FAIL");
    return b0 && b1;
}

static bool program_all(const CpldConfig& cfg) {
    bulk_erase();

    uart_puts("[CPLD] Programming block 0 (3328 words)...\r\n");
    program_block(0x0000, cfg.block_0, CPLD_BLOCK_0_SIZE);

    uart_puts("[CPLD] Programming block 1 (512 words)...\r\n");
    program_block(0x0001, cfg.block_1, CPLD_BLOCK_1_SIZE);

    bool ok = verify_all(cfg);

    if (ok) {
        // Post-program notification: write block_0[0] with bit 10 cleared
        sector_select(0x0000);
        shift_ir(IR_BITS, IR_ISC_PROGRAM);
        jtag_runtest_tck(93);
        shift_dr(16, cfg.block_0[0] & 0xFBFF);
        jtag_runtest_tck(1800);
        shift_dr(16, cfg.block_0[1]);
        jtag_runtest_tck(1800);
        shift_dr(16, cfg.block_0[2]);
        jtag_runtest_tck(1800);
        shift_dr(16, cfg.block_0[3]);
        jtag_runtest_tck(1800);
    }

    return ok;
}

// ---------------------------------------------------------------------------
// JTAG GPIO setup
// ---------------------------------------------------------------------------
static void jtag_gpio_init() {
    // SCU pin mux for JTAG pins
    static constexpr uint32_t PIN_MODE = (1u << 4) | (1u << 6);  // EPUN + EZI

    // TMS: P1_8 → GPIO1[1], func 0
    scu_set_pinmode(CPLD_TMS_SCU_GRP, CPLD_TMS_SCU_PIN, 0, PIN_MODE);
    // TCK: P6_1 → GPIO3[0], func 0
    scu_set_pinmode(CPLD_TCK_SCU_GRP, CPLD_TCK_SCU_PIN, 0, PIN_MODE);
    // TDI: P6_2 → GPIO3[1], func 0
    scu_set_pinmode(CPLD_TDI_SCU_GRP, CPLD_TDI_SCU_PIN, 0, PIN_MODE);
    // TDO: P1_5 → GPIO1[8], func 0
    scu_set_pinmode(CPLD_TDO_SCU_GRP, CPLD_TDO_SCU_PIN, 0, PIN_MODE);

    // Preload output values
    jtag_tdi(true);
    jtag_tms(true);
    jtag_tck(false);

    // Set directions: TMS, TCK, TDI = outputs; TDO = input
    gpio_set_dir(PORT_CPLD_TMS, PIN_CPLD_TMS, true);
    gpio_set_dir(PORT_CPLD_TCK, PIN_CPLD_TCK, true);
    gpio_set_dir(PORT_CPLD_TDI, PIN_CPLD_TDI, true);
    gpio_set_dir(PORT_CPLD_TDO, PIN_CPLD_TDO, false);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
CpldStatus portapack_cpld_init() {
    uart_puts("[CPLD] Initialising JTAG...\r\n");
    jtag_gpio_init();

    jtag_reset();
    jtag_run_test_idle();

    // Check IDCODE
    if (!cpld_idcode_ok()) {
        return CpldStatus::IdcodeFail;
    }

    // Enter ISC mode
    cpld_sample();
    cpld_bypass();
    cpld_enable();

    // Check silicon ID
    if (!cpld_silicon_id_ok()) {
        cpld_disable();
        return CpldStatus::SiliconIdFail;
    }

    // Verify existing bitstream
    const CpldConfig& cfg = cpld_config_h4m;
    bool ok = verify_all(cfg);

    if (!ok) {
        // Needs programming
        uart_puts("[CPLD] Bitstream mismatch — programming...\r\n");
        ok = program_all(cfg);
        if (!ok) {
            cpld_disable();
            return CpldStatus::ProgramFail;
        }
        uart_puts("[CPLD] Programming complete!\r\n");
    } else {
        uart_puts("[CPLD] Bitstream already correct.\r\n");
    }

    // Exit ISC mode and reload SRAM
    cpld_disable();
    cpld_bypass();
    cpld_sample();
    cpld_clamp();
    cpld_disable();

    uart_puts("[CPLD] Ready.\r\n");
    return CpldStatus::Success;
}
