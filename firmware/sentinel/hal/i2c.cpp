// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 I2C Master Driver Implementation
//
// Polling state machine driven by the I2C STAT register.
// Each state transition: poll SI bit, read STAT, take action.

#include "i2c.hpp"
#include "lpc4320_regs.hpp"
#include "clocks.hpp"

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static I2C_Type* i2c_regs(uint8_t bus) {
    return (bus == 0) ? LPC_I2C0 : LPC_I2C1;
}

// Wait for SI (interrupt flag) to be set, indicating state machine advanced.
// Returns false on timeout (bus hung / not initialised).
static bool i2c_wait_si(I2C_Type* i2c) {
    uint32_t timeout = 1000000u;
    while (!(i2c->CONSET & I2C_SI)) {
        if (--timeout == 0u) return false;
    }
    return true;
}

// Clear SI bit (write 1 to CONCLR bit 3).
static void i2c_clear_si(I2C_Type* i2c) {
    i2c->CONCLR = I2C_SI;
}

// Issue a STOP condition and clear all relevant bits.
static void i2c_stop(I2C_Type* i2c) {
    i2c->CONSET = I2C_STO;
    i2c->CONCLR = I2C_SI | I2C_STA | I2C_AA;
    // Wait for STOP to complete (STO bit self-clears)
    uint32_t timeout = 100000u;
    while ((i2c->CONSET & I2C_STO) && --timeout) { }
}

// ---------------------------------------------------------------------------
// i2c_init
// ---------------------------------------------------------------------------
bool i2c_init(uint8_t bus, uint32_t freq_hz) {
    I2C_Type* i2c = i2c_regs(bus);

    // Disable I2C
    i2c->CONCLR = I2C_I2EN | I2C_SI | I2C_STA | I2C_AA;

    // The I2C clock is derived from PCLK.
    // LPC43xx I2C operates from APB clock.
    // PCLK for I2C0 (APB1) and I2C1 (APB3) = CPU_FREQ = 204 MHz
    // SCL period = (SCLH + SCLL) / PCLK
    // For 400 kHz: SCLH + SCLL = 204MHz / 400kHz = 510 cycles
    //   Typical: SCLH = 255, SCLL = 255 (50% duty)
    // For 100 kHz: SCLH + SCLL = 204MHz / 100kHz = 2040 cycles
    //   Typical: SCLH = 1020, SCLL = 1020

    const uint32_t pclk = clocks_get_cpu_hz();
    uint32_t total = pclk / freq_hz;
    if (total < 4) total = 4;

    uint32_t sclh = total / 2;
    uint32_t scll = total - sclh;

    i2c->SCLH = sclh;
    i2c->SCLL = scll;

    // Enable I2C
    i2c->CONSET = I2C_I2EN;

    return true;
}

// ---------------------------------------------------------------------------
// i2c_write
// ---------------------------------------------------------------------------
bool i2c_write(uint8_t bus, uint8_t addr7, const uint8_t* data, size_t len) {
    I2C_Type* i2c = i2c_regs(bus);
    bool ok = true;

    // Issue START
    i2c->CONCLR = I2C_SI;
    i2c->CONSET = I2C_STA;

    if (!i2c_wait_si(i2c)) { i2c_stop(i2c); return false; }
    uint32_t stat = i2c->STAT;

    if (stat != I2C_STAT_START && stat != I2C_STAT_REP_START) {
        i2c_stop(i2c);
        return false;
    }

    // Send SLA+W (address with write bit)
    i2c->CONCLR = I2C_STA;
    i2c->DAT = (static_cast<uint32_t>(addr7) << 1) | 0u; // R/W = 0 (write)
    i2c_clear_si(i2c);

    if (!i2c_wait_si(i2c)) { i2c_stop(i2c); return false; }
    stat = i2c->STAT;
    if (stat != I2C_STAT_SLA_W_ACK) {
        ok = false;
        goto stop;
    }

    // Send data bytes
    for (size_t i = 0; i < len; i++) {
        i2c->DAT = data[i];
        i2c_clear_si(i2c);

        if (!i2c_wait_si(i2c)) { ok = false; goto stop; }
        stat = i2c->STAT;
        if (stat != I2C_STAT_DAT_W_ACK) {
            ok = false;
            goto stop;
        }
    }

stop:
    i2c_stop(i2c);
    return ok;
}

// ---------------------------------------------------------------------------
// i2c_read
// ---------------------------------------------------------------------------
bool i2c_read(uint8_t bus, uint8_t addr7, uint8_t* data, size_t len) {
    I2C_Type* i2c = i2c_regs(bus);
    bool ok = true;

    if (len == 0) return true;

    // Issue START
    i2c->CONCLR = I2C_SI;
    i2c->CONSET = I2C_STA;

    if (!i2c_wait_si(i2c)) { i2c_stop(i2c); return false; }
    uint32_t stat = i2c->STAT;
    if (stat != I2C_STAT_START && stat != I2C_STAT_REP_START) {
        i2c_stop(i2c);
        return false;
    }

    // Send SLA+R
    i2c->CONCLR = I2C_STA;
    i2c->DAT = (static_cast<uint32_t>(addr7) << 1) | 1u; // R/W = 1 (read)

    // For all but the last byte, assert ACK; for the last byte, send NACK.
    if (len > 1) {
        i2c->CONSET = I2C_AA;
    } else {
        i2c->CONCLR = I2C_AA;
    }
    i2c_clear_si(i2c);

    if (!i2c_wait_si(i2c)) { i2c_stop(i2c); return false; }
    stat = i2c->STAT;
    if (stat != I2C_STAT_SLA_R_ACK) {
        ok = false;
        goto stop;
    }

    // Receive data bytes
    for (size_t i = 0; i < len; i++) {
        // Prepare ACK/NACK for *next* byte
        if (i + 1 < len) {
            i2c->CONSET = I2C_AA;
        } else {
            i2c->CONCLR = I2C_AA;  // NACK the last byte
        }
        i2c_clear_si(i2c);

        if (!i2c_wait_si(i2c)) { ok = false; goto stop; }
        stat = i2c->STAT;

        if (stat != I2C_STAT_DAT_R_ACK && stat != I2C_STAT_DAT_R_NACK) {
            ok = false;
            goto stop;
        }
        data[i] = static_cast<uint8_t>(i2c->DAT);
    }

stop:
    i2c_stop(i2c);
    return ok;
}

// ---------------------------------------------------------------------------
// i2c_write_read — write then repeated-START then read
// ---------------------------------------------------------------------------
bool i2c_write_read(uint8_t bus, uint8_t addr7,
                    const uint8_t* wr, size_t wr_len,
                    uint8_t*       rd, size_t rd_len) {
    I2C_Type* i2c = i2c_regs(bus);
    bool ok = true;

    // --- Write phase ---
    i2c->CONCLR = I2C_SI;
    i2c->CONSET = I2C_STA;

    if (!i2c_wait_si(i2c)) { i2c_stop(i2c); return false; }
    uint32_t stat = i2c->STAT;
    if (stat != I2C_STAT_START && stat != I2C_STAT_REP_START) {
        i2c_stop(i2c);
        return false;
    }

    i2c->CONCLR = I2C_STA;
    i2c->DAT = (static_cast<uint32_t>(addr7) << 1) | 0u;
    i2c_clear_si(i2c);

    if (!i2c_wait_si(i2c)) { i2c_stop(i2c); return false; }
    stat = i2c->STAT;
    if (stat != I2C_STAT_SLA_W_ACK) { ok = false; goto stop; }

    for (size_t i = 0; i < wr_len; i++) {
        i2c->DAT = wr[i];
        i2c_clear_si(i2c);
        if (!i2c_wait_si(i2c)) { ok = false; goto stop; }
        if (i2c->STAT != I2C_STAT_DAT_W_ACK) { ok = false; goto stop; }
    }

    // --- Repeated START for read phase ---
    if (rd_len == 0) goto stop;

    i2c->CONSET = I2C_STA;
    i2c_clear_si(i2c);

    if (!i2c_wait_si(i2c)) { ok = false; goto stop; }
    stat = i2c->STAT;
    if (stat != I2C_STAT_REP_START && stat != I2C_STAT_START) { ok = false; goto stop; }

    i2c->CONCLR = I2C_STA;
    i2c->DAT = (static_cast<uint32_t>(addr7) << 1) | 1u;
    if (rd_len > 1) {
        i2c->CONSET = I2C_AA;
    } else {
        i2c->CONCLR = I2C_AA;
    }
    i2c_clear_si(i2c);

    if (!i2c_wait_si(i2c)) { ok = false; goto stop; }
    if (i2c->STAT != I2C_STAT_SLA_R_ACK) { ok = false; goto stop; }

    for (size_t i = 0; i < rd_len; i++) {
        if (i + 1 < rd_len) {
            i2c->CONSET = I2C_AA;
        } else {
            i2c->CONCLR = I2C_AA;
        }
        i2c_clear_si(i2c);
        if (!i2c_wait_si(i2c)) { ok = false; goto stop; }
        stat = i2c->STAT;
        if (stat != I2C_STAT_DAT_R_ACK && stat != I2C_STAT_DAT_R_NACK) { ok = false; goto stop; }
        rd[i] = static_cast<uint8_t>(i2c->DAT);
    }

stop:
    i2c_stop(i2c);
    return ok;
}
