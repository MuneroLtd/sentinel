// SPDX-License-Identifier: MIT
// Project Sentinel — LPC4320 SSP (SPI) Master Driver Implementation

#include "ssp.hpp"
#include "lpc4320_regs.hpp"
#include "clocks.hpp"

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static SSP_Type* ssp_regs(uint8_t bus) {
    return (bus == 0) ? LPC_SSP0 : LPC_SSP1;
}

// Drain the RX FIFO (discard any stale data).
static void ssp_drain_rx(SSP_Type* ssp) {
    while (ssp->SR & SSP_SR_RNE) {
        (void)ssp->DR;
    }
}

// Wait until the SSP is not busy and TX FIFO is empty.
static void ssp_wait_done(SSP_Type* ssp) {
    // Wait for TX FIFO to empty first
    while (!(ssp->SR & SSP_SR_TFE)) { }
    // Then wait for BSY to clear (shift register draining)
    while (ssp->SR & SSP_SR_BSY)    { }
}

// ---------------------------------------------------------------------------
// ssp_init
// ---------------------------------------------------------------------------
// Compute CPSR (even prescaler 2..254) and SCR (0..255) such that:
//   SCK = PCLK / (CPSR * (SCR + 1))  as close to clock_hz as possible.
//
// Strategy: iterate CPSR from 2..254 and pick the (CPSR, SCR) pair
// that gives the highest frequency <= clock_hz.
void ssp_init(uint8_t bus, uint32_t clock_hz, uint8_t cpol, uint8_t cpha) {
    SSP_Type* ssp = ssp_regs(bus);

    // Disable SSP during setup
    ssp->CR1 = 0;

    const uint32_t pclk = clocks_get_cpu_hz(); // SSP clocked from BASE_SSP_CLK = PLL1

    // Find best prescaler
    uint32_t best_cpsr = 2;
    uint32_t best_scr  = 0;
    uint32_t best_freq = 0;

    for (uint32_t cpsr = 2; cpsr <= 254; cpsr += 2) {
        for (uint32_t scr = 0; scr <= 255; scr++) {
            uint32_t f = pclk / (cpsr * (scr + 1));
            if (f <= clock_hz && f > best_freq) {
                best_freq = f;
                best_cpsr = cpsr;
                best_scr  = scr;
            }
        }
    }

    // Clock prescaler register
    ssp->CPSR = best_cpsr;

    // CR0: 8-bit SPI frame, chosen CPOL/CPHA, SCR
    uint32_t cr0 = SSP_CR0_DSS(8) | SSP_CR0_FRF_SPI;
    if (cpol) cr0 |= SSP_CR0_CPOL;
    if (cpha) cr0 |= SSP_CR0_CPHA;
    cr0 |= (best_scr & 0xFFu) << 8;
    ssp->CR0 = cr0;

    // Drain any stale RX data
    ssp_drain_rx(ssp);

    // CR1: enable as master (MS=0), no loopback
    ssp->CR1 = SSP_CR1_SSE;
}

// ---------------------------------------------------------------------------
// ssp_transfer — single byte full-duplex
// ---------------------------------------------------------------------------
uint8_t ssp_transfer(uint8_t bus, uint8_t byte) {
    SSP_Type* ssp = ssp_regs(bus);

    // Wait for TX FIFO to have space
    while (!(ssp->SR & SSP_SR_TNF)) { }
    ssp->DR = byte;

    // Wait for RX FIFO to have data
    while (!(ssp->SR & SSP_SR_RNE)) { }
    return static_cast<uint8_t>(ssp->DR);
}

// ---------------------------------------------------------------------------
// ssp_write — transmit-only (discard RX)
// ---------------------------------------------------------------------------
void ssp_write(uint8_t bus, const uint8_t* data, size_t len) {
    SSP_Type* ssp = ssp_regs(bus);

    // We pump data into the TX FIFO (depth=8) and drain RX FIFO simultaneously
    // to prevent RX FIFO overflow.
    size_t tx_count = 0;
    size_t rx_count = 0;

    while (rx_count < len) {
        // Fill TX FIFO as much as possible
        while (tx_count < len && (ssp->SR & SSP_SR_TNF)) {
            ssp->DR = data[tx_count++];
        }
        // Drain RX FIFO
        while (rx_count < tx_count && (ssp->SR & SSP_SR_RNE)) {
            (void)ssp->DR;
            rx_count++;
        }
    }

    // Wait for shift register to complete
    while (ssp->SR & SSP_SR_BSY) { }
}

// ---------------------------------------------------------------------------
// ssp_read — receive only (transmit 0xFF)
// ---------------------------------------------------------------------------
void ssp_read(uint8_t bus, uint8_t* data, size_t len) {
    SSP_Type* ssp = ssp_regs(bus);

    size_t tx_count = 0;
    size_t rx_count = 0;

    while (rx_count < len) {
        while (tx_count < len && (ssp->SR & SSP_SR_TNF)) {
            ssp->DR = 0xFFu;
            tx_count++;
        }
        while (rx_count < tx_count && (ssp->SR & SSP_SR_RNE)) {
            data[rx_count++] = static_cast<uint8_t>(ssp->DR);
        }
    }

    while (ssp->SR & SSP_SR_BSY) { }
}

// ---------------------------------------------------------------------------
// ssp_write_read — full-duplex
// ---------------------------------------------------------------------------
void ssp_write_read(uint8_t bus, const uint8_t* tx, uint8_t* rx, size_t len) {
    SSP_Type* ssp = ssp_regs(bus);

    size_t tx_count = 0;
    size_t rx_count = 0;

    while (rx_count < len) {
        while (tx_count < len && (ssp->SR & SSP_SR_TNF)) {
            ssp->DR = tx[tx_count++];
        }
        while (rx_count < tx_count && (ssp->SR & SSP_SR_RNE)) {
            rx[rx_count++] = static_cast<uint8_t>(ssp->DR);
        }
    }

    while (ssp->SR & SSP_SR_BSY) { }
}

// ---------------------------------------------------------------------------
// ssp_write16 — send a 16-bit word as two bytes, MSB first
// ---------------------------------------------------------------------------
void ssp_write16(uint8_t bus, uint16_t word) {
    SSP_Type* ssp = ssp_regs(bus);

    // Wait for room in TX FIFO (need 2 slots)
    while (!(ssp->SR & SSP_SR_TNF)) { }
    ssp->DR = (word >> 8) & 0xFFu;

    while (!(ssp->SR & SSP_SR_TNF)) { }
    ssp->DR = word & 0xFFu;

    // Drain the two RX bytes so FIFO doesn't overflow
    while (!(ssp->SR & SSP_SR_RNE)) { }
    (void)ssp->DR;
    while (!(ssp->SR & SSP_SR_RNE)) { }
    (void)ssp->DR;
}
