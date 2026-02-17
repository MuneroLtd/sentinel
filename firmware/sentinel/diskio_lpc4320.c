// SPDX-License-Identifier: MIT
// Project Sentinel — FatFS Disk I/O Glue for LPC4320 SD Card (SPI/SSP)
//
// Implements the standard FatFS diskio interface for an SD/SDHC card
// connected to the LPC4320 SSP0 peripheral in SPI mode.
//
// Pin usage (from bsp/portapack_pins.hpp):
//   SSP0 SCK/MOSI/MISO — configured in hal/ssp.c
//   SD_CS_GPIO_PORT / SD_CS_GPIO_PIN — chip select (active-low)
//
// SD card SPI command protocol:
//   CMD0  (0x40) — GO_IDLE_STATE          → R1
//   CMD8  (0x48) — SEND_IF_COND           → R7 (voltage check)
//   CMD16 (0x50) — SET_BLOCKLEN           → R1
//   CMD17 (0x51) — READ_SINGLE_BLOCK      → R1 + data block
//   CMD24 (0x58) — WRITE_BLOCK            → R1 + data block
//   CMD58 (0x7A) — READ_OCR               → R3
//   ACMD41 (CMD55 + CMD41) — SD_SEND_OP_COND → R1
//   ACMD41 with HCS bit set → enables SDHC
//
// Data token: 0xFE for single-block read/write.
// Data response token: lower 4 bits — 0x05 = data accepted.
//
// Block size is always 512 bytes (SET_BLOCKLEN 512 at init).

#include "ff.h"       // FATFS type definitions (DSTATUS, DRESULT, etc.)
#include "diskio.h"   // FatFS diskio interface
#include "ffconf.h"   // FF_MIN_SS, FF_MAX_SS

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// HAL and BSP includes (C-compatible)
// The SSP and GPIO HAL headers are C++; we call them via thin C wrappers
// declared in hal/ssp.h and hal/gpio.h. For simplicity we duplicate the
// relevant inline calls using direct register access, which avoids needing
// C++ wrapper headers in this .c file.
//
// LPC4320 SSP0 base address: 0x40083000
// LPC4320 GPIO base:         0x400F4000 (GPIO word-bit interface)
// ---------------------------------------------------------------------------

// GPIO byte-access interface for CS control (LPC4320 UM10503, §17.4.2)
// GPIO port n, pin p: address = GPIO_BYTE_BASE + (port << 5) + pin
#define GPIO_BYTE_BASE  0x400F4000u

static inline void gpio_set(uint8_t port, uint8_t pin, bool high)
{
    volatile uint8_t* reg =
        (volatile uint8_t*)(GPIO_BYTE_BASE + ((uint32_t)port << 5) + pin);
    *reg = high ? 1u : 0u;
}

// SD card CS from portapack_pins.hpp (SD_CS_GPIO_PORT=0, SD_CS_GPIO_PIN=6)
#define SD_CS_PORT  0u
#define SD_CS_PIN   6u

static inline void sd_cs_high(void) { gpio_set(SD_CS_PORT, SD_CS_PIN, true);  }
static inline void sd_cs_low(void)  { gpio_set(SD_CS_PORT, SD_CS_PIN, false); }

// ---------------------------------------------------------------------------
// SSP0 register definitions (UM10503 §20)
// ---------------------------------------------------------------------------
#define SSP0_BASE   0x40083000u

#define SSP0_CR0    (*(volatile uint32_t*)(SSP0_BASE + 0x00u))
#define SSP0_CR1    (*(volatile uint32_t*)(SSP0_BASE + 0x04u))
#define SSP0_DR     (*(volatile uint32_t*)(SSP0_BASE + 0x08u))
#define SSP0_SR     (*(volatile uint32_t*)(SSP0_BASE + 0x0Cu))
#define SSP0_CPSR   (*(volatile uint32_t*)(SSP0_BASE + 0x10u))
#define SSP0_IMSC   (*(volatile uint32_t*)(SSP0_BASE + 0x14u))

#define SSP_SR_TFE  (1u << 0)  // TX FIFO empty
#define SSP_SR_TNF  (1u << 1)  // TX FIFO not full
#define SSP_SR_RNE  (1u << 2)  // RX FIFO not empty
#define SSP_SR_RFF  (1u << 3)  // RX FIFO full
#define SSP_SR_BSY  (1u << 4)  // Busy

// Transfer one byte over SSP0 (blocking poll)
static uint8_t ssp0_byte(uint8_t out)
{
    while (!(SSP0_SR & SSP_SR_TNF)) {}  // Wait TX not full
    SSP0_DR = out;
    while (!(SSP0_SR & SSP_SR_RNE)) {}  // Wait RX not empty
    return (uint8_t)(SSP0_DR & 0xFFu);
}

// Drain any residual bytes from RX FIFO
static void ssp0_flush(void)
{
    while (SSP0_SR & SSP_SR_RNE) { (void)SSP0_DR; }
}

static uint8_t sd_xchg(uint8_t b)  { return ssp0_byte(b); }
static uint8_t sd_recv(void)        { return ssp0_byte(0xFFu); }
static void    sd_send(uint8_t b)   { (void)ssp0_byte(b); }

// ---------------------------------------------------------------------------
// SD card state
// ---------------------------------------------------------------------------
static volatile DSTATUS s_status = STA_NOINIT;
static bool             s_sdhc   = false;   // true if SDHC/SDXC card

// ---------------------------------------------------------------------------
// Low-level SD SPI helpers
// ---------------------------------------------------------------------------

// Wait until the DO line is high (card releases it after a write) or timeout.
// Returns true on success.
static bool sd_wait_ready(uint32_t timeout_ms)
{
    // Simple busy-loop count approximating timeout_ms at ~200 MHz AHB
    uint32_t count = timeout_ms * 20000u;
    while (count--) {
        if (sd_recv() == 0xFFu) return true;
    }
    return false;
}

// Send a command and return the R1 response byte.
// For ACMD, caller must first send CMD55.
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc;

    // Select the card
    sd_cs_low();

    // Stuff byte (give card time to de-select previous)
    sd_recv();

    // Command frame: start bit | tx bit | index (6 bits) | arg (32) | CRC (7) | end bit
    sd_send(cmd | 0x40u);
    sd_send((uint8_t)(arg >> 24));
    sd_send((uint8_t)(arg >> 16));
    sd_send((uint8_t)(arg >>  8));
    sd_send((uint8_t)(arg));

    // Pre-computed CRC for CMD0 and CMD8 (only ones that need valid CRC in SPI mode)
    if (cmd == 0x00u) crc = 0x95u;
    else if (cmd == 0x08u) crc = 0x87u;
    else crc = 0x01u;  // Dummy stop bit; CRC is ignored in SPI mode otherwise
    sd_send(crc);

    // Skip one byte if CMD12 (stop transmission)
    if (cmd == 0x0Cu) sd_recv();

    // Poll for response (max 10 bytes)
    uint8_t resp = 0xFFu;
    for (int i = 0; i < 10; ++i) {
        resp = sd_recv();
        if (!(resp & 0x80u)) break;  // Valid response has bit7 = 0
    }
    return resp;
}

// ---------------------------------------------------------------------------
// disk_initialize (physical drive 0 = SD card)
// ---------------------------------------------------------------------------
DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;

    s_status = STA_NOINIT;
    s_sdhc   = false;

    // Ensure SSP0 is enabled (assume hal/ssp.cpp already configured it;
    // if not, we at least flush any garbage from FIFO)
    ssp0_flush();

    // -----------------------------------------------------------------------
    // SD card power-up sequence (SPI mode):
    //   1. Send >= 74 clock pulses with CS deasserted and DI high
    //   2. Assert CS
    //   3. Send CMD0 — expect R1 = 0x01 (idle state)
    //   4. Send CMD8 — check voltage range
    //   5. Send ACMD41 with HCS bit — wait for ready (R1 = 0x00)
    //   6. If CMD8 accepted: check OCR, set s_sdhc flag
    //   7. Send CMD16 to set block length to 512
    // -----------------------------------------------------------------------

    // Step 1: 10 × 0xFF = 80 clocks with CS high
    sd_cs_high();
    for (int i = 0; i < 10; ++i) sd_recv();

    // Step 2+3: CMD0 — go idle
    uint8_t r = sd_cmd(0x00u, 0u);
    if (r != 0x01u) {
        sd_cs_high();
        return STA_NOINIT;
    }

    // Step 4: CMD8 — check if SDv2 card (voltage 2.7–3.6 V, pattern 0xAA)
    bool sdv2 = false;
    r = sd_cmd(0x08u, 0x000001AAu);
    if (r == 0x01u) {
        // Read R7 (4 bytes)
        uint8_t r7[4];
        r7[0] = sd_recv();
        r7[1] = sd_recv();
        r7[2] = sd_recv();
        r7[3] = sd_recv();
        // Confirm voltage range (bits 11:8 = 0001) and echo pattern (0xAA)
        if ((r7[2] & 0x0Fu) == 0x01u && r7[3] == 0xAAu) {
            sdv2 = true;
        }
    }

    // Step 5: ACMD41 — initialise (send CMD55 + CMD41 in a loop)
    uint32_t acmd41_arg = sdv2 ? 0x40000000u : 0u;  // HCS bit for SDv2
    uint32_t timeout    = 100000u;  // ~1 second at typical clock

    do {
        // CMD55 (APP_CMD)
        sd_cmd(0x37u, 0u);
        sd_cs_high();
        sd_recv();
        // CMD41 (SD_SEND_OP_COND)
        r = sd_cmd(0x29u, acmd41_arg);
    } while (r == 0x01u && --timeout);

    if (r != 0x00u) {
        // Initialisation failed
        sd_cs_high();
        return STA_NOINIT;
    }

    // Step 6: For SDv2, read OCR (CMD58) and check CCS bit for SDHC
    if (sdv2) {
        r = sd_cmd(0x3Au, 0u);  // CMD58
        uint8_t ocr[4];
        ocr[0] = sd_recv();
        ocr[1] = sd_recv();
        ocr[2] = sd_recv();
        ocr[3] = sd_recv();
        if (r == 0x00u && (ocr[0] & 0x40u)) {
            s_sdhc = true;  // SDHC / SDXC card
        }
    }

    // Step 7: CMD16 — set block length to 512 bytes (SDSCs only; SDHC ignores it)
    if (!s_sdhc) {
        r = sd_cmd(0x10u, 512u);  // CMD16
        if (r != 0x00u) {
            sd_cs_high();
            return STA_NOINIT;
        }
    }

    sd_cs_high();
    sd_recv();

    s_status = 0;  // Ready
    return s_status;
}

// ---------------------------------------------------------------------------
// disk_status
// ---------------------------------------------------------------------------
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return s_status;
}

// ---------------------------------------------------------------------------
// disk_read
// ---------------------------------------------------------------------------
DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || (s_status & STA_NOINIT)) return RES_NOTRDY;
    if (!count) return RES_PARERR;

    // Convert sector number to byte address for non-SDHC cards
    uint32_t addr = s_sdhc ? (uint32_t)sector : (uint32_t)(sector * 512u);

    uint8_t cmd = (count == 1u) ? 0x51u : 0x52u;  // CMD17 or CMD18

    uint8_t r = sd_cmd(cmd, addr);
    if (r != 0x00u) {
        sd_cs_high();
        return RES_ERROR;
    }

    do {
        // Wait for data token (0xFE)
        uint32_t wait = 200000u;
        while (--wait) {
            if (sd_recv() == 0xFEu) break;
        }
        if (wait == 0u) {
            sd_cs_high();
            return RES_ERROR;
        }

        // Read 512 data bytes
        for (int i = 0; i < 512; ++i) {
            *buff++ = sd_recv();
        }
        // Discard 2-byte CRC
        sd_recv();
        sd_recv();
    } while (--count);

    if (cmd == 0x52u) {
        // CMD18: send STOP_TRANSMISSION (CMD12)
        sd_cmd(0x0Cu, 0u);
    }

    sd_cs_high();
    sd_recv();

    return RES_OK;
}

// ---------------------------------------------------------------------------
// disk_write
// ---------------------------------------------------------------------------
#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || (s_status & STA_NOINIT)) return RES_NOTRDY;
    if (s_status & STA_PROTECT) return RES_WRPRT;
    if (!count) return RES_PARERR;

    uint32_t addr = s_sdhc ? (uint32_t)sector : (uint32_t)(sector * 512u);

    if (count == 1u) {
        // Single block write: CMD24
        uint8_t r = sd_cmd(0x58u, addr);
        if (r != 0x00u) {
            sd_cs_high();
            return RES_ERROR;
        }

        // Send data token 0xFE
        sd_send(0xFEu);

        for (int i = 0; i < 512; ++i) {
            sd_send(*buff++);
        }
        // Dummy CRC
        sd_send(0xFFu);
        sd_send(0xFFu);

        // Data response token: lower nibble 0x05 = accepted
        uint8_t resp = sd_recv() & 0x1Fu;
        if (resp != 0x05u) {
            sd_cs_high();
            return RES_ERROR;
        }

        // Wait for write to complete
        if (!sd_wait_ready(500u)) {
            sd_cs_high();
            return RES_ERROR;
        }
    } else {
        // Multiple block write: CMD25
        uint8_t r = sd_cmd(0x59u, addr);
        if (r != 0x00u) {
            sd_cs_high();
            return RES_ERROR;
        }

        do {
            // Send data token 0xFC (multiple-block token)
            sd_send(0xFCu);

            for (int i = 0; i < 512; ++i) {
                sd_send(*buff++);
            }
            sd_send(0xFFu);
            sd_send(0xFFu);

            uint8_t resp = sd_recv() & 0x1Fu;
            if (resp != 0x05u) {
                sd_cs_high();
                return RES_ERROR;
            }

            if (!sd_wait_ready(500u)) {
                sd_cs_high();
                return RES_ERROR;
            }
        } while (--count);

        // Send stop token for CMD25
        sd_send(0xFDu);
        sd_recv();

        if (!sd_wait_ready(500u)) {
            sd_cs_high();
            return RES_ERROR;
        }
    }

    sd_cs_high();
    sd_recv();

    return RES_OK;
}
#endif /* FF_FS_READONLY == 0 */

// ---------------------------------------------------------------------------
// disk_ioctl
// ---------------------------------------------------------------------------
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff)
{
    if (pdrv != 0 || (s_status & STA_NOINIT)) return RES_NOTRDY;

    DRESULT res = RES_ERROR;

    switch (cmd) {

    case CTRL_SYNC:
        // Ensure card is not busy
        sd_cs_low();
        if (sd_wait_ready(1000u)) res = RES_OK;
        sd_cs_high();
        break;

    case GET_SECTOR_COUNT:
        // Send CMD9 (SEND_CSD) and parse CSD register to determine capacity
        {
            uint8_t csd[16];
            uint8_t r = sd_cmd(0x09u, 0u);
            if (r == 0x00u) {
                // Wait for data token
                uint32_t wait = 100000u;
                while (--wait && sd_recv() != 0xFEu) {}
                if (wait) {
                    for (int i = 0; i < 16; ++i) csd[i] = sd_recv();
                    sd_recv(); sd_recv();  // CRC

                    LBA_t sectors;
                    if ((csd[0] >> 6) == 1u) {
                        // CSD version 2 (SDHC / SDXC)
                        uint32_t c_size = (uint32_t)(csd[9])
                                        | ((uint32_t)(csd[8]) << 8)
                                        | ((uint32_t)(csd[7] & 0x3Fu) << 16);
                        sectors = (LBA_t)(c_size + 1u) * 1024u;
                    } else {
                        // CSD version 1 (SDSC)
                        uint32_t c_size_mult = (uint32_t)((csd[10] & 0x80u) >> 7)
                                             | (uint32_t)((csd[9]  & 0x03u) << 1);
                        uint32_t c_size      = (uint32_t)((csd[8]  & 0xC0u) >> 6)
                                             | (uint32_t)((csd[7]) << 2)
                                             | (uint32_t)((csd[6]  & 0x03u) << 10);
                        uint32_t read_bl_len = (uint32_t)(csd[5]  & 0x0Fu);
                        sectors = (LBA_t)((c_size + 1u)
                                          << (c_size_mult + 2u + read_bl_len - 9u));
                    }
                    *(LBA_t*)buff = sectors;
                    res = RES_OK;
                }
            }
            sd_cs_high();
        }
        break;

    case GET_BLOCK_SIZE:
        // Erasable block granularity in sectors (1 = unknown / 512-byte granularity)
        *(DWORD*)buff = 1u;
        res = RES_OK;
        break;

    case GET_SECTOR_SIZE:
        // Always 512 for SD cards
        *(WORD*)buff = 512u;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
        break;
    }

    return res;
}
