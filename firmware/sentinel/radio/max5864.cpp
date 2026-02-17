// SPDX-License-Identifier: MIT
// Project Sentinel — MAX5864 Baseband ADC/DAC Driver Implementation
//
// Reference: MAX5864 datasheet (Analog Devices), Table 3 (Operating Modes),
// and HackRF firmware max5864.c (original by Mike Ossmann, BSD 2-clause).
//
// The MAX5864 is controlled entirely by writing a single mode byte over SPI.
// There are no readable registers.
// The actual IQ sample stream is handled by the LPC4320 SGPIO+DMA subsystem
// (not in this file).

#include "max5864.hpp"
#include "hal/ssp.hpp"
#include "hal/gpio.hpp"
#include "bsp/portapack_pins.hpp"

// ---------------------------------------------------------------------------
// SPI bus assignment (SSP0, shared with RFFC5072/MAX2837; CS distinguishes)
// ---------------------------------------------------------------------------
static constexpr uint8_t MAX5864_SSP_BUS = 0u; // SSP0

// ---------------------------------------------------------------------------
// Mode byte constants (MAX5864 datasheet Table 3)
// ---------------------------------------------------------------------------
static constexpr uint8_t MAX5864_MODE_SHUTDOWN   = 0x00u;
static constexpr uint8_t MAX5864_MODE_IDLE        = 0x01u;
static constexpr uint8_t MAX5864_MODE_RX          = 0x02u;
static constexpr uint8_t MAX5864_MODE_TX          = 0x03u;
static constexpr uint8_t MAX5864_MODE_XCVR        = 0x04u;
static constexpr uint8_t MAX5864_MODE_STANDBY     = 0x05u;

// ---------------------------------------------------------------------------
// Low-level: send one mode byte
// ---------------------------------------------------------------------------
static void max5864_write_mode(uint8_t mode) {
    gpio_cs_low(PORT_MAX5864_CS, PIN_MAX5864_CS);
    ssp_transfer(MAX5864_SSP_BUS, mode);
    gpio_cs_high(PORT_MAX5864_CS, PIN_MAX5864_CS);
}

// ---------------------------------------------------------------------------
// max5864_init
// ---------------------------------------------------------------------------
bool max5864_init() {
    // Configure CS as output, deasserted (high)
    gpio_set_dir(PORT_MAX5864_CS, PIN_MAX5864_CS, true);
    gpio_cs_high(PORT_MAX5864_CS, PIN_MAX5864_CS);

    // Power up into RX mode (ADC active, DAC off)
    max5864_write_mode(MAX5864_MODE_RX);
    return true;
}

// ---------------------------------------------------------------------------
// max5864_rx_mode — ADCs on, DACs off
// ---------------------------------------------------------------------------
void max5864_rx_mode() {
    max5864_write_mode(MAX5864_MODE_RX);
}

// ---------------------------------------------------------------------------
// max5864_tx_mode — DACs on, ADCs off
// ---------------------------------------------------------------------------
void max5864_tx_mode() {
    max5864_write_mode(MAX5864_MODE_TX);
}

// ---------------------------------------------------------------------------
// max5864_xcvr_mode — ADCs and DACs both active (FDD)
// ---------------------------------------------------------------------------
void max5864_xcvr_mode() {
    max5864_write_mode(MAX5864_MODE_XCVR);
}

// ---------------------------------------------------------------------------
// max5864_idle — converters off, reference and clock running
// ---------------------------------------------------------------------------
void max5864_idle() {
    max5864_write_mode(MAX5864_MODE_IDLE);
}

// ---------------------------------------------------------------------------
// max5864_standby — reference on, clock off (fast wakeup)
// ---------------------------------------------------------------------------
void max5864_standby() {
    max5864_write_mode(MAX5864_MODE_STANDBY);
}

// ---------------------------------------------------------------------------
// max5864_shutdown — all circuits off
// ---------------------------------------------------------------------------
void max5864_shutdown() {
    max5864_write_mode(MAX5864_MODE_SHUTDOWN);
}
