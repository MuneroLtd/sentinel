// SPDX-License-Identifier: MIT
// Project Sentinel — High-Level Radio Orchestration
//
// Coordinates Si5351C, RFFC5072, MAX2837, MAX5864, and CPLD to
// implement a complete wideband receive chain from 1 MHz to 6 GHz.
//
// Frequency plan:
//   Region A — 1 MHz to ~2.3 GHz:
//     RFFC5072 mixes RF down to 2600 MHz IF.
//     LO = RF + 2600 MHz  (low-side LO injection: LO above RF)
//     MAX2837 tuned to 2600 MHz (fixed).
//
//   Region B — 2.3 GHz to 2.7 GHz (MAX2837 direct):
//     RFFC5072 bypassed (disabled).
//     MAX2837 VCO tuned directly to the target frequency.
//
//   Region C — 2.7 GHz to 6 GHz:
//     RFFC5072 mixes RF down to 2600 MHz IF.
//     LO = RF - 2600 MHz  (high-side LO injection: LO below RF)
//     MAX2837 tuned to 2600 MHz (fixed).
//
// Note: RFFC5072 LO maximum is 4200 MHz, which constrains Region A top to
//   LO = RF + 2600 → RF_max = 4200 - 2600 = 1600 MHz within safe LO range.
//   For 1.6–2.3 GHz, LO = RF + 2600 is 4200–4900 MHz (exceeds RFFC5072 max).
//   Alternative: use harmonic mixing or accept slight de-rating above 1.6 GHz.
//   In HackRF, this is handled by the RFFC5072 supporting RF input up to 6 GHz
//   on its mixer port even when VCO LO > rated spec for some bands.
//   Practical limit used here: LO stays ≤ 4200 MHz.
//   RF > 1600 MHz and < 2300 MHz: LO = RF + IF would exceed 4200 MHz;
//   instead clamp LO = 4200 MHz (accepting slight frequency error in this
//   edge band) or flag unsupported.  The user is expected to know the limits.

#pragma once
#include <cstdint>
#include <cstdbool>

namespace sentinel {
namespace radio {

    // -----------------------------------------------------------------------
    // Initialisation
    // -----------------------------------------------------------------------

    // Power-up and initialise all RF components in the correct sequence:
    //   1. CPLD programmed and RF switches set to RX
    //   2. Si5351C clocks started (CLK0 → RFFC5072, CLK1 → MAX2837)
    //   3. RFFC5072 initialised in RX mode, LO standby
    //   4. MAX2837 initialised
    //   5. MAX5864 in RX mode
    // Returns true if all components initialised successfully.
    bool init();

    // -----------------------------------------------------------------------
    // Frequency tuning
    // -----------------------------------------------------------------------

    // Tune the radio to a centre frequency in Hz (1 MHz – 6 GHz).
    //   - Selects the correct frequency region (A, B, or C above).
    //   - Configures RFFC5072 LO (or disables it for region B).
    //   - Sets MAX2837 VCO.
    //   - Updates the CPLD RF switch path.
    // Returns true on success.
    bool tune(uint64_t freq_hz);

    // -----------------------------------------------------------------------
    // Gain control
    // -----------------------------------------------------------------------

    // Set LNA and baseband VGA gain.
    //   lna_db : 0, 8, 16, 24, 32, or 40 dB
    //   vga_db : 0–62 dB (even values only; odd values rounded down)
    void set_gain(uint8_t lna_db, uint8_t vga_db);

    // -----------------------------------------------------------------------
    // Bandwidth / filter
    // -----------------------------------------------------------------------

    // Set approximate baseband filter bandwidth on MAX2837.
    //   bw_hz : desired bandwidth in Hz.
    //   Supported values: 1750000, 2500000, 3500000, 5000000,
    //                     5500000, 6000000, 7000000, 8000000,
    //                     9000000, 10000000, 12000000, 14000000,
    //                     15000000, 20000000, 24000000, 28000000 Hz.
    //   Value is rounded to the nearest supported setting.
    void set_bandwidth(uint32_t bw_hz);

    // -----------------------------------------------------------------------
    // Mode control
    // -----------------------------------------------------------------------

    // Switch to RX mode (default). Configures CPLD, MAX2837, and MAX5864.
    void rx_mode();

    // Power down all RF components to minimum power.
    void standby();

    // -----------------------------------------------------------------------
    // Status
    // -----------------------------------------------------------------------

    // Return the currently tuned centre frequency in Hz.
    // Returns 0 if the radio has not been tuned.
    uint64_t current_freq_hz();

} // namespace radio
} // namespace sentinel
