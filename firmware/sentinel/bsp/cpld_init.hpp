// SPDX-License-Identifier: MIT
// Project Sentinel — PortaPack CPLD JTAG Programmer
//
// Programs the PortaPack H4M CPLD (AG256SL100 / MAX V compatible)
// via JTAG bit-banging.  The CPLD contains LCD bus routing logic
// that must be loaded before the ILI9341 LCD can be accessed.
//
// Ported from portapack-mayhem: cpld_max5.cpp, cpld_update.cpp,
// jtag.cpp, jtag_target_gpio.hpp.

#pragma once
#include <cstdint>
#include <cstddef>

// CPLD bitstream configuration
struct CpldConfig {
    const uint16_t* block_0;     // CFM block: 3328 words
    const uint16_t* block_1;     // UFM block: 512 words
};

static constexpr size_t CPLD_BLOCK_0_SIZE = 3328;
static constexpr size_t CPLD_BLOCK_1_SIZE = 512;

// H4M bitstream (defined in cpld_data_h4m.cpp)
extern const CpldConfig cpld_config_h4m;

// Status codes
enum class CpldStatus {
    Success = 0,
    IdcodeFail,
    SiliconIdFail,
    ProgramFail,
};

// Initialise the PortaPack CPLD: verify and reprogram if necessary.
// Must be called before lcd_init().
// Returns CpldStatus::Success if CPLD is ready.
CpldStatus portapack_cpld_init();
