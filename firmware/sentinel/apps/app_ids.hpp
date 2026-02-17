// SPDX-License-Identifier: MIT
// Project Sentinel - Application identifiers
#pragma once

#include <cstdint>

namespace sentinel {

enum class AppID : uint8_t {
    NONE      = 0,
    DASHBOARD = 1,
    SPECTRUM  = 2,
    FM_RADIO  = 3,
    ADS_B     = 4,
    SCANNER   = 5,
    CAPTURE   = 6,
};

} // namespace sentinel
