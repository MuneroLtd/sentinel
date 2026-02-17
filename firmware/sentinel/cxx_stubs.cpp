// SPDX-License-Identifier: MIT
// Minimal C++ runtime stubs for -nostdlib -fno-exceptions bare-metal firmware

#include <cstddef>

void operator delete(void*) noexcept {}
void operator delete(void*, std::size_t) noexcept {}

// Pure virtual call handler (should never be reached)
extern "C" void __cxa_pure_virtual() { for (;;); }
