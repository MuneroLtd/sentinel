cmake_minimum_required(VERSION 3.16)

# Project Sentinel — Top-level CMake
#
# Builds in two stages:
#   1. firmware/baseband  → Cortex-M0 baseband DSP binary (baseband.bin)
#   2. firmware/sentinel  → Cortex-M4 application firmware (sentinel_m4.bin)
#      The M4 links the M0 binary as a raw section so the startup code can
#      copy it to M0 SRAM before releasing the M0 from reset.
#
# Usage (from project root):
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build -j$(nproc)
#
# Or use the convenience Makefile wrapper at the project root.

project(ProjectSentinel NONE)

# Cross-compilation toolchain is required.
# Users must pass -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
# or it will be picked up automatically from the Makefile.
if(NOT CMAKE_TOOLCHAIN_FILE)
    message(WARNING
        "CMAKE_TOOLCHAIN_FILE not set. "
        "Pass -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake "
        "or use the top-level Makefile.")
endif()

# Build M0 baseband DSP firmware first.
# The resulting binary at <build>/baseband_build/baseband.bin is consumed
# by the sentinel_build step.
add_subdirectory(firmware/baseband baseband_build)

# Build M4 sentinel application firmware.
# The CMakeLists.txt in firmware/sentinel references the M0 binary path
# relative to CMAKE_BINARY_DIR (set by the top-level cmake invocation).
add_subdirectory(firmware/sentinel sentinel_build)
