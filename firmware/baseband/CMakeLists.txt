# SPDX-License-Identifier: MIT
# Project Sentinel — Cortex-M0 Baseband DSP Firmware
# CMakeLists.txt — standalone CMake build for the M0 binary
#
# Produces baseband.elf and baseband.bin (raw binary for loading into M4 firmware image).
# Build from the project root:
#   cmake -B build/baseband -S firmware/baseband \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build/baseband

cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# Toolchain must be configured BEFORE project() so CMake uses it for the ABI
# checks.  Set CMAKE_TOOLCHAIN_FILE on the command line, or configure the
# variables here if not using a separate toolchain file.
# ---------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME    Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# If no explicit toolchain prefix is given, assume arm-none-eabi- is on PATH
if(NOT DEFINED CROSS_COMPILE)
    set(CROSS_COMPILE arm-none-eabi-)
endif()

set(CMAKE_C_COMPILER   ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)
set(CMAKE_ASM_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_OBJCOPY      ${CROSS_COMPILE}objcopy)
set(CMAKE_SIZE         ${CROSS_COMPILE}size)

# Prevent CMake from testing the compiler with -rdynamic (unsupported on bare-metal)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(sentinel_baseband C CXX ASM)

# ---------------------------------------------------------------------------
# Cortex-M0 CPU flags
# ---------------------------------------------------------------------------

set(CPU_FLAGS
    -mcpu=cortex-m0
    -mthumb
    -mfloat-abi=soft        # No FPU on Cortex-M0
)

string(JOIN " " CPU_FLAGS_STR ${CPU_FLAGS})

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

set(COMMON_FLAGS
    "${CPU_FLAGS_STR}"
    -O2
    -ffunction-sections
    -fdata-sections
    -nostdlib
    -Wall
    -Wextra
    -Wno-unused-parameter
)

string(JOIN " " COMMON_FLAGS_STR ${COMMON_FLAGS})

set(CMAKE_C_FLAGS   "${COMMON_FLAGS_STR}")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS_STR} -fno-exceptions -fno-rtti -std=c++17 -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS_STR}")

# ---------------------------------------------------------------------------
# Linker flags
# ---------------------------------------------------------------------------

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/m0.ld")

set(CMAKE_EXE_LINKER_FLAGS
    "${CPU_FLAGS_STR} -T${LINKER_SCRIPT} -Wl,--gc-sections -nostdlib -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/baseband.map"
)

# ---------------------------------------------------------------------------
# CMSIS-DSP (Apache 2.0 — compatible with MIT firmware)
# ---------------------------------------------------------------------------
# Expected path: firmware/third_party/CMSIS_5/
# CMSIS-DSP source is structured with individual Source subdirectories.

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../third_party/CMSIS_5")
set(CMSIS_CORE_INCLUDE "${CMSIS_ROOT}/CMSIS/Core/Include")

# CMSIS-DSP is in its own standalone repo (split from CMSIS_5)
set(CMSIS_DSP_ROOT     "${CMAKE_SOURCE_DIR}/../third_party/CMSIS-DSP")
set(CMSIS_DSP_INCLUDE  "${CMSIS_DSP_ROOT}/Include")
set(CMSIS_DSP_PRIVATE  "${CMSIS_DSP_ROOT}/PrivateInclude")
set(CMSIS_DSP_SOURCE   "${CMSIS_DSP_ROOT}/Source")

# Only compile the CMSIS-DSP modules we actually use:
#   - arm_cfft_q15        (ComplexMathFunctions)
#   - arm_cmplx_mag_q15   (ComplexMathFunctions)
#   - arm_rfft_q15        (TransformFunctions — included for completeness)
#   - arm_cfft_init_q15   (TransformFunctions)
#   Support functions (common tables used by all)

# CMSIS-DSP requires these compile definitions for Cortex-M0 (no DSP extension)
add_library(CMSISDSP STATIC
    # Transform functions
    ${CMSIS_DSP_SOURCE}/TransformFunctions/arm_cfft_q15.c
    ${CMSIS_DSP_SOURCE}/TransformFunctions/arm_cfft_init_q15.c
    ${CMSIS_DSP_SOURCE}/TransformFunctions/arm_cfft_radix4_q15.c
    ${CMSIS_DSP_SOURCE}/TransformFunctions/arm_rfft_q15.c
    ${CMSIS_DSP_SOURCE}/TransformFunctions/arm_rfft_init_q15.c
    ${CMSIS_DSP_SOURCE}/TransformFunctions/arm_bitreversal2.c
    # Complex math
    ${CMSIS_DSP_SOURCE}/ComplexMathFunctions/arm_cmplx_mag_q15.c
    # Fast math (arm_sqrt_q31 used by arm_cmplx_mag_q15)
    ${CMSIS_DSP_SOURCE}/FastMathFunctions/arm_sqrt_q31.c
    # Common tables (twiddle factors, bit-reversal tables — gc-sections strips unused sizes)
    ${CMSIS_DSP_SOURCE}/CommonTables/arm_common_tables.c
    ${CMSIS_DSP_SOURCE}/CommonTables/arm_const_structs.c
)

target_compile_definitions(CMSISDSP PUBLIC
    ARM_MATH_CM0           # Tell CMSIS we're on Cortex-M0
    ARM_MATH_LOOPUNROLL    # Allow loop unrolling
    # Disable features not available on M0
    DISABLEFLOAT16
)

target_include_directories(CMSISDSP PUBLIC
    ${CMSIS_CORE_INCLUDE}
    ${CMSIS_DSP_INCLUDE}
    ${CMSIS_DSP_PRIVATE}
)

target_compile_options(CMSISDSP PRIVATE ${CPU_FLAGS} -O2 -ffunction-sections -fdata-sections -nostdlib)

# ---------------------------------------------------------------------------
# Baseband firmware executable
# ---------------------------------------------------------------------------

add_executable(baseband
    m0_vectors.cpp
    libc_stubs.c
    main_m0.cpp
    dsp/fm_demod.cpp
    dsp/spectrum.cpp
    dsp/adsb.cpp
    dsp/scanner.cpp
    dsp/rssi.cpp
)

target_include_directories(baseband PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}          # for m0_startup.hpp
    ${CMAKE_SOURCE_DIR}/../shared        # for ipc/ipc_protocol.hpp
    ${CMSIS_CORE_INCLUDE}
    ${CMSIS_DSP_INCLUDE}
)

# Link CMSIS-DSP and libgcc (for __aeabi_idiv, __aeabi_lmul, etc. — Cortex-M0 has no HW divide)
# Use the compiler to find the correct multilib libgcc for our CPU flags.
execute_process(
    COMMAND ${CMAKE_C_COMPILER} ${CPU_FLAGS} -print-libgcc-file-name
    OUTPUT_VARIABLE LIBGCC_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
target_link_libraries(baseband PRIVATE CMSISDSP ${LIBGCC_PATH})

# Depend on the linker script so re-link happens if it changes
set_target_properties(baseband PROPERTIES
    LINK_DEPENDS "${LINKER_SCRIPT}"
)

# ---------------------------------------------------------------------------
# Post-build: create raw binary and print size
# ---------------------------------------------------------------------------

add_custom_command(TARGET baseband POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:baseband>
            ${CMAKE_CURRENT_BINARY_DIR}/baseband.bin
    COMMENT "Creating baseband.bin (raw binary for M4 firmware image)"
)

add_custom_command(TARGET baseband POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:baseband>
    COMMENT "Baseband firmware size:"
)
