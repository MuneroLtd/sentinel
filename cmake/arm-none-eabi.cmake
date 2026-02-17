# SPDX-License-Identifier: MIT
# Project Sentinel — CMake Toolchain File for arm-none-eabi GCC
#
# Target: bare-metal ARM Cortex-M (both M4 and M0 sub-cores).
# Tested with arm-none-eabi-gcc 13.x (Arm GNU Toolchain).
#
# Usage:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#
# The toolchain sets CMAKE_TRY_COMPILE_TARGET_TYPE to STATIC_LIBRARY so that
# CMake does not attempt to link a test executable (which would fail without
# a proper linker script during the compiler detection phase).

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Prevent CMake from running link tests (no OS, no libc, no startup file)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---------------------------------------------------------------------------
# Toolchain binaries
# ---------------------------------------------------------------------------
find_program(CMAKE_C_COMPILER
    NAMES arm-none-eabi-gcc
    REQUIRED
    DOC "arm-none-eabi C compiler"
)

find_program(CMAKE_CXX_COMPILER
    NAMES arm-none-eabi-g++
    REQUIRED
    DOC "arm-none-eabi C++ compiler"
)

find_program(CMAKE_ASM_COMPILER
    NAMES arm-none-eabi-gcc
    REQUIRED
    DOC "arm-none-eabi assembler (invoked via gcc)"
)

find_program(CMAKE_OBJCOPY
    NAMES arm-none-eabi-objcopy
    REQUIRED
    DOC "arm-none-eabi objcopy"
)

find_program(CMAKE_SIZE
    NAMES arm-none-eabi-size
    REQUIRED
    DOC "arm-none-eabi size"
)

find_program(CMAKE_GDB
    NAMES arm-none-eabi-gdb
    DOC "arm-none-eabi GDB (optional — for debug sessions)"
)

# ---------------------------------------------------------------------------
# Tell CMake the compilers are confirmed working.
# Without these, CMake will re-run compiler detection on each configure.
# ---------------------------------------------------------------------------
set(CMAKE_C_COMPILER_WORKS   1 CACHE BOOL "" FORCE)
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE BOOL "" FORCE)
set(CMAKE_ASM_COMPILER_WORKS 1 CACHE BOOL "" FORCE)

# ---------------------------------------------------------------------------
# Sysroot / search paths
# When cross-compiling, do not search host paths for libraries or headers.
# ---------------------------------------------------------------------------
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---------------------------------------------------------------------------
# Default language standards for all targets that use this toolchain.
# Individual target CMakeLists.txt may override these.
# ---------------------------------------------------------------------------
set(CMAKE_C_STANDARD   11 CACHE STRING "Default C standard")
set(CMAKE_CXX_STANDARD 17 CACHE STRING "Default C++ standard")
