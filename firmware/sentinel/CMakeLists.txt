cmake_minimum_required(VERSION 3.16)

# Project Sentinel — M4 Sentinel Firmware
# Standalone CMake for the Cortex-M4F application firmware.
# Requires: arm-none-eabi toolchain (see cmake/arm-none-eabi.cmake).
# The M0 baseband binary must be built first; it is linked in as a raw
# binary section so the M4 can copy it to M0 SRAM at boot.

project(sentinel_m4 C CXX ASM)

# ---------------------------------------------------------------------------
# CPU and compiler flags for Cortex-M4F with hardware FPU
# ---------------------------------------------------------------------------
set(CPU_FLAGS
    "-mcpu=cortex-m4"
    "-mthumb"
    "-mfpu=fpv4-sp-d16"
    "-mfloat-abi=hard"
)
string(JOIN " " CPU_FLAGS_STR ${CPU_FLAGS})

set(COMMON_FLAGS
    "${CPU_FLAGS_STR}"
    "-O2"
    "-ffunction-sections"
    "-fdata-sections"
    "-fno-exceptions"
    "-fno-rtti"
    "-nostdlib"
    "-Wall"
    "-Wextra"
)
string(JOIN " " COMMON_FLAGS_STR ${COMMON_FLAGS})

set(CMAKE_C_FLAGS   "${COMMON_FLAGS_STR}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS_STR} -std=c++17" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${CPU_FLAGS_STR}"                CACHE STRING "" FORCE)

# ---------------------------------------------------------------------------
# FreeRTOS kernel
# Fetched via git subtree into freertos/FreeRTOS-Kernel/
# ---------------------------------------------------------------------------
set(FREERTOS_PORT   GCC_ARM_CM4F CACHE STRING "" FORCE)
set(FREERTOS_HEAP   4            CACHE STRING "" FORCE)

# Tell FreeRTOS-Kernel where to find FreeRTOSConfig.h
set(FREERTOS_CONFIG_FILE_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/freertos
    CACHE STRING "" FORCE)

add_subdirectory(freertos/FreeRTOS-Kernel freertos_kernel)

# ---------------------------------------------------------------------------
# FatFS — SD card filesystem
# Fetched into third_party/fatfs/ (BSD-1-Clause equivalent, permissive)
# ---------------------------------------------------------------------------
add_library(fatfs STATIC
    ${CMAKE_SOURCE_DIR}/../../third_party/fatfs/ff.c
    ${CMAKE_SOURCE_DIR}/../../third_party/fatfs/ffsystem.c
    ${CMAKE_SOURCE_DIR}/../../third_party/fatfs/ffunicode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/diskio_lpc4320.c
)

target_include_directories(fatfs PUBLIC
    ${CMAKE_SOURCE_DIR}/../../third_party/fatfs
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../shared
)

# fatfs is a C library; apply only C flags
set_target_properties(fatfs PROPERTIES
    C_STANDARD   11
    LINKER_LANGUAGE C
)

# ---------------------------------------------------------------------------
# Sentinel M4 firmware executable
# ---------------------------------------------------------------------------
add_executable(sentinel_m4
    # Startup / main
    main.cpp

    # HAL
    hal/clocks.cpp
    hal/gpio.cpp
    hal/ssp.cpp
    hal/i2c.cpp
    hal/uart.cpp

    # BSP
    bsp/lcd_ili9341.cpp
    bsp/audio_wm8731.cpp

    # Radio front-end drivers
    radio/si5351.cpp
    radio/rffc5072.cpp
    radio/max2837.cpp
    radio/max5864.cpp
    radio/cpld.cpp
    radio/radio_ctrl.cpp

    # UI
    ui/font.cpp
    ui/canvas.cpp
    ui/widgets.cpp

    # Event bus
    event_bus/event_bus.cpp

    # Tasks
    tasks/task_orchestrator.cpp
    tasks/task_app.cpp
    tasks/task_radio_manager.cpp
    tasks/task_bg_scanner.cpp
    tasks/task_esp32_comms.cpp
    tasks/task_knowledge_base.cpp
    tasks/task_logger.cpp

    # Drivers
    drivers/i2c_esp32.cpp

    # Apps
    apps/sentinel_app_base.cpp
    apps/app_home_dashboard.cpp
    apps/app_spectrum.cpp
    apps/app_fm.cpp
    apps/app_adsb.cpp
    apps/app_scanner.cpp
    apps/app_capture.cpp

    # FreeRTOS runtime stats counter
    freertos/sentinel_runtime.cpp

    # Bare-metal runtime stubs
    libc_stubs.c
    cxx_stubs.cpp
)

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
target_include_directories(sentinel_m4 PRIVATE
    # Firmware root (so #include "hal/foo.hpp" etc. work from any source file)
    ${CMAKE_CURRENT_SOURCE_DIR}
    # Project root — needed for includes like "firmware/sentinel/ui/widgets.hpp"
    # that appear in sentinel_app_base.hpp and similar cross-cutting headers.
    ${CMAKE_SOURCE_DIR}/../../
    # Shared IPC protocol header (firmware/shared/ipc/ipc_protocol.hpp)
    ${CMAKE_SOURCE_DIR}/../shared
    # CMSIS-Core for Cortex-M4 intrinsics (__asm volatile("wfi"), DWT, etc.)
    ${CMAKE_SOURCE_DIR}/../../third_party/CMSIS_5/CMSIS/Core/Include
)

# ---------------------------------------------------------------------------
# Linker options
#   - Use the project-specific linker script
#   - Link M0 baseband binary as a raw binary section (.m0_binary)
#     The M4 startup code copies it to M0 SRAM at address 0x10080000.
# ---------------------------------------------------------------------------
set(BASEBAND_BIN ${CMAKE_BINARY_DIR}/../baseband/baseband.bin)

target_link_options(sentinel_m4 PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/sentinel_m4.ld
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# Embed M0 baseband binary: switch input format to binary, include file,
# then switch back to ELF so remaining libraries link correctly.
target_link_libraries(sentinel_m4 PRIVATE
    -Wl,--format=binary
    -Wl,${BASEBAND_BIN}
    -Wl,--format=default
)

# Find runtime libraries for -nostdlib bare-metal build
set(_M4_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
foreach(_lib libgcc.a libc_nano.a libnosys.a libm.a)
    execute_process(
        COMMAND ${CMAKE_C_COMPILER} ${_M4_FLAGS} -print-file-name=${_lib}
        OUTPUT_VARIABLE _path
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    list(APPEND M4_RUNTIME_LIBS ${_path})
endforeach()

target_link_libraries(sentinel_m4 PRIVATE
    freertos_kernel
    fatfs
    ${M4_RUNTIME_LIBS}
)

# ---------------------------------------------------------------------------
# Post-build: produce raw binary for spiflash programming
# ---------------------------------------------------------------------------
add_custom_command(TARGET sentinel_m4 POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
            -O binary
            $<TARGET_FILE:sentinel_m4>
            ${CMAKE_CURRENT_BINARY_DIR}/sentinel_m4.bin
    COMMENT "Creating sentinel_m4.bin for flash programming"
)

find_program(ARM_SIZE arm-none-eabi-size)
if(ARM_SIZE)
    add_custom_command(TARGET sentinel_m4 POST_BUILD
        COMMAND ${ARM_SIZE} $<TARGET_FILE:sentinel_m4>
        COMMENT "Firmware size report"
    )
endif()
