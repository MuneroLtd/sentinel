# SPDX-License-Identifier: MIT
# Project Sentinel — Top-level build convenience wrapper
#
# Delegates to CMake but provides short, memorable targets.
# The arm-none-eabi toolchain must be on PATH.
#
# Targets:
#   all          Build M0 baseband and M4 sentinel firmware (default)
#   baseband     Build M0 baseband DSP firmware only
#   sentinel     Build M4 application firmware (requires baseband)
#   flash-lpc    Flash sentinel_m4.bin to HackRF via hackrf_spiflash
#   clean        Remove build directory
#   help         Print this help

BUILD_DIR ?= build
JOBS      ?= $(shell nproc 2>/dev/null || echo 4)
TOOLCHAIN := cmake/arm-none-eabi.cmake
BUILD_TYPE ?= Release

# CMake invocation flags shared by both sub-builds
CMAKE_BASE_FLAGS := \
	-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN) \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

.PHONY: all baseband sentinel flash-lpc clean help

# ---------------------------------------------------------------------------
# Default target: build everything
# ---------------------------------------------------------------------------
all: baseband sentinel

# ---------------------------------------------------------------------------
# M0 baseband DSP firmware
# ---------------------------------------------------------------------------
baseband:
	@echo "--- Configuring baseband (M0) ---"
	cmake -B $(BUILD_DIR)/baseband \
	      -S firmware/baseband \
	      $(CMAKE_BASE_FLAGS)
	@echo "--- Building baseband (M0) ---"
	cmake --build $(BUILD_DIR)/baseband -j$(JOBS)

# ---------------------------------------------------------------------------
# M4 sentinel application firmware
# Depends on baseband because the M4 ELF embeds the M0 binary.
# ---------------------------------------------------------------------------
sentinel: baseband
	@echo "--- Configuring sentinel (M4) ---"
	cmake -B $(BUILD_DIR)/sentinel \
	      -S firmware/sentinel \
	      $(CMAKE_BASE_FLAGS)
	@echo "--- Building sentinel (M4) ---"
	cmake --build $(BUILD_DIR)/sentinel -j$(JOBS)

# ---------------------------------------------------------------------------
# Flash the sentinel firmware to HackRF's SPI NOR flash.
# Requires hackrf_spiflash from the hackrf host tools.
# The HackRF must be in DFU/bootloader mode (hold DFU button during USB plug-in).
# ---------------------------------------------------------------------------
flash-lpc: sentinel
	@echo "--- Flashing sentinel_m4.bin ---"
	hackrf_spiflash -w $(BUILD_DIR)/sentinel/sentinel_m4.bin
	@echo "--- Done. Power-cycle the device. ---"

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------
clean:
	rm -rf $(BUILD_DIR)
	@echo "Build directory removed."

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------
help:
	@echo ""
	@echo "Project Sentinel Build System"
	@echo "============================="
	@echo ""
	@echo "  make                  Build everything (baseband + sentinel)"
	@echo "  make all              Same as above"
	@echo "  make baseband         Build M0 baseband firmware only"
	@echo "  make sentinel         Build M4 sentinel firmware (needs baseband)"
	@echo "  make flash-lpc        Flash sentinel to HackRF via hackrf_spiflash"
	@echo "  make clean            Remove build directory"
	@echo "  make help             Show this help"
	@echo ""
	@echo "Options:"
	@echo "  BUILD_DIR=<dir>       Build output directory (default: build)"
	@echo "  BUILD_TYPE=<type>     CMake build type: Release|Debug (default: Release)"
	@echo "  JOBS=<n>              Parallel jobs (default: nproc)"
	@echo ""
	@echo "Prerequisites:"
	@echo "  arm-none-eabi-gcc, arm-none-eabi-g++, arm-none-eabi-objcopy"
	@echo "  cmake >= 3.16"
	@echo "  hackrf_spiflash (for flash-lpc target)"
	@echo ""
