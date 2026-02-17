# Project Sentinel

Custom firmware for the **HackRF One + PortaPack H4M** that replaces the stock Mayhem firmware with an intelligent, context-aware RF monitoring platform.

## What is this?

The [HackRF One](https://greatscottgadgets.com/hackrf/) is a 1 MHz - 6 GHz software-defined radio (SDR). The [PortaPack H4M](https://www.aliexpress.com/item/portapack-h4m.html) is an add-on board with an LCD, audio codec, buttons, and an ESP32-S3 — turning the HackRF into a standalone handheld device.

Most people run the [Mayhem firmware](https://github.com/portapack-mayhem/mayhem-firmware), which provides a collection of standalone radio tools. Sentinel is different: it runs **FreeRTOS** on the M4 core with an **event bus** for inter-task communication, enabling background scanning, concurrent apps, and ESP32 co-processor integration.

## Hardware

| Component | Role |
|-----------|------|
| LPC4320 Cortex-M4 @ 204 MHz | Application core — FreeRTOS, UI, radio control |
| LPC4320 Cortex-M0 @ 204 MHz | Baseband DSP — IQ capture via SGPIO from CPLD |
| MAX5864 / MAX2837 / RFFC5072 | ADC/DAC, RF transceiver, mixer/synthesizer |
| XC2C64A CPLD | Routes 8-bit parallel IQ data to SGPIO bus |
| ESP32-S3 (on H4M) | Co-processor — Wi-Fi, GPS, sensors, MQTT bridge |
| 320x240 ILI9341 LCD | Touchscreen display (via EPM240 CPLD on H4M) |

## Architecture

```
┌─────────────────────────────────────────────────┐
│                 SPI Flash (1 MB)                 │
│   sentinel_m4.bin (M4 firmware + embedded M0)    │
└──────────────────────┬──────────────────────────┘
                       │ boots
        ┌──────────────┴──────────────┐
        │        Cortex-M4            │
        │    FreeRTOS + Event Bus     │
        │                             │
        │  Tasks:                     │
        │   - App (UI + user input)   │
        │   - Radio Manager           │
        │   - Background Scanner      │
        │   - ESP32 Comms (I2C/SPI)   │
        │   - Knowledge Base (SD)     │
        │   - Logger (UART console)   │
        └──────────────┬──────────────┘
                       │ IPC (shared SRAM)
        ┌──────────────┴──────────────┐
        │        Cortex-M0            │
        │   Bare-metal baseband DSP   │
        │                             │
        │  SGPIO ISR → ping-pong buf  │
        │  FM demod / signal detect   │
        └─────────────────────────────┘
```

## IQ Data Path

```
MAX5864 ADC → XC2C64A CPLD → SGPIO parallel bus (8-bit, ext clock)
    → M0 exchange ISR → ping-pong buffers in AHB SRAM
    → M4 DSP tasks read via IPC
```

## Memory Map

| Region | Address | Size | Used by |
|--------|---------|------|---------|
| SPI Flash | `0x00000000` | 1 MB | Firmware (XIP via SPIFI) |
| M4 SRAM0 | `0x10000000` | 128 KB | .data, .bss, stack, FreeRTOS heap |
| M0 SRAM | `0x10080000` | 32 KB | M0 baseband binary (copied at boot) |
| Shared IPC | `0x10088000` | 8 KB | Inter-core communication |
| AHB SRAM lo | `0x20000000` | 16 KB | M0 ping-pong DMA buffers |
| AHB SRAM hi | `0x20004000` | 16 KB | M4 heap / static arrays |

## Project Structure

```
mayh4m/
├── firmware/
│   ├── sentinel/          # M4 application firmware (FreeRTOS)
│   │   ├── hal/           #   Hardware abstraction (clocks, UART, SGPIO, GPIO, I2C, SSP)
│   │   ├── bsp/           #   Board support (pin definitions for H4M)
│   │   ├── tasks/         #   FreeRTOS tasks (app, radio, scanner, esp32, logger, etc.)
│   │   ├── event_bus/     #   Inter-task publish/subscribe event system
│   │   ├── ipc/           #   M4 <-> M0 shared memory protocol
│   │   ├── freertos/      #   FreeRTOS kernel + port
│   │   ├── main.cpp       #   M4 entry point and boot sequence
│   │   └── sentinel_m4.ld #   M4 linker script
│   ├── baseband/          # M0 baseband DSP firmware (bare-metal)
│   │   ├── dsp/           #   DSP routines (FM demod, filters)
│   │   ├── main_m0.cpp    #   M0 entry point + SGPIO ISR
│   │   └── m0.ld          #   M0 linker script
│   └── third_party/       # CMSIS-5, CMSIS-DSP
├── esp32-bridge/          # ESP32-S3 co-processor firmware (ESP-IDF)
├── sdcard/                # SD card filesystem template
├── docs/                  # Documentation
│   └── flashing.md        #   Flashing guide (all platforms)
├── cmake/                 # CMake toolchain files
├── Makefile               # Top-level build wrapper
├── SPEC.md                # MVP specification
└── PLAN.md                # Implementation plan
```

## Building

Requires `arm-none-eabi-gcc`, `arm-none-eabi-g++`, `arm-none-eabi-objcopy`, and `cmake >= 3.16`.

```bash
make              # Build everything (M0 baseband + M4 sentinel)
make sentinel     # Build M4 only (auto-builds M0 first)
make baseband     # Build M0 only
make clean        # Remove build directory
```

Build output:
- `build/sentinel/sentinel_m4.bin` — Flash this (includes embedded M0 binary)
- `build/baseband/baseband.bin` — M0 binary (for reference; already embedded above)

## Flashing

The firmware is written to the HackRF's SPI NOR flash. Both the HackRF and PortaPack run from this single chip.

```bash
# Enter DFU mode: hold DFU button while plugging in USB
make flash-lpc

# Or manually:
hackrf_spiflash -w build/sentinel/sentinel_m4.bin

# Then power-cycle the device
```

**Windows users**: Install the WinUSB driver via [Zadig](https://zadig.akeo.ie/) first.

See [docs/flashing.md](docs/flashing.md) for detailed instructions (Linux, macOS, Windows), DFU-to-RAM for development, and JTAG/SWD debugging.

### Restoring Mayhem / Stock Firmware

```bash
hackrf_spiflash -w hackrf_one_usb.bin       # Stock HackRF firmware
hackrf_spiflash -w mayhem_vX.X.X_firmware.bin  # Mayhem firmware
```

## Serial Debug Console

Connect a USB-UART adapter to USART0 (P2_0 TX, P2_1 RX) at **115200 8N1**.

```
[SENTINEL] Boot
[SENTINEL] Core clock: 204000000 Hz
[SENTINEL] Configuring SGPIO pins...
[SENTINEL] Tasks created. Starting scheduler...
```

Interactive commands: `status`, `sgpio`, `mem`, `help`

## Current Status

**v0.1.0** — Foundation release:
- FreeRTOS running on M4 with 7 concurrent tasks
- SGPIO IQ data path from CPLD to M0 via exchange interrupts
- Event bus for inter-task communication
- Serial debug console with command parser
- HAL drivers: clocks (204 MHz PLL), UART, SGPIO, GPIO, I2C, SSP

**Not yet implemented:**
- ESP32-S3 co-processor communication
- FM audio demodulation (DSP pipeline stubbed)
- SD card filesystem
- LCD display driver
- Touch/encoder input (H4M EPM240 CPLD driver needed)

## License

[MIT](LICENSE)
