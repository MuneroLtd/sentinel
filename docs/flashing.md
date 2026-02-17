# Flashing the Sentinel Firmware

## Build Output

After a successful build (`make` or `make sentinel`), the following binaries are produced:

| File | Description |
|------|-------------|
| `build/baseband/baseband.bin` | M0 baseband DSP firmware (~14 KB) |
| `build/sentinel/sentinel_m4.bin` | M4 application firmware (~122 KB, includes M0 binary embedded) |

Only `sentinel_m4.bin` needs to be flashed. It contains the M0 baseband binary embedded within it (in the `.m0_binary` section). At boot, the M4 startup code copies the M0 binary from flash to M0 SRAM (0x10080000) before releasing the M0 core.

## Why Not hackrf.app (Mayhem Hub)?

[hackrf.app](https://hackrf.app) is a browser-based tool for the **Mayhem firmware** specifically.
It uses the Web Serial API to talk to a PortaPack that is already running Mayhem firmware
in "normal mode." It sends firmware update commands over Mayhem's serial protocol and
downloads `.tar` packages from the Mayhem release API.

It **cannot** flash Sentinel firmware because:
- It requires Mayhem firmware already running on the device
- It only understands Mayhem's `.tar` firmware package format
- It doesn't do raw SPI flash writes

To flash Sentinel, use `hackrf_spiflash` (all platforms) or `dfu-util` as described below.

## Method 1: hackrf_spiflash (Recommended)

The HackRF One uses a Winbond W25Q80BV SPI NOR flash chip (1 MB) memory-mapped at 0x00000000 via the LPC4320's SPIFI peripheral. The `hackrf_spiflash` utility writes directly to this flash.

### Prerequisites

**Linux:**
```bash
# Debian/Ubuntu
sudo apt install hackrf

# Arch Linux
sudo pacman -S hackrf
```

**macOS:**
```bash
brew install hackrf
```

**Windows:**

1. Download the latest HackRF release from
   https://github.com/greatscottgadgets/hackrf/releases
   (the Windows zip contains `hackrf_spiflash.exe` and other tools).

2. Extract to a folder and add it to your PATH, or run from the extracted directory.

3. Install the USB driver with **Zadig** (required for Windows to talk to HackRF in DFU mode):
   - Download Zadig from https://zadig.akeo.ie/
   - Plug in HackRF while holding the DFU button
   - In Zadig, select **Options > List All Devices**
   - Select **HackRF One** (or "Unknown Device" with VID `1FC9`, PID `000C`)
   - Set the driver to **WinUSB** and click **Install Driver**
   - This only needs to be done once

**From source (any platform):**
```bash
git clone https://github.com/greatscottgadgets/hackrf.git
cd hackrf/host && mkdir build && cd build
cmake .. && make && sudo make install
```

### Steps

1. **Enter DFU mode**: Hold the **DFU button** (small button near USB connector) while plugging in the USB cable. The LEDs should show the DFU bootloader pattern (LED3 blinks).

2. **Flash the firmware**:

   Linux/macOS:
   ```bash
   make flash-lpc
   ```
   Or manually:
   ```bash
   hackrf_spiflash -w build/sentinel/sentinel_m4.bin
   ```

   Windows (from the directory containing `hackrf_spiflash.exe`):
   ```cmd
   hackrf_spiflash.exe -w path\to\sentinel_m4.bin
   ```

3. **Power-cycle the device**: Unplug and re-plug USB. The firmware boots from SPI flash immediately.

4. **Verify boot**: Connect a USB-UART adapter to USART0 (P2_0 TX / P2_1 RX) at 115200 8N1. You should see:
   ```
   [SENTINEL] Boot
   [SENTINEL] Core clock: 204000000 Hz
   [SENTINEL] Configuring SGPIO pins...
   [SENTINEL] Initialising SGPIO...
   [SENTINEL] Clearing IPC shared memory...
   [SENTINEL] Copying M0 firmware: XXXX bytes to 0x10080000
   [SENTINEL] Starting SGPIO capture...
   [SENTINEL] Releasing M0 core...
   [SENTINEL] M0 core started.
   [SENTINEL] EventBus ready.
   [SENTINEL] Tasks created. Starting scheduler...
   ```

### Restoring Stock HackRF Firmware

To revert to the official HackRF firmware:

```bash
hackrf_spiflash -w /path/to/hackrf_one_usb.bin
```

The official binary can be downloaded from the [HackRF releases page](https://github.com/greatscottgadgets/hackrf/releases).

## Method 2: DFU (RAM-only, for development)

DFU mode loads firmware into SRAM only -- it does not persist across power cycles. This is useful for rapid iteration during development.

### Prerequisites

**Linux/macOS:**
```bash
# Debian/Ubuntu
sudo apt install dfu-util

# macOS
brew install dfu-util
```

**Windows:**
Download `dfu-util` from https://dfu-util.sourceforge.net/releases/ (Windows binaries included).
The same Zadig WinUSB driver setup from Method 1 applies here.

### Steps

1. **Enter DFU mode** (same as above -- hold DFU button during USB plug-in).

2. **Upload via DFU**:
   ```bash
   dfu-util -d 1fc9:000c -a 0 -D build/sentinel/sentinel_m4.bin
   ```
   The LPC4320 DFU bootloader has USB VID:PID `1fc9:000c`.

   Windows:
   ```cmd
   dfu-util.exe -d 1fc9:000c -a 0 -D path\to\sentinel_m4.bin
   ```

3. **Firmware runs immediately from RAM**. No power-cycle needed, but firmware is lost on next power-cycle.

> **Note**: DFU to RAM has a size limit (~128 KB usable in SRAM0). The sentinel firmware should fit, but if it grows beyond this, use SPI flash instead.

## Method 3: JTAG/SWD (for debugging)

Use a JTAG/SWD probe (Black Magic Probe, J-Link, or ST-Link with OpenOCD) for interactive debugging.

### OpenOCD + GDB

1. **Connect SWD probe** to the HackRF JTAG header (1.27mm pitch, near the antenna connector). Pinout:
   - Pin 1: VCC (3.3V reference)
   - Pin 2: SWDIO
   - Pin 3: GND
   - Pin 4: SWCLK
   - Pin 5: nRESET

2. **Start OpenOCD**:
   ```bash
   openocd -f interface/cmsis-dap.cfg -f target/lpc4350.cfg
   ```
   (Substitute your probe's interface file, e.g. `jlink.cfg` or `stlink.cfg`. The LPC4320 uses the same OpenOCD target as LPC4350.)

3. **Connect GDB**:
   ```bash
   arm-none-eabi-gdb build/sentinel/sentinel_m4.elf
   (gdb) target remote :3333
   (gdb) monitor reset halt
   (gdb) load
   (gdb) continue
   ```

### Black Magic Probe

```bash
arm-none-eabi-gdb build/sentinel/sentinel_m4.elf
(gdb) target extended-remote /dev/ttyACM0
(gdb) monitor swdp_scan
(gdb) attach 1
(gdb) load
(gdb) run
```

## Serial Debug Console

Once booted, the firmware provides an interactive debug console over USART0.

### Connection

| Signal | Pin | PortaPack Header |
|--------|-----|-----------------|
| TX | P2_0 (USART0 TXD) | See schematic |
| RX | P2_1 (USART0 RXD) | See schematic |
| GND | GND | Any ground pad |

Settings: **115200 baud, 8N1, no flow control**

**Terminal programs:** PuTTY or TeraTerm (Windows), `screen /dev/ttyUSB0 115200` (Linux/macOS),
or any serial terminal. On Windows the adapter appears as a COM port in Device Manager.

### Available Commands

| Command | Description |
|---------|-------------|
| `status` | Task states, free heap, stack high-water marks |
| `sgpio` | SGPIO register dump (enabled slices, counters) |
| `mem` | Memory usage (SRAM0, AHBRAM, IPC region) |
| `help` | List available commands |

## Memory Map Reference

```
0x00000000 ┌──────────────────────┐
           │  SPI Flash (1 MB)    │  sentinel_m4.bin lives here
           │  .vectors            │  Cortex-M4 vector table
           │  .text / .rodata     │  Code + constants
           │  .m0_binary          │  Embedded M0 baseband binary
           │  .data (LMA)         │  Initialised data image
0x000FFFFF └──────────────────────┘

0x10000000 ┌──────────────────────┐
           │  M4 SRAM0 (128 KB)   │  .data, .bss, stack, FreeRTOS heap
0x1001FFFF └──────────────────────┘

0x10080000 ┌──────────────────────┐
           │  M0 SRAM (32 KB)     │  M0 baseband binary (copied at boot)
0x10087FFF └──────────────────────┘

0x10088000 ┌──────────────────────┐
           │  Shared IPC (8 KB)   │  Inter-core communication
0x10089FFF └──────────────────────┘

0x20000000 ┌──────────────────────┐
           │  AHB SRAM lo (16 KB) │  M0 DMA ping-pong buffers
0x20003FFF ├──────────────────────┤
           │  AHB SRAM hi (16 KB) │  M4 heap / static arrays
0x20007FFF └──────────────────────┘
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `hackrf_spiflash` says "No HackRF found" | Device not in DFU mode | Hold DFU button while plugging in USB |
| No serial output after flash | Wrong UART pins or baud rate | Verify P2_0/P2_1 connections, 115200 8N1 |
| Boot halts at "M0 firmware too large" | M0 binary exceeds 32 KB limit | Check baseband.bin size; reduce code |
| SGPIO counters not incrementing | CPLD not clocking SGPIO8 | Verify CPLD firmware is programmed; check CLK2 pin |
| `dfu-util` can't find device | Wrong VID:PID or permissions | Use `lsusb` to verify; try `sudo` or add udev rules |
| Windows: "No HackRF found" / "Access denied" | Wrong USB driver | Run Zadig, select the HackRF device, install WinUSB driver |
| Windows: device not listed in Zadig | DFU mode not entered | Unplug, hold DFU button, plug in again; check Device Manager for "Unknown Device" with VID 1FC9 |
