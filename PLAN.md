# Project Sentinel - Implementation Plan

**Based on:** SPEC.md v0.1
**Firmware reference:** `/home/dan/projects/mayhem-firmware/` (Mayhem v2.x)
**Date:** February 2026

---

## Architectural Corrections vs SPEC.md

Before implementation begins, two critical corrections to SPEC.md section 5.3.1:

### 1. Core Roles Are Reversed in SPEC

The SPEC states "M0 core continues to run baseband DSP. The M4 core runs the RTOS kernel." This is backwards. Actual Mayhem architecture:

| Core | Actual Role | SPEC.md (incorrect) |
|------|-------------|---------------------|
| **Cortex-M4** | Application / UI / Control | "RTOS kernel + app tasks" |
| **Cortex-M0** | Baseband DSP (real-time) | "Baseband DSP" |

The M4 has the FPU, runs application code and the event loop. The M0 handles real-time sample processing. Sentinel will follow the same division: **FreeRTOS on M4, M0 baseband untouched initially.**

### 2. Mayhem Already Has an RTOS

SPEC calls the Mayhem scheduler "cooperative." It is not — it uses **ChibiOS/RT** (a preemptive RTOS) on both cores. The work is replacing ChibiOS on the M4 application core with FreeRTOS, not adding an RTOS to a bare-metal system.

---

## Repository Structure

Fork Mayhem, keep upstream separation clear:

```
sentinel-firmware/
├── firmware/
│   ├── sentinel/              # NEW: All Sentinel additions
│   │   ├── freertos/          # FreeRTOS kernel + LPC43xx M4 port
│   │   ├── event_bus/         # Event bus implementation
│   │   ├── tasks/             # FreeRTOS task implementations
│   │   │   ├── task_radio_manager.cpp
│   │   │   ├── task_bg_scanner.cpp
│   │   │   ├── task_knowledge_base.cpp
│   │   │   ├── task_esp32_comms.cpp
│   │   │   └── task_ui_renderer.cpp
│   │   └── apps/              # Sentinel apps (ported from Mayhem)
│   ├── application/           # Mayhem application (M4) - minimally modified
│   ├── baseband/              # Mayhem baseband (M0) - untouched Phase 0-2
│   └── common/                # Shared M4/M0 headers
├── esp32-bridge/              # Sentinel Bridge (ESP-IDF project)
│   ├── main/
│   │   ├── tasks/
│   │   │   ├── gps_task.c
│   │   │   ├── mqtt_task.c
│   │   │   ├── iq_stream_task.c
│   │   │   └── web_task.c
│   │   └── protocols/
│   │       ├── i2c_slave.c
│   │       └── spi_slave.c
└── sdcard/
    └── SENTINEL/              # Default SD card content
```

**Key principle:** All Mayhem source files in `firmware/application/` and `firmware/baseband/` are modified as little as possible. New functionality lives in `firmware/sentinel/`. This keeps diffs small and upstream merges tractable.

---

## Phase 0: Toolchain and Hardware Verification

**Goal:** Working build environment. Physical hardware confirmed functional. GPIO pinout documented.

### 0.1 Build Mayhem from Source

```bash
git clone https://github.com/portapack-mayhem/mayhem-firmware sentinel-firmware
cd sentinel-firmware
git remote rename origin upstream
git checkout -b sentinel-dev

# Verify build
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=firmware/toolchain-arm-cortex-m.cmake
make -j$(nproc)
# Produces: portapack-mayhem-firmware.bin
```

**Toolchain requirement:** `arm-none-eabi-gcc` 9.2.1+ (newer versions tested against upstream CI).

Flash and verify stock Mayhem functionality before making any changes.

### 0.2 GPIO Header Probe

The H4M GPIO header exposes LPC4320 pins. Key pins to identify:

| Signal | LPC4320 Function | Method |
|--------|------------------|--------|
| I2C SDA | I2C0 or I2C1 | Probe with multimeter while running I2C scan |
| I2C SCL | I2C0 or I2C1 | As above |
| SPI CLK | SSP0 or SSP1 | Check LPC4320 SCU register SFSP*_* for SSP mapping |
| SPI MOSI | SSP0_MOSI | Verify via datasheet pin alternate function table |
| SPI MISO | SSP0_MISO | As above |
| SPI CS | GPIO | Any available GPIO |
| GND | - | Continuity to device ground |
| 3.3V | - | Verify voltage with DMM |

**Reference files:**
- `firmware/common/hackrf_gpio.hpp` - Existing GPIO definitions
- NXP UM10503 LPC43xx User Manual, Chapter 7 (SCU register map)

Existing `i2c_pp.hpp` wraps ChibiOS I2C driver at I2C bus 0 (used for Si5351 clock gen, sensors). The ESP32 I2C connection must use a free bus (I2C1) or we multiplex onto I2C0.

**SPI availability check:** The existing radio SPI uses SSP1 (MAX283x at 20MHz). SSP0 may be available on the GPIO header — verify by checking `firmware/application/radio.cpp` SSP configuration and cross-referencing which LPC43xx SSP pins are physically exposed on the header.

**Exit criteria:** GPIO pinout diagram complete. I2C bus and SPI peripheral identified. Stock Mayhem flashed and verified FM audio works.

### 0.3 ESP32 Initial Wiring

Minimum initial wiring (I2C only):

```
H4M GPIO Header    ESP32-S3 Dev Board
─────────────────  ──────────────────
3.3V               3.3V
GND                GND
I2C_SDA (3.3V)     GPIO8 (with 4.7kΩ pullup to 3.3V)
I2C_SCL (3.3V)     GPIO9 (with 4.7kΩ pullup to 3.3V)
```

Flash stock ESP32PP firmware and verify I2C communication works using the existing sensor code in Mayhem (`firmware/application/apps/ert_rx_app.hpp` or similar I2C sensor app).

---

## Phase 1: FreeRTOS Foundation

**Goal:** FreeRTOS running on M4. FM audio works identically to stock Mayhem.

### 1.1 FreeRTOS Integration

Add FreeRTOS as a git subtree (not submodule, for build simplicity):

```bash
git subtree add --prefix firmware/sentinel/freertos \
    https://github.com/FreeRTOS/FreeRTOS-Kernel.git \
    main --squash
```

**Port selection:** Use `GCC/ARM_CM4F` portable layer — this is the standard Cortex-M4 with FPU port. Already works for LPC43xx.

**FreeRTOS configuration** (`firmware/sentinel/freertos/FreeRTOSConfig.h`):

```c
#define configCPU_CLOCK_HZ              204000000UL
#define configTICK_RATE_HZ              1000        // 1ms tick
#define configMAX_PRIORITIES            8
#define configMINIMAL_STACK_SIZE        256         // 1KB (in words)
#define configTOTAL_HEAP_SIZE           (40 * 1024) // 40KB heap
#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK             1
#define configUSE_TICK_HOOK             0
#define configUSE_MUTEXES               1
#define configUSE_COUNTING_SEMAPHORES   1
#define configUSE_TASK_NOTIFICATIONS    1
#define configUSE_TIMERS                1
#define configSUPPORT_STATIC_ALLOCATION 1           // Static alloc for tasks
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configUSE_TRACE_FACILITY        0           // Disable for production
```

**Memory allocation:** Use `heap_4.c` (coalescing allocator). 40KB of the "App working memory" pool from the SPEC.

### 1.2 ChibiOS Removal Strategy

The M4 application currently initialises ChibiOS in `firmware/application/main.cpp`:
- `halInit()` - HAL drivers (keep these, they init clocks, SPI, I2C)
- `chSysInit()` - ChibiOS kernel (replace with FreeRTOS)
- `chThdSleepMilliseconds()` and event loops (replace with FreeRTOS equivalents)

**Strategy:** Rather than a big-bang replacement, use a shim approach:

1. Create `firmware/sentinel/freertos/chibios_shim.hpp` that maps common ChibiOS calls to FreeRTOS equivalents for transitional compatibility
2. Replace ChibiOS kernel init in `main.cpp` with FreeRTOS init
3. Convert the existing event loop (`portapack::events_process()`) into the first FreeRTOS task
4. Remove ChibiOS from the build system once all callers are ported

**Key ChibiOS → FreeRTOS mappings:**

| ChibiOS | FreeRTOS Equivalent |
|---------|---------------------|
| `chSysInit()` | `vTaskStartScheduler()` |
| `chThdCreateStatic(wa, size, prio, func, arg)` | `xTaskCreateStatic(func, name, size/4, arg, prio, stack, tcb)` |
| `chThdSleepMilliseconds(n)` | `vTaskDelay(pdMS_TO_TICKS(n))` |
| `chMBPost(&mb, msg, TIME_INFINITE)` | `xQueueSend(queue, &msg, portMAX_DELAY)` |
| `chEvtWaitAny(mask)` | `xTaskNotifyWait(0, mask, &bits, portMAX_DELAY)` |
| `chSysLock()` / `chSysUnlock()` | `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` |
| `chVTSet(&vt, ticks, cb, arg)` | `xTimerCreate(name, ticks, pdFALSE, arg, cb)` |

### 1.3 Initial Task Set (Phase 1)

Keep it minimal — only two tasks to validate the foundation:

```c
// firmware/sentinel/tasks/task_app.cpp
// Wraps the existing Mayhem UI event loop as a FreeRTOS task
void task_app(void* pvParameters) {
    // Initialize portapack hardware (was in main())
    portapack::init();

    for (;;) {
        // Replaces portapack::events_process() busy loop
        // with FreeRTOS notification-driven dispatch
        uint32_t notification;
        xTaskNotifyWait(0, UINT32_MAX, &notification, pdMS_TO_TICKS(10));
        portapack::events_dispatch(notification);
    }
}

// firmware/sentinel/tasks/task_idle.cpp
// FreeRTOS idle hook - replaces ChibiOS idle thread
void vApplicationIdleHook(void) {
    __WFI(); // Wait for interrupt (power saving)
}
```

### 1.4 Serial Debug Logging

The USB connection presents as a CDC serial device on PC side. Add a logging task:

```c
// firmware/sentinel/tasks/task_logger.cpp
// Wraps USB CDC write into a thread-safe log queue
void log(const char* task, const char* fmt, ...);
// Outputs: "[1234ms][RADIO_MGR] Frequency set to 100.0MHz\n"
```

Use a 512-byte ring buffer + FreeRTOS queue to avoid blocking callers.

### 1.5 M0 Baseband: Untouched

The M0 baseband code is left completely unchanged in Phase 1. The existing `portapack_shared_memory.hpp` IPC between M4 and M0 continues to work identically — the M4 side now drives it from a FreeRTOS task instead of a ChibiOS thread, which is transparent to the M0.

**Exit criteria:** FM audio plays. `minicom /dev/ttyACM0 115200` shows task startup log. Two FreeRTOS tasks (App + Logger) confirmed running via RTOS stats dump.

---

## Phase 2: Event Bus and Background Scanner

**Goal:** Event bus operational. Background scanner running. Home dashboard shows live events.

### 2.1 Event Bus Implementation

**File:** `firmware/sentinel/event_bus/event_bus.hpp` / `.cpp`

The event bus is a typed, publish-subscribe system on top of FreeRTOS queues.

```cpp
// 32-byte message (from SPEC Appendix B)
struct __attribute__((packed)) BusEvent {
    uint8_t  type;          // EventType enum
    uint32_t timestamp_ms;  // xTaskGetTickCount() * portTICK_PERIOD_MS
    uint8_t  source;        // TaskID enum
    uint8_t  payload[26];   // Type-specific data
};
static_assert(sizeof(BusEvent) == 32, "BusEvent must be 32 bytes");

// Event type IDs (from SPEC Appendix B)
enum class EventType : uint8_t {
    SIGNAL_DETECTED   = 0x01,
    GPS_UPDATE        = 0x02,
    SENSOR_DATA       = 0x03,
    APP_STATE         = 0x04,
    RADIO_REQUEST     = 0x05,
    RADIO_RELEASE     = 0x06,
    AIRCRAFT_POSITION = 0x07,
    CAPTURE_START     = 0x08,
    CAPTURE_STOP      = 0x09,
    SYSTEM_STATUS     = 0x0A,
};

class EventBus {
public:
    static EventBus& instance();

    // Subscribe: register a queue to receive events of given types
    // Returns subscriber ID or -1 on failure
    int subscribe(QueueHandle_t queue, uint32_t event_type_mask);
    void unsubscribe(int subscriber_id);

    // Publish: routes event to all subscribed queues
    // Called from any task or ISR (ISR-safe version provided)
    void publish(const BusEvent& event);
    void publish_from_isr(const BusEvent& event, BaseType_t* pxHigherPriorityTaskWoken);

private:
    static constexpr int MAX_SUBSCRIBERS = 8;
    static constexpr int BUS_QUEUE_DEPTH = 64;

    struct Subscriber {
        QueueHandle_t queue;
        uint32_t      mask;
        bool          active;
    };

    Subscriber subscribers[MAX_SUBSCRIBERS]{};
    QueueHandle_t internal_queue;   // Single master queue (fanout on publish)
    SemaphoreHandle_t lock;         // Protects subscriber table
};
```

**Memory cost:** 64 × 32 bytes = 2048 bytes for master queue + subscriber table overhead ≈ 2.1KB total. Within spec.

### 2.2 Radio Manager Task

**File:** `firmware/sentinel/tasks/task_radio_manager.cpp`

The Radio Manager arbitrates access to the M0 baseband between the foreground app and background scanner. It uses the existing `ReceiverModel` / `baseband_api` interface.

```cpp
// Priority: high (configMAX_PRIORITIES - 2)
void task_radio_manager(void* pvParameters) {
    QueueHandle_t event_queue = xQueueCreate(8, sizeof(BusEvent));
    EventBus::instance().subscribe(event_queue,
        EVENT_MASK(EventType::RADIO_REQUEST) |
        EVENT_MASK(EventType::RADIO_RELEASE));

    TaskID current_owner = TaskID::NONE;

    for (;;) {
        BusEvent event;
        xQueueReceive(event_queue, &event, portMAX_DELAY);

        if (event.type == EventType::RADIO_REQUEST) {
            auto* req = reinterpret_cast<RadioRequestPayload*>(event.payload);
            if (current_owner == TaskID::NONE) {
                grant_radio(req->requesting_task, req->freq, req->bandwidth, req->mode);
                current_owner = static_cast<TaskID>(event.source);
            } else if (is_lower_priority(event.source, current_owner)) {
                suspend_task(event.source); // Queue the request
            } else {
                preempt_radio(current_owner, event.source);
                current_owner = static_cast<TaskID>(event.source);
            }
        } else if (event.type == EventType::RADIO_RELEASE) {
            current_owner = TaskID::NONE;
            // Resume any pending lower-priority request
            resume_suspended_request();
        }
    }
}
```

### 2.3 Background Scanner Task

**File:** `firmware/sentinel/tasks/task_bg_scanner.cpp`

Reads `/SENTINEL/scan_config.json` at startup. Sweeps frequency ranges when radio is available.

```cpp
// Priority: medium (configMAX_PRIORITIES / 2)
void task_bg_scanner(void* pvParameters) {
    ScanConfig config = load_scan_config("/SENTINEL/scan_config.json");

    for (;;) {
        // Request radio access
        publish_radio_request(TaskID::BG_SCANNER, config.current_freq,
                              config.bandwidth, RadioMode::ENERGY_DETECT);

        // Wait for grant (or timeout = radio busy, back off)
        uint32_t granted;
        if (!xTaskNotifyWait(0, UINT32_MAX, &granted, pdMS_TO_TICKS(500))) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Back off
            continue;
        }

        // Tune and measure energy
        set_frequency(config.current_freq);
        vTaskDelay(pdMS_TO_TICKS(config.dwell_ms));
        int16_t rssi = read_rssi(); // via shared_memory stats

        if (rssi > config.threshold_dbm) {
            BusEvent event = make_signal_detected_event(
                config.current_freq, rssi, current_gps_position());
            EventBus::instance().publish(event);
        }

        // Release and step to next frequency
        publish_radio_release(TaskID::BG_SCANNER);
        config.advance_frequency();
        vTaskDelay(pdMS_TO_TICKS(5)); // Yield
    }
}
```

**Default scan_config.json:**
```json
{
  "ranges": [
    { "start_hz": 430000000, "end_hz": 440000000, "step_hz": 25000 },
    { "start_hz": 863000000, "end_hz": 870000000, "step_hz": 25000 }
  ],
  "dwell_ms": 50,
  "threshold_dbm": -80
}
```

### 2.4 Knowledge Base Writer Task

**File:** `firmware/sentinel/tasks/task_knowledge_base.cpp`

```cpp
// Priority: low (1, just above idle)
// 16-byte binary record (from SPEC section 6.3)
struct __attribute__((packed)) ObservationRecord {
    uint32_t timestamp;     // seconds since epoch
    uint32_t frequency_hz;
    int16_t  rssi_dbm;
    int32_t  gps_lat;       // × 1e6 fixed point, packed as 24-bit
    // ... see SPEC section 6.3 for full layout
};

void task_knowledge_base(void* pvParameters) {
    QueueHandle_t event_queue = xQueueCreate(32, sizeof(BusEvent));
    EventBus::instance().subscribe(event_queue, EVENT_MASK(EventType::SIGNAL_DETECTED));

    ObservationRecord ring_buffer[256]; // 4KB
    uint32_t write_idx = 0;
    uint32_t last_flush_tick = 0;

    for (;;) {
        BusEvent event;
        // Wait up to 10s for events; flush periodically regardless
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(10000))) {
            ring_buffer[write_idx++ % 256] = extract_observation(event);
        }

        uint32_t now = xTaskGetTickCount();
        bool flush_needed = (write_idx % 192 == 0) || // 75% of 256
                            (now - last_flush_tick > pdMS_TO_TICKS(60000));

        if (flush_needed && write_idx > 0) {
            flush_to_sdcard(ring_buffer, write_idx);
            last_flush_tick = now;
        }
    }
}
```

### 2.5 Home Dashboard App

**File:** `firmware/sentinel/apps/app_home_dashboard.cpp`

Replace `firmware/application/apps/ui_menu.hpp` main menu with the Sentinel dashboard. The dashboard runs as part of the UI task (not a separate task — UI must stay single-threaded for display safety).

Dashboard layout (320×240):
```
┌─────────────────────────────────────────┐  ← Status bar (20px)
│ ▓ 87%  GPS: FIX  WiFi: OK  12:34:56    │
├─────────────────────────────────────────┤
│                                         │
│   [Waterfall: 128-bin mini spectrum]    │  ← 80px
│                                         │
├─────────────────────────────────────────┤
│ Activity Feed:                          │  ← Event log (5 lines)
│ 12:34:51  433.920MHz  -73dBm  DETECTED  │
│ 12:34:48  869.000MHz  -81dBm  DETECTED  │
│ 12:34:41  GPS  51.5074°N 0.1278°W      │
│ 12:34:35  BME280  21.4°C  56% RH       │
│ 12:34:28  433.920MHz  -71dBm  DETECTED  │
├─────────────────────────────────────────┤
│ [Spectrum] [FM Radio] [ADS-B] [Scanner] │  ← Quick launch (40px)
└─────────────────────────────────────────┘
```

The dashboard subscribes to the event bus and updates at 4Hz via a FreeRTOS timer callback that sends a notification to the UI task.

**Exit criteria:** Dashboard shows live signal detections. Background scanner pauses when FM app opens (via Radio Manager), resumes on exit. Observations log written to `/SENTINEL/knowledge_base/YYYY-MM-DD.bin`.

---

## Phase 3: ESP32-S3 Co-processor (Sentinel Bridge)

**Goal:** GPS on display. IQ streaming to homelab. MQTT publishing signal events.

### 3.1 Sentinel Bridge Firmware Architecture

**Project:** `esp32-bridge/` (ESP-IDF 5.x with FreeRTOS native)

```
main/
├── main.c                  # App entry point, task creation
├── sentinel_config.h       # Pins, MQTT topic strings, timing constants
├── tasks/
│   ├── task_gps.c          # UART→NMEA parse→GPS_UPDATE publish via I2C
│   ├── task_bme280.c       # I2C→BME280→SENSOR_DATA publish
│   ├── task_i2c_slave.c    # I2C slave: responds to LPC4320 commands
│   ├── task_mqtt.c         # Wi-Fi + MQTT publish
│   ├── task_iq_stream.c    # SPI slave→UDP forward
│   └── task_webserver.c    # HTTP status page
└── protocols/
    ├── sentinel_i2c.h      # I2C command protocol (from SPEC Appendix D)
    └── sentinel_events.h   # Shared event struct definition
```

**ESP32 pin assignments:**
```c
// i2c_slave.c (receives commands from LPC4320)
#define I2C_SLAVE_SDA_PIN   GPIO_NUM_8
#define I2C_SLAVE_SCL_PIN   GPIO_NUM_9
#define I2C_SLAVE_ADDR      0x42

// GPS (NEO-6M via UART)
#define GPS_UART_NUM        UART_NUM_1
#define GPS_TX_PIN          GPIO_NUM_17
#define GPS_RX_PIN          GPIO_NUM_18

// BME280 (I2C master, separate bus from I2C slave)
#define BME280_I2C_PORT     I2C_NUM_1
#define BME280_SDA_PIN      GPIO_NUM_21
#define BME280_SCL_PIN      GPIO_NUM_22

// SPI slave (IQ streaming from LPC4320)
#define SPI_MOSI_PIN        GPIO_NUM_11
#define SPI_MISO_PIN        GPIO_NUM_13
#define SPI_CLK_PIN         GPIO_NUM_12
#define SPI_CS_PIN          GPIO_NUM_10
```

### 3.2 I2C Command Protocol

Implement SPEC Appendix D as the LPC4320-side driver:

**File:** `firmware/sentinel/tasks/task_esp32_comms.cpp`

```cpp
// Priority: medium
void task_esp32_comms(void* pvParameters) {
    QueueHandle_t event_queue = xQueueCreate(16, sizeof(BusEvent));
    EventBus::instance().subscribe(event_queue,
        EVENT_MASK(EventType::SYSTEM_STATUS) |
        EVENT_MASK(EventType::SIGNAL_DETECTED));

    TickType_t gps_poll_ticks = 0;
    TickType_t sensor_poll_ticks = 0;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        // Poll GPS at 1Hz
        if (now - gps_poll_ticks >= pdMS_TO_TICKS(1000)) {
            auto gps = esp32_i2c_get_gps();   // CMD 0x01
            if (gps.fix_quality > 0) {
                BusEvent ev = make_gps_update_event(gps);
                EventBus::instance().publish(ev);
            }
            gps_poll_ticks = now;
        }

        // Poll BME280 at 0.1Hz
        if (now - sensor_poll_ticks >= pdMS_TO_TICKS(10000)) {
            auto sensors = esp32_i2c_get_sensors(); // CMD 0x02
            BusEvent ev = make_sensor_data_event(sensors);
            EventBus::instance().publish(ev);
            sensor_poll_ticks = now;
        }

        // Forward events to ESP32 for MQTT publishing
        BusEvent event;
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100))) {
            esp32_i2c_publish_event(event); // CMD 0x05
        }
    }
}
```

### 3.3 IQ Sample Streaming

**M0 side tap:** Add a new baseband message type to request IQ streaming output:

```cpp
// firmware/common/message.hpp - add new message
struct IQStreamConfig {
    uint32_t decimation_factor; // 1, 2, 4, 8 (target sample rate)
    uint32_t target_ip;
    uint16_t target_port;
};

struct IQStreamMessage : public Message {
    static constexpr ID id = ID::IQStreamConfig;
    IQStreamConfig config;
};
```

The M0 baseband processor accumulates a decimated IQ buffer and signals M4 via `app_local_queue` when a buffer is ready. The M4 `task_esp32_comms` drains this buffer and sends it over SPI to the ESP32.

**SPI driver:** New file `firmware/sentinel/drivers/spi_esp32.cpp` — wraps existing SSP0/SSP1 driver for DMA-based TX to ESP32.

**If SPI not available on GPIO header:** Fall back to UART at 3.5Mbaud (hardware UART via `firmware/common/uart.hpp`). Achieves ~2Mbps effective throughput — sufficient for decimated narrowband streaming.

### 3.4 MQTT Topic Map

ESP32 publishes to:
- `sentinel/{device_id}/signal_detected` — `{"freq":433920000,"rssi":-73,"lat":51.507,"lon":-0.127,"ts":1708123456}`
- `sentinel/{device_id}/gps` — `{"lat":51.507,"lon":-0.127,"alt":25,"sats":8,"fix":1}`
- `sentinel/{device_id}/sensors` — `{"temp_c":21.4,"humidity_pct":56,"pressure_hpa":1013}`
- `sentinel/{device_id}/status` — `{"uptime_s":3600,"battery_pct":87,"sd_free_mb":28000}`

### 3.5 Web Status Page

Served by `esp_http_server` on ESP32. Single HTML page with auto-refresh every 10s. Displays GPS position (embedded Leaflet.js map if CDN available, static coordinates otherwise), last 5 signal detections, and sensor readings.

**Exit criteria:** GPS coordinates visible on PortaPack dashboard. IQ samples decoded by GNU Radio on homelab (`osmocom_source` → `file_sink`, verify carrier at configured frequency). Signal detections appear on MQTT via `mosquitto_sub -t 'sentinel/#'`. Web page accessible at ESP32 IP.

---

## Phase 4: App Porting and Integration

**Goal:** All five core apps running as FreeRTOS-integrated RTOS tasks with full event bus participation.

### 4.1 App-to-Task Migration Pattern

Each ported Mayhem app follows this pattern:

1. Keep existing UI view class (extends `ui::View`) unchanged
2. Wrap the app in a FreeRTOS task that manages its lifecycle:
   - Publishes `APP_STATE` event on launch (so Radio Manager knows)
   - Subscribes to relevant bus events on init
   - Sends `RADIO_REQUEST` before tuning
   - Sends `RADIO_RELEASE` on exit
3. Add event bus publications for app-specific events

**Example — Spectrum Analyser integration:**

```cpp
// firmware/sentinel/apps/app_spectrum.cpp

// Existing: proc_wideband_spectrum (M0) → UpdateSpectrum message → SpectrumView (M4)
// Add: SpectrumView peaks → SIGNAL_DETECTED event on bus
//      Dashboard SIGNAL_DETECTED → auto-tune Spectrum to detected frequency

class SentinelSpectrumApp : public SpectrumAnalysisView {
    QueueHandle_t bus_queue;

    void on_message(const Message* const message) override {
        SpectrumAnalysisView::on_message(message); // Existing rendering

        if (message->id == Message::ID::UpdateSpectrum) {
            auto* update = static_cast<const UpdateSpectrumMessage*>(message);
            // Publish peak above threshold as signal event
            if (update->peak_db > DETECTION_THRESHOLD) {
                EventBus::instance().publish(make_signal_detected_event(
                    update->peak_frequency_hz, update->peak_db, cached_gps));
            }
        }
    }

    // Subscribe to SIGNAL_DETECTED to auto-tune
    void handle_bus_event(const BusEvent& ev) {
        if (ev.type == EventType::SIGNAL_DETECTED && auto_tune_enabled) {
            auto* payload = reinterpret_cast<const SignalDetectedPayload*>(ev.payload);
            set_target_frequency(payload->frequency_hz);
        }
    }
};
```

### 4.2 Apps to Port

| App | Key Files to Wrap | New Bus Events | Bus Subscriptions |
|-----|-------------------|----------------|-------------------|
| **Spectrum Analyser** | `spectrum_analysis_app.*` | SIGNAL_DETECTED (peaks) | SIGNAL_DETECTED (auto-tune) |
| **FM Audio** | `analog_audio_app.*` | APP_STATE | GPS_UPDATE (display coords) |
| **ADS-B** | `ads_b_rx_app.*` | AIRCRAFT_POSITION | GPS_UPDATE (own position) |
| **Scanner** | `scanner_app.*` | SIGNAL_DETECTED | — |
| **Signal Capture** | `capture_app.*` | CAPTURE_START, CAPTURE_STOP | GPS_UPDATE (geo-tag) |

### 4.3 ADS-B Own-Position Integration

ADS-B app displays aircraft positions relative to own position. Currently requires manual lat/lon entry. With GPS_UPDATE events:

```cpp
// In ADS-B receiver message handler
void on_gps_update(const GPSUpdatePayload& gps) {
    own_position = {gps.latitude, gps.longitude};
    map_view.set_own_position(own_position);
    map_view.set_dirty(); // Trigger repaint
}
```

### 4.4 Signal Capture GPS Tagging

Captures are auto-named with GPS coordinates when fix is available:

```
/SENTINEL/captures/capture_433920_20260217_143022_N51507W00127.iq
```

The existing `CaptureView` is extended to prepend an 8-byte GPS header to each capture file: `[lat_int32][lon_int32]`.

### 4.5 End-to-End Integration Test

Run 4-hour endurance test:
1. Background scanner active (5 frequency ranges, 50ms dwell)
2. User switches between FM, Spectrum, ADS-B apps
3. GPS logging continuously
4. MQTT publishing all events
5. Monitor via `mosquitto_sub`, verify no message gaps > 30s
6. Check SD card observation log size grows at expected rate
7. Connect via USB serial: verify no heap exhaustion, stack high-water marks stable

**Check:** FreeRTOS `vTaskList()` and `vTaskGetRunTimeStats()` via serial command. Ensure no task stack overflows (sentinel canary values).

**Exit criteria:** All five apps functional. MQTT continuous. Observation log on SD. 4-hour run without crash or audio glitch.

---

## Memory Budget (Revised)

Actual Mayhem M4 SRAM layout (from `memory_map.hpp`):

| Region | Start | Size | Owner |
|--------|-------|------|-------|
| Local SRAM 0 | 0x10000000 | 96KB | M4 code |
| Local SRAM 1 | 0x10080000 | 32KB | M4 code continuation |
| Shared region | 0x10088000 | 8KB | M4/M0 IPC |
| AHB SRAM 0 | 0x20000000 | 32KB | M4 data |
| AHB SRAM 1 | 0x20008000 | 16KB | M4 data |
| AHB SRAM 2 | 0x2000C000 | 16KB | M4 data |

**Total M4 data SRAM: ~64KB** (plus shared 8KB)

**Sentinel M4 data allocation:**

| Component | Size | Notes |
|-----------|------|-------|
| FreeRTOS kernel | 8KB | Kernel objects, timer queues |
| Task stacks × 7 | 28KB | 4KB per task (tunable down) |
| Event bus (64 msgs) | 2.1KB | Master queue + subscriber table |
| FreeRTOS heap | 14KB | Dynamic alloc pool (heap_4) |
| Display buffer | 0KB | Unchanged, in its own SRAM region |
| Baseband IPC (shared) | 8KB | Existing shared_memory, unchanged |
| Knowledge base ring buf | 4KB | 256 × 16-byte records |
| SPI TX DMA buffer | 4KB | IQ streaming to ESP32 |
| SD card driver | 2KB | FatFS work area |
| Logger ring buffer | 0.5KB | Serial debug |
| **Total** | **~70.6KB** | Fits within 72KB M4 data SRAM |

**Warning zones:**
- Task stacks are the biggest flex point. Use `uxTaskGetStackHighWaterMark()` to profile each task and trim aggressively.
- FreeRTOS heap is lean at 14KB. Monitor with `xPortGetFreeHeapSize()` in status task.

---

## Risk Register (Updated)

| Risk | Mitigation | Phase |
|------|-----------|-------|
| SPI not on GPIO header | UART fallback at 3.5Mbaud — narrowband IQ streaming still works | 0 |
| ChibiOS HAL drivers break under FreeRTOS scheduler | HAL init happens before scheduler starts; ChibiOS HAL is independent of ChibiOS kernel. Most drivers (SSP, I2C, DMA) will work. Test each peripheral after FreeRTOS init. | 1 |
| M0 baseband timing upset by M4 FreeRTOS preemption | M0 runs independently. M4 preemption doesn't affect M0 timing. The `portapack_shared_memory` IPC is lock-free; safe under any M4 scheduling. | 1 |
| FM audio regression | A/B test immediately after Phase 1. FM quality is the Phase 1 exit gate. Any regression = revert and debug before proceeding. | 1 |
| Memory exhaustion | Profile task stacks in Phase 2. Add `SYSTEM_STATUS` event that reports `xPortGetFreeHeapSize()`. Set low-memory alert at <4KB free. | 2 |
| SD card blocking tasks | File I/O is blocking. Knowledge base writer runs at lowest priority. Never call FatFS from high-priority tasks. Use the task queue to hand off writes. | 2 |
| I2C bus collision (LPC4320 I2C used for Si5351 clock gen) | Use separate I2C bus for ESP32 (I2C1), or add mutex on I2C0. Profile bus traffic — Si5351 reconfiguration is infrequent. | 3 |
| ESP32 SPI slave overrun | Start with I2C-only integration. Add SPI with DMA and explicit flow control. Implement backpressure: LPC4320 checks ESP32 buffer status before sending. | 3 |

---

## File-by-File Change Map

### Files Modified (Mayhem originals, minimal changes)

| File | Change | Reason |
|------|--------|--------|
| `firmware/application/main.cpp` | Replace `chSysInit()` → `xTaskCreate()` + `vTaskStartScheduler()` | Phase 1: RTOS swap |
| `firmware/application/portapack.hpp` | Extract `init()` from `main()` into callable function | Phase 1: Task-friendly init |
| `firmware/CMakeLists.txt` | Add FreeRTOS source files, remove ChibiOS kernel sources | Phase 1: Build |
| `firmware/common/message.hpp` | Add `IQStreamConfig`, `IQStreamData` message types | Phase 3: IQ streaming |

### Files Created (Sentinel additions only)

| File | Phase |
|------|-------|
| `firmware/sentinel/freertos/FreeRTOSConfig.h` | 1 |
| `firmware/sentinel/freertos/chibios_shim.hpp` | 1 |
| `firmware/sentinel/event_bus/event_bus.hpp/.cpp` | 2 |
| `firmware/sentinel/tasks/task_app.cpp` | 1 |
| `firmware/sentinel/tasks/task_logger.cpp` | 1 |
| `firmware/sentinel/tasks/task_radio_manager.cpp` | 2 |
| `firmware/sentinel/tasks/task_bg_scanner.cpp` | 2 |
| `firmware/sentinel/tasks/task_knowledge_base.cpp` | 2 |
| `firmware/sentinel/tasks/task_esp32_comms.cpp` | 3 |
| `firmware/sentinel/apps/app_home_dashboard.cpp` | 2 |
| `firmware/sentinel/apps/app_spectrum.cpp` | 4 |
| `firmware/sentinel/apps/app_fm.cpp` | 4 |
| `firmware/sentinel/apps/app_adsb.cpp` | 4 |
| `firmware/sentinel/apps/app_scanner.cpp` | 4 |
| `firmware/sentinel/apps/app_capture.cpp` | 4 |
| `esp32-bridge/main/main.c` | 3 |
| `esp32-bridge/main/tasks/task_gps.c` | 3 |
| `esp32-bridge/main/tasks/task_mqtt.c` | 3 |
| `esp32-bridge/main/tasks/task_iq_stream.c` | 3 |
| `esp32-bridge/main/tasks/task_webserver.c` | 3 |
| `esp32-bridge/main/tasks/task_i2c_slave.c` | 3 |

---

## Development Checkpoints

### After Phase 0
- [ ] `arm-none-eabi-gcc --version` shows 9.2.1+
- [ ] Mayhem builds clean from source (`make -j8` no errors)
- [ ] Device boots Mayhem, FM radio works
- [ ] GPIO header pinout documented with multimeter readings
- [ ] ESP32 responds to I2C probe (`i2cdetect` equivalent in Mayhem sensor app)

### After Phase 1
- [ ] Serial log shows `[RTOS] FreeRTOS scheduler started`
- [ ] Serial log shows task startup for App and Logger tasks
- [ ] FM audio quality A/B matched to stock Mayhem
- [ ] `vTaskList()` output readable via serial at any time

### After Phase 2
- [ ] Dashboard visible at boot (replaces stock Mayhem menu)
- [ ] Signal detections appear in dashboard activity feed
- [ ] Opening FM app pauses background scanner (no interference)
- [ ] SD card shows `/SENTINEL/knowledge_base/YYYY-MM-DD.bin` growing
- [ ] `vTaskGetRunTimeStats()` shows all 6 tasks running

### After Phase 3
- [ ] GPS coordinates update on dashboard every second
- [ ] `mosquitto_sub -v -t 'sentinel/#'` shows MQTT messages
- [ ] GNU Radio `osmocom_source → file_sink` captures IQ from LPC4320 stream
- [ ] ESP32 web page accessible from browser on LAN

### After Phase 4
- [ ] All 5 apps launch from dashboard quick-launch buttons
- [ ] ADS-B shows aircraft positions with own-position marker from GPS
- [ ] Signal captures in `/SENTINEL/captures/` include GPS in filename
- [ ] 4-hour run: no crashes, no audio glitches, MQTT continuous
- [ ] Stack high-water marks all > 20% of allocated stack size

---

## References

- Mayhem firmware source: `/home/dan/projects/mayhem-firmware/`
- Key IPC: `firmware/common/portapack_shared_memory.hpp`
- Key messages: `firmware/common/message.hpp`
- App pattern: `firmware/application/apps/analog_audio_app.hpp`
- Baseband pattern: `firmware/baseband/proc_nfm_audio.hpp`
- Radio control: `firmware/application/radio.cpp`
- I2C driver: `firmware/common/i2c_pp.hpp`
- Memory map: `firmware/common/memory_map.hpp`
- [FreeRTOS LPC43xx port](https://freertos.org/FreeRTOS-for-LPC4350-Cortex-M4F-and-Cortex-M0-Keil.html)
- [NXP LPC43xx dual-core examples](https://community.nxp.com/t5/LPCware-Archive-Content/LPC43XX-Dual-Core-Examples/ta-p/1123502)
- [Mayhem firmware wiki: Boot Process](https://github.com/portapack-mayhem/mayhem-firmware/wiki/Boot-Process)
- [Mayhem firmware wiki: Access Radio Hardware](https://github.com/portapack-mayhem/mayhem-firmware/wiki/Access-Radio-Hardware)
