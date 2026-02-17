// SPDX-License-Identifier: MIT
// Project Sentinel — Background RF Energy Scanner Task (task_bg_scanner)
//
// Continuously sweeps a configurable frequency range, reporting signal
// detection events to the EventBus.  Pauses automatically when another
// task (e.g. an app) requests exclusive radio access.
//
// Configuration: /SENTINEL/config/scan_config.json on SD card.
// Parsed with strstr/sscanf — no external JSON library required.
//
// Defaults (used if file is absent or unparseable):
//   freq_start_hz : 430 000 000  (430 MHz)
//   freq_stop_hz  : 440 000 000  (440 MHz)
//   step_hz       :      25 000  (25 kHz)
//   dwell_ms      :          50
//   threshold_dbm :         -80

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ff.h"

#include "../event_bus/event_bus.hpp"
#include "../event_bus/event_types.hpp"
#include "../tasks/task_ids.hpp"
#include "ipc/ipc_protocol.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>

extern "C" void sentinel_log(const char* tag, const char* fmt, ...);

// ---------------------------------------------------------------------------
// Radio ownership flag
//
// g_radio_owned_by is the TaskID of the current radio owner.
// task_radio_manager.cpp publishes RADIO_REQUEST / RADIO_RELEASE events and
// notifies the requesting task.  The scanner watches those events on its own
// bus queue and maintains a local shadow flag.
// ---------------------------------------------------------------------------
namespace sentinel {
    // g_radio_available: true when BG_SCANNER may use the radio.
    // Written only from task_bg_scanner (protected by its own task context).
    // Declared extern so other translation units can read it if needed.
    bool g_radio_available = true;
} // namespace sentinel

// ---------------------------------------------------------------------------
// Scan configuration
// ---------------------------------------------------------------------------
namespace {

struct ScanConfig {
    uint32_t freq_start_hz  = 430000000u;
    uint32_t freq_stop_hz   = 440000000u;
    uint32_t step_hz        =     25000u;
    uint32_t dwell_ms       =        50u;
    int32_t  threshold_dbm  =       -80;
};

// ---------------------------------------------------------------------------
// parse_scan_config
//
// Opens /SENTINEL/config/scan_config.json from the FatFS-mounted SD card.
// Uses strstr + sscanf to extract the five fields — no JSON library needed.
// On any error the supplied default config is returned unchanged.
// ---------------------------------------------------------------------------
static ScanConfig parse_scan_config(const ScanConfig& defaults)
{
    ScanConfig cfg = defaults;

    FATFS fs;
    if (f_mount(&fs, "", 1) != FR_OK) {
        sentinel_log("BG_SCAN", "SD mount failed — using default scan config");
        return cfg;
    }

    FIL   file;
    FRESULT rc = f_open(&file, "/SENTINEL/config/scan_config.json", FA_READ);
    if (rc != FR_OK) {
        sentinel_log("BG_SCAN", "scan_config.json not found — using defaults");
        f_unmount("");
        return cfg;
    }

    // Read the whole file into a local buffer (max 512 bytes)
    static char buf[512];
    UINT bytes_read = 0;
    f_read(&file, buf, sizeof(buf) - 1u, &bytes_read);
    f_close(&file);
    f_unmount("");
    buf[bytes_read] = '\0';

    // Parse each field with strstr + sscanf
    const char* p;
    uint32_t u;
    int32_t  s;

    if ((p = strstr(buf, "\"freq_start_hz\"")) != nullptr)
        if (sscanf(p, "%*[^:]: %u", &u) == 1) cfg.freq_start_hz = u;

    if ((p = strstr(buf, "\"freq_stop_hz\"")) != nullptr)
        if (sscanf(p, "%*[^:]: %u", &u) == 1) cfg.freq_stop_hz = u;

    if ((p = strstr(buf, "\"step_hz\"")) != nullptr)
        if (sscanf(p, "%*[^:]: %u", &u) == 1) cfg.step_hz = u;

    if ((p = strstr(buf, "\"dwell_ms\"")) != nullptr)
        if (sscanf(p, "%*[^:]: %u", &u) == 1) cfg.dwell_ms = u;

    if ((p = strstr(buf, "\"threshold_dbm\"")) != nullptr)
        if (sscanf(p, "%*[^:]: %d", &s) == 1) cfg.threshold_dbm = s;

    // Sanity-clamp
    if (cfg.step_hz == 0u)   cfg.step_hz   = defaults.step_hz;
    if (cfg.dwell_ms == 0u)  cfg.dwell_ms  = defaults.dwell_ms;
    if (cfg.freq_stop_hz <= cfg.freq_start_hz)
        cfg.freq_stop_hz = cfg.freq_start_hz + cfg.step_hz;

    sentinel_log("BG_SCAN",
                 "Config: start=%u stop=%u step=%u dwell=%u thr=%d",
                 (unsigned)cfg.freq_start_hz,
                 (unsigned)cfg.freq_stop_hz,
                 (unsigned)cfg.step_hz,
                 (unsigned)cfg.dwell_ms,
                 (int)cfg.threshold_dbm);

    return cfg;
}

// ---------------------------------------------------------------------------
// send_radio_command — helpers to write into IPC shared memory
// ---------------------------------------------------------------------------
static void ipc_set_mode(sentinel::ipc::BasebandMode mode, uint32_t freq_hz)
{
    auto& shm = sentinel::ipc::shared();
    shm.cmd_mode    = mode;
    shm.cmd_freq_hz = freq_hz;
    shm.cmd_bw_hz   = 200000u;  // 200 kHz default scan bandwidth
    sentinel::ipc::send_command(sentinel::ipc::Command::SET_MODE);
}

static void ipc_tune(uint32_t freq_hz)
{
    sentinel::ipc::shared().cmd_freq_hz = freq_hz;
    sentinel::ipc::send_command(sentinel::ipc::Command::TUNE);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// task_bg_scanner_entry
//
// Entry point for the background scanner task.
// Priority: BG_SCANNER (3).  Stack: 512 words (2 KB).
// ---------------------------------------------------------------------------
extern "C" void task_bg_scanner_entry(void* /*pvParameters*/)
{
    sentinel_log("BG_SCAN", "Background scanner task started");

    // -----------------------------------------------------------------------
    // 1. Set up event bus subscription (RADIO_REQUEST, RADIO_RELEASE)
    // -----------------------------------------------------------------------
    static StaticQueue_t s_queue_static;
    static uint8_t       s_queue_storage[16 * sizeof(sentinel::BusEvent)];
    const QueueHandle_t rx_queue = xQueueCreateStatic(
        16,
        sizeof(sentinel::BusEvent),
        s_queue_storage,
        &s_queue_static);

    const uint32_t mask =
        EVENT_MASK(sentinel::EventType::RADIO_REQUEST) |
        EVENT_MASK(sentinel::EventType::RADIO_RELEASE);

    const int sub_id = sentinel::EventBus::instance().subscribe(rx_queue, mask);
    if (sub_id < 0) {
        sentinel_log("BG_SCAN", "ERROR: EventBus subscribe failed — task halting");
        vTaskSuspend(nullptr);
    }

    // -----------------------------------------------------------------------
    // 2. Load scan configuration from SD card
    // -----------------------------------------------------------------------
    const ScanConfig cfg = parse_scan_config(ScanConfig{});

    // -----------------------------------------------------------------------
    // 3. Main scan loop
    // -----------------------------------------------------------------------
    uint32_t current_freq = cfg.freq_start_hz;
    TickType_t last_status_tick = xTaskGetTickCount();
    static constexpr uint32_t STATUS_INTERVAL_MS = 30000u;

    // Signal ring read shadow (local copy of signal_rd)
    uint8_t signal_rd_local = sentinel::ipc::shared().signal_rd;

    for (;;) {

        // --- Check bus events (non-blocking) ---
        sentinel::BusEvent ev{};
        while (xQueueReceive(rx_queue, &ev, 0) == pdTRUE) {
            const auto type = static_cast<sentinel::EventType>(ev.type);
            if (type == sentinel::EventType::RADIO_REQUEST) {
                // Another task wants the radio — pause scan
                if (sentinel::g_radio_available) {
                    sentinel_log("BG_SCAN", "Radio preempted — pausing scan");
                    sentinel::g_radio_available = false;
                    // Tell M0 to go idle
                    sentinel::ipc::shared().cmd_mode = sentinel::ipc::BasebandMode::IDLE;
                    sentinel::ipc::send_command(sentinel::ipc::Command::SET_MODE);
                }
            } else if (type == sentinel::EventType::RADIO_RELEASE) {
                // Releasing task is done — resume scan
                if (!sentinel::g_radio_available) {
                    sentinel_log("BG_SCAN", "Radio released — resuming scan");
                    sentinel::g_radio_available = true;
                    // Re-enter scanner mode from where we left off
                    ipc_set_mode(sentinel::ipc::BasebandMode::SCANNER, current_freq);
                }
            }
        }

        // --- Wait if radio is not available ---
        if (!sentinel::g_radio_available) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- First iteration: enter SCANNER mode ---
        if (current_freq == cfg.freq_start_hz) {
            ipc_set_mode(sentinel::ipc::BasebandMode::SCANNER, current_freq);
        }

        // --- Tune to current frequency ---
        ipc_tune(current_freq);

        // --- Dwell ---
        vTaskDelay(pdMS_TO_TICKS(cfg.dwell_ms));

        // --- Read RSSI ---
        const int16_t rssi = sentinel::ipc::shared().rssi_dbm_x10;
        // rssi is in dBm × 10; threshold is in dBm
        const bool above_threshold = (rssi > cfg.threshold_dbm * 10);

        // --- Drain signal detection ring ---
        {
            const uint8_t wr = sentinel::ipc::shared().signal_wr;
            while (signal_rd_local != wr) {
                const auto& sig = sentinel::ipc::shared().signals[
                    signal_rd_local % sentinel::ipc::SIGNAL_RING_DEPTH];

                // Build and publish SIGNAL_DETECTED event
                sentinel::BusEvent out = sentinel::BusEvent::make(
                    sentinel::EventType::SIGNAL_DETECTED,
                    sentinel::TaskID::BG_SCANNER);
                out.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                auto& p = out.as<sentinel::SignalDetectedPayload>();
                p.frequency_hz     = sig.freq_hz;
                p.rssi_dbm         = sig.rssi_dbm_x10;
                p.bandwidth_est_hz = 0u;   // not estimated by M0 scanner
                p.duration_ms      = sig.duration_ms;
                p.gps_lat          = 0;    // filled by ESP32 comms task
                p.gps_lon          = 0;

                sentinel::EventBus::instance().publish(out);

                signal_rd_local = (signal_rd_local + 1u) % 256u;
            }
            // Write back updated rd
            sentinel::ipc::shared().signal_rd = signal_rd_local;
        }

        // Also publish if RSSI alone exceeds threshold (not caught by M0 ring)
        if (above_threshold) {
            sentinel::BusEvent out = sentinel::BusEvent::make(
                sentinel::EventType::SIGNAL_DETECTED,
                sentinel::TaskID::BG_SCANNER);
            out.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            auto& p = out.as<sentinel::SignalDetectedPayload>();
            p.frequency_hz     = current_freq;
            p.rssi_dbm         = rssi;
            p.bandwidth_est_hz = 0u;
            p.duration_ms      = static_cast<uint16_t>(cfg.dwell_ms);
            p.gps_lat          = 0;
            p.gps_lon          = 0;

            sentinel::EventBus::instance().publish(out);
        }

        // --- Advance to next frequency ---
        current_freq += cfg.step_hz;
        if (current_freq > cfg.freq_stop_hz) {
            current_freq = cfg.freq_start_hz;
        }

        // --- Periodic SYSTEM_STATUS event (every 30 s) ---
        const TickType_t now = xTaskGetTickCount();
        if ((now - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
            last_status_tick = now;

            sentinel::BusEvent status_ev = sentinel::BusEvent::make(
                sentinel::EventType::SYSTEM_STATUS,
                sentinel::TaskID::BG_SCANNER);
            status_ev.timestamp_ms = now * portTICK_PERIOD_MS;

            auto& sp = status_ev.as<sentinel::SystemStatusPayload>();
            sp.battery_pct   = 100u;     // TODO: ADC battery monitor
            sp.sd_free_mb    = 0u;       // TODO: FatFS f_getfree
            sp.uptime_s      = static_cast<uint32_t>(now / 1000u);
            sp.cpu_load_pct  = 0u;       // TODO: FreeRTOS run-time stats
            sp.heap_free_pct = static_cast<uint8_t>(
                (xPortGetFreeHeapSize() * 100u) / configTOTAL_HEAP_SIZE);
            __builtin_memset(sp.pad, 0, sizeof(sp.pad));

            sentinel::EventBus::instance().publish(status_ev);
        }
    }

    // Unreachable
    sentinel::EventBus::instance().unsubscribe(sub_id);
}
