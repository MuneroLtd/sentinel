/*
 * Project Sentinel - SD Card Observation Logger Task
 *
 * Subscribes to SIGNAL_DETECTED events and writes compact 16-byte records
 * to a binary observation file on the SD card. GPS position is geo-tagged
 * from GPS_UPDATE events.
 *
 * Record format (16 bytes, packed):
 *   Offset  Size  Field
 *   0       4     timestamp_s      — RTOS tick / 1000 (approx seconds since boot)
 *   4       4     frequency_hz     — Centre frequency in Hz
 *   8       2     rssi_dbm         — RSSI × 10 (int16, 0.1 dBm resolution)
 *   10      3     gps_lat_24       — latitude  × 1e6, stored as 24-bit signed
 *   13      3     gps_lon_24       — longitude × 1e6, stored as 24-bit signed
 *
 * RAM ring buffer: 256 × 16 = 4096 bytes
 * Flush trigger:   60s elapsed OR 192/256 records used (75% full)
 *
 * File layout on SD:
 *   /SENTINEL/knowledge_base/YYYY-MM-DD.bin   — raw binary records (appended)
 *   /SENTINEL/knowledge_base/YYYY-MM-DD.idx   — (frequency, file_offset) pairs,
 *                                               rebuilt on every flush
 *
 * The index is rebuilt from scratch on each flush to keep it accurate;
 * we only append to .bin, so the index must cover the entire file.
 *
 * Date derivation:
 *   We do not have a real-time clock; we use GPS date from GPS_UPDATE events
 *   when available. If no GPS fix, we substitute "0000-00-00".
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ff.h"

#include "../event_bus/event_bus.hpp"
#include "../event_bus/event_types.hpp"
#include "../tasks/task_ids.hpp"

#include <cstdio>

extern "C" void sentinel_log(const char* tag, const char* fmt, ...);

// ---------------------------------------------------------------------------
// Record definition (16 bytes packed, verified by static_assert below)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) ObservationRecord {
    uint32_t timestamp_s;
    uint32_t frequency_hz;
    int16_t  rssi_dbm;
    uint8_t  gps_lat_24[3];   // 24-bit two's complement, little-endian
    uint8_t  gps_lon_24[3];   // 24-bit two's complement, little-endian
};
static_assert(sizeof(ObservationRecord) == 16, "ObservationRecord must be 16 bytes");

// Index entry: (frequency, file byte offset)
struct __attribute__((packed)) IndexEntry {
    uint32_t frequency_hz;
    uint32_t file_offset;     // byte offset of matching record in .bin file
};
static_assert(sizeof(IndexEntry) == 8, "IndexEntry must be 8 bytes");

// ---------------------------------------------------------------------------
// Helper: pack a 32-bit signed integer into 3 bytes (little-endian 24-bit)
// ---------------------------------------------------------------------------
static void pack_24(uint8_t dest[3], int32_t value)
{
    dest[0] = static_cast<uint8_t>( value        & 0xFF);
    dest[1] = static_cast<uint8_t>((value >>  8) & 0xFF);
    dest[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
}

// ---------------------------------------------------------------------------
// Ring buffer
// ---------------------------------------------------------------------------
namespace {

static constexpr int RING_CAPACITY    = 256;
static constexpr int FLUSH_THRESHOLD  = 192;   // 75 % of 256
static constexpr uint32_t FLUSH_INTERVAL_MS = 60000u;

struct RingBuffer {
    ObservationRecord records[RING_CAPACITY];
    int   head  = 0;  // next write slot
    int   count = 0;  // number of valid records
};

static RingBuffer      s_ring{};
static SemaphoreHandle_t s_ring_mutex = nullptr;

// ---------------------------------------------------------------------------
// GPS cache (updated from GPS_UPDATE events)
// ---------------------------------------------------------------------------
static int32_t  s_gps_lat    = 0;
static int32_t  s_gps_lon    = 0;
static bool     s_gps_valid  = false;

// Current date string: "YYYY-MM-DD\0"
static char s_date_str[16] = "0000-00-00";

// ---------------------------------------------------------------------------
// ring_push: add a record to the ring buffer.
// If full, oldest record is overwritten.
// ---------------------------------------------------------------------------
static void ring_push(const ObservationRecord& rec)
{
    if (s_ring_mutex && xSemaphoreTake(s_ring_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_ring.records[s_ring.head] = rec;
        s_ring.head = (s_ring.head + 1) % RING_CAPACITY;
        if (s_ring.count < RING_CAPACITY) {
            ++s_ring.count;
        }
        xSemaphoreGive(s_ring_mutex);
    }
}

// ---------------------------------------------------------------------------
// flush_to_sd
//
// Appends all records in the ring buffer to the .bin file, then rebuilds the
// .idx file by scanning the entire .bin file.
//
// Returns the number of records flushed, or -1 on error.
// ---------------------------------------------------------------------------
static int flush_to_sd()
{
    // Take a snapshot of the ring buffer under mutex, then release
    ObservationRecord snapshot[RING_CAPACITY];
    int count = 0;

    if (s_ring_mutex && xSemaphoreTake(s_ring_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        count = s_ring.count;
        if (count > 0) {
            // Records are stored in insertion order starting from:
            //   oldest_slot = (head - count + RING_CAPACITY) % RING_CAPACITY
            const int oldest = (s_ring.head - count + RING_CAPACITY) % RING_CAPACITY;
            for (int i = 0; i < count; ++i) {
                snapshot[i] = s_ring.records[(oldest + i) % RING_CAPACITY];
            }
        }
        s_ring.count = 0;
        s_ring.head  = 0;
        xSemaphoreGive(s_ring_mutex);
    }

    if (count == 0) return 0;

    // Ensure directory exists
    f_mkdir("/SENTINEL");
    f_mkdir("/SENTINEL/knowledge_base");

    // Build file paths
    char bin_path[64];
    char idx_path[64];
    snprintf(bin_path, sizeof(bin_path), "/SENTINEL/knowledge_base/%s.bin", s_date_str);
    snprintf(idx_path, sizeof(idx_path), "/SENTINEL/knowledge_base/%s.idx", s_date_str);

    FATFS fs;
    FIL   bin_file;
    FRESULT rc;

    if (f_mount(&fs, "", 1) != FR_OK) {
        sentinel_log("KB", "SD mount failed — %d records dropped", count);
        return -1;
    }

    // Append snapshot to .bin file
    rc = f_open(&bin_file, bin_path, FA_WRITE | FA_OPEN_APPEND);
    if (rc != FR_OK) {
        sentinel_log("KB", "Cannot open %s (rc=%d)", bin_path, (int)rc);
        f_unmount("");
        return -1;
    }

    UINT written = 0;
    f_write(&bin_file, snapshot, count * sizeof(ObservationRecord), &written);
    f_close(&bin_file);

    sentinel_log("KB", "Flushed %d records to %s", count, bin_path);

    // Rebuild the index: open .bin, scan entire file, write sorted index
    FIL idx_file;
    FIL bin_scan;

    rc = f_open(&bin_scan, bin_path, FA_READ);
    if (rc != FR_OK) {
        sentinel_log("KB", "Cannot reopen %s for index build", bin_path);
        f_unmount("");
        return count;
    }

    rc = f_open(&idx_file, idx_path, FA_WRITE | FA_CREATE_ALWAYS);
    if (rc != FR_OK) {
        f_close(&bin_scan);
        sentinel_log("KB", "Cannot open %s for write", idx_path);
        f_unmount("");
        return count;
    }

    // Simple insertion-sort index build: read all records, write sorted (freq, offset) pairs.
    // For embedded use with ≤256 records per flush this is acceptable (O(n²) but n is small).
    // We allocate index entries on the stack — 256 × 8 = 2048 bytes.
    static IndexEntry idx_entries[RING_CAPACITY];
    int idx_count = 0;
    ObservationRecord scan_rec{};
    UINT bytes_read = 0;
    FSIZE_t offset  = 0;

    while (true) {
        f_read(&bin_scan, &scan_rec, sizeof(scan_rec), &bytes_read);
        if (bytes_read < sizeof(scan_rec)) break;
        if (idx_count < RING_CAPACITY) {
            idx_entries[idx_count].frequency_hz = scan_rec.frequency_hz;
            idx_entries[idx_count].file_offset  = static_cast<uint32_t>(offset);
            ++idx_count;
        }
        offset += sizeof(scan_rec);
    }
    f_close(&bin_scan);

    // Insertion sort by frequency
    for (int i = 1; i < idx_count; ++i) {
        IndexEntry key = idx_entries[i];
        int j = i - 1;
        while (j >= 0 && idx_entries[j].frequency_hz > key.frequency_hz) {
            idx_entries[j + 1] = idx_entries[j];
            --j;
        }
        idx_entries[j + 1] = key;
    }

    UINT idx_written = 0;
    f_write(&idx_file, idx_entries, idx_count * sizeof(IndexEntry), &idx_written);
    f_close(&idx_file);

    sentinel_log("KB", "Index rebuilt: %d entries in %s", idx_count, idx_path);

    f_unmount("");
    return count;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// task_knowledge_base_fn
//
// Priority: KNOWLEDGE_BASE (1) — lowest user task; SD writes are not latency-
// critical and should not compete with radio or UI tasks.
// ---------------------------------------------------------------------------
extern "C" void task_knowledge_base_fn(void* /*pvParameters*/)
{
    sentinel_log("KB", "Knowledge base task started");

    s_ring_mutex = xSemaphoreCreateMutex();

    // Local event receive queue
    static StaticQueue_t s_queue_static;
    static uint8_t       s_queue_storage[16 * sizeof(sentinel::BusEvent)];
    const QueueHandle_t  rx_queue = xQueueCreateStatic(
        16,
        sizeof(sentinel::BusEvent),
        s_queue_storage,
        &s_queue_static);

    // Subscribe to SIGNAL_DETECTED and GPS_UPDATE
    const uint32_t mask =
        EVENT_MASK(sentinel::EventType::SIGNAL_DETECTED) |
        EVENT_MASK(sentinel::EventType::GPS_UPDATE);

    const int sub_id = sentinel::EventBus::instance().subscribe(rx_queue, mask);
    if (sub_id < 0) {
        sentinel_log("KB", "ERROR: EventBus subscribe failed");
        vTaskSuspend(nullptr);
    }

    TickType_t last_flush_tick = xTaskGetTickCount();
    sentinel::BusEvent ev{};

    for (;;) {
        // Wait up to 5s for an event; then check flush timer
        const BaseType_t got = xQueueReceive(rx_queue, &ev, pdMS_TO_TICKS(5000));

        if (got == pdTRUE) {
            const auto type = static_cast<sentinel::EventType>(ev.type);

            if (type == sentinel::EventType::GPS_UPDATE) {
                const auto& gp = ev.as<sentinel::GPSUpdatePayload>();
                s_gps_lat   = gp.latitude;
                s_gps_lon   = gp.longitude;
                s_gps_valid = (gp.fix_quality > 0);
                // TODO: update s_date_str from GPS date when available in payload
            }
            else if (type == sentinel::EventType::SIGNAL_DETECTED) {
                const auto& sig = ev.as<sentinel::SignalDetectedPayload>();

                ObservationRecord rec{};
                rec.timestamp_s  = xTaskGetTickCount() / 1000;
                rec.frequency_hz = sig.frequency_hz;
                rec.rssi_dbm     = sig.rssi_dbm;
                pack_24(rec.gps_lat_24, s_gps_valid ? s_gps_lat : sig.gps_lat);
                pack_24(rec.gps_lon_24, s_gps_valid ? s_gps_lon : sig.gps_lon);

                ring_push(rec);
            }
        }

        // Flush decision: time-based or threshold-based
        const TickType_t now = xTaskGetTickCount();
        const int current_count = [&]() {
            int n = 0;
            if (s_ring_mutex && xSemaphoreTake(s_ring_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                n = s_ring.count;
                xSemaphoreGive(s_ring_mutex);
            }
            return n;
        }();

        const bool time_trigger      = (now - last_flush_tick) >= pdMS_TO_TICKS(FLUSH_INTERVAL_MS);
        const bool threshold_trigger = (current_count >= FLUSH_THRESHOLD);

        if ((time_trigger || threshold_trigger) && current_count > 0) {
            sentinel_log("KB", "Flush triggered (time=%d threshold=%d count=%d)",
                         (int)time_trigger, (int)threshold_trigger, current_count);
            flush_to_sd();
            last_flush_tick = now;
        }
    }

    sentinel::EventBus::instance().unsubscribe(sub_id);
}
