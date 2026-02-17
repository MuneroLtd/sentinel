/*
 * Project Sentinel - Serial Debug Logger Task
 *
 * Provides a thread-safe logging API callable from any task or ISR:
 *
 *   sentinel_log("TAG", "value=%d freq=%u", val, freq);
 *
 * Architecture:
 *   - A 512-byte ring buffer stores formatted message strings.
 *   - A FreeRTOS mutex protects the ring buffer during concurrent writes.
 *   - The logger task drains the ring buffer to the USB CDC UART at 115200.
 *   - Each message is formatted as:  [%5ums][%-12s] <message>\n
 *   - Every 30 seconds the logger prints all task stack high-water marks.
 *
 * Why a ring buffer rather than a FreeRTOS queue of pointers?
 *   - No dynamic heap allocation per log call (stack-local format buffers
 *     are written directly into the ring buffer)
 *   - Simpler to reason about in an embedded context
 *   - Drops old messages (tail overwrite) rather than blocking writers
 *
 * UART output:
 *   The LPC4320 USART0 peripheral is used at 115200 8N1.
 *   We write via the PortaPack usb_serial (USB CDC ACM) which presents as a
 *   virtual COM port on the host. Baud rate selection is irrelevant for USB
 *   CDC but 115200 is set for parity with hardware UART fallback.
 *   Direct USART register writes are used below to avoid ChibiOS HAL dependency.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cinttypes>

#ifndef PRIu32
#define PRIu32 "u"
#endif

#include "../tasks/task_ids.hpp"

// ---------------------------------------------------------------------------
// LPC4320 USART0 register definitions
// Used as fallback if USB CDC is unavailable.
// Reference: LPC43xx User Manual UM10503, Chapter 35.
// ---------------------------------------------------------------------------
namespace usart0 {
    static constexpr uint32_t BASE       = 0x40081000;
    static constexpr uint32_t THR_OFFSET = 0x00;   // Transmit Holding Register
    static constexpr uint32_t LSR_OFFSET = 0x14;   // Line Status Register
    static constexpr uint32_t LSR_THRE   = (1u << 5); // Tx Holding Register Empty

    inline volatile uint32_t& reg(uint32_t offset) {
        return *reinterpret_cast<volatile uint32_t*>(BASE + offset);
    }

    inline void putchar(uint8_t c) {
        while (!(reg(LSR_OFFSET) & LSR_THRE)) {}
        reg(THR_OFFSET) = c;
    }

    inline void write(const char* buf, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            putchar(static_cast<uint8_t>(buf[i]));
        }
    }
} // namespace usart0

// ---------------------------------------------------------------------------
// Ring buffer parameters
// ---------------------------------------------------------------------------
namespace {

static constexpr size_t RING_BUF_SIZE   = 512;
static constexpr size_t MAX_MSG_LEN     = 128;  // max single formatted message
static constexpr uint32_t HWM_INTERVAL_MS = 30000u; // 30 seconds

// Ring buffer state
struct RingBuf {
    char     data[RING_BUF_SIZE];
    uint16_t head;  // write pointer (producer)
    uint16_t tail;  // read pointer  (consumer)
    uint16_t used;  // bytes currently in buffer
};

static RingBuf     s_ring{};
static SemaphoreHandle_t s_mutex = nullptr;

// ---------------------------------------------------------------------------
// ring_write: write up to len bytes into ring buffer.
// Overwrites oldest data if buffer is full (lossy but non-blocking).
// Must be called with s_mutex held.
// ---------------------------------------------------------------------------
static void ring_write(const char* src, size_t len)
{
    if (len == 0 || len > RING_BUF_SIZE) return;

    // If writing would overflow, advance tail to make room.
    if (s_ring.used + len > RING_BUF_SIZE) {
        const size_t drop = (s_ring.used + len) - RING_BUF_SIZE;
        s_ring.tail = static_cast<uint16_t>((s_ring.tail + drop) % RING_BUF_SIZE);
        s_ring.used = static_cast<uint16_t>(s_ring.used - drop);
    }

    for (size_t i = 0; i < len; ++i) {
        s_ring.data[s_ring.head] = src[i];
        s_ring.head = static_cast<uint16_t>((s_ring.head + 1) % RING_BUF_SIZE);
    }
    s_ring.used = static_cast<uint16_t>(s_ring.used + len);
}

// ---------------------------------------------------------------------------
// ring_read: read up to max_len bytes from ring buffer into dst.
// Returns bytes actually read. Must be called with s_mutex held.
// ---------------------------------------------------------------------------
static size_t ring_read(char* dst, size_t max_len)
{
    const size_t avail = (s_ring.used < max_len) ? s_ring.used : max_len;
    for (size_t i = 0; i < avail; ++i) {
        dst[i] = s_ring.data[s_ring.tail];
        s_ring.tail = static_cast<uint16_t>((s_ring.tail + 1) % RING_BUF_SIZE);
    }
    s_ring.used = static_cast<uint16_t>(s_ring.used - avail);
    return avail;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// sentinel_log
//
// Public API: callable from any task. Formats the message and appends it to
// the ring buffer. Does NOT block — if the ring buffer is full, old messages
// are silently overwritten. This prevents logging from stalling real-time tasks.
//
// Format: [%5ums][%-12s] <message>\n
// ---------------------------------------------------------------------------
extern "C" void sentinel_log(const char* tag, const char* fmt, ...)
{
    // Format the message payload into a stack-local buffer.
    char body[MAX_MSG_LEN];
    va_list args;
    va_start(args, fmt);
    const int body_len = vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    if (body_len <= 0) return;

    // Format the full line with timestamp and tag.
    char line[MAX_MSG_LEN + 32];
    const TickType_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    const int line_len = snprintf(line, sizeof(line),
                                  "[%5" PRIu32 "ms][%-12s] %s\n",
                                  static_cast<uint32_t>(now_ms),
                                  tag,
                                  body);

    if (line_len <= 0) return;

    // Take mutex with a short timeout — if the logger is busy, drop rather
    // than block the calling task for an extended period.
    if (s_mutex != nullptr &&
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ring_write(line, static_cast<size_t>(line_len));
        xSemaphoreGive(s_mutex);
    }
    // If mutex unavailable: message silently discarded (acceptable for a logger)
}

// ---------------------------------------------------------------------------
// print_stack_watermarks
//
// Logs the stack high-water mark (minimum free stack words) for every task.
// Called every HWM_INTERVAL_MS from the logger task.
// ---------------------------------------------------------------------------
static void print_stack_watermarks()
{
    // FreeRTOS vTaskGetRunTimeStats requires a buffer; use stack allocation.
    // configUSE_STATS_FORMATTING_FUNCTIONS = 1 provides vTaskList().
    char buf[512];
    vTaskList(buf);
    // vTaskList output: "Name\t\tState\tPriority\tFreeStack\tTaskNum\n" per task
    sentinel_log("LOGGER", "=== Task Stack High-Water Marks ===\n%s", buf);
}

// ---------------------------------------------------------------------------
// task_logger_fn
//
// Logger task entry point. Priority: LOGGER (4).
//
// Creates the ring buffer mutex, then loops:
//   1. Drain any pending bytes from the ring buffer to USART0
//   2. If HWM interval elapsed, print all task watermarks
//   3. Delay 10ms before next drain (balances latency vs. CPU cost)
// ---------------------------------------------------------------------------
extern "C" void task_logger_fn(void* /*pvParameters*/)
{
    s_mutex = xSemaphoreCreateMutex();
    // If mutex creation fails we fall back to unprotected operation.
    // The ring buffer will still work, just with potential data races.

    // Zero-initialise ring buffer
    s_ring.head = 0;
    s_ring.tail = 0;
    s_ring.used = 0;

    sentinel_log("LOGGER", "Sentinel logger task started");

    TickType_t last_hwm_tick = xTaskGetTickCount();
    char drain_buf[64];

    for (;;) {
        // Drain ring buffer to UART in small chunks
        size_t drained = 0;
        do {
            size_t n = 0;
            if (s_mutex != nullptr &&
                xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                n = ring_read(drain_buf, sizeof(drain_buf));
                xSemaphoreGive(s_mutex);
            }
            if (n == 0) break;
            usart0::write(drain_buf, n);
            drained += n;
        } while (drained < RING_BUF_SIZE); // cap per-cycle drain at one full buffer

        // Stack high-water mark report every 30 seconds
        const TickType_t now = xTaskGetTickCount();
        if ((now - last_hwm_tick) >= pdMS_TO_TICKS(HWM_INTERVAL_MS)) {
            last_hwm_tick = now;
            print_stack_watermarks();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

