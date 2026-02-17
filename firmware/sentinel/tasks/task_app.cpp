// SPDX-License-Identifier: MIT
// Project Sentinel — Application Task (task_app)
//
// Owns the display and drives the active application at ~60 Hz.
// Handles joystick navigation input and M0 audio notification.
//
// App switching API:
//   void sentinel_push_app(AppBase* app)  — switch to a new app
//
// The home dashboard is held in static storage and is never deleted.
// User apps allocate from a static pool (see below) or are stack-local
// singletons kept alive by the caller.

#include "FreeRTOS.h"
#include "task.h"

#include "../event_bus/event_bus.hpp"
#include "../event_bus/event_types.hpp"
#include "../tasks/task_ids.hpp"
#include "../apps/sentinel_app_base.hpp"
#include "../apps/app_ids.hpp"
#include "../ui/widgets.hpp"
#include "../hal/gpio.hpp"
#include "../bsp/portapack_pins.hpp"
#include "../bsp/audio_wm8731.hpp"
#include "ipc/ipc_protocol.hpp"

#include <cstdint>

// Forward declaration of log API (task_logger.cpp)
extern "C" void sentinel_log(const char* tag, const char* fmt, ...);

// Forward declaration of HomeDashboardApp (apps/app_home_dashboard.hpp)
// We include it opaquely to avoid pulling in app headers from the task layer.
namespace sentinel {
    class HomeDashboardApp;
}

// The actual header is included only for static construction:
#include "../apps/app_home_dashboard.hpp"

// ---------------------------------------------------------------------------
// Global current app pointer
// ---------------------------------------------------------------------------

namespace sentinel {

// The active application. Set by sentinel_push_app().
static AppBase* g_current_app = nullptr;

// Static home dashboard instance — lives for the entire firmware lifetime.
// Placement in BSS ensures zero-initialisation before main().
static HomeDashboardApp g_home_dashboard_instance;

// Task handle for the app task (registered by task_orchestrator.cpp).
extern TaskHandle_t g_task_handles[];

} // namespace sentinel

// ---------------------------------------------------------------------------
// sentinel_push_app
//
// Public API callable from any app to switch to a new app.
// Safe to call from within on_tick() or on_nav() of the current app;
// the transition happens at the top of the next 60 Hz tick.
//
// Passing nullptr switches back to the home dashboard.
// ---------------------------------------------------------------------------
void sentinel_push_app(sentinel::AppBase* new_app)
{
    using namespace sentinel;

    if (g_current_app != nullptr) {
        g_current_app->on_exit();
    }

    if (new_app == nullptr) {
        new_app = &g_home_dashboard_instance;
    }

    g_current_app = new_app;
    g_current_app->on_enter();

    sentinel_log("APP_TASK", "Switched to app_id=%d",
                 static_cast<int>(g_current_app->app_id()));
}

// ---------------------------------------------------------------------------
// Joystick debounce state
// ---------------------------------------------------------------------------
namespace {

struct NavState {
    bool     pressed;
    bool     reported;
    uint32_t press_tick;  // tick at which the pin went active-low
};

static NavState s_nav[5]{};  // UP, DOWN, LEFT, RIGHT, SELECT

static constexpr uint32_t DEBOUNCE_MS = 10u;

// Read all 5 nav pins and return a debounced NavKey event (or NONE).
// Called once per 60 Hz tick.
static sentinel::ui::NavKey poll_joystick()
{
    using sentinel::ui::NavKey;

    // Pin mapping: index 0-4 → UP, DOWN, LEFT, RIGHT, SELECT
    static constexpr struct {
        uint8_t port;
        uint8_t pin;
        NavKey  key;
    } k_pins[5] = {
        { NAV_GPIO_PORT, NAV_UP_PIN,     NavKey::UP     },
        { NAV_GPIO_PORT, NAV_DOWN_PIN,   NavKey::DOWN   },
        { NAV_GPIO_PORT, NAV_LEFT_PIN,   NavKey::LEFT   },
        { NAV_GPIO_PORT, NAV_RIGHT_PIN,  NavKey::RIGHT  },
        { NAV_GPIO_PORT, NAV_SELECT_PIN, NavKey::SELECT },
    };

    const uint32_t now = xTaskGetTickCount();
    NavKey result = NavKey::NONE;

    for (int i = 0; i < 5; ++i) {
        // Pins are active-low: gpio_read returns false when key is pressed.
        const bool active = !gpio_read(k_pins[i].port, k_pins[i].pin);

        if (active && !s_nav[i].pressed) {
            // Rising edge: start debounce timer
            s_nav[i].pressed    = true;
            s_nav[i].reported   = false;
            s_nav[i].press_tick = now;
        } else if (!active && s_nav[i].pressed) {
            // Released: clear state
            s_nav[i].pressed  = false;
            s_nav[i].reported = false;
        }

        // Report the key once per press, after debounce interval
        if (s_nav[i].pressed && !s_nav[i].reported &&
            (now - s_nav[i].press_tick) >= DEBOUNCE_MS) {
            s_nav[i].reported = true;
            result = k_pins[i].key;  // Last active key wins (one per tick)
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Handle M0 audio notification
// ---------------------------------------------------------------------------
static void handle_m0_audio()
{
    using namespace sentinel::ipc;

    auto& shm = shared();

    const uint32_t wr = shm.audio_wr_pos;
    const uint32_t rd = shm.audio_rd_pos;

    if (wr == rd) return;  // Nothing new

    // Samples are laid out as a linear ring of AUDIO_BUF_SAMPLES int16 values.
    // We read from rd to wr (modulo AUDIO_BUF_SAMPLES).
    const uint32_t avail = wr - rd;  // unsigned wrap handles ring correctly
    const uint32_t start = rd % AUDIO_BUF_SAMPLES;
    const uint32_t end   = wr % AUDIO_BUF_SAMPLES;

    if (start < end) {
        // Contiguous region
        audio_feed_samples(&shm.audio_buf[start], end - start);
    } else {
        // Wrapped: two segments
        audio_feed_samples(&shm.audio_buf[start], AUDIO_BUF_SAMPLES - start);
        audio_feed_samples(&shm.audio_buf[0],     end);
    }

    // Advance read pointer
    shm.audio_rd_pos = wr;

    // Clear the flag
    shm.m0_flags &= ~M0_FLAG_AUDIO;

    (void)avail;  // suppress unused warning
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// task_app_entry
//
// Application task entry point. Priority: APP_TASK (3).
// Stack: 1024 words (4 KB).
// ---------------------------------------------------------------------------
extern "C" void task_app_entry(void* /*pvParameters*/)
{
    sentinel_log("APP_TASK", "Application task started");

    // Initialise the home dashboard and make it current
    sentinel_push_app(&sentinel::g_home_dashboard_instance);

    for (;;) {
        // --- 1. Drain bus events and call app logic ---
        sentinel::g_current_app->on_tick();

        // --- 2. Redraw screen ---
        sentinel::g_current_app->on_paint();

        // --- 3. Joystick input ---
        const sentinel::ui::NavKey key = poll_joystick();
        if (key != sentinel::ui::NavKey::NONE) {
            sentinel::g_current_app->on_nav(key);
        }

        // --- 4. Wait for next 60 Hz frame ---
        vTaskDelay(pdMS_TO_TICKS(16));

        // --- 5. Handle M0 notifications (audio, etc.) ---
        const uint32_t m0f = sentinel::ipc::shared().m0_flags;

        if (m0f & sentinel::ipc::M0_FLAG_AUDIO) {
            handle_m0_audio();
        }
        // Other M0 flags (SPECTRUM, ADSB, SIGNAL, RSSI) are consumed by the
        // active app via on_tick() / EventBus polling within AppBase.
    }
}
