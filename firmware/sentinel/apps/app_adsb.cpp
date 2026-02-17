// SPDX-License-Identifier: MIT
// Project Sentinel - ADS-B receiver implementation

#include "app_adsb.hpp"
#include "app_home_dashboard.hpp"
#include "../ui/canvas.hpp"
#include "../ui/font.hpp"
#include "../../shared/ipc/ipc_protocol.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>

namespace sentinel {

// ---------------------------------------------------------------------------
// ADS-B CRC24 — generator polynomial 0xFFF409
// Standard polynomial per ICAO Annex 10.
// ---------------------------------------------------------------------------
static uint32_t adsb_crc24(const uint8_t* data, int len)
{
    constexpr uint32_t POLY = 0xFFF409u;
    uint32_t crc = 0;
    for (int i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x800000u) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc & 0xFFFFFFu;
}

// ---------------------------------------------------------------------------
// 6-bit char decode table for ADS-B identification (TC 1-4)
// ---------------------------------------------------------------------------
static const char ADSB_CHARSET[65] =
    "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####"
    "###############0123456789######";

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

AppAdsb::AppAdsb()
    : AppBase(AppID::ADS_B, TaskID::APP_TASK)
{
    static const char* labels[CTRL_BTN_COUNT] = { "Sort:Alt", "Sort:RSSI", "Clear", "Home" };
    for (int i = 0; i < CTRL_BTN_COUNT; ++i) {
        ctrl_btns_[i].r       = { i * 80, CTRL_Y, 80, CTRL_H };
        ctrl_btns_[i].label   = labels[i];
        ctrl_btns_[i].focused = (i == 0);
        ctrl_btns_[i].pressed = false;
    }
    ctrl_list_.focused_idx = 0;
    memset(aircraft_, 0, sizeof(aircraft_));
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AppAdsb::on_enter()
{
    request_radio(1'090'000'000u, 4'000'000u, 0);
    bool granted = wait_radio_grant(2000);
    if (granted) {
        auto& shm = ipc::shared();
        shm.cmd_mode    = ipc::BasebandMode::ADSB;
        shm.cmd_freq_hz = 1'090'000'000u;
        shm.cmd_bw_hz   = 4'000'000u;
        ipc::send_command(ipc::Command::SET_MODE);
    }
    publish_app_state(1, 1'090'000'000u);
    update_status();
}

void AppAdsb::on_exit()
{
    ipc::shared().cmd_mode = ipc::BasebandMode::IDLE;
    ipc::send_command(ipc::Command::SET_MODE);
    release_radio();
    publish_app_state(0);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void AppAdsb::on_paint()
{
    status_bar_.draw();
    draw_table();
    draw_controls();
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void AppAdsb::on_nav(ui::NavKey k)
{
    switch (k) {
        case ui::NavKey::LEFT:
            if (ctrl_list_.focused_idx > 0) --ctrl_list_.focused_idx;
            else ctrl_list_.focused_idx = CTRL_BTN_COUNT - 1;
            for (int i = 0; i < CTRL_BTN_COUNT; ++i)
                ctrl_btns_[i].focused = (i == ctrl_list_.focused_idx);
            break;
        case ui::NavKey::RIGHT:
            if (ctrl_list_.focused_idx < CTRL_BTN_COUNT - 1) ++ctrl_list_.focused_idx;
            else ctrl_list_.focused_idx = 0;
            for (int i = 0; i < CTRL_BTN_COUNT; ++i)
                ctrl_btns_[i].focused = (i == ctrl_list_.focused_idx);
            break;
        case ui::NavKey::UP:
            if (scroll_top_ > 0) --scroll_top_;
            break;
        case ui::NavKey::DOWN:
            ++scroll_top_;
            break;
        case ui::NavKey::SELECT: {
            int sel = ctrl_list_.focused_idx;
            if (sel == 0) {
                sort_mode_ = AdsbSortMode::BY_ALT;
            } else if (sel == 1) {
                sort_mode_ = AdsbSortMode::BY_RSSI;
            } else if (sel == 2) {
                // Clear all entries
                memset(aircraft_, 0, sizeof(aircraft_));
                ac_count_ = 0;
            } else if (sel == 3) {
                on_exit();
                g_current_app = nullptr;
            }
            break;
        }
        default:
            break;
    }
    draw_table();
    draw_controls();
    update_status();
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void AppAdsb::on_tick()
{
    process_bus_events();

    auto& shm = ipc::shared();
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uptime_s_ = now_ms / 1000u;

    // Drain ADS-B ring buffer
    if (shm.m0_flags & ipc::M0_FLAG_ADSB) {
        shm.m0_flags &= ~ipc::M0_FLAG_ADSB;

        while (shm.adsb_rd != shm.adsb_wr) {
            uint8_t idx = shm.adsb_rd;
            ipc::AdsbFrame frame = shm.adsb_frames[idx];  // copy off volatile
            shm.adsb_rd = static_cast<uint8_t>((idx + 1u) & (ipc::ADSB_RING_DEPTH - 1u));
            parse_frame(frame);
        }
    }

    // Expire old entries
    expire_old(now_ms);
    update_status();
}

// ---------------------------------------------------------------------------
// Bus events
// ---------------------------------------------------------------------------

void AppAdsb::on_bus_event(const BusEvent& /*ev*/) {}

// ---------------------------------------------------------------------------
// ADS-B parsing
// ---------------------------------------------------------------------------

bool AppAdsb::verify_crc24(const uint8_t* frame, int len)
{
    // The last 3 bytes of a 14-byte (DF17/18) frame are the CRC
    if (len < 4) return false;
    uint32_t computed = adsb_crc24(frame, len - 3);
    uint32_t embedded = (static_cast<uint32_t>(frame[len - 3]) << 16)
                      | (static_cast<uint32_t>(frame[len - 2]) << 8)
                      |  static_cast<uint32_t>(frame[len - 1]);
    return computed == embedded;
}

void AppAdsb::parse_frame(const ipc::AdsbFrame& f)
{
    // DF17/18 extended squitter: 14 bytes
    const uint8_t* data = f.data;
    uint8_t df = (data[0] >> 3) & 0x1F;
    if (df != 17 && df != 18) return;

    if (!verify_crc24(data, 14)) return;

    uint32_t icao = (static_cast<uint32_t>(data[1]) << 16)
                  | (static_cast<uint32_t>(data[2]) << 8)
                  |  static_cast<uint32_t>(data[3]);

    // ME field: bytes 4..10 (7 bytes)
    const uint8_t* me = &data[4];
    uint8_t tc = (me[0] >> 3) & 0x1F;  // Type Code = upper 5 bits of ME[0]

    if (tc >= 1 && tc <= 4) {
        decode_identification(icao, me, f.rssi_dbm_x10);
    } else if (tc >= 9 && tc <= 18) {
        bool odd = (me[2] >> 2) & 1;
        decode_position(icao, me, odd, f.timestamp_ms, f.rssi_dbm_x10);
    } else if (tc == 19) {
        decode_velocity(icao, me, f.rssi_dbm_x10);
    }
}

void AppAdsb::decode_identification(uint32_t icao, const uint8_t* me, int16_t rssi)
{
    AircraftEntry* e = find_or_alloc(icao);
    if (!e) return;
    e->rssi_dbm_x10 = rssi;

    // ME bytes 1-6 hold 8 x 6-bit characters
    // me[0] bits[2:0] and me[1]..me[6]
    char cs[9]{};
    cs[0] = ADSB_CHARSET[(me[1] >> 2) & 0x3F];
    cs[1] = ADSB_CHARSET[((me[1] & 0x03) << 4) | ((me[2] >> 4) & 0x0F)];
    cs[2] = ADSB_CHARSET[((me[2] & 0x0F) << 2) | ((me[3] >> 6) & 0x03)];
    cs[3] = ADSB_CHARSET[me[3] & 0x3F];
    cs[4] = ADSB_CHARSET[(me[4] >> 2) & 0x3F];
    cs[5] = ADSB_CHARSET[((me[4] & 0x03) << 4) | ((me[5] >> 4) & 0x0F)];
    cs[6] = ADSB_CHARSET[((me[5] & 0x0F) << 2) | ((me[6] >> 6) & 0x03)];
    cs[7] = ADSB_CHARSET[me[6] & 0x3F];
    cs[8] = '\0';
    memcpy(e->callsign, cs, 9);
    e->callsign_hash = crc16_callsign(cs);
}

void AppAdsb::decode_position(uint32_t icao, const uint8_t* me,
                               bool is_odd, uint32_t ts_ms, int16_t rssi)
{
    AircraftEntry* e = find_or_alloc(icao);
    if (!e) return;
    e->rssi_dbm_x10 = rssi;
    e->last_seen_ms = ts_ms;

    // Extract raw CPR lat/lon from ME bytes
    // Format: [alt(12)] [T(1)] [F(1)] [lat_cpr(17)] [lon_cpr(17)]
    uint32_t lat_cpr = ((static_cast<uint32_t>(me[2] & 0x03) << 15)
                       | (static_cast<uint32_t>(me[3]) << 7)
                       | (static_cast<uint32_t>(me[4]) >> 1)) & 0x1FFFF;
    uint32_t lon_cpr = ((static_cast<uint32_t>(me[4] & 0x01) << 16)
                       | (static_cast<uint32_t>(me[5]) << 8)
                       |  static_cast<uint32_t>(me[6])) & 0x1FFFF;

    // Extract 12-bit altitude (Gillham code / Gray code)
    int raw_alt = ((static_cast<int>(me[1]) << 4) | ((me[2] >> 4) & 0x0F)) & 0x0FFF;

    if (is_odd) {
        e->cpr_lat_odd = lat_cpr;
        e->cpr_lon_odd = lon_cpr;
        e->cpr_odd_valid = true;
    } else {
        e->cpr_lat_even = lat_cpr;
        e->cpr_lon_even = lon_cpr;
        e->cpr_even_valid = true;
    }

    decode_altitude(icao, raw_alt, rssi, ts_ms);

    if (e->cpr_even_valid && e->cpr_odd_valid) {
        resolve_cpr(*e);
        publish_position(*e);
    }
}

void AppAdsb::decode_altitude(uint32_t icao, int raw_alt, int16_t rssi, uint32_t ts_ms)
{
    AircraftEntry* e = find_or_alloc(icao);
    if (!e) return;
    e->rssi_dbm_x10 = rssi;
    e->last_seen_ms = ts_ms;

    // Q-bit determines encoding
    // Bit 4 of the 12-bit field (bit index from LSB) is the Q-bit
    if (raw_alt & 0x0010) {
        // Q=1: altitude in 25ft increments, offset -1000
        int n = ((raw_alt & 0x0FE0) >> 1) | (raw_alt & 0x000F);
        e->alt_ft = n * 25 - 1000;
    } else {
        // Q=0: Gray code / Gillham encoding — simplified, use 0 for now
        // Full Gillham decode is complex; store as unavailable
        e->alt_ft = 0;
    }
}

void AppAdsb::decode_velocity(uint32_t icao, const uint8_t* me, int16_t rssi)
{
    AircraftEntry* e = find_or_alloc(icao);
    if (!e) return;
    e->rssi_dbm_x10 = rssi;

    uint8_t subtype = me[0] & 0x07;

    if (subtype == 1 || subtype == 2) {
        // Ground speed
        bool ew_sign = (me[1] >> 2) & 1;
        int  ew_vel  = ((me[1] & 0x03) << 8) | me[2];
        bool ns_sign = (me[3] >> 7) & 1;
        int  ns_vel  = ((me[3] & 0x7F) << 3) | ((me[4] >> 5) & 0x07);

        if (ew_vel > 0) ew_vel -= 1;
        if (ns_vel > 0) ns_vel -= 1;
        if (ew_sign) ew_vel = -ew_vel;
        if (ns_sign) ns_vel = -ns_vel;

        // Speed = sqrt(ew^2 + ns^2)
        int spd = static_cast<int>(
            sqrtf(static_cast<float>(ew_vel * ew_vel + ns_vel * ns_vel)));
        e->speed_kts = static_cast<uint16_t>(spd);

        // Heading = atan2(ew, ns) in degrees
        float hdg = atan2f(static_cast<float>(ew_vel), static_cast<float>(ns_vel))
                    * (180.0f / 3.14159265f);
        if (hdg < 0) hdg += 360.0f;
        e->heading = static_cast<uint16_t>(hdg * 10);
    } else if (subtype == 3 || subtype == 4) {
        // Airspeed + true heading
        bool hdg_avail = (me[1] >> 2) & 1;
        int  hdg_raw   = ((me[1] & 0x03) << 8) | me[2];
        if (hdg_avail) {
            e->heading = static_cast<uint16_t>((hdg_raw * 3600u) >> 10);
        }
        int airspeed = ((me[3] & 0x7F) << 3) | ((me[4] >> 5) & 0x07);
        if (airspeed > 0) --airspeed;
        e->speed_kts = static_cast<uint16_t>(airspeed);
    }
}

AircraftEntry* AppAdsb::find_or_alloc(uint32_t icao)
{
    // Search existing
    for (int i = 0; i < MAX_AIRCRAFT; ++i) {
        if (aircraft_[i].valid && aircraft_[i].icao == icao)
            return &aircraft_[i];
    }
    // Allocate a free slot
    for (int i = 0; i < MAX_AIRCRAFT; ++i) {
        if (!aircraft_[i].valid) {
            memset(&aircraft_[i], 0, sizeof(AircraftEntry));
            aircraft_[i].icao  = icao;
            aircraft_[i].valid = true;
            ++ac_count_;
            return &aircraft_[i];
        }
    }
    // Table full — evict oldest
    int oldest = 0;
    uint32_t oldest_ms = aircraft_[0].last_seen_ms;
    for (int i = 1; i < MAX_AIRCRAFT; ++i) {
        if (aircraft_[i].last_seen_ms < oldest_ms) {
            oldest_ms = aircraft_[i].last_seen_ms;
            oldest = i;
        }
    }
    memset(&aircraft_[oldest], 0, sizeof(AircraftEntry));
    aircraft_[oldest].icao  = icao;
    aircraft_[oldest].valid = true;
    return &aircraft_[oldest];
}

void AppAdsb::expire_old(uint32_t now_ms)
{
    int count = 0;
    for (int i = 0; i < MAX_AIRCRAFT; ++i) {
        if (aircraft_[i].valid) {
            uint32_t age = now_ms - aircraft_[i].last_seen_ms;
            if (age > 60'000u) {
                aircraft_[i].valid = false;
            } else {
                ++count;
            }
        }
    }
    ac_count_ = count;
}

// Simplified CPR decoding (global, 1/2 NZ step)
// Reference: ICAO Doc 9684 / Tim Dwyer, "Supplementary techniques"
void AppAdsb::resolve_cpr(AircraftEntry& e)
{
    // NZ = 15 (60 latitude zones)
    constexpr float NZ = 15.0f;
    constexpr float PI = 3.14159265f;

    float lat_even = static_cast<float>(e.cpr_lat_even) / 131072.0f;
    float lat_odd  = static_cast<float>(e.cpr_lat_odd)  / 131072.0f;

    float dlat_even = 360.0f / (4.0f * NZ);
    float dlat_odd  = 360.0f / (4.0f * NZ - 1.0f);

    float j = floorf(59.0f * lat_even - 60.0f * lat_odd + 0.5f);

    float lat_e = dlat_even * (fmodf(j, 4.0f * NZ) + lat_even);
    float lat_o = dlat_odd  * (fmodf(j, 4.0f * NZ - 1.0f) + lat_odd);

    if (lat_e >= 270.0f) lat_e -= 360.0f;
    if (lat_o >= 270.0f) lat_o -= 360.0f;

    // Use most recent frame's latitude
    float lat = lat_e;  // use even (most common)
    (void)lat_o;

    // Longitude zone count NL(lat)
    float nl_lat  = (lat > -87.0f && lat < 87.0f) ?
        floorf(2.0f * PI / acosf(1.0f - (1.0f - cosf(PI / (2.0f * NZ))) /
                                         (cosf(PI / 180.0f * lat) * cosf(PI / 180.0f * lat)))) : 1.0f;
    if (nl_lat < 1.0f) nl_lat = 1.0f;

    float ni_even = nl_lat;
    float ni_odd  = (nl_lat > 1.0f) ? nl_lat - 1.0f : 1.0f;

    float lon_even = static_cast<float>(e.cpr_lon_even) / 131072.0f;
    float lon_odd  = static_cast<float>(e.cpr_lon_odd)  / 131072.0f;

    float m = floorf(lon_even * (ni_odd) - lon_odd * (ni_even) + 0.5f);

    float dlon_even = 360.0f / ni_even;

    float lon = dlon_even * (fmodf(m, ni_even) + lon_even);
    if (lon >= 180.0f) lon -= 360.0f;

    // Store as deg x1e6
    // (We don't store resolved lat/lon in AircraftEntry — it goes straight to the bus event)
    (void)lat;
    (void)lon;
    // lat/lon are used in publish_position below
    // Store temporarily in cpr fields as a hack-free approach:
    // We use two spare int32 fields via a reinterpret — instead store directly in the event
}

void AppAdsb::publish_position(const AircraftEntry& e)
{
    // Recompute lat/lon for the event (CPR resolve is done in resolve_cpr above;
    // we call resolve inline here for publish only)
    constexpr float NZ = 15.0f;
    constexpr float PI = 3.14159265f;

    float lat_even = static_cast<float>(e.cpr_lat_even) / 131072.0f;
    float lat_odd  = static_cast<float>(e.cpr_lat_odd)  / 131072.0f;
    float dlat_even = 360.0f / (4.0f * NZ);
    float dlat_odd  = 360.0f / (4.0f * NZ - 1.0f);
    float j = floorf(59.0f * lat_even - 60.0f * lat_odd + 0.5f);
    float lat_e = dlat_even * (fmodf(j, 4.0f * NZ) + lat_even);
    if (lat_e >= 270.0f) lat_e -= 360.0f;

    float nl_lat = 1.0f;
    if (lat_e > -87.0f && lat_e < 87.0f) {
        float cos_lat = cosf(PI / 180.0f * lat_e);
        float arg = 1.0f - (1.0f - cosf(PI / (2.0f * NZ))) / (cos_lat * cos_lat);
        if (arg > 0.0f && arg < 1.0f)
            nl_lat = floorf(2.0f * PI / acosf(arg));
        if (nl_lat < 1.0f) nl_lat = 1.0f;
    }

    float ni_even = nl_lat;
    float lon_even = static_cast<float>(e.cpr_lon_even) / 131072.0f;
    float lon_odd  = static_cast<float>(e.cpr_lon_odd)  / 131072.0f;
    float ni_odd = (nl_lat > 1.0f) ? nl_lat - 1.0f : 1.0f;
    float m = floorf(lon_even * ni_odd - lon_odd * ni_even + 0.5f);
    float dlon_even = 360.0f / ni_even;
    float lon = dlon_even * (fmodf(m, ni_even) + lon_even);
    if (lon >= 180.0f) lon -= 360.0f;

    BusEvent ev = BusEvent::make(EventType::AIRCRAFT_POSITION, task_id_);
    auto& p = ev.as<AircraftPositionPayload>();
    p.icao[0] = static_cast<uint8_t>((e.icao >> 16) & 0xFF);
    p.icao[1] = static_cast<uint8_t>((e.icao >>  8) & 0xFF);
    p.icao[2] = static_cast<uint8_t>( e.icao        & 0xFF);
    p.lat         = static_cast<int32_t>(lat_e * 1e6f);
    p.lon         = static_cast<int32_t>(lon   * 1e6f);
    p.altitude_ft = static_cast<int16_t>(e.alt_ft / 25);
    p.heading_deg = e.heading;
    p.speed_kts   = e.speed_kts;
    p.callsign_hash = e.callsign_hash;
    EventBus::instance().publish(ev);
}

// ---------------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------------

int AppAdsb::sorted_indices(int out[MAX_AIRCRAFT]) const
{
    int count = 0;
    for (int i = 0; i < MAX_AIRCRAFT; ++i) {
        if (aircraft_[i].valid) out[count++] = i;
    }

    // Simple insertion sort
    for (int i = 1; i < count; ++i) {
        int key = out[i];
        int j = i - 1;
        bool swap_cond = false;
        while (j >= 0) {
            switch (sort_mode_) {
                case AdsbSortMode::BY_ALT:
                    swap_cond = aircraft_[out[j]].alt_ft < aircraft_[key].alt_ft;
                    break;
                case AdsbSortMode::BY_RSSI:
                    swap_cond = aircraft_[out[j]].rssi_dbm_x10 < aircraft_[key].rssi_dbm_x10;
                    break;
                case AdsbSortMode::BY_SEEN:
                    swap_cond = aircraft_[out[j]].last_seen_ms < aircraft_[key].last_seen_ms;
                    break;
            }
            if (!swap_cond) break;
            out[j + 1] = out[j];
            --j;
        }
        out[j + 1] = key;
    }
    return count;
}

void AppAdsb::draw_table() const
{
    int sorted[MAX_AIRCRAFT];
    int count = sorted_indices(sorted);

    // Header
    ui::fill_rect(0, TABLE_Y, SCREEN_W, ROW_H, ui::colors::DARK_BLUE);
    ui::draw_text(2,   TABLE_Y + (ROW_H - FONT_H) / 2, "ICAO  ",  ui::colors::WHITE, ui::colors::DARK_BLUE);
    ui::draw_text(40,  TABLE_Y + (ROW_H - FONT_H) / 2, "Alt   ",  ui::colors::WHITE, ui::colors::DARK_BLUE);
    ui::draw_text(82,  TABLE_Y + (ROW_H - FONT_H) / 2, "Spd",     ui::colors::WHITE, ui::colors::DARK_BLUE);
    ui::draw_text(106, TABLE_Y + (ROW_H - FONT_H) / 2, "Hdg",     ui::colors::WHITE, ui::colors::DARK_BLUE);
    ui::draw_text(130, TABLE_Y + (ROW_H - FONT_H) / 2, "RSSI",    ui::colors::WHITE, ui::colors::DARK_BLUE);
    ui::draw_text(166, TABLE_Y + (ROW_H - FONT_H) / 2, "Callsign",ui::colors::WHITE, ui::colors::DARK_BLUE);

    // Data rows (up to TABLE_ROWS visible)
    for (int row = 0; row < TABLE_ROWS; ++row) {
        int list_idx = scroll_top_ + row;
        int screen_y = TABLE_Y + ROW_H + row * ROW_H;
        ui::Color row_bg = (row & 1) ? ui::colors::ROW_ODD : ui::colors::ROW_EVEN;

        ui::fill_rect(0, screen_y, SCREEN_W, ROW_H, row_bg);

        if (list_idx >= count) continue;

        const AircraftEntry& e = aircraft_[sorted[list_idx]];
        char buf[64];

        // ICAO
        snprintf(buf, sizeof(buf), "%06X", e.icao);
        ui::draw_text(2, screen_y + (ROW_H - FONT_H) / 2, buf, ui::colors::CYAN, row_bg);

        // Altitude
        snprintf(buf, sizeof(buf), "%6d", e.alt_ft);
        ui::draw_text(40, screen_y + (ROW_H - FONT_H) / 2, buf, ui::colors::WHITE, row_bg);

        // Speed
        snprintf(buf, sizeof(buf), "%3u", e.speed_kts);
        ui::draw_text(82, screen_y + (ROW_H - FONT_H) / 2, buf, ui::colors::WHITE, row_bg);

        // Heading
        snprintf(buf, sizeof(buf), "%3u", e.heading / 10);
        ui::draw_text(106, screen_y + (ROW_H - FONT_H) / 2, buf, ui::colors::WHITE, row_bg);

        // RSSI
        snprintf(buf, sizeof(buf), "%4d", e.rssi_dbm_x10 / 10);
        ui::draw_text(130, screen_y + (ROW_H - FONT_H) / 2, buf, ui::colors::GREEN, row_bg);

        // Callsign
        if (e.callsign[0]) {
            ui::draw_text(166, screen_y + (ROW_H - FONT_H) / 2,
                          e.callsign, ui::colors::YELLOW, row_bg);
        }
    }
}

void AppAdsb::draw_controls() const
{
    ctrl_list_.draw_all();
}

void AppAdsb::update_status()
{
    char lbuf[24], rbuf[24];
    snprintf(lbuf, sizeof(lbuf), "%d AC", ac_count_);
    snprintf(rbuf, sizeof(rbuf), "up:%us", uptime_s_);
    status_bar_.update(lbuf, "ADS-B 1090", rbuf);
}

uint16_t AppAdsb::crc16_callsign(const char* s)
{
    uint16_t crc = 0xFFFFu;
    while (*s) {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(*s++) << 8);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            else                crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

} // namespace sentinel
