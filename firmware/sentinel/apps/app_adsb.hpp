// SPDX-License-Identifier: MIT
// Project Sentinel - ADS-B receiver application
//
// Layout (320x240):
//   Row   0-15  : StatusBar (count | "ADS-B 1090" | uptime)
//   Row  16-207 : Aircraft list table (max 10 visible rows, 19px each)
//   Row 208-239 : [Sort:Alt] [Sort:RSSI] [Clear] [Home]
#pragma once

#include "sentinel_app_base.hpp"
#include "../ui/widgets.hpp"
#include <cstdint>

namespace sentinel {

// Per-aircraft tracking entry
struct AircraftEntry {
    uint32_t icao;           // 24-bit ICAO address
    int32_t  alt_ft;         // altitude in feet (from CPR/barometric)
    uint16_t speed_kts;      // speed in knots
    uint16_t heading;        // heading degrees x10
    int16_t  rssi_dbm_x10;  // last RSSI
    uint32_t last_seen_ms;   // tick at last update
    uint16_t callsign_hash;  // CRC16 of callsign
    char     callsign[9];    // up to 8 chars + NUL
    // CPR state for position resolution
    uint32_t cpr_lat_even;
    uint32_t cpr_lon_even;
    uint32_t cpr_lat_odd;
    uint32_t cpr_lon_odd;
    bool     cpr_even_valid;
    bool     cpr_odd_valid;
    bool     valid;          // entry in use
};

enum class AdsbSortMode : uint8_t {
    BY_ALT   = 0,
    BY_RSSI  = 1,
    BY_SEEN  = 2,
};

class AppAdsb : public AppBase {
public:
    static constexpr int MAX_AIRCRAFT = 32;
    static constexpr int TABLE_ROWS   = 10;
    static constexpr int ROW_H        = 19;

    AppAdsb();

    void on_enter() override;
    void on_exit()  override;
    void on_paint() override;
    void on_nav(ui::NavKey k) override;
    void on_tick()  override;

protected:
    void on_bus_event(const BusEvent& ev) override;

private:
    static constexpr int TABLE_Y  = 16;
    static constexpr int CTRL_Y   = 208;
    static constexpr int CTRL_H   = 32;
    static constexpr int SCREEN_W = 320;

    AircraftEntry aircraft_[MAX_AIRCRAFT]{};
    int           ac_count_{0};         // occupied entries

    AdsbSortMode  sort_mode_{AdsbSortMode::BY_ALT};
    int           scroll_top_{0};       // first visible row index
    uint32_t      uptime_s_{0};

    ui::StatusBar status_bar_{};

    static constexpr int CTRL_BTN_COUNT = 4;
    ui::Button     ctrl_btns_[CTRL_BTN_COUNT]{};
    ui::ButtonList ctrl_list_{ctrl_btns_, CTRL_BTN_COUNT, 0};

    // --- ADS-B parsing ---
    bool  verify_crc24(const uint8_t* frame, int len);
    void  parse_frame(const ipc::AdsbFrame& f);
    void  decode_identification(uint32_t icao, const uint8_t* me, int16_t rssi);
    void  decode_position(uint32_t icao, const uint8_t* me, bool is_odd, uint32_t ts_ms, int16_t rssi);
    void  decode_velocity(uint32_t icao, const uint8_t* me, int16_t rssi);
    void  decode_altitude(uint32_t icao, int raw_alt, int16_t rssi, uint32_t ts_ms);
    AircraftEntry* find_or_alloc(uint32_t icao);
    void  expire_old(uint32_t now_ms);
    void  resolve_cpr(AircraftEntry& e);
    void  publish_position(const AircraftEntry& e);

    // --- Display ---
    void draw_table() const;
    void draw_controls() const;
    void update_status();
    int  sorted_indices(int out[MAX_AIRCRAFT]) const;

    static uint16_t crc16_callsign(const char* s);
};

} // namespace sentinel
