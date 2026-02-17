// SPDX-License-Identifier: MIT
// Project Sentinel — ESP32 Bridge I2C Driver
//
// Clean driver for the ESP32 co-processor using the Project Sentinel HAL.
// Does NOT use portapack:: namespace or any Mayhem firmware code.
//
// All functions are safe to call from any FreeRTOS task.  They are NOT
// ISR-safe (I2C transfers block in a polling state machine).
//
// I2C bus and address are defined in bsp/portapack_pins.hpp:
//   ESP32_I2C_BUS  = 1 (I2C1)
//   ESP32_I2C_ADDR = 0x42

#pragma once

#include <cstdint>
#include <cstdbool>

#include "../event_bus/event_types.hpp"   // GPSUpdatePayload, SensorDataPayload, BusEvent

namespace sentinel::esp32 {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Initialise the I2C bus and verify the ESP32 is responding.
// Returns true if the ESP32 acknowledges its address.
// Must be called once before any other function in this namespace.
bool init(void);

// ---------------------------------------------------------------------------
// Sensor polling
// ---------------------------------------------------------------------------

// Query the ESP32 for the latest GPS fix (CMD 0x01).
// Fills 'out' and returns true if the ESP32 returned a valid fix (fix>0).
// Returns false on I2C error or if no GPS fix is available.
bool get_gps(GPSUpdatePayload& out);

// Query the ESP32 for the latest BME280 sensor reading (CMD 0x02).
// Fills 'out' and returns true on success.
bool get_sensors(SensorDataPayload& out);

// ---------------------------------------------------------------------------
// Event forwarding
// ---------------------------------------------------------------------------

// Forward a 32-byte BusEvent to the ESP32 for MQTT relay (CMD 0x05).
// Returns true on success.
bool send_event(const BusEvent& ev);

// ---------------------------------------------------------------------------
// Network configuration
// ---------------------------------------------------------------------------

// Configure the ESP32 Wi-Fi credentials.
// ssid and password are null-terminated strings (max 31 chars each).
// Returns true if the configuration was accepted.
bool set_wifi(const char* ssid, const char* password);

// Configure the MQTT broker.
// broker_ip is a packed big-endian IPv4 address.
// Returns true if the configuration was accepted.
bool set_mqtt(uint32_t broker_ip, uint16_t port);

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

// Read the ESP32 status word (CMD 0x03).
// Returns true and fills out the fields on success.
bool get_status(bool* wifi_ok, bool* mqtt_ok, uint16_t* uptime_s);

} // namespace sentinel::esp32
