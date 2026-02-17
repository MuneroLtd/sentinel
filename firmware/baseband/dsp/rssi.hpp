// SPDX-License-Identifier: MIT
// Project Sentinel — Cortex-M0 Baseband DSP Firmware
// dsp/rssi.hpp — Continuous RSSI measurement (runs in all operating modes)
//
// Every 10 ms, computes average magnitude over the last 8192 samples.
// Writes result to ipc::shared().rssi_dbm_x10, increments rssi_seq,
// sets M0_FLAG_RSSI, and calls notify_m4().

#pragma once

#include <cstdint>
#include <cstddef>

/// Initialise (or reset) the RSSI accumulator state.
void rssi_init();

/// Process one DMA buffer of IQ samples for RSSI measurement.
/// Accumulates samples internally; publishes when 8192 samples are collected
/// AND at least 10 ms has elapsed since the last update.
/// @param iq_buf   Pointer to interleaved int8_t I,Q pairs
/// @param n_pairs  Number of IQ pairs (DMA_BUF_SAMPLES = 2048)
void rssi_process(const int8_t* iq_buf, size_t n_pairs);

/// Tick function: call from main loop on WFE wake to flush any pending
/// RSSI measurement when in IDLE mode (no DSP pipeline running).
void rssi_tick();
