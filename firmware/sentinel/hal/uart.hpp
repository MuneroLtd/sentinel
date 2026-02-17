// SPDX-License-Identifier: MIT
// Project Sentinel — USART0 Debug UART Driver
//
// Blocking transmit only.  No receive buffering.
// Supports a minimal printf subset: %d, %u, %x, %X, %s, %c, %%.

#pragma once
#include <cstdint>
#include <cstdarg>

// Configure USART0 for 8N1 at the given baud rate.
// Call after clocks_init() since it reads the CPU clock.
void uart_init(uint32_t baud);

// Transmit a single character (blocks until THR is empty).
void uart_putc(char c);

// Transmit a null-terminated string.
void uart_puts(const char* s);

// Minimal printf. Supports: %d, %u, %x, %X, %s, %c, %%.
// Returns the number of characters written.
int uart_printf(const char* fmt, ...);

// Check if receive data is available (LSR RDR bit).
bool uart_rx_ready();

// Non-blocking read. Returns the received character, or -1 if no data.
int uart_getc_nb();
