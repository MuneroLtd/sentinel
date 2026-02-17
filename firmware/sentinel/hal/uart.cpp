// SPDX-License-Identifier: MIT
// Project Sentinel — USART0 Debug UART Driver Implementation
//
// Baud rate calculation:
//   The LPC4320 USART has a fractional baud rate generator.
//   DLM:DLL sets the integer divisor; FDR sets MULVAL and DIVADDVAL.
//   BAUD = PCLK / (16 × DL × (1 + DIVADDVAL/MULVAL))
//
//   For simplicity we use the fractional divider to get close.
//   We iterate over MULVAL (1..15) and DIVADDVAL (0..MULVAL-1) and find the
//   combination that gives the smallest baud-rate error.

#include "uart.hpp"
#include "lpc4320_regs.hpp"
#include "clocks.hpp"
#include <cstdarg>
#include <cstring>

// ---------------------------------------------------------------------------
// uart_init
// ---------------------------------------------------------------------------
void uart_init(uint32_t baud) {
    UART_Type* uart = LPC_USART0;

    const uint32_t pclk = clocks_get_cpu_hz(); // UART0 clocked from BASE_UART0_CLK = PLL1

    // Find best DL + fractional divider combination
    // BAUD = PCLK / (16 × DL × (1 + DIVADDVAL/MULVAL))
    uint32_t best_dl   = 1;
    uint32_t best_mul  = 1;
    uint32_t best_div  = 0;
    uint32_t best_err  = 0xFFFFFFFFu;

    for (uint32_t mul = 1; mul <= 15; mul++) {
        for (uint32_t dv = 0; dv < mul; dv++) {
            // DL = PCLK / (16 * baud * (1 + dv/mul))
            //    = PCLK * mul / (16 * baud * (mul + dv))
            uint32_t numerator   = pclk * mul;
            uint32_t denominator = 16u * baud * (mul + dv);
            uint32_t dl = (numerator + denominator / 2) / denominator;
            if (dl == 0) dl = 1;

            // Actual baud
            uint32_t actual_baud = pclk * mul / (16u * dl * (mul + dv));
            uint32_t err = (actual_baud > baud) ? (actual_baud - baud) : (baud - actual_baud);

            if (err < best_err) {
                best_err = err;
                best_dl  = dl;
                best_mul = mul;
                best_div = dv;
            }
        }
    }

    // Enable DLAB to set divisor
    uart->LCR = UART_LCR_DLAB | UART_LCR_WORD_8BIT;
    uart->DLL = best_dl & 0xFFu;
    uart->DLM = (best_dl >> 8) & 0xFFu;

    // Clear DLAB, set 8N1
    uart->LCR = UART_LCR_WORD_8BIT;

    // Fractional divider: MULVAL[7:4], DIVADDVAL[3:0]
    uart->FDR = (best_mul << 4) | best_div;

    // Enable and reset FIFOs (RX + TX) with 1-byte trigger
    uart->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_RST | UART_FCR_TX_RST;

    // No modem flow control
    uart->MCR = 0;

    // Transmit enable
    uart->TER = (1u << 7);
}

// ---------------------------------------------------------------------------
// uart_putc
// ---------------------------------------------------------------------------
void uart_putc(char c) {
    UART_Type* uart = LPC_USART0;
    // Wait for Transmitter Holding Register Empty
    while (!(uart->LSR & UART_LSR_THRE)) { }
    uart->THR = static_cast<uint32_t>(static_cast<unsigned char>(c));
}

// ---------------------------------------------------------------------------
// uart_puts
// ---------------------------------------------------------------------------
void uart_puts(const char* s) {
    while (*s) {
        uart_putc(*s++);
    }
}

// ---------------------------------------------------------------------------
// Minimal printf helpers
// ---------------------------------------------------------------------------

static int uart_print_uint(uint32_t val, int base, bool uppercase, int min_width) {
    static const char digits_lower[] = "0123456789abcdef";
    static const char digits_upper[] = "0123456789ABCDEF";
    const char* digits = uppercase ? digits_upper : digits_lower;

    char buf[16];
    int  idx = 0;
    int  count = 0;

    if (val == 0) {
        buf[idx++] = '0';
    } else {
        while (val > 0) {
            buf[idx++] = digits[val % static_cast<uint32_t>(base)];
            val /= static_cast<uint32_t>(base);
        }
    }

    // Pad with spaces (no zero-padding support needed here)
    int len = idx;
    while (len < min_width) {
        uart_putc(' ');
        count++;
        len++;
    }

    // Reverse
    for (int i = idx - 1; i >= 0; i--) {
        uart_putc(buf[i]);
        count++;
    }
    return count;
}

static int uart_print_int(int32_t val, int min_width) {
    int count = 0;
    if (val < 0) {
        uart_putc('-');
        count++;
        val = -val;
        min_width--;
    }
    count += uart_print_uint(static_cast<uint32_t>(val), 10, false, min_width);
    return count;
}

// ---------------------------------------------------------------------------
// uart_printf
// ---------------------------------------------------------------------------
int uart_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    int count = 0;

    while (*fmt) {
        if (*fmt != '%') {
            uart_putc(*fmt++);
            count++;
            continue;
        }
        fmt++; // skip '%'

        // Optional minimum width
        int width = 0;
        while (*fmt >= '0' && *fmt <= '9') {
            width = width * 10 + (*fmt - '0');
            fmt++;
        }

        switch (*fmt) {
            case 'd': {
                int32_t v = va_arg(args, int);
                count += uart_print_int(v, width);
                break;
            }
            case 'u': {
                uint32_t v = va_arg(args, uint32_t);
                count += uart_print_uint(v, 10, false, width);
                break;
            }
            case 'x': {
                uint32_t v = va_arg(args, uint32_t);
                count += uart_print_uint(v, 16, false, width);
                break;
            }
            case 'X': {
                uint32_t v = va_arg(args, uint32_t);
                count += uart_print_uint(v, 16, true, width);
                break;
            }
            case 's': {
                const char* s = va_arg(args, const char*);
                if (!s) s = "(null)";
                int slen = 0;
                while (s[slen]) slen++;
                while (slen < width) { uart_putc(' '); count++; slen++; }
                while (*s) { uart_putc(*s++); count++; }
                break;
            }
            case 'c': {
                char c = static_cast<char>(va_arg(args, int));
                uart_putc(c);
                count++;
                break;
            }
            case '%': {
                uart_putc('%');
                count++;
                break;
            }
            case '\0':
                goto done;
            default:
                uart_putc('%');
                uart_putc(*fmt);
                count += 2;
                break;
        }
        fmt++;
    }

done:
    va_end(args);
    return count;
}
