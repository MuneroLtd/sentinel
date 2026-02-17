// SPDX-License-Identifier: MIT
// Project Sentinel — Minimal libc stubs for bare-metal M0
//
// Provides memset, memcpy, and memmove so we can link with -nostdlib
// without pulling in the full C library.

#include <stddef.h>
#include <stdint.h>

void* memset(void* dst, int val, size_t n)
{
    uint8_t* d = (uint8_t*)dst;
    while (n--) *d++ = (uint8_t)val;
    return dst;
}

void* memcpy(void* dst, const void* src, size_t n)
{
    uint8_t*       d = (uint8_t*)dst;
    const uint8_t* s = (const uint8_t*)src;
    while (n--) *d++ = *s++;
    return dst;
}

void* memmove(void* dst, const void* src, size_t n)
{
    uint8_t*       d = (uint8_t*)dst;
    const uint8_t* s = (const uint8_t*)src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n;
        s += n;
        while (n--) *--d = *--s;
    }
    return dst;
}
