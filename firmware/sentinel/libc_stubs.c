// SPDX-License-Identifier: MIT
// Minimal C runtime stubs for -nostdlib bare-metal M4 firmware

#include <stddef.h>
#include <stdint.h>

void* memset(void* s, int c, size_t n) {
    unsigned char* p = (unsigned char*)s;
    while (n--) *p++ = (unsigned char)c;
    return s;
}

void* memcpy(void* dest, const void* src, size_t n) {
    unsigned char* d = (unsigned char*)dest;
    const unsigned char* s = (const unsigned char*)src;
    while (n--) *d++ = *s++;
    return dest;
}

void* memmove(void* dest, const void* src, size_t n) {
    unsigned char* d = (unsigned char*)dest;
    const unsigned char* s = (const unsigned char*)src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n; s += n;
        while (n--) *--d = *--s;
    }
    return dest;
}

int memcmp(const void* s1, const void* s2, size_t n) {
    const unsigned char* a = (const unsigned char*)s1;
    const unsigned char* b = (const unsigned char*)s2;
    while (n--) {
        if (*a != *b) return *a - *b;
        a++; b++;
    }
    return 0;
}

// FatFS requires get_fattime() for timestamping files
// Returns a packed DWORD: bits[31:25]=year-1980, [24:21]=month, [20:16]=day,
//                         [15:11]=hour, [10:5]=min, [4:0]=sec/2
// Default: 2025-01-01 00:00:00
uint32_t get_fattime(void) {
    return ((uint32_t)(2025 - 1980) << 25)
         | ((uint32_t)1 << 21)
         | ((uint32_t)1 << 16);
}

// C++ ABI requires __dso_handle for static destructors (which we never run)
void* __dso_handle = 0;

// __cxa_atexit — register static destructor (no-op in bare metal)
int __cxa_atexit(void (*destructor)(void*), void* arg, void* dso) {
    (void)destructor; (void)arg; (void)dso;
    return 0;
}
