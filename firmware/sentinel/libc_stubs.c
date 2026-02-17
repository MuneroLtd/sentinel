// SPDX-License-Identifier: MIT
// Minimal C runtime stubs for -nostdlib bare-metal M4 firmware
//
// CRITICAL: GCC at -O2 recognises byte-loop patterns and replaces them with
// calls to memset/memcpy/memmove/memcmp — even inside the implementations
// themselves, creating infinite recursion that overflows the stack.
// Compile this file with -fno-builtin (set in CMakeLists.txt) or use the
// inhibit_loop_to_libcall pragma to prevent this.

#include <stddef.h>
#include <stdint.h>
#include <errno.h>

#pragma GCC optimize("-fno-tree-loop-distribute-patterns")

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

// -------------------------------------------------------------------------
// C++ ABI stubs — bare-metal firmware never exits, so destructor
// registration is a no-op.  We provide BOTH __cxa_atexit (Itanium ABI)
// and __aeabi_atexit (ARM EABI) so the linker never pulls newlib's
// versions (which internally call _malloc_r and crash without _sbrk).
// -------------------------------------------------------------------------
void* __dso_handle = 0;

int __cxa_atexit(void (*destructor)(void*), void* arg, void* dso) {
    (void)destructor; (void)arg; (void)dso;
    return 0;
}

// ARM EABI variant — compiler-generated init_array wrappers call this.
// Argument order differs from __cxa_atexit: (obj, dtor, dso) vs (dtor, obj, dso).
int __aeabi_atexit(void* obj, void (*dtor)(void*), void* dso) {
    (void)obj; (void)dtor; (void)dso;
    return 0;
}

// -------------------------------------------------------------------------
// _sbrk — newlib heap allocator back-end
//
// newlib-nano's snprintf, sscanf, and other stdio functions call _malloc_r
// internally (e.g. to initialise the _REENT structure on first use).
// Without a working _sbrk, malloc returns NULL and newlib dereferences it
// → HardFault.
//
// The heap grows upward from _end (end of BSS) toward _heap_limit
// (defined in the linker script as _stack_top - 2KB guard).
// -------------------------------------------------------------------------
extern char _end;          /* linker symbol: end of BSS */
extern char _heap_limit;   /* linker symbol: max heap address */

static char* heap_ptr = 0;

void* _sbrk(int incr) {
    if (heap_ptr == 0) {
        heap_ptr = &_end;
    }

    char* prev = heap_ptr;
    char* new_ptr = heap_ptr + incr;

    if (new_ptr >= &_heap_limit) {
        errno = ENOMEM;
        return (void*)-1;
    }

    heap_ptr = new_ptr;
    return prev;
}

// _sbrk_r — reentrant wrapper that newlib calls instead of _sbrk directly
struct _reent;
void* _sbrk_r(struct _reent* r, int incr) {
    (void)r;
    return _sbrk(incr);
}
