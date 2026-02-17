/* SPDX-License-Identifier: MIT
 * Project Sentinel — Cortex-M0 Baseband DSP Firmware
 * m0.ld — Linker script for M0 sub-core
 *
 * The LPC4320 M0 sub-core runs entirely from SRAM — there is no separate
 * M0 flash.  M4 loads the M0 image into SRAM at boot before releasing M0
 * from reset.
 *
 * Memory layout for M0 (LPC4320 SRAM0, 8 KB):
 *   0x10080000 ──► code + rodata + data  (6 KB maximum)
 *   0x10081800 ──► stack base  (2 KB stack grows downward from 0x10082000)
 *   0x10082000 ──► top of M0 SRAM / initial stack pointer
 *
 * The shared IPC memory lives at 0x10088000 (SRAM1 upper 8 KB) and is NOT
 * part of this linker script — it is accessed by address via the ipc_protocol.hpp
 * accessor functions.
 *
 * DMA ping-pong buffers (g_dma_buf) are placed in .bss here (SRAM0).
 * They total 2 × 4096 = 8192 bytes — this exceeds the 6 KB code area!
 *
 * Resolution: DMA buffers are placed in a separate SRAM region.
 * LPC4320 also has SRAM_ETB at 0x20000000 (16 KB), accessible by M0.
 * We put the 8 KB DMA buffers there, keeping M0 SRAM for code and stack.
 *
 * Final layout:
 *   0x10080000  M0_SRAM (8 KB): vector table, code, rodata, .data, .bss (non-DMA)
 *   0x10082000  top of stack (stack = 2 KB, 0x10081800..0x10082000)
 *   0x20000000  ETB_SRAM (16 KB): DMA buffers (8 KB), free remainder
 */

/* ---------------------------------------------------------------------------
 * Memory regions
 * --------------------------------------------------------------------------- */

MEMORY
{
    /* M0 SRAM: code + data + stack */
    M0_SRAM  (rwx) : ORIGIN = 0x10080000, LENGTH = 8K

    /* LPC4320 ETB SRAM: for large DMA buffers (accessed by M0 and GPDMA) */
    ETB_SRAM (rw)  : ORIGIN = 0x20000000, LENGTH = 16K
}

/* ---------------------------------------------------------------------------
 * Entry point
 * --------------------------------------------------------------------------- */

ENTRY(m0_main)

/* ---------------------------------------------------------------------------
 * Sections
 * --------------------------------------------------------------------------- */

SECTIONS
{
    /* -----------------------------------------------------------------------
     * Vector table — MUST be at the very start of M0 SRAM.
     * CREG_M0APPMEMMAP is set by M4 to 0x10080000 before releasing M0.
     * The first word is the initial SP; the second is the reset handler.
     * ----------------------------------------------------------------------- */
    .vectors :
    {
        KEEP(*(.vectors))
        KEEP(*(.isr_vector))
    } > M0_SRAM

    /* -----------------------------------------------------------------------
     * Code and read-only data
     * ----------------------------------------------------------------------- */
    .text :
    {
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)

        /* CMSIS-DSP twiddle/bitrev tables (read-only) */
        *(.constdata)
        *(.constdata.*)

        . = ALIGN(4);
        _etext = .;
    } > M0_SRAM

    /* -----------------------------------------------------------------------
     * Initialised data — copied from flash (here: from the binary image)
     * to SRAM at startup.  For M0 SRAM-only execution, M4 copies the entire
     * image, so .data LMA = VMA (both in M0_SRAM).
     * ----------------------------------------------------------------------- */
    .data :
    {
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > M0_SRAM

    /* -----------------------------------------------------------------------
     * Uninitialised data — zeroed by CRT or startup code.
     * Excludes DMA buffers (placed separately in ETB_SRAM below).
     * ----------------------------------------------------------------------- */
    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > M0_SRAM

    /* -----------------------------------------------------------------------
     * Stack — top of M0_SRAM.
     * Initial SP is loaded from word 0 of the vector table.
     * We use a stack guard symbol here for overflow detection if desired.
     * ----------------------------------------------------------------------- */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _stack_bottom = .;
        . += 0x800;  /* 2 KB stack */
        . = ALIGN(8);
        _stack_top = .;
    } > M0_SRAM

    /* -----------------------------------------------------------------------
     * DMA ping-pong buffers in ETB SRAM.
     * g_dma_buf[0] and g_dma_buf[1] are 4096 bytes each (8192 total).
     * GPDMA can access ETB SRAM via the AHB matrix on LPC4320.
     * ----------------------------------------------------------------------- */
    .dma_buffers (NOLOAD) :
    {
        . = ALIGN(4);
        _sdma = .;
        *(.dma_buffers)
        *(.dma_buffers.*)
        /* If the DMA buffers are in .bss (no section attribute used), they
         * land in the .bss above.  To force them here, declare with:
         *   __attribute__((section(".dma_buffers")))
         * See main_m0.cpp where g_dma_buf is declared. */
        . = ALIGN(4);
        _edma = .;
    } > ETB_SRAM

    /* -----------------------------------------------------------------------
     * Debug sections — discarded from final image
     * ----------------------------------------------------------------------- */
    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.ARM.attributes)
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }

    /* -----------------------------------------------------------------------
     * DWARF debug info (preserved in ELF for debugging, stripped from .bin)
     * ----------------------------------------------------------------------- */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_ranges   0 : { *(.debug_ranges) }

    /* -----------------------------------------------------------------------
     * Sanity check: ensure we have not overflowed M0 SRAM.
     * Stack top must equal M0_SRAM end (0x10082000).
     * ----------------------------------------------------------------------- */
    ASSERT(_stack_top <= 0x10082000,
           "ERROR: M0 code + data + stack exceeds 8 KB SRAM!")

    ASSERT(SIZEOF(.text) + SIZEOF(.data) + SIZEOF(.bss) <= 6144,
           "ERROR: M0 code + data exceeds 6 KB (leaving too little for stack!)")
}

/* Provide symbols expected by minimal CRT / startup (if used) */
PROVIDE(_end    = _ebss);
PROVIDE(end     = _ebss);
