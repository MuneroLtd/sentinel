/* SPDX-License-Identifier: MIT */
/* Project Sentinel — LPC4320 Cortex-M4 Linker Script                        */
/*                                                                             */
/* Memory map (LPC4320 UM10503):                                               */
/*   Flash (SPI NOR via SPIFI, memory-mapped): 0x00000000 – 0x000FFFFF  1 MB  */
/*   M4 SRAM0 (local bus):                     0x10000000 – 0x1001FFFF 128 KB */
/*   AHB SRAM (M0 DMA buffers):                0x20000000 – 0x20003FFF  16 KB */
/*   AHB SRAM (M4 heap/statics):               0x20004000 – 0x20007FFF  16 KB */
/*   M0 SRAM (reserved):                       0x10080000 – 0x10087FFF   8 KB */
/*   Shared SRAM (IPC, reserved):              0x10088000 – 0x10089FFF   8 KB */
/*                                                                             */
/* The M4 stack lives at the top of SRAM0.                                    */
/* The M0 binary is linked into Flash (.m0_binary section).  The startup      */
/* code copies it to 0x10080000 before releasing the M0 from reset.           */
/* The shared IPC region (0x10088000) is NOT described here; it is accessed   */
/* only through the sentinel::ipc::shared() accessor.                         */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)

MEMORY
{
    /* SPI-mapped Flash (execute-in-place via SPIFI) */
    FLASH  (rx)  : ORIGIN = 0x00000000, LENGTH = 1024K

    /* Cortex-M4 local SRAM0: stack + data + BSS + FreeRTOS heap */
    SRAM0  (rwx) : ORIGIN = 0x10000000, LENGTH = 128K

    /* AHB SRAM bank (upper 16 KB): use for FreeRTOS heap and large static arrays.
     * Lower 16 KB (0x20000000–0x20003FFF) is reserved for M0 DMA ping-pong
     * buffers (ETB_SRAM in m0.ld). */
    AHBRAM (rwx) : ORIGIN = 0x20004000, LENGTH = 16K

    /* M0 SRAM — reserved; M4 startup copies baseband binary here */
    /* Not listed as allocatable; the startup routine writes it raw. */

    /* Shared IPC SRAM — reserved; accessed via pointer, not by linker */
    /* Not listed as allocatable. */
}

/* Stack size: 4 KB at the top of SRAM0 */
_stack_size = 0x1000;
_stack_top  = ORIGIN(SRAM0) + LENGTH(SRAM0);  /* 0x10020000 */

SECTIONS
{
    /* ------------------------------------------------------------------ */
    /* Interrupt vector table — must be first in Flash at 0x00000000      */
    /* ------------------------------------------------------------------ */
    .vectors :
    {
        KEEP(*(.vectors))
    } > FLASH

    /* ------------------------------------------------------------------ */
    /* Text: code and read-only data                                       */
    /* ------------------------------------------------------------------ */
    .text :
    {
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.glue_7 .glue_7t)
        *(.gcc_except_table)
        . = ALIGN(4);
        KEEP(*(.init))
        KEEP(*(.fini))
    } > FLASH

    /* ARM exception unwind tables */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    /* C++ static constructor/destructor tables */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* ------------------------------------------------------------------ */
    /* M0 baseband binary blob (linked in from baseband.bin)              */
    /* The startup routine copies _m0_binary_start .. _m0_binary_end      */
    /* to 0x10080000 (M0 SRAM) before releasing the M0 from reset.        */
    /* ------------------------------------------------------------------ */
    .m0_binary :
    {
        . = ALIGN(4);
        _m0_binary_start = .;
        KEEP(*(.m0_binary))
        /* When linked with -b binary the object has one section named    */
        /* .data; objcopy renames it to the basename. We catch both.      */
        KEEP(*(SORT_NONE(.data)))
        . = ALIGN(4);
        _m0_binary_end = .;
    } > FLASH

    _m0_binary_size = _m0_binary_end - _m0_binary_start;

    /* ------------------------------------------------------------------ */
    /* Initialised data (ROM image → SRAM0)                               */
    /* ------------------------------------------------------------------ */
    _data_load = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _data_start = .;
        *(.data .data.*)
        . = ALIGN(4);
        _data_end = .;
    } > SRAM0 AT > FLASH

    /* ------------------------------------------------------------------ */
    /* BSS (zero-initialised)                                              */
    /* ------------------------------------------------------------------ */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > SRAM0

    /* ------------------------------------------------------------------ */
    /* FreeRTOS heap (heap_4 uses a static array in BSS; no extra section  */
    /* needed here — the array is covered by .bss above).                  */
    /* ------------------------------------------------------------------ */

    /* ------------------------------------------------------------------ */
    /* Stack: grows downward from _stack_top                              */
    /* The FreeRTOS port allocates task stacks from the heap or static    */
    /* arrays; this is only the initial MSP stack used before the         */
    /* scheduler starts.                                                   */
    /* ------------------------------------------------------------------ */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _stack_bottom = .;
        . = . + _stack_size;
        . = ALIGN(8);
    } > SRAM0

    /* ------------------------------------------------------------------ */
    /* AHB SRAM: DMA-friendly buffers (not zero-initialised automatically) */
    /* Place objects here with __attribute__((section(".ahbram")))         */
    /* ------------------------------------------------------------------ */
    .ahbram (NOLOAD) :
    {
        . = ALIGN(4);
        _ahbram_start = .;
        *(.ahbram .ahbram.*)
        . = ALIGN(4);
        _ahbram_end = .;
    } > AHBRAM

    /* ------------------------------------------------------------------ */
    /* Discard C++ exception handling sections (not used: -fno-exceptions) */
    /* ------------------------------------------------------------------ */
    /DISCARD/ :
    {
        *(.ARM.attributes)
        *(.note.*)
        *(.comment)
        *(.eh_frame)
    }
}

/* Expose symbols used by startup code */
PROVIDE(_etext = LOADADDR(.data) + SIZEOF(.data));
PROVIDE(_end   = _bss_end);
PROVIDE(end    = _bss_end);

/* Sanity: ensure stack does not overlap BSS */
ASSERT(_stack_bottom >= _bss_end,
       "Stack and BSS overlap — increase SRAM0 or reduce stack size")

/* Ensure the IPC shared region is never mapped by the linker */
ASSERT(ORIGIN(SRAM0) + LENGTH(SRAM0) <= 0x10080000,
       "SRAM0 region must not overlap M0 SRAM at 0x10080000")
