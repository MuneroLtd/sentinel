/* Minimal linker script for LPC4320 blinky test */
OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(blinky_main)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 1024K
    SRAM0 (rwx) : ORIGIN = 0x10000000, LENGTH = 96K
}

_stack_top = ORIGIN(SRAM0) + LENGTH(SRAM0);  /* 0x10018000 */

SECTIONS
{
    .vectors :
    {
        KEEP(*(.vectors))
    } > FLASH

    .text :
    {
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > FLASH
}
