/* SPDX-License-Identifier: MIT */
/* Project Sentinel — Embedded CPLD XSVF bitstream                            */
/*                                                                             */
/* Links the XC2C64A sgpio_if XSVF image into .rodata so cpld_init() can      */
/* program the CPLD at boot via the JTAG bit-bang routines in cpld.cpp.        */

    .syntax unified
    .section .rodata, "a", %progbits
    .align  2

    .global cpld_xsvf_data
    .type   cpld_xsvf_data, %object
cpld_xsvf_data:
    .incbin "radio/sgpio_if/default.xsvf"
    .size   cpld_xsvf_data, . - cpld_xsvf_data

    .global cpld_xsvf_length
    .type   cpld_xsvf_length, %object
cpld_xsvf_length:
    .word   cpld_xsvf_length - cpld_xsvf_data
    .size   cpld_xsvf_length, 4

    .end
