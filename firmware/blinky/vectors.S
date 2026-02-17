    .syntax unified
    .arch   armv7e-m
    .thumb

    .section .vectors, "a", %progbits
    .align 2
    .globl _vectors
_vectors:
    .word   _stack_top
    .word   blinky_main
    .word   blinky_main
    .word   blinky_main
    .word   blinky_main
    .word   blinky_main
    .word   blinky_main
    .word   0               /* checksum placeholder */
