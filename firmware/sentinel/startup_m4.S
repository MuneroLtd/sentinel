/* SPDX-License-Identifier: MIT */
/* Project Sentinel — Cortex-M4F Startup for LPC4320                          */
/*                                                                             */
/* Provides:                                                                   */
/*   - Vector table (.vectors section, placed at 0x00000000 by linker script)  */
/*   - Reset_Handler: .data copy, .bss zero, C++ constructors, branch to main  */
/*   - Default_Handler: infinite loop for unhandled interrupts                 */
/*                                                                             */
/* FreeRTOS (GCC ARM_CM4F port) requires these exact symbol names for the      */
/* SVCall, PendSV, and SysTick vectors — the port asserts them at runtime.     */

    .syntax unified
    .arch   armv7e-m
    .fpu    fpv4-sp-d16
    .thumb

/* ========================================================================= */
/* Vector Table                                                               */
/* ========================================================================= */
    .section .vectors, "a", %progbits
    .align   2
    .globl   _vectors
_vectors:
    /* Exception  #  Offset  Description                                     */
    .word   _stack_top              /*  0  0x000  Initial MSP                 */
    .word   Reset_Handler           /*  1  0x004  Reset                       */
    .word   NMI_Handler             /*  2  0x008  NMI                         */
    .word   HardFault_Handler       /*  3  0x00C  Hard Fault                  */
    .word   MemManage_Handler       /*  4  0x010  MPU Fault                   */
    .word   BusFault_Handler        /*  5  0x014  Bus Fault                   */
    .word   UsageFault_Handler      /*  6  0x018  Usage Fault                 */
    .word   0                       /*  7  0x01C  Boot checksum (patched post-build) */
    .word   0                       /*  8  0x020  Reserved                    */
    .word   0                       /*  9  0x024  Reserved                    */
    .word   0                       /* 10  0x028  Reserved                    */
    .word   vPortSVCHandler         /* 11  0x02C  SVCall (FreeRTOS)           */
    .word   DebugMon_Handler        /* 12  0x030  Debug Monitor               */
    .word   0                       /* 13  0x034  Reserved                    */
    .word   xPortPendSVHandler      /* 14  0x038  PendSV (FreeRTOS)          */
    .word   xPortSysTickHandler     /* 15  0x03C  SysTick (FreeRTOS)         */

    /* LPC4320 peripheral IRQs (IRQ0..IRQ36 = 37 vectors) */
    .word   DAC_IRQHandler          /* 16  IRQ 0  DAC                        */
    .word   M0APP_IRQHandler        /* 17  IRQ 1  M0APP                      */
    .word   DMA_IRQHandler          /* 18  IRQ 2  DMA                        */
    .word   0                       /* 19  IRQ 3  Reserved                   */
    .word   FLASH_EEPROM_IRQHandler /* 20  IRQ 4  Flash/EEPROM               */
    .word   ETH_IRQHandler          /* 21  IRQ 5  Ethernet                   */
    .word   SDIO_IRQHandler         /* 22  IRQ 6  SDIO                       */
    .word   LCD_IRQHandler          /* 23  IRQ 7  LCD                        */
    .word   USB0_IRQHandler         /* 24  IRQ 8  USB0                       */
    .word   USB1_IRQHandler         /* 25  IRQ 9  USB1                       */
    .word   SCT_IRQHandler          /* 26  IRQ10  SCT                        */
    .word   RIT_IRQHandler          /* 27  IRQ11  RIT / WWDT                 */
    .word   TIMER0_IRQHandler       /* 28  IRQ12  Timer 0                    */
    .word   TIMER1_IRQHandler       /* 29  IRQ13  Timer 1                    */
    .word   TIMER2_IRQHandler       /* 30  IRQ14  Timer 2                    */
    .word   TIMER3_IRQHandler       /* 31  IRQ15  Timer 3                    */
    .word   MCPWM_IRQHandler        /* 32  IRQ16  Motor Control PWM          */
    .word   ADC0_IRQHandler         /* 33  IRQ17  ADC0                       */
    .word   I2C0_IRQHandler         /* 34  IRQ18  I2C0                       */
    .word   I2C1_IRQHandler         /* 35  IRQ19  I2C1                       */
    .word   SPI_IRQHandler          /* 36  IRQ20  SPI (INT)                  */
    .word   ADC1_IRQHandler         /* 37  IRQ21  ADC1                       */
    .word   SSP0_IRQHandler         /* 38  IRQ22  SSP0                       */
    .word   SSP1_IRQHandler         /* 39  IRQ23  SSP1                       */
    .word   USART0_IRQHandler       /* 40  IRQ24  USART0                     */
    .word   UART1_IRQHandler        /* 41  IRQ25  UART1                      */
    .word   USART2_IRQHandler       /* 42  IRQ26  USART2                     */
    .word   USART3_IRQHandler       /* 43  IRQ27  USART3                     */
    .word   I2S0_IRQHandler         /* 44  IRQ28  I2S0                       */
    .word   I2S1_IRQHandler         /* 45  IRQ29  I2S1                       */
    .word   SPIFI_IRQHandler        /* 46  IRQ30  SPIFI                      */
    .word   SGPIO_IRQHandler        /* 47  IRQ31  SGPIO                      */
    .word   PIN_INT0_IRQHandler     /* 48  IRQ32  GPIO pin int 0             */
    .word   PIN_INT1_IRQHandler     /* 49  IRQ33  GPIO pin int 1             */
    .word   PIN_INT2_IRQHandler     /* 50  IRQ34  GPIO pin int 2             */
    .word   PIN_INT3_IRQHandler     /* 51  IRQ35  GPIO pin int 3             */
    .word   PIN_INT4_IRQHandler     /* 52  IRQ36  GPIO pin int 4             */
    .word   PIN_INT5_IRQHandler     /* 53  IRQ37  GPIO pin int 5             */
    .word   PIN_INT6_IRQHandler     /* 54  IRQ38  GPIO pin int 6             */
    .word   PIN_INT7_IRQHandler     /* 55  IRQ39  GPIO pin int 7             */
    .word   GINT0_IRQHandler        /* 56  IRQ40  GPIO group int 0           */
    .word   GINT1_IRQHandler        /* 57  IRQ41  GPIO group int 1           */
    .word   EVENTROUTER_IRQHandler  /* 58  IRQ42  Event router               */
    .word   C_CAN1_IRQHandler       /* 59  IRQ43  C_CAN1                     */
    .word   0                       /* 60  IRQ44  Reserved                   */
    .word   0                       /* 61  IRQ45  Reserved                   */
    .word   ATIMER_IRQHandler       /* 62  IRQ46  Alarm timer                */
    .word   RTC_IRQHandler          /* 63  IRQ47  RTC                        */
    .word   0                       /* 64  IRQ48  Reserved                   */
    .word   WWDT_IRQHandler         /* 65  IRQ49  WWDT                       */
    .word   0                       /* 66  IRQ50  Reserved                   */
    .word   C_CAN0_IRQHandler       /* 67  IRQ51  C_CAN0                     */
    .word   QEI_IRQHandler          /* 68  IRQ52  QEI                        */

    .size   _vectors, . - _vectors

/* ========================================================================= */
/* Reset Handler                                                              */
/* ========================================================================= */
    .section .text.Reset_Handler, "ax", %progbits
    .align  2
    .globl  Reset_Handler
    .type   Reset_Handler, %function
    .thumb_func
Reset_Handler:

    /* ============================================================= */
    /* DIAGNOSTIC: Blink LED1 (GPIO2[1], P4_1) at each startup stage */
    /* Pure register writes — no C/C++ dependencies.                 */
    /*                                                               */
    /*   Stage 0: 1 blink  → Reset_Handler reached                  */
    /*   Stage 1: 2 blinks → .data copy done                        */
    /*   Stage 2: 3 blinks → .bss zero done                         */
    /*   Stage 3: 4 blinks → .ahbram zero done                      */
    /*   Stage 4: 5 blinks → init_array done (C++ ctors complete)   */
    /*   Stage 5: 6 blinks → about to call sentinel_main            */
    /*                                                               */
    /* Expected total pattern: 1, pause, 2, pause, 3, pause, 4,     */
    /* pause, 5, pause, 6, then sentinel_main LED blinks take over.  */
    /* ============================================================= */

    /* --- Set up LED1: P4_1 = GPIO2[1] --- */
    /* SCU P4_1 at 0x40086204: func 0 (GPIO), no pull */
    ldr     r6, =0x40086204
    movs    r7, #0
    str     r7, [r6]

    /* GPIO2 DIR: set bit 1 as output */
    ldr     r6, =0x400F6008     /* GPIO_PORT_BASE + 0x2000 + 2*4 */
    ldr     r7, [r6]
    orr     r7, r7, #2          /* bit 1 */
    str     r7, [r6]

    /* --- Stage 0: blink 1 time — we're alive --- */
    movs    r0, #1
    bl      .Ldiag_blink

    /* ---- 1. Copy .data from Flash (LMA) to SRAM (VMA) ---- */
    ldr     r0, =_data_start
    ldr     r1, =_data_end
    ldr     r2, =_data_load
.Lcopy_data:
    cmp     r0, r1
    bge     .Ldata_done
    ldr     r3, [r2], #4
    str     r3, [r0], #4
    b       .Lcopy_data
.Ldata_done:

    /* --- Stage 1: blink 2 times — .data copy done --- */
    movs    r0, #2
    bl      .Ldiag_blink

    /* ---- 2. Zero .bss ---- */
.Lzero_bss:
    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    movs    r2, #0
.Lzero_loop:
    cmp     r0, r1
    bge     .Lbss_done
    str     r2, [r0], #4
    b       .Lzero_loop
.Lbss_done:

    /* --- Stage 2: blink 3 times — .bss zero done --- */
    movs    r0, #3
    bl      .Ldiag_blink

    /* ---- 2b. Zero .ahbram (FreeRTOS heap + task stacks) ---- */
    /* The .ahbram section is NOLOAD so we must zero it explicitly.   */
    /* Without this, pvPortMalloc reads garbage heap metadata.         */
.Lzero_ahbram:
    ldr     r0, =_ahbram_start
    ldr     r1, =_ahbram_end
    movs    r2, #0
.Lahbram_loop:
    cmp     r0, r1
    bge     .Lahbram_done
    str     r2, [r0], #4
    b       .Lahbram_loop
.Lahbram_done:

    /* --- Stage 3: blink 4 times — .ahbram zero done --- */
    movs    r0, #4
    bl      .Ldiag_blink

    /* ---- 3. C++ static constructors ---- */
.Lcall_init:
    /* .preinit_array */
    ldr     r4, =__preinit_array_start
    ldr     r5, =__preinit_array_end
.Lpreinit_loop:
    cmp     r4, r5
    bge     .Linit_array
    ldr     r0, [r4], #4
    blx     r0
    b       .Lpreinit_loop

    /* .init_array */
.Linit_array:
    ldr     r4, =__init_array_start
    ldr     r5, =__init_array_end
.Linit_loop:
    cmp     r4, r5
    bge     .Linit_done
    ldr     r0, [r4], #4
    blx     r0
    b       .Linit_loop
.Linit_done:

    /* --- Stage 4: blink 5 times — init_array complete --- */
    movs    r0, #5
    bl      .Ldiag_blink

    /* ---- 4. Jump to application ---- */
    /* --- Stage 5: blink 6 times — about to call sentinel_main --- */
    movs    r0, #6
    bl      .Ldiag_blink

.Lcall_main:
    bl      sentinel_main

    /* ---- 5. Hang if sentinel_main returns ---- */
.Lhang:
    b       .Lhang

/* --------------------------------------------------------------------- */
/* Diagnostic blink subroutine                                           */
/* Input: r0 = number of blinks                                          */
/* Clobbers: r0, r1, r2, r3, lr (caller must use bl)                     */
/* Blinks LED1 (GPIO2[1]) with ~200ms on/off at 12-96 MHz IRC/PLL       */
/* --------------------------------------------------------------------- */
.Ldiag_blink:
    push    {r4, r5, lr}
    mov     r4, r0              /* r4 = blink count */
    /* GPIO2 SET register for LED1 */
    ldr     r5, =0x400F6208     /* GPIO_PORT_BASE + 0x2200 + 2*4 */
.Ldb_loop:
    cbz     r4, .Ldb_pause
    /* LED ON */
    movs    r3, #2              /* bit 1 */
    str     r3, [r5]            /* GPIO2 SET */
    /* Delay ~200ms */
    ldr     r2, =800000
.Ldb_on_dly:
    subs    r2, #1
    bne     .Ldb_on_dly
    /* LED OFF */
    ldr     r1, =0x400F6288     /* GPIO_PORT_BASE + 0x2280 + 2*4 */
    str     r3, [r1]            /* GPIO2 CLR */
    /* Delay ~200ms */
    ldr     r2, =800000
.Ldb_off_dly:
    subs    r2, #1
    bne     .Ldb_off_dly
    subs    r4, #1
    b       .Ldb_loop
.Ldb_pause:
    /* Pause ~600ms between stages */
    ldr     r2, =2400000
.Ldb_pause_dly:
    subs    r2, #1
    bne     .Ldb_pause_dly
    pop     {r4, r5, pc}

    .size   Reset_Handler, . - Reset_Handler

/* ========================================================================= */
/* Default Handler — infinite loop for unhandled interrupts                   */
/* ========================================================================= */
    .section .text.Default_Handler, "ax", %progbits
    .align  2
    .globl  Default_Handler
    .type   Default_Handler, %function
    .thumb_func
Default_Handler:
    b       .
    .size   Default_Handler, . - Default_Handler

/* ========================================================================= */
/* Weak aliases — override in C/C++ by defining the same symbol               */
/* FreeRTOS handlers (vPortSVCHandler, xPortPendSVHandler,                    */
/* xPortSysTickHandler) are NOT weak — they are provided by the FreeRTOS      */
/* ARM_CM4F port and must link at build time.                                 */
/* ========================================================================= */
    .macro  weak_alias name
    .weak   \name
    .thumb_set \name, Default_Handler
    .endm

    /* Cortex-M4 system exceptions */
    weak_alias NMI_Handler
    weak_alias HardFault_Handler
    weak_alias MemManage_Handler
    weak_alias BusFault_Handler
    weak_alias UsageFault_Handler
    weak_alias DebugMon_Handler

    /* LPC4320 peripheral IRQs */
    weak_alias DAC_IRQHandler
    weak_alias M0APP_IRQHandler
    weak_alias DMA_IRQHandler
    weak_alias FLASH_EEPROM_IRQHandler
    weak_alias ETH_IRQHandler
    weak_alias SDIO_IRQHandler
    weak_alias LCD_IRQHandler
    weak_alias USB0_IRQHandler
    weak_alias USB1_IRQHandler
    weak_alias SCT_IRQHandler
    weak_alias RIT_IRQHandler
    weak_alias TIMER0_IRQHandler
    weak_alias TIMER1_IRQHandler
    weak_alias TIMER2_IRQHandler
    weak_alias TIMER3_IRQHandler
    weak_alias MCPWM_IRQHandler
    weak_alias ADC0_IRQHandler
    weak_alias I2C0_IRQHandler
    weak_alias I2C1_IRQHandler
    weak_alias SPI_IRQHandler
    weak_alias ADC1_IRQHandler
    weak_alias SSP0_IRQHandler
    weak_alias SSP1_IRQHandler
    weak_alias USART0_IRQHandler
    weak_alias UART1_IRQHandler
    weak_alias USART2_IRQHandler
    weak_alias USART3_IRQHandler
    weak_alias I2S0_IRQHandler
    weak_alias I2S1_IRQHandler
    weak_alias SPIFI_IRQHandler
    weak_alias SGPIO_IRQHandler
    weak_alias PIN_INT0_IRQHandler
    weak_alias PIN_INT1_IRQHandler
    weak_alias PIN_INT2_IRQHandler
    weak_alias PIN_INT3_IRQHandler
    weak_alias PIN_INT4_IRQHandler
    weak_alias PIN_INT5_IRQHandler
    weak_alias PIN_INT6_IRQHandler
    weak_alias PIN_INT7_IRQHandler
    weak_alias GINT0_IRQHandler
    weak_alias GINT1_IRQHandler
    weak_alias EVENTROUTER_IRQHandler
    weak_alias C_CAN1_IRQHandler
    weak_alias ATIMER_IRQHandler
    weak_alias RTC_IRQHandler
    weak_alias WWDT_IRQHandler
    weak_alias C_CAN0_IRQHandler
    weak_alias QEI_IRQHandler

    .end
