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
    /* ---- 1. Copy .data from Flash (LMA) to SRAM (VMA) ---- */
    ldr     r0, =_data_start
    ldr     r1, =_data_end
    ldr     r2, =_data_load
.Lcopy_data:
    cmp     r0, r1
    bge     .Lzero_bss
    ldr     r3, [r2], #4
    str     r3, [r0], #4
    b       .Lcopy_data

    /* ---- 2. Zero .bss ---- */
.Lzero_bss:
    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    movs    r2, #0
.Lzero_loop:
    cmp     r0, r1
    bge     .Lcall_init
    str     r2, [r0], #4
    b       .Lzero_loop

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
    bge     .Lcall_main
    ldr     r0, [r4], #4
    blx     r0
    b       .Linit_loop

    /* ---- 4. Jump to application ---- */
.Lcall_main:
    bl      sentinel_main

    /* ---- 5. Hang if sentinel_main returns ---- */
.Lhang:
    b       .Lhang

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
