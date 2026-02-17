/* Test: full startup_m4.S → minimal blink (no FreeRTOS, no clocks_init) */
/* This tests whether .data copy, .bss zero, and init_array work.       */
/* If this blinks: startup is OK, crash is in sentinel_main().           */
/* If this doesn't blink: startup code or init_array is crashing.        */

#include <stdint.h>

#define SCU_BASE        0x40086000u
#define SCU_SFSP4_1     (*(volatile uint32_t *)(SCU_BASE + 0x208))
#define SCU_SFSP4_2     (*(volatile uint32_t *)(SCU_BASE + 0x20C))
#define SCU_SFSP6_12    (*(volatile uint32_t *)(SCU_BASE + 0x330))

#define GPIO_BASE       0x400F4000u
#define GPIO_DIR2       (*(volatile uint32_t *)(GPIO_BASE + 0x2008))
#define GPIO_SET2       (*(volatile uint32_t *)(GPIO_BASE + 0x2208))
#define GPIO_CLR2       (*(volatile uint32_t *)(GPIO_BASE + 0x2288))

#define CCU1_BASE       0x40051000u
#define CCU1_CLK_M4_GPIO_CFG (*(volatile uint32_t *)(CCU1_BASE + 0x088))

/* Use a .data variable to prove .data copy works */
static volatile uint32_t blink_count = 3;

/* Use a .bss variable to prove .bss zero works */
static volatile uint32_t bss_test;

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm volatile("nop");
    }
}

/* This is called by startup_m4.S as sentinel_main */
void sentinel_main(void) __attribute__((noreturn));
void sentinel_main(void) {
    /* Enable GPIO clock */
    CCU1_CLK_M4_GPIO_CFG = 0x01;

    /* Configure LED pins */
    SCU_SFSP4_1  = 0x00;
    SCU_SFSP4_2  = 0x00;
    SCU_SFSP6_12 = 0x00;
    GPIO_DIR2 |= (1u << 1) | (1u << 2) | (1u << 8);

    /* Blink pattern: LED1 blinks 'blink_count' times fast (proves .data works) */
    for (uint32_t i = 0; i < blink_count; i++) {
        GPIO_SET2 = (1u << 1);
        delay(1000000);
        GPIO_CLR2 = (1u << 1);
        delay(1000000);
    }

    /* If bss_test == 0, .bss was zeroed correctly → blink LED2 */
    if (bss_test == 0) {
        GPIO_SET2 = (1u << 2);
        delay(3000000);
        GPIO_CLR2 = (1u << 2);
        delay(1000000);
    }

    /* Then blink all 3 LEDs forever (proves we survived init_array) */
    while (1) {
        GPIO_SET2 = (1u << 1) | (1u << 2) | (1u << 8);
        delay(2000000);
        GPIO_CLR2 = (1u << 1) | (1u << 2) | (1u << 8);
        delay(2000000);
    }
}
