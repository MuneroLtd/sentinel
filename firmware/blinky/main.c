/* Absolute minimal blinky test for HackRF H4M / LPC4320 */
/* No libraries, no startup, no .data, no .bss — pure register banging */

#include <stdint.h>

/* SCU pin mux registers */
#define SCU_BASE        0x40086000u
#define SCU_SFSP4_1     (*(volatile uint32_t *)(SCU_BASE + 0x208))  /* LED1: P4_1 → GPIO2[1] */
#define SCU_SFSP4_2     (*(volatile uint32_t *)(SCU_BASE + 0x20C))  /* LED2: P4_2 → GPIO2[2] */
#define SCU_SFSP6_12    (*(volatile uint32_t *)(SCU_BASE + 0x330))  /* LED3: P6_12 → GPIO2[8] */

/* GPIO registers */
#define GPIO_BASE       0x400F4000u
#define GPIO_DIR2       (*(volatile uint32_t *)(GPIO_BASE + 0x2008))  /* GPIO port 2 direction */
#define GPIO_SET2       (*(volatile uint32_t *)(GPIO_BASE + 0x2208))  /* GPIO port 2 set */
#define GPIO_CLR2       (*(volatile uint32_t *)(GPIO_BASE + 0x2288))  /* GPIO port 2 clear */

/* CCU1 GPIO clock */
#define CCU1_BASE       0x40051000u
#define CCU1_CLK_M4_GPIO_CFG (*(volatile uint32_t *)(CCU1_BASE + 0x088))

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm volatile("nop");
    }
}

/* Entry point — called directly from vector table */
void blinky_main(void) __attribute__((noreturn));
void blinky_main(void) {
    /* Enable GPIO clock */
    CCU1_CLK_M4_GPIO_CFG = 0x01;  /* RUN bit */

    /* Configure LED pins as GPIO (function 0), no pull-up/down */
    SCU_SFSP4_1  = 0x00;   /* P4_1  → GPIO2[1] (func 0) */
    SCU_SFSP4_2  = 0x00;   /* P4_2  → GPIO2[2] (func 0) */
    SCU_SFSP6_12 = 0x00;   /* P6_12 → GPIO2[8] (func 0) */

    /* Set LED pins as output */
    GPIO_DIR2 |= (1u << 1) | (1u << 2) | (1u << 8);

    /* Blink all 3 LEDs forever */
    while (1) {
        GPIO_SET2 = (1u << 1) | (1u << 2) | (1u << 8);  /* LEDs ON */
        delay(2000000);
        GPIO_CLR2 = (1u << 1) | (1u << 2) | (1u << 8);  /* LEDs OFF */
        delay(2000000);
    }
}

/* Vector table is in vectors.S */
