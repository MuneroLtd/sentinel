// SPDX-License-Identifier: MIT
// Project Sentinel — ILI9341 320×240 LCD Driver Implementation
//
// Full ILI9341 SPI initialisation sequence based on the ILI9341 v1.11 datasheet
// and the Adafruit ILI9341 library reference init sequence.
// Operates in SPI 4-line mode: SCK, MOSI, MISO, CS.
// DC (D/CX) pin selects command (low) vs data (high).

#include "lcd_ili9341.hpp"
#include "portapack_pins.hpp"
#include "../hal/ssp.hpp"
#include "../hal/gpio.hpp"
#include "../hal/lpc4320_regs.hpp"
#include <cstddef>

// ---------------------------------------------------------------------------
// ILI9341 command bytes (from ILI9341 datasheet)
// ---------------------------------------------------------------------------
static constexpr uint8_t ILI9341_NOP        = 0x00;
static constexpr uint8_t ILI9341_SWRESET    = 0x01;  // Software reset
static constexpr uint8_t ILI9341_RDDID      = 0x04;
static constexpr uint8_t ILI9341_RDDST      = 0x09;
static constexpr uint8_t ILI9341_SLPIN      = 0x10;
static constexpr uint8_t ILI9341_SLPOUT     = 0x11;  // Sleep out
static constexpr uint8_t ILI9341_PTLON      = 0x12;
static constexpr uint8_t ILI9341_NORON      = 0x13;  // Normal display mode on
static constexpr uint8_t ILI9341_RDMODE     = 0x0A;
static constexpr uint8_t ILI9341_RDMADCTL   = 0x0B;
static constexpr uint8_t ILI9341_RDPIXFMT   = 0x0C;
static constexpr uint8_t ILI9341_RDIMGFMT   = 0x0D;
static constexpr uint8_t ILI9341_RDSELFDIAG = 0x0F;
static constexpr uint8_t ILI9341_INVOFF     = 0x20;
static constexpr uint8_t ILI9341_INVON      = 0x21;
static constexpr uint8_t ILI9341_GAMMASET   = 0x26;
static constexpr uint8_t ILI9341_DISPOFF    = 0x28;
static constexpr uint8_t ILI9341_DISPON     = 0x29;  // Display on
static constexpr uint8_t ILI9341_CASET      = 0x2A;  // Column address set
static constexpr uint8_t ILI9341_PASET      = 0x2B;  // Page (row) address set
static constexpr uint8_t ILI9341_RAMWR      = 0x2C;  // Memory write
static constexpr uint8_t ILI9341_RAMRD      = 0x2E;
static constexpr uint8_t ILI9341_PTLAR      = 0x30;
static constexpr uint8_t ILI9341_VSCRDEF    = 0x33;
static constexpr uint8_t ILI9341_MADCTL     = 0x36;  // Memory access control
static constexpr uint8_t ILI9341_VSCRSADD   = 0x37;
static constexpr uint8_t ILI9341_PIXFMT     = 0x3A;  // Pixel format set
static constexpr uint8_t ILI9341_FRMCTR1    = 0xB1;
static constexpr uint8_t ILI9341_FRMCTR2    = 0xB2;
static constexpr uint8_t ILI9341_FRMCTR3    = 0xB3;
static constexpr uint8_t ILI9341_INVCTR     = 0xB4;
static constexpr uint8_t ILI9341_DFUNCTR    = 0xB6;  // Display function control
static constexpr uint8_t ILI9341_PWCTR1     = 0xC0;
static constexpr uint8_t ILI9341_PWCTR2     = 0xC1;
static constexpr uint8_t ILI9341_PWCTR3     = 0xC2;
static constexpr uint8_t ILI9341_PWCTR4     = 0xC3;
static constexpr uint8_t ILI9341_PWCTR5     = 0xC4;
static constexpr uint8_t ILI9341_VMCTR1     = 0xC5;
static constexpr uint8_t ILI9341_VMCTR2     = 0xC7;
static constexpr uint8_t ILI9341_RDID1      = 0xDA;
static constexpr uint8_t ILI9341_RDID2      = 0xDB;
static constexpr uint8_t ILI9341_RDID3      = 0xDC;
static constexpr uint8_t ILI9341_RDID4      = 0xDD;
static constexpr uint8_t ILI9341_GMCTRP1    = 0xE0;  // Positive gamma correction
static constexpr uint8_t ILI9341_GMCTRN1    = 0xE1;  // Negative gamma correction
static constexpr uint8_t ILI9341_IF_CTL     = 0xF6;  // Interface control

// MADCTL rotation bits
static constexpr uint8_t MADCTL_MY  = 0x80;  // Row address order
static constexpr uint8_t MADCTL_MX  = 0x40;  // Column address order
static constexpr uint8_t MADCTL_MV  = 0x20;  // Row/column exchange
static constexpr uint8_t MADCTL_ML  = 0x10;  // Vertical refresh order
static constexpr uint8_t MADCTL_BGR = 0x08;  // BGR colour filter
static constexpr uint8_t MADCTL_MH  = 0x04;  // Horizontal refresh order

// Landscape rotation: MX | MV → 320 wide × 240 tall
static constexpr uint8_t MADCTL_LANDSCAPE = MADCTL_MX | MADCTL_MV;

// ---------------------------------------------------------------------------
// Simple microsecond delay (rough, calibrated for 96 MHz core)
// Each iteration ≈ 5 cycles → 96M/5 = ~19.2M iter/s
// ---------------------------------------------------------------------------
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 19u;
    while (count--) {
        __asm volatile("nop");
    }
}

static void delay_ms(uint32_t ms) {
    delay_us(ms * 1000u);
}

// ---------------------------------------------------------------------------
// Low-level GPIO helpers (inline wrappers that use the portapack pin defs)
// ---------------------------------------------------------------------------
static inline void cs_assert()   { gpio_write(LCD_CS_GPIO_PORT,  LCD_CS_GPIO_PIN,  false); }
static inline void cs_deassert() { gpio_write(LCD_CS_GPIO_PORT,  LCD_CS_GPIO_PIN,  true);  }
static inline void dc_cmd()      { gpio_write(LCD_DC_GPIO_PORT,  LCD_DC_GPIO_PIN,  false); }
static inline void dc_data()     { gpio_write(LCD_DC_GPIO_PORT,  LCD_DC_GPIO_PIN,  true);  }
static inline void rst_assert()  { gpio_write(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN, false); }
static inline void rst_deassert(){ gpio_write(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN, true);  }

// ---------------------------------------------------------------------------
// Send one command byte
// ---------------------------------------------------------------------------
static void lcd_write_cmd(uint8_t cmd) {
    dc_cmd();
    cs_assert();
    ssp_transfer(LCD_SSP_BUS, cmd);
    cs_deassert();
}

// ---------------------------------------------------------------------------
// Send one data byte
// ---------------------------------------------------------------------------
static void lcd_write_data(uint8_t data) {
    dc_data();
    cs_assert();
    ssp_transfer(LCD_SSP_BUS, data);
    cs_deassert();
}

// ---------------------------------------------------------------------------
// Send command followed by data bytes (CS held for entire transaction)
// ---------------------------------------------------------------------------
static void lcd_cmd_data(uint8_t cmd, const uint8_t* data, size_t len) {
    dc_cmd();
    cs_assert();
    ssp_transfer(LCD_SSP_BUS, cmd);
    dc_data();
    ssp_write(LCD_SSP_BUS, data, len);
    cs_deassert();
}

// ---------------------------------------------------------------------------
// lcd_init — hardware reset + full initialisation sequence
// ---------------------------------------------------------------------------
void lcd_init() {
    // Configure GPIO pin directions
    gpio_set_dir(LCD_CS_GPIO_PORT,  LCD_CS_GPIO_PIN,  true);  // output
    gpio_set_dir(LCD_DC_GPIO_PORT,  LCD_DC_GPIO_PIN,  true);  // output
    gpio_set_dir(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN, true);  // output

    // Configure SCU mux for GPIO
    scu_set_pinmode(LCD_CS_SCU_GRP,  LCD_CS_SCU_PIN,  LCD_CS_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_IBUF);
    scu_set_pinmode(LCD_DC_SCU_GRP,  LCD_DC_SCU_PIN,  LCD_DC_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_IBUF);
    scu_set_pinmode(LCD_RST_SCU_GRP, LCD_RST_SCU_PIN, LCD_RST_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_IBUF);

    // Configure SSP1 pin mux for SPI (MOSI, MISO, SCK)
    scu_set_pinmode(SSP1_SCK_SCU_GRP,  SSP1_SCK_SCU_PIN,  SSP1_SCK_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_FAST);
    scu_set_pinmode(SSP1_MOSI_SCU_GRP, SSP1_MOSI_SCU_PIN, SSP1_MOSI_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_FAST);
    scu_set_pinmode(SSP1_MISO_SCU_GRP, SSP1_MISO_SCU_PIN, SSP1_MISO_SCU_FUNC,
                    SCU_MODE_INACT | SCU_MODE_IBUF | SCU_MODE_FAST);

    // Initialise SSP1 at ~10 MHz, SPI mode 0 (CPOL=0, CPHA=0)
    ssp_init(LCD_SSP_BUS, 10000000u, 0, 0);

    // Deassert CS, put DC in data mode
    cs_deassert();
    dc_data();

    // Hardware reset sequence
    rst_deassert();
    delay_ms(5);
    rst_assert();
    delay_ms(20);
    rst_deassert();
    delay_ms(150);

    // --- ILI9341 initialisation sequence ---

    // Software reset
    lcd_write_cmd(ILI9341_SWRESET);
    delay_ms(5);

    // Exit sleep
    lcd_write_cmd(ILI9341_SLPOUT);
    delay_ms(120);  // ILI9341 datasheet: must wait 120ms after SLPOUT

    // Power control A
    { uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
      lcd_cmd_data(0xCB, d, sizeof(d)); }

    // Power control B
    { uint8_t d[] = {0x00, 0xC1, 0x30};
      lcd_cmd_data(0xCF, d, sizeof(d)); }

    // Driver timing control A
    { uint8_t d[] = {0x85, 0x00, 0x78};
      lcd_cmd_data(0xE8, d, sizeof(d)); }

    // Driver timing control B
    { uint8_t d[] = {0x00, 0x00};
      lcd_cmd_data(0xEA, d, sizeof(d)); }

    // Power on sequence control
    { uint8_t d[] = {0x64, 0x03, 0x12, 0x81};
      lcd_cmd_data(0xED, d, sizeof(d)); }

    // Pump ratio control
    { uint8_t d[] = {0x20};
      lcd_cmd_data(0xF7, d, sizeof(d)); }

    // Power control 1: VRH[5:0] = 4.60V
    { uint8_t d[] = {0x23};
      lcd_cmd_data(ILI9341_PWCTR1, d, sizeof(d)); }

    // Power control 2: SAP[2:0] BT[3:0]
    { uint8_t d[] = {0x10};
      lcd_cmd_data(ILI9341_PWCTR2, d, sizeof(d)); }

    // VCOM control 1: VMH, VML
    { uint8_t d[] = {0x3E, 0x28};
      lcd_cmd_data(ILI9341_VMCTR1, d, sizeof(d)); }

    // VCOM control 2
    { uint8_t d[] = {0x86};
      lcd_cmd_data(ILI9341_VMCTR2, d, sizeof(d)); }

    // Memory access control: landscape, BGR
    { uint8_t d[] = {MADCTL_LANDSCAPE | MADCTL_BGR};
      lcd_cmd_data(ILI9341_MADCTL, d, sizeof(d)); }

    // Pixel format: 16 bits per pixel
    { uint8_t d[] = {0x55};  // 0x55 = 16 bpp for MCU and RGB interfaces
      lcd_cmd_data(ILI9341_PIXFMT, d, sizeof(d)); }

    // Frame rate control: 119 Hz
    { uint8_t d[] = {0x00, 0x18};
      lcd_cmd_data(ILI9341_FRMCTR1, d, sizeof(d)); }

    // Display function control
    { uint8_t d[] = {0x08, 0x82, 0x27};
      lcd_cmd_data(ILI9341_DFUNCTR, d, sizeof(d)); }

    // Disable 3-gamma
    { uint8_t d[] = {0x00};
      lcd_cmd_data(0xF2, d, sizeof(d)); }

    // Gamma curve selected: curve 1
    { uint8_t d[] = {0x01};
      lcd_cmd_data(ILI9341_GAMMASET, d, sizeof(d)); }

    // Positive gamma correction
    { uint8_t d[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
                     0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03,
                     0x0E, 0x09, 0x00};
      lcd_cmd_data(ILI9341_GMCTRP1, d, sizeof(d)); }

    // Negative gamma correction
    { uint8_t d[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
                     0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C,
                     0x31, 0x36, 0x0F};
      lcd_cmd_data(ILI9341_GMCTRN1, d, sizeof(d)); }

    // Normal display mode on
    lcd_write_cmd(ILI9341_NORON);
    delay_ms(10);

    // Display on
    lcd_write_cmd(ILI9341_DISPON);
    delay_ms(100);

    // Clear the screen to black
    lcd_clear(LCD_BLACK);
}

// ---------------------------------------------------------------------------
// lcd_set_window — set column address (CASET) and row address (PASET)
// ---------------------------------------------------------------------------
void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column address set
    {
        uint8_t d[4] = {
            static_cast<uint8_t>(x0 >> 8),
            static_cast<uint8_t>(x0 & 0xFF),
            static_cast<uint8_t>(x1 >> 8),
            static_cast<uint8_t>(x1 & 0xFF)
        };
        lcd_cmd_data(ILI9341_CASET, d, sizeof(d));
    }
    // Page (row) address set
    {
        uint8_t d[4] = {
            static_cast<uint8_t>(y0 >> 8),
            static_cast<uint8_t>(y0 & 0xFF),
            static_cast<uint8_t>(y1 >> 8),
            static_cast<uint8_t>(y1 & 0xFF)
        };
        lcd_cmd_data(ILI9341_PASET, d, sizeof(d));
    }
    // Memory write command (data follows)
    lcd_write_cmd(ILI9341_RAMWR);
}

// ---------------------------------------------------------------------------
// lcd_draw_pixel
// ---------------------------------------------------------------------------
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color565) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;

    lcd_set_window(x, y, x, y);

    dc_data();
    cs_assert();
    ssp_transfer(LCD_SSP_BUS, static_cast<uint8_t>(color565 >> 8));
    ssp_transfer(LCD_SSP_BUS, static_cast<uint8_t>(color565 & 0xFF));
    cs_deassert();
}

// ---------------------------------------------------------------------------
// lcd_fill_rect — optimised bulk fill
// ---------------------------------------------------------------------------
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color565) {
    if (w == 0 || h == 0) return;
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;

    // Clip
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH  - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    lcd_set_window(x, y, x + w - 1u, y + h - 1u);

    const uint8_t hi = static_cast<uint8_t>(color565 >> 8);
    const uint8_t lo = static_cast<uint8_t>(color565 & 0xFF);

    uint32_t count = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);

    dc_data();
    cs_assert();

    // Push pixels; SSP FIFO is 8 deep, so we pump pairs without stalling often.
    while (count--) {
        // Wait for TX slot, write high byte
        while (!(LPC_SSP1->SR & SSP_SR_TNF)) { }
        LPC_SSP1->DR = hi;
        // Wait for TX slot, write low byte
        while (!(LPC_SSP1->SR & SSP_SR_TNF)) { }
        LPC_SSP1->DR = lo;
    }

    // Wait for all bytes to shift out
    while (LPC_SSP1->SR & SSP_SR_BSY) { }
    // Drain RX FIFO
    while (LPC_SSP1->SR & SSP_SR_RNE) { (void)LPC_SSP1->DR; }

    cs_deassert();
}

// ---------------------------------------------------------------------------
// lcd_blit — transfer a pixel buffer to the LCD
// ---------------------------------------------------------------------------
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* pixels) {
    if (w == 0 || h == 0 || !pixels) return;

    lcd_set_window(x, y, x + w - 1u, y + h - 1u);

    uint32_t count = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);

    dc_data();
    cs_assert();

    const uint16_t* p = pixels;
    // We drain the RX FIFO periodically to prevent overflow.
    uint32_t tx_pending = 0;

    while (count--) {
        uint16_t pixel = *p++;

        // Wait for space in TX FIFO
        while (!(LPC_SSP1->SR & SSP_SR_TNF)) {
            // Drain RX to make forward progress
            if (LPC_SSP1->SR & SSP_SR_RNE) { (void)LPC_SSP1->DR; tx_pending--; }
        }
        LPC_SSP1->DR = static_cast<uint8_t>(pixel >> 8);
        tx_pending++;

        while (!(LPC_SSP1->SR & SSP_SR_TNF)) {
            if (LPC_SSP1->SR & SSP_SR_RNE) { (void)LPC_SSP1->DR; tx_pending--; }
        }
        LPC_SSP1->DR = static_cast<uint8_t>(pixel & 0xFF);
        tx_pending++;

        // Drain aggressively
        while (tx_pending > 0 && (LPC_SSP1->SR & SSP_SR_RNE)) {
            (void)LPC_SSP1->DR;
            tx_pending--;
        }
    }

    // Flush remaining
    while (LPC_SSP1->SR & SSP_SR_BSY) { }
    while (LPC_SSP1->SR & SSP_SR_RNE) { (void)LPC_SSP1->DR; }

    cs_deassert();
}
