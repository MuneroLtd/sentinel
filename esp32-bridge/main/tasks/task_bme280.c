#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "sentinel_config.h"
#include "shared_state.h"

static const char *TAG = "bme280";

// ---------------------------------------------------------------------------
// BME280 register map
// ---------------------------------------------------------------------------
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_CHIP_ID_VALUE    0x60

// Calibration registers bank 1: 0x88-0xA1 (26 bytes: T1..P9 + reserved + H1)
#define BME280_REG_CALIB_00     0x88
#define BME280_CALIB_DATA1_LEN  26

// Calibration registers bank 2: 0xE1-0xE7 (7 bytes: H2..H6)
#define BME280_REG_CALIB_26     0xE1
#define BME280_CALIB_DATA2_LEN  7

// ---------------------------------------------------------------------------
// Calibration data struct (matches BME280 datasheet §4.2.2)
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

static bme280_calib_t s_calib;

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

static esp_err_t bme280_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(BME280_I2C_PORT,
                                      BME280_I2C_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
}

static esp_err_t bme280_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BME280_I2C_PORT,
                                        BME280_I2C_ADDR,
                                        &reg, 1,
                                        data, len,
                                        pdMS_TO_TICKS(100));
}

// ---------------------------------------------------------------------------
// Calibration data loading
// ---------------------------------------------------------------------------

static esp_err_t bme280_read_calibration(void)
{
    uint8_t d1[BME280_CALIB_DATA1_LEN];
    uint8_t d2[BME280_CALIB_DATA2_LEN];

    esp_err_t err = bme280_read_regs(BME280_REG_CALIB_00, d1, sizeof(d1));
    if (err != ESP_OK) {
        return err;
    }

    err = bme280_read_regs(BME280_REG_CALIB_26, d2, sizeof(d2));
    if (err != ESP_OK) {
        return err;
    }

    // --- Temperature calibration ---
    s_calib.dig_T1 = (uint16_t)(d1[1] << 8 | d1[0]);
    s_calib.dig_T2 = (int16_t) (d1[3] << 8 | d1[2]);
    s_calib.dig_T3 = (int16_t) (d1[5] << 8 | d1[4]);

    // --- Pressure calibration ---
    s_calib.dig_P1 = (uint16_t)(d1[7]  << 8 | d1[6]);
    s_calib.dig_P2 = (int16_t) (d1[9]  << 8 | d1[8]);
    s_calib.dig_P3 = (int16_t) (d1[11] << 8 | d1[10]);
    s_calib.dig_P4 = (int16_t) (d1[13] << 8 | d1[12]);
    s_calib.dig_P5 = (int16_t) (d1[15] << 8 | d1[14]);
    s_calib.dig_P6 = (int16_t) (d1[17] << 8 | d1[16]);
    s_calib.dig_P7 = (int16_t) (d1[19] << 8 | d1[18]);
    s_calib.dig_P8 = (int16_t) (d1[21] << 8 | d1[20]);
    s_calib.dig_P9 = (int16_t) (d1[23] << 8 | d1[22]);

    // d1[24] is reserved
    s_calib.dig_H1 = d1[25];

    // --- Humidity calibration (bank 2) ---
    s_calib.dig_H2 = (int16_t)(d2[1] << 8 | d2[0]);
    s_calib.dig_H3 = d2[2];
    // H4 and H5 share a register byte
    s_calib.dig_H4 = (int16_t)((int16_t)(d2[3] << 4) | (d2[4] & 0x0F));
    s_calib.dig_H5 = (int16_t)((int16_t)(d2[5] << 4) | (d2[4] >> 4));
    s_calib.dig_H6 = (int8_t)d2[6];

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Compensation formulas (integer arithmetic, from BME280 datasheet §8.2)
// ---------------------------------------------------------------------------

/**
 * @brief Compensate raw temperature.
 * @param adc_T  Raw temperature ADC value (20-bit)
 * @param t_fine Output: fine temperature value used by P and H compensation
 * @return       Temperature in degrees C × 100 (e.g. 2345 = 23.45 °C)
 */
static int32_t bme280_compensate_temp(int32_t adc_T, int32_t *t_fine)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
             ((int32_t)s_calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
             ((int32_t)s_calib.dig_T3)) >> 14;

    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;  // T in 0.01 degrees C
    return T;
}

/**
 * @brief Compensate raw pressure.
 * @param adc_P  Raw pressure ADC value (20-bit)
 * @param t_fine Fine temperature from compensate_temp()
 * @return       Pressure in Pascals (Q24.8 fixed point / 256 = Pa)
 *               Returns 0 if result would overflow.
 */
static uint32_t bme280_compensate_press(int32_t adc_P, int32_t t_fine)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((INT64_C(1) << 47) + var1) * ((int64_t)s_calib.dig_P1)) >> 33;

    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

    // p is in Q24.8 format; divide by 256 to get Pa
    return (uint32_t)(p >> 8);
}

/**
 * @brief Compensate raw humidity.
 * @param adc_H  Raw humidity ADC value (16-bit)
 * @param t_fine Fine temperature from compensate_temp()
 * @return       Humidity in %RH × 1024 (Q22.10 fixed point / 1024 = %RH)
 */
static uint32_t bme280_compensate_hum(int32_t adc_H, int32_t t_fine)
{
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)s_calib.dig_H4) << 20) -
                   (((int32_t)s_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                  (((((((v_x1_u32r * ((int32_t)s_calib.dig_H6)) >> 10) *
                       (((v_x1_u32r * ((int32_t)s_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                     ((int32_t)2097152)) * ((int32_t)s_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)s_calib.dig_H1)) >> 4);

    if (v_x1_u32r < 0) v_x1_u32r = 0;
    if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;

    // Return Q22.10: divide by 1024 to get %RH
    return (uint32_t)(v_x1_u32r >> 12);
}

// ---------------------------------------------------------------------------
// Task entry point
// ---------------------------------------------------------------------------
void task_bme280_run(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Initialising I2C master on port %d SDA=%d SCL=%d",
             BME280_I2C_PORT, BME280_SDA_PIN, BME280_SCL_PIN);

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BME280_SDA_PIN,
        .scl_io_num       = BME280_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BME280_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(BME280_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(BME280_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // --- Verify chip ID ---
    uint8_t chip_id = 0;
    esp_err_t err = bme280_read_regs(BME280_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != BME280_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "BME280 not found (id=0x%02X, err=%d) — sensor task sleeping",
                 chip_id, err);
        // Sensor is optional; task suspends itself
        vTaskSuspend(NULL);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "BME280 found, chip_id=0x%02X", chip_id);

    // --- Soft reset ---
    bme280_write_reg(BME280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- Load calibration data ---
    err = bme280_read_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data: %d", err);
        vTaskSuspend(NULL);
        vTaskDelete(NULL);
        return;
    }

    // --- Configure sensor ---
    // ctrl_hum: osrs_h = 1 (×1 oversampling for humidity)  — must be written BEFORE ctrl_meas
    bme280_write_reg(BME280_REG_CTRL_HUM, 0x01);
    // config: t_sb = 101 (1000 ms standby), filter = 0 (off), spi3w_en = 0
    //   bits: t_sb[7:5] = 101, filter[4:2] = 000, spi3w_en[0] = 0 → 0xA0
    bme280_write_reg(BME280_REG_CONFIG, 0xA0);
    // ctrl_meas: osrs_t = 010 (×2), osrs_p = 101 (×16), mode = 11 (normal)
    //   bits: osrs_t[7:5] = 010, osrs_p[4:2] = 101, mode[1:0] = 11 → 0x57
    bme280_write_reg(BME280_REG_CTRL_MEAS, 0x57);

    ESP_LOGI(TAG, "BME280 configured in normal mode (1000 ms standby)");

    // Allow first measurement to complete
    vTaskDelay(pdMS_TO_TICKS(1100));

    for (;;) {
        // Wait for measurement status (optional — we poll on schedule)
        uint8_t status = 0;
        bme280_read_regs(BME280_REG_STATUS, &status, 1);
        if (status & 0x08) {
            // Measurement in progress — wait a bit more
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Read 8 bytes of raw ADC data starting at 0xF7:
        //   press_msb[0], press_lsb[1], press_xlsb[2],
        //   temp_msb[3],  temp_lsb[4],  temp_xlsb[5],
        //   hum_msb[6],   hum_lsb[7]
        uint8_t raw[8];
        err = bme280_read_regs(BME280_REG_PRESS_MSB, raw, sizeof(raw));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed: %d", err);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        int32_t adc_P = ((int32_t)raw[0] << 12) |
                        ((int32_t)raw[1] << 4)  |
                        ((int32_t)raw[2] >> 4);

        int32_t adc_T = ((int32_t)raw[3] << 12) |
                        ((int32_t)raw[4] << 4)  |
                        ((int32_t)raw[5] >> 4);

        int32_t adc_H = ((int32_t)raw[6] << 8) | (int32_t)raw[7];

        // Compensate
        int32_t  t_fine   = 0;
        int32_t  temp_c100 = bme280_compensate_temp(adc_T, &t_fine);
        uint32_t press_pa  = bme280_compensate_press(adc_P, t_fine);
        uint32_t hum_q1024 = bme280_compensate_hum(adc_H, t_fine);

        // Convert to shared state units:
        //   temp_c_x10  = temp_c100 / 10
        //   humidity_x10 = (hum_q1024 / 1024) * 10 = hum_q1024 * 10 / 1024
        int16_t  temp_x10 = (int16_t)(temp_c100 / 10);
        uint16_t hum_x10  = (uint16_t)((hum_q1024 * 10) / 1024);

        shared_state_lock();
        g_state.temp_c_x10   = temp_x10;
        g_state.humidity_x10 = hum_x10;
        g_state.pressure_pa  = press_pa;
        shared_state_unlock();

        ESP_LOGI(TAG, "T=%d.%d°C H=%d.%d%% P=%lu Pa",
                 temp_x10 / 10, abs(temp_x10 % 10),
                 hum_x10 / 10, hum_x10 % 10,
                 (unsigned long)press_pa);

        // Wait for next measurement cycle (standby is 1000 ms; poll at 10 s)
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    vTaskDelete(NULL);
}
