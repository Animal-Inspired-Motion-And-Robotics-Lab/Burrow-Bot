#include "ldc1101.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef LDC_MOSI
#define LDC_MOSI 9
#endif

#ifndef LDC_MISO
#define LDC_MISO 8
#endif

#ifndef LDC_SCK
#define LDC_SCK 7
#endif

#ifndef LDC_CS
#define LDC_CS 4
#endif

#ifndef LDC_SPI_HOST
#define LDC_SPI_HOST SPI3_HOST
#endif

#ifndef LDC_SPI_CLOCK_HZ
#define LDC_SPI_CLOCK_HZ (100 * 1000)
#endif

#define F_CLKIN 16000000.0f

#define REG_RP_SET 0x01
#define REG_TC1 0x02
#define REG_TC2 0x03
#define REG_DIG_CONF 0x04
#define REG_ALT_CONFIG 0x05
#define REG_INTB_MODE 0x0A
#define REG_START_CONFIG 0x0B
#define REG_CHIP_ID 0x3F

#define REG_RP_DATA_MSB 0x22
#define REG_RP_DATA_LSB 0x21

#define REG_L_DATA_MSB 0x24
#define REG_L_DATA_LSB 0x23

#define REG_STATUS 0x20

#define REG_LHR_RCOUNT_LSB 0x30
#define REG_LHR_RCOUNT_MSB 0x31
#define REG_LHR_OFFSET_LSB 0x32
#define REG_LHR_OFFSET_MSB 0x33
#define REG_LHR_DATA_LSB 0x38
#define REG_LHR_DATA_MID 0x39
#define REG_LHR_DATA_MSB 0x3A
#define REG_D_CONFIG 0x0C

typedef struct {
    float resistance;
    uint8_t bits;
} rp_entry_t;

static const rp_entry_t rp_table[8] = {
    {96000.0f, 0b000},
    {48000.0f, 0b001},
    {24000.0f, 0b010},
    {12000.0f, 0b011},
    {6000.0f, 0b100},
    {3000.0f, 0b101},
    {1500.0f, 0b110},
    {750.0f, 0b111},
};

typedef struct {
    uint8_t reg;
    float rp_min_ohms;
    float rp_max_ohms;
    uint8_t high_q;
} rp_config_t;

typedef struct {
    uint8_t reg;
    float response_time_periods;
    float conversion_time_sec;
} dig_conf_result_t;

static rp_config_t rp_cfg_global;
static dig_conf_result_t dig_cfg_global;
static ldc1101_mode_t current_mode;
static uint16_t lhr_rcount = 0xFFFF;

static void get_c1_settings(ldc_speed_mode_t mode, float* C1, uint8_t* C1_bits) {
    // Per datasheet §8.6.3: TC1.C1 maps b00→0.75pF, b01→1.5pF, b10→3pF, b11→6pF.
    switch (mode) {
        case LDC_SPEED_ACCURACY_MAX:
            *C1 = 6e-12f;
            *C1_bits = 0b11;
            break;
        case LDC_SPEED_BALANCED_1:
            *C1 = 3e-12f;
            *C1_bits = 0b10;
            break;
        case LDC_SPEED_BALANCED_2:
            *C1 = 1.5e-12f;
            *C1_bits = 0b01;
            break;
        case LDC_SPEED_FAST:
        default:
            *C1 = 0.75e-12f;
            *C1_bits = 0b00;
            break;
    }
}

static void get_c2_settings(ldc_speed_mode_t mode, float* C2, uint8_t* C2_bits) {
    switch (mode) {
        case LDC_SPEED_ACCURACY_MAX:
            *C2 = 24e-12f;
            *C2_bits = 0b11;
            break;
        case LDC_SPEED_BALANCED_1:
            *C2 = 12e-12f;
            *C2_bits = 0b10;
            break;
        case LDC_SPEED_BALANCED_2:
            *C2 = 6e-12f;
            *C2_bits = 0b01;
            break;
        case LDC_SPEED_FAST:
        default:
            *C2 = 3e-12f;
            *C2_bits = 0b00;
            break;
    }
}

static rp_config_t ldc1101_make_rp_set(float L_h, float Q, float C_sensor) {
    rp_config_t result;

    float f = 1.0f / (2.0f * (float)M_PI * sqrtf(L_h * C_sensor));
    float Rp_est = Q * 2.0f * (float)M_PI * f * L_h;

    uint8_t high_q_bit = (Q > 50.0f) ? 1 : 0;

    float upper = 1.25f * Rp_est;
    float lower = 0.75f * Rp_est;

    uint8_t rp_max_bits = rp_table[0].bits;
    uint8_t rp_min_bits = rp_table[7].bits;

    float rp_max_val = rp_table[0].resistance;
    float rp_min_val = rp_table[7].resistance;

    for (int i = 7; i >= 0; i--) {
        if (rp_table[i].resistance >= upper) {
            rp_max_bits = rp_table[i].bits;
            rp_max_val = rp_table[i].resistance;
            break;
        }
    }

    // rp_table is sorted largest→smallest, so iterate i=0..7 to pick the
    // largest bin ≤ lower (per datasheet §9.1.4 step 3). If nothing qualifies,
    // rp_min_val stays at the initial 750 Ω (rp_table[7]).
    for (int i = 0; i <= 7; i++) {
        if (rp_table[i].resistance <= lower) {
            rp_min_bits = rp_table[i].bits;
            rp_min_val = rp_table[i].resistance;
            break;
        }
    }

    if (rp_min_val > rp_max_val) {
        rp_min_bits = rp_max_bits;
        rp_min_val = rp_max_val;
    }

    result.reg = (high_q_bit << 7) | (rp_max_bits << 4) | (rp_min_bits);
    result.rp_min_ohms = rp_min_val;
    result.rp_max_ohms = rp_max_val;
    result.high_q = high_q_bit;

    return result;
}

static uint8_t ldc1101_make_tc1(float L_h, float C_sensor, ldc_speed_mode_t mode) {
    const float VAMP = 0.6f;

    float C1;
    uint8_t C1_bits;
    get_c1_settings(mode, &C1, &C1_bits);

    float f = 1.0f / (2.0f * (float)M_PI * sqrtf(L_h * C_sensor));
    float f_min = 0.9f * f;

    float R1_required = sqrtf(2.0f) / ((float)M_PI * VAMP * f_min * C1);

    float code_f = (417000.0f - R1_required) / 12770.0f;
    int code = (int)roundf(code_f);

    if (code < 0) code = 0;
    if (code > 31) code = 31;

    return (uint8_t)((C1_bits << 6) | code);
}

static uint8_t ldc1101_make_tc2(float C_sensor, float rp_min_ohms, ldc_speed_mode_t mode) {
    float C2;
    uint8_t C2_bits;

    get_c2_settings(mode, &C2, &C2_bits);

    float R2_required = (2.0f * rp_min_ohms * C_sensor) / C2;
    float code_f = (835000.0f - R2_required) / 12770.0f;
    int code = (int)roundf(code_f);

    if (code < 0) code = 0;
    if (code > 63) code = 63;

    return (uint8_t)((C2_bits << 6) | code);
}

static dig_conf_result_t ldc1101_make_dig_conf(float L_h, float C_sensor, ldc_speed_mode_t mode) {
    dig_conf_result_t result;

    float f_sensor = 0.8f * (1.0f / (2.0f * (float)M_PI * sqrtf(L_h * C_sensor)));
    float min_freq_calc = 16.0f - (8000000.0f / f_sensor);

    if (min_freq_calc < 0) min_freq_calc = 0;
    if (min_freq_calc > 15) min_freq_calc = 15;

    uint8_t min_freq_bits = (uint8_t)min_freq_calc;

    uint8_t resp_time_bits;
    float response_time;

    switch (mode) {
        case LDC_SPEED_ACCURACY_MAX:
            resp_time_bits = 0b111;
            response_time = 6144.0f;
            break;
        case LDC_SPEED_BALANCED_1:
            resp_time_bits = 0b101;
            response_time = 2048.0f;
            break;
        case LDC_SPEED_BALANCED_2:
            resp_time_bits = 0b011;
            response_time = 512.0f;
            break;
        case LDC_SPEED_FAST:
        default:
            resp_time_bits = 0b001;
            response_time = 128.0f;
            break;
    }

    float conversion_time = response_time / (3.0f * f_sensor);

    result.reg = (uint8_t)((min_freq_bits << 4) | resp_time_bits);
    result.response_time_periods = response_time;
    result.conversion_time_sec = conversion_time;

    return result;
}

static spi_device_handle_t spi;

static void spi_init(void) {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = LDC_MOSI;
    bus.miso_io_num = LDC_MISO;
    bus.sclk_io_num = LDC_SCK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(LDC_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = LDC_SPI_CLOCK_HZ;
    dev.mode = 0;
    dev.spics_io_num = LDC_CS;
    dev.queue_size = 1;
    dev.cs_ena_pretrans = 2;
    dev.cs_ena_posttrans = 2;

    ESP_ERROR_CHECK(spi_bus_add_device(LDC_SPI_HOST, &dev, &spi));
}

static uint8_t ldc_read(uint8_t addr) {
    uint8_t tx[2] = {(uint8_t)(addr | 0x80), 0x00};
    uint8_t rx[2] = {0};

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    return rx[1];
}

static void ldc_write(uint8_t addr, uint8_t val) {
    uint8_t tx[2] = {(uint8_t)(addr & 0x7F), val};

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;

    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

void ldc1101_configure(float L_h, float C_sensor, float Q, ldc1101_mode_t mode,
                       ldc_speed_mode_t speed_mode, int switch_enable, int switch_gpio) {
    if (switch_gpio >= 0) {
        gpio_config_t io_conf = {};
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << switch_gpio);
        gpio_config(&io_conf);

        gpio_set_level((gpio_num_t)switch_gpio, switch_enable ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    current_mode = mode;

    rp_cfg_global = ldc1101_make_rp_set(L_h, Q, C_sensor);
    uint8_t tc1_reg = ldc1101_make_tc1(L_h, C_sensor, speed_mode);
    uint8_t tc2_reg = ldc1101_make_tc2(C_sensor, rp_cfg_global.rp_min_ohms, speed_mode);
    dig_cfg_global = ldc1101_make_dig_conf(L_h, C_sensor, speed_mode);

    ldc_write(REG_START_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    ldc_write(REG_RP_SET, rp_cfg_global.reg);
    ldc_write(REG_TC1, tc1_reg);
    ldc_write(REG_TC2, tc2_reg);
    ldc_write(REG_DIG_CONF, dig_cfg_global.reg);

    if (mode == LDC1101_MODE_LHR) {
        if (ldc_read(REG_START_CONFIG) != 0x01) {
            printf("WARNING: Failed to enter sleep mode\n");
            abort();
        }

        ldc_write(REG_ALT_CONFIG, 0x01);
        ldc_write(REG_D_CONFIG, 0x01);
        lhr_rcount = 0xFFFF;
        ldc_write(REG_LHR_RCOUNT_LSB, (uint8_t)(lhr_rcount & 0xFF));
        ldc_write(REG_LHR_RCOUNT_MSB, (uint8_t)((lhr_rcount >> 8) & 0xFF));
        ldc_write(REG_LHR_OFFSET_MSB, 0x00);
        ldc_write(REG_LHR_OFFSET_LSB, 0x00);
    } else {
        ldc_write(REG_ALT_CONFIG, 0x00);
        // §9.1.4 requires DOK_REPORT=0 for RP measurements; force in case a
        // previous LHR configuration left it set.
        ldc_write(REG_D_CONFIG, 0x00);
    }

    ldc_write(REG_INTB_MODE, 0x00);
    ldc_write(REG_START_CONFIG, 0x00);
}

ldc1101_measurement_t ldc1101_read(float C_sensor) {
    ldc1101_measurement_t result;

    if (current_mode == LDC1101_MODE_LHR) {
        // §9.1.10 Eq 14: t_CONV = (55 + RCOUNT × 16) / f_CLKIN
        float lhr_time_sec = (55.0f + (float)lhr_rcount * 16.0f) / F_CLKIN;
        uint32_t delay_ms = (uint32_t)(lhr_time_sec * 1000.0f) + 1;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        result.Rp_ohms = NAN;
    } else {
        // Poll DRDYB; bail out on NO_SENSOR_OSC or timeout so a misconfigured
        // chip can't deadlock the caller.
        const uint32_t timeout_ms = 100;
        uint32_t waited_ms = 0;
        uint8_t status;
        while (1) {
            status = ldc_read(REG_STATUS);
            if (!(status & (1 << 6))) break;            // DRDYB cleared
            if (status & (1 << 7)) {                    // NO_SENSOR_OSC
                printf("LDC1101: NO_SENSOR_OSC (status=0x%02X) — RPMIN likely too high for sensor\n", status);
                result.Rp_ohms = NAN;
                result.L_uH = NAN;
                result.timestamp_ms = esp_log_timestamp();
                return result;
            }
            if (waited_ms >= timeout_ms) {
                printf("LDC1101: DRDYB timeout (status=0x%02X)\n", status);
                result.Rp_ohms = NAN;
                result.L_uH = NAN;
                result.timestamp_ms = esp_log_timestamp();
                return result;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            waited_ms++;
        }

        uint8_t rp_lsb = ldc_read(REG_RP_DATA_LSB);
        uint8_t rp_msb = ldc_read(REG_RP_DATA_MSB);
        uint16_t rp_raw = (uint16_t)(((uint16_t)rp_msb << 8) | rp_lsb);

        float x = (float)rp_raw / 65535.0f;
        float Rp_ohms;

        if (rp_cfg_global.high_q) {
            Rp_ohms = rp_cfg_global.rp_min_ohms / (1.0f - x);
        } else {
            Rp_ohms = (rp_cfg_global.rp_max_ohms * rp_cfg_global.rp_min_ohms) /
                      (rp_cfg_global.rp_max_ohms * (1.0f - x) + rp_cfg_global.rp_min_ohms * x);
        }

        result.Rp_ohms = Rp_ohms;
    }

    float f;

    if (current_mode == LDC1101_MODE_LHR) {
        uint8_t lhr_lsb = ldc_read(REG_LHR_DATA_LSB);
        uint8_t lhr_mid = ldc_read(REG_LHR_DATA_MID);
        uint8_t lhr_msb = ldc_read(REG_LHR_DATA_MSB);

        uint32_t lhr_raw = ((uint32_t)lhr_msb << 16) | ((uint32_t)lhr_mid << 8) | lhr_lsb;

        if (lhr_raw == 0) {
            f = 0.0f;
        } else {
            f = (F_CLKIN * (float)lhr_raw) / 16777216.0f;
        }
    } else {
        uint8_t l_lsb = ldc_read(REG_L_DATA_LSB);
        uint8_t l_msb = ldc_read(REG_L_DATA_MSB);
        uint16_t l_raw = (uint16_t)(((uint16_t)l_msb << 8) | l_lsb);

        if (l_raw == 0) {
            f = 0.0f;
        } else {
            f = (F_CLKIN * dig_cfg_global.response_time_periods) / (3.0f * l_raw);
        }
    }

    float L_val = 1.0f / ((2.0f * (float)M_PI * f) * (2.0f * (float)M_PI * f) * C_sensor);

    result.L_uH = L_val * 1e6f;
    result.timestamp_ms = esp_log_timestamp();

    return result;
}

void ldc1101_init(void) {
    spi_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("CHIP_ID = 0x%02X\n", ldc_read(REG_CHIP_ID));
}

void ldc1101_sleep(void) {
    ldc_write(REG_START_CONFIG, 0x01);
}

void ldc1101_wake(void) {
    ldc_write(REG_START_CONFIG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
}
