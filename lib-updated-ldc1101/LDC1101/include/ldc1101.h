#ifndef LDC1101_H
#define LDC1101_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LDC1101_MODE_RP_L,
    LDC1101_MODE_LHR
} ldc1101_mode_t;

typedef enum {
    LDC_SPEED_ACCURACY_MAX = 1,
    LDC_SPEED_BALANCED_1,
    LDC_SPEED_BALANCED_2,
    LDC_SPEED_FAST
} ldc_speed_mode_t;

typedef struct {
    float Rp_ohms;
    float L_uH;
    uint32_t timestamp_ms;
} ldc1101_measurement_t;

void ldc1101_init(void);

void ldc1101_configure(
    float L_h,
    float C_sensor,
    float Q,
    ldc1101_mode_t mode,
    ldc_speed_mode_t speed_mode,
    int switch_enable,
    int switch_gpio);

ldc1101_measurement_t ldc1101_read(float C_sensor);

void ldc1101_sleep(void);
void ldc1101_wake(void);

#ifdef __cplusplus
}
#endif

#endif
