#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include <stdint.h>

/* Dedicated software I2C: PA8=SCL, PC9=SDA.
 * Sensor's current 7-bit address, without the read/write bit.
 * Changing this macro does not reprogram the sensor's hardware address. */
#define VL53L0X_I2C_ADDR 0x29U
#define VL53L0X_TIMEOUT_MS 200U
#define VL53L0X_TIMING_BUDGET_US 100000UL

typedef enum {
    VL53L0X_OK = 0,
    VL53L0X_ERROR_I2C,
    VL53L0X_ERROR_TIMEOUT,
    VL53L0X_ERROR_ID,
    VL53L0X_ERROR_ARGUMENT,
    VL53L0X_ERROR_STATE,
    VL53L0X_ERROR_RANGE,
    VL53L0X_NOT_READY,
    VL53L0X_ERROR_CONFIG
} VL53L0X_Status;

typedef struct {
    uint16_t distance_mm;
    uint8_t raw_range_status; /* Device register bits [6:3], NOT ST API RangeStatus. */
} VL53L0X_Result;

/* Call delay_init(168) first. XSHUT must be high; no GPIO1 connection needed.
 * One task owns this driver. Calls must not overlap or run from an ISR.
 * Init performs SPAD setup, default tuning and VHV/phase calibration.
 */
VL53L0X_Status VL53L0X_Init(void);
VL53L0X_Status VL53L0X_ReadSingle(VL53L0X_Result *result);
VL53L0X_Status VL53L0X_StartContinuous(void);
VL53L0X_Status VL53L0X_StopContinuous(void);
/* Nonblocking sensor readiness check; NOT_READY leaves result unchanged. */
VL53L0X_Status VL53L0X_ReadContinuous(VL53L0X_Result *result);

#endif
