#ifndef MANGO_NAV_CONFIG_H
#define MANGO_NAV_CONFIG_H

/* 日常只调这一组。距离 mm，速度 m/s，角度 rad。 */
#define NAV_CALIBRATED             1 /* 实车标定 nav_calibration.c 后改为 1 */
#define NAV_SPEED_CRUISE           0.28f
#define NAV_SPEED_APPROACH         0.18f
#define NAV_SPEED_LONGITUDINAL_APPROACH 0.12f
#define NAV_SPEED_CROSS_CRUISE     0.48f
#define NAV_SPEED_CROSS_APPROACH   0.18f
#define NAV_SPEED_ALIGN            0.18f    
#define NAV_CROSS_NUDGE_MPS        0.08f 
#define NAV_CROSS_NUDGE_MM         10.0f
#define NAV_POINT_TOL_MM           50.0f
#define NAV_CENTER_DEADBAND_MM     10.0f
#define NAV_CROSS_SLOWDOWN_MM      800.0f
#define NAV_LONGITUDINAL_SLOWDOWN_MM 300.0f
#define NAV_SLOWDOWN_MM            300.0f /* 返程和退场 */
#ifndef NAV_FILTER_FRAMES
#define NAV_FILTER_FRAMES          1U /* 可选 1、3、5；到点/恢复共用此窗口 */
#endif

/* 固定实现参数，通常不用逐点调整。 */
#define NAV_BLE_TELEMETRY_ENABLE    1
#define NAV_BLE_BAUD                9600U
#define NAV_BLE_PERIOD_MS           300U
#define NAV_LIDAR_TTL_MS            300U
#define NAV_LIDAR_MIN_MM            1U
#define NAV_LIDAR_MAX_MM            6000U
#define NAV_POSITION_KP            0.0030f /* m/s per mm */
#define NAV_CENTER_KP              0.0015f
#define NAV_SCENE_TOL_MM           100.0f
#define NAV_SIDE_FIELD_BAND_MM     80.0f
#define NAV_ENTRY_CAPTURE_MM       80.0f
#define NAV_HEADING_KP             1.8f
#define NAV_HEADING_KD             0.75f
#define NAV_HEADING_MAX_RAD_S      0.25f
#define NAV_HEADING_TOL_RAD        0.035f
#define NAV_HEADING_RATE_TOL_RAD_S 0.03f
#define NAV_STOP_WHEEL_MPS         0.02f
#define NAV_CROSS_STALL_MS         300U
#define NAV_CROSS_STALL_PWM_RATIO  0.95f
#define NAV_CROSS_NUDGE_MAX_MS     1500U
#define NAV_ACCEL_MPS2             0.80f
#define NAV_ANGULAR_ACCEL_RAD_S2   1.0f
#define NAV_WHEEL_LIMIT_MPS        0.50f
#define NAV_PWM_LIMIT              16700.0f
#define NAV_WHEEL_KP               300.0f
#define NAV_WHEEL_KI               300.0f
/* 起步补偿按 A/B/C/D 轮分别标定。Kv 单位为 PWM/(m/s)。 */
#define NAV_WHEEL_START_PWM_A      2000.0f
#define NAV_WHEEL_START_PWM_B      2000.0f
#define NAV_WHEEL_START_PWM_C      2000.0f
#define NAV_WHEEL_START_PWM_D      2000.0f
#define NAV_WHEEL_KS_PWM           500.0f
#define NAV_WHEEL_KV_PWM           2000.0f
#define NAV_WHEEL_LOAD_PWM         500.0f
#define NAV_KEY_DEBOUNCE_MS        30U
#define NAV_IMU_CAL_SAMPLES        200U /* 100 Hz 静止采样，约 2 秒 */
#define NAV_IMU_TTL_MS             100U
#define NAV_GYRO_SIGN              1.0f /* 正向转动车头，显示 yaw 应增加；反之改 -1 */
#define NAV_GYRO_LSB_PER_DPS       65.5f /* 本商家工程两款 IMU 均设 ±500 dps */

#if NAV_FILTER_FRAMES != 1 && NAV_FILTER_FRAMES != 3 && NAV_FILTER_FRAMES != 5
#error NAV_FILTER_FRAMES must be 1, 3 or 5.
#endif
#endif
