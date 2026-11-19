#ifndef __HEADING_CONTROL_H
#define __HEADING_CONTROL_H

#include "sys.h"
#include "nav_config.h"

/* 重载底盘 IMU 航向保持初始参数，运行于 100 Hz 底盘任务。
 * 角度单位 rad，角速度单位 rad/s；陀螺仪使用启动校准后的 Z 轴值。
 * 此组参数用于降低过偏，仍需按实际载荷、地面和电机响应标定。
 * 手动航向保持、路线航向保持、原串口航向保持共用本控制器。 */
#define HEADING_CONTROL_ENABLE                1       // 航向保持开关
#define HEADING_CONTROL_RATE_HZ               100.0f  // 控制频率，Hz
#define HEADING_CONTROL_GYRO_FULL_SCALE_DPS   (32768.0f / NAV_GYRO_LSB_PER_DPS)   // 陀螺仪量程，度/秒，与 IMU 配置一致
#define HEADING_CONTROL_KP                    2.40f   // 加强同角度误差下的纠偏
#define HEADING_CONTROL_KI                    0.40f   // Limited integral for sustained load bias
#define HEADING_CONTROL_KD                    0.75f   // 配套提高角速度阻尼
#define HEADING_CONTROL_MAX_ANGULAR_VELOCITY   0.80f   // 加大纠偏上限，仍需实车标定
#define HEADING_CONTROL_MIN_ANGULAR_VELOCITY   0.16f   // 大误差且近乎静止时的起转助力，不是永久最低输出
#define HEADING_CONTROL_MIN_FULL_ERROR_RAD    0.0523599f // Full startup assist at 3 degrees
#define HEADING_CONTROL_MIN_ASSIST_RATE       0.03f   // 超过此角速度不再强加最低起转输出，保留阻尼结果
#define HEADING_CONTROL_INTEGRAL_RELEASE_RAD   0.003f  // Release parked effort within about 0.17 degrees
#define HEADING_CONTROL_INTEGRAL_LIMIT        0.5f    // 积分状态限幅
#define HEADING_CONTROL_YAW_TOLERANCE          0.01f   // 航向容差，约 0.57度
#define HEADING_CONTROL_GYRO_TOLERANCE         0.03f   // 同时满足角度和角速度容差后输出零
#define HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE 0.0f // 角速度积分死区
/* Full longitudinal tuning at 120 mm/s; linearly blend below it. */
#define HEADING_CONTROL_LONGITUDINAL_FULL_MPS       0.12f
#define HEADING_CONTROL_LONGITUDINAL_KP             3.60f
#define HEADING_CONTROL_LONGITUDINAL_KD             1.00f
#define HEADING_CONTROL_LONGITUDINAL_YAW_TOLERANCE  0.00349066f /* 0.2 degrees */
#define HEADING_CONTROL_LONGITUDINAL_GYRO_TOLERANCE 0.01f

float HeadingControl_NormalizeAngle(float angle);
void HeadingControl_Reset(void);
float HeadingControl_GetIntegralOutput(void);
float HeadingControl_Update(short gyro_z_raw);
float HeadingControl_HoldYaw(float target_yaw, float measured_yaw, short gyro_z_raw, float forward_mps);

#endif
