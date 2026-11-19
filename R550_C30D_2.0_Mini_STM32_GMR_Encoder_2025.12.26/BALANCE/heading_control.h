#ifndef __HEADING_CONTROL_H
#define __HEADING_CONTROL_H

#include "sys.h"
#include "nav_config.h"

/* Trial: use REBUILD's PD yaw hold for manual, route and visual motion.
 * Gains/tolerances come from nav_config.h; the sensor task owns odometry yaw.
 * Units: rad and rad/s. No integral or minimum-turn boost is applied. */
#define HEADING_CONTROL_ENABLE                1       // 航向保持开关
#define HEADING_CONTROL_RATE_HZ               100.0f  // 控制频率，Hz
#define HEADING_CONTROL_GYRO_FULL_SCALE_DPS   (32768.0f / NAV_GYRO_LSB_PER_DPS)   // 陀螺仪量程，度/秒，与 IMU 配置一致
#define HEADING_CONTROL_KP                    NAV_HEADING_KP
#define HEADING_CONTROL_KI                    0.0f   // REBUILD uses PD only
#define HEADING_CONTROL_KD                    NAV_HEADING_KD
#define HEADING_CONTROL_MAX_ANGULAR_VELOCITY   NAV_HEADING_MAX_RAD_S
#define HEADING_CONTROL_MIN_ANGULAR_VELOCITY   0.0f   // 大误差且近乎静止时的起转助力，不是永久最低输出
#define HEADING_CONTROL_MIN_FULL_ERROR_RAD    0.0523599f // Full startup assist at 3 degrees
#define HEADING_CONTROL_MIN_ASSIST_RATE       0.03f   // 超过此角速度不再强加最低起转输出，保留阻尼结果
#define HEADING_CONTROL_INTEGRAL_RELEASE_RAD   0.003f  // Release parked effort within about 0.17 degrees
#define HEADING_CONTROL_INTEGRAL_LIMIT        0.5f    // 积分状态限幅
#define HEADING_CONTROL_YAW_TOLERANCE          NAV_HEADING_TOL_RAD
#define HEADING_CONTROL_GYRO_TOLERANCE         NAV_HEADING_RATE_TOL_RAD_S
#define HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE 0.0f // 角速度积分死区

float HeadingControl_NormalizeAngle(float angle);
void HeadingControl_Reset(void);
float HeadingControl_GetIntegralOutput(void);
float HeadingControl_Update(short gyro_z_raw);
float HeadingControl_HoldYaw(float target_yaw, float measured_yaw, short gyro_z_raw);

#endif
