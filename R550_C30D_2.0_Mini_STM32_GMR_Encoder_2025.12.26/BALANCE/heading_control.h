#ifndef __HEADING_CONTROL_H
#define __HEADING_CONTROL_H

#include "sys.h"

/* 重载底盘 IMU 航向保持初始参数，运行于 100 Hz 底盘任务。
 * 角度单位 rad，角速度单位 rad/s；陀螺仪使用启动校准后的 Z 轴值。
 * 此组参数用于降低过偏，仍需按实际载荷、地面和电机响应标定。
 * 手动航向保持、路线航向保持、原串口航向保持共用本控制器。 */
#define HEADING_CONTROL_ENABLE                1       // 航向保持开关
#define HEADING_CONTROL_RATE_HZ               100.0f  // 控制频率，Hz
#define HEADING_CONTROL_GYRO_FULL_SCALE_DPS   500.0f   // 陀螺仪量程，度/秒，与 IMU 配置一致
#define HEADING_CONTROL_KP                    0.80f   // 比例：原 1.32，降低同角度误差下的驱动力度
#define HEADING_CONTROL_KI                    0.0f    // 积分：重载试调阶段保持关闭
#define HEADING_CONTROL_KD                    0.40f   // 角速度阻尼：原 0.18，提前抑制仍在继续的旋转
#define HEADING_CONTROL_MAX_ANGULAR_VELOCITY   0.35f   // 最大纠偏角速度：原 1.0，限制大角度纠偏强度
#define HEADING_CONTROL_MIN_ANGULAR_VELOCITY   0.08f   // 大误差且近乎静止时的起转助力，不是永久最低输出
#define HEADING_CONTROL_MIN_FULL_ERROR_RAD    0.0872665f // 5度：小于此误差时，起转助力随误差逐渐减小
#define HEADING_CONTROL_MIN_ASSIST_RATE       0.03f   // 超过此角速度不再强加最低起转输出，保留阻尼结果
#define HEADING_CONTROL_INTEGRAL_LIMIT        0.5f    // 积分状态限幅
#define HEADING_CONTROL_YAW_TOLERANCE          0.01f   // 航向容差，约 0.57度
#define HEADING_CONTROL_GYRO_TOLERANCE         0.03f   // 同时满足角度和角速度容差后输出零
#define HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE 0.002f // 角速度积分死区

void HeadingControl_Reset(void);
float HeadingControl_Update(short gyro_z_raw);
float HeadingControl_HoldYaw(float target_yaw, float measured_yaw, short gyro_z_raw);

#endif
