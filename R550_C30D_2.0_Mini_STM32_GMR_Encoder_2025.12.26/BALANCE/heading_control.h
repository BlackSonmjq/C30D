#ifndef __HEADING_CONTROL_H
#define __HEADING_CONTROL_H

#include "sys.h"

/*
 * Chassis-side IMU heading hold.
 *
 * The controller runs from Balance_task at 100 Hz and uses the Z-axis gyro
 * sample after the IMU task has removed its startup bias. These defaults are
 * the same as the former ROS heading_controller node.
 */
#define HEADING_CONTROL_ENABLE                1
#define HEADING_CONTROL_RATE_HZ               100.0f
#define HEADING_CONTROL_GYRO_FULL_SCALE_DPS   500.0f
#define HEADING_CONTROL_KP                    1.32f
#define HEADING_CONTROL_KI                    0.0f
#define HEADING_CONTROL_KD                    0.18f
#define HEADING_CONTROL_MAX_ANGULAR_VELOCITY  1.0f
#define HEADING_CONTROL_MIN_ANGULAR_VELOCITY  0.08f
#define HEADING_CONTROL_INTEGRAL_LIMIT        0.5f
#define HEADING_CONTROL_YAW_TOLERANCE         0.01f
#define HEADING_CONTROL_GYRO_TOLERANCE        0.03f
#define HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE 0.002f

void HeadingControl_Reset(void);
float HeadingControl_Update(short gyro_z_raw);

#endif
