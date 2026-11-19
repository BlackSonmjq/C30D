#ifndef MANGO_NAV_STM32_H
#define MANGO_NAV_STM32_H
#include "nav_core.h"
typedef struct {
    uint32_t now_ms, lidar_sequence, lidar_age_ms;
    uint16_t raw_mm[4];
    float yaw_rad, main_error_mm, cross_error_mm, wheel_max_mps;
    float command[3], wheel_target[4], wheel_actual[4], wheel_pwm[4];
    uint8_t state, reason, target, confirmed, estimated_point, work_stopped, imu_ready;
    float filtered_mm[4], reference_target_mm[4], heading_error_rad;
    float battery_voltage, start_front_mm;
    uint8_t valid_mask, reference_mask, can_fresh;
    uint8_t motor_allowed, hardware_enable, software_stop, manual_mode;
} NavStatus;
void NavBoard_Init(void);
void NavBoard_Tick1ms(void); /* SysTick: 不调用 FreeRTOS API */
uint32_t NavBoard_Millis(void);
void NavBoard_CanRx(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t dlc,const uint8_t *data);
void NavBoard_PublishImu(float yaw_rad,int ready);
/* 每个有效报告发布一次。分别传入模式键（START/Menu）和导航启动键（△/Y）的按下状态。 */
void RemoteBoard_Publish(uint8_t ly,uint8_t rx,int mode_pressed,int nav_start_pressed);
void RemoteBoard_Disconnect(void);
void NavBoard_ControlStep(NavOutput *out);
void NavBoard_PublishDrive(int motor_allowed);
void NavBoard_ReadStatus(NavStatus *out);
void NavBoard_Draw(void);
void NavBoard_DebugSend(void);
/* 可由将来的机构任务调用；不直接改速度。 */
void Nav_RequestWorkPause(void);
void Nav_RequestWorkContinue(void);
int Nav_WorkStopped(void);
#endif
