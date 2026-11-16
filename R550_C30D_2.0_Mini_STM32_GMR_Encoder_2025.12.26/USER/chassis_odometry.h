#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H
#include <stdint.h>

/* 起跑时车头方向为 +x，车左为 +y；保持车头朝图下时，地图向上对应 -x。 */
typedef struct {
    float x_mm, y_mm;
    float yaw_rad;
    float vx_mps, vy_mps; /* 本次编码器位移 / 实际采样间隔得到的车体反馈速度。 */
    uint32_t tick_ms;
    uint32_t sample_dt_ms, max_sample_dt_ms; /* 实际间隔及本次起跑以来的最大间隔。 */
    uint8_t valid;
} ChassisOdomFrame;

void ChassisOdom_Reset(uint32_t now_ms);
/* 每次读取并清零编码器后调用一次，传入本次计数对应的位移，单位 mm。
 * 平移直接累加；只有 IMU 角速度积分使用实际时间，避免采样抖动放大位移。 */
void ChassisOdom_UpdateDelta(float forward_mm, float lateral_mm, short gyro_z,
                            uint32_t now_ms, uint8_t calibrated);
void ChassisOdom_Read(ChassisOdomFrame *frame);
/* 只校正当前地标能观测的轴；0 为 x，1 为 y。 */
void ChassisOdom_CorrectAxis(uint8_t axis, float position_mm);
#endif
