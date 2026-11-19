#ifndef MANGO_NAV_CORE_H
#define MANGO_NAV_CORE_H
#include "nav_can.h"
#include "nav_config.h"
typedef enum { NAV_BOOT, NAV_READY, NAV_RUN, NAV_ALIGN, NAV_WAIT_LIDAR,
               NAV_PAUSING, NAV_PAUSED, NAV_STOPPED, NAV_DONE } NavState;
typedef enum { NAV_OK, NAV_NEED_CALIBRATION, NAV_NEED_IMU, NAV_NEED_LIDAR,
               NAV_NEED_X, NAV_NEED_Y, NAV_BAD_SCENE, NAV_TOO_CLOSE,
               NAV_DISABLED, NAV_USER_STOP, NAV_WORK_PAUSE } NavReason;
typedef struct {
    uint32_t now_ms;
    float yaw_rad, yaw_rate_rad_s, wheel_max_mps, forward_step_mm;
    uint8_t imu_ready, enabled, start, stop, pause, resume;
    int8_t loaded_lateral; /* 接近满 PWM 的横移方向：左 +1、右 -1 */
} NavInput;
typedef struct {
    float vx, vy, wz; /* 车体坐标：前+、左+、逆时针+ */
    uint8_t drive_enabled, quick_stop, reset_pi, work_stopped;
} NavOutput;
typedef struct {
    float samples[NAV_FILTER_FRAMES], value;
    uint32_t last_sequence;
    uint8_t count, next, role;
} NavFilter;
typedef struct {
    const NavCalibration *cal;
    NavState state;
    NavReason reason;
    NavOutput output;
    NavLidar lidar;
    NavFilter filter[4];
    uint32_t last_sequence;
    float start_front_mm, heading_origin_rad;
    float main_error_mm, cross_error_mm, heading_error_rad;
    float last_x_mm, last_front_mm; /* 最近采信的场地左距和前距 */
    uint8_t position_axes;
    uint8_t calibrated, target, confirmed, estimated_point;
    uint8_t arrival_count, recovery_count, zero_reset_done, pause_latched;
    uint8_t cross_nudge_state;
    uint32_t cross_nudge_since_ms;
    float cross_nudge_remaining_mm;
    /* 只读遥测镜像；不作为控制输入。目标与本次 observe 使用的参考一致。 */
    float reference_target_mm[4];
    uint8_t reference_mask;
} NavCore;
void Nav_Init(NavCore *n, const NavCalibration *cal, int calibrated);
float Nav_HeadingError(float reference_rad,float yaw_rad);
float Nav_HeadingCorrection(float error_rad,float yaw_rate_rad_s);
void Nav_Tick(NavCore *n, const NavInput *in, const NavLidar *lidar);
int Nav_IsActive(const NavCore *n);
const char *Nav_StateName(NavState state);
const char *Nav_ReasonName(NavReason reason);
#endif
