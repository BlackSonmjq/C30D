#include "route_debug.h"
#include "route_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static RouteDebugFrame latest_frame, fault_frame;
static uint8_t frame_seen, fault_seen;
typedef char RouteFloatMustHaveFourBytes[(sizeof(float) == 4U) ? 1 : -1];

void RouteDebug_Reset(void)
{
    taskENTER_CRITICAL();
    memset(&latest_frame, 0, sizeof(latest_frame));
    memset(&fault_frame, 0, sizeof(fault_frame));
    frame_seen = fault_seen = 0U;
    taskEXIT_CRITICAL();
}

void RouteDebug_Capture(const RouteController *controller, const RouteInputs *in,
                        float vx_mps, float vy_mps)
{
    RouteDebugFrame frame;
    RouteLidarDirection sensor = controller->status.sensor;
    frame.route = controller->status;
    frame.odom = in->odom;
    frame.tick_ms = in->now_ms;
    frame.lidar_sequence = in->lidar.sequence;
    memcpy(frame.raw_mm, in->lidar.mm, sizeof(frame.raw_mm));
    frame.command_vx_mps = vx_mps;
    frame.command_vy_mps = vy_mps;
    frame.can_fresh = in->lidar.fresh;
    frame.frozen = 0U;
    frame.lidar_age_ms = sensor < ROUTE_LIDAR_COUNT && controller->laser[sensor].count == 3U ?
        (float)(uint32_t)(in->now_ms - controller->laser[sensor].valid_at) : -1.0f;

    /* 只复制内存，不在导航任务中格式化字符串或等待串口。
     * 即使停车后搬动车，第一份 FAULT 输入和计算结果也不会被覆盖。 */
    taskENTER_CRITICAL();
    latest_frame = frame;
    frame_seen = 1U;
    if (!fault_seen && frame.route.phase == ROUTE_FAULT) {
        fault_frame = frame;
        fault_frame.frozen = 1U;
        fault_seen = 1U;
    }
    taskEXIT_CRITICAL();
}

uint8_t RouteDebug_Read(RouteDebugFrame *frame)
{
    uint8_t seen;
    taskENTER_CRITICAL();
    seen = frame_seen;
    if (seen) *frame = fault_seen ? fault_frame : latest_frame;
    taskEXIT_CRITICAL();
    return seen;
}

uint8_t RouteDebug_ReadFault(RouteDebugFrame *frame)
{
    uint8_t seen;
    taskENTER_CRITICAL();
    seen = fault_seen;
    if (seen) *frame = fault_frame;
    taskEXIT_CRITICAL();
    return seen;
}

uint16_t RouteDebug_BuildPacket(uint8_t *bytes, uint16_t capacity)
{
    RouteDebugFrame f;
    float channels[ROUTE_VOFA_CHANNEL_COUNT];
    uint32_t bits;
    uint16_t channel;
    if (bytes == 0 || capacity < ROUTE_VOFA_FRAME_BYTES || !RouteDebug_Read(&f)) return 0U;

    channels[0] = (float)f.tick_ms;
    channels[1] = (float)f.route.target_point;
    channels[2] = (float)f.route.phase;
    channels[3] = (float)f.route.reason;
    channels[4] = f.route.progress_mm;
    channels[5] = f.route.length_mm;
    channels[6] = (float)f.raw_mm[ROUTE_LIDAR_FRONT];
    channels[7] = (float)f.raw_mm[ROUTE_LIDAR_BEHIND];
    channels[8] = (float)f.raw_mm[ROUTE_LIDAR_LEFT];
    channels[9] = (float)f.raw_mm[ROUTE_LIDAR_RIGHT];
    channels[10] = (float)f.route.measured_mm;
    channels[11] = (float)f.route.target_mm;
    channels[12] = f.route.odom_remaining_mm;
    channels[13] = f.route.laser_remaining_mm;
    channels[14] = f.route.mismatch_mm;
    channels[15] = (float)f.odom.sample_dt_ms;
    channels[16] = (float)f.odom.max_sample_dt_ms;
    channels[17] = f.odom.yaw_rad * (180.0f / 3.14159265358979323846f);
    channels[18] = f.command_vx_mps;
    channels[19] = f.command_vy_mps;
    channels[20] = f.odom.vx_mps;
    channels[21] = f.odom.vy_mps;
    channels[22] = (float)f.route.stop_flags;
    channels[23] = (float)f.frozen;
    channels[24] = (float)f.lidar_sequence;
    channels[25] = f.lidar_age_ms;
    channels[26] = (float)f.can_fresh;
    channels[27] = (float)f.odom.valid;
    channels[28] = f.odom.x_mm;
    channels[29] = f.odom.y_mm;
    channels[30] = f.route.gate_mm;
    channels[31] = (float)f.route.lane;
    channels[32] = f.route.estimated_end_mm;
    channels[33] = f.route.control_remaining_mm;
    channels[34] = f.route.wall_innovation_mm;
    channels[35] = (float)f.route.wall_locked;
    channels[36] = (float)f.route.capture_confirmed;
    channels[37] = (float)f.route.odom_untrusted;
    channels[38] = (float)f.route.search_elapsed_ms;
    channels[39] = (float)f.route.reference_idle_ms;
    channels[40] = (float)f.route.confidence;
    channels[41] = (float)f.route.reject_reason;
    channels[42] = (float)f.route.good_frames;
    channels[43] = (float)f.route.bad_frames;
    channels[44] = (float)f.route.speed_limited;
    channels[45] = (float)f.route.odom_warn_count;
    channels[46] = (float)f.route.recovering;
    channels[47] = (float)f.route.recovery_resets;
    channels[48] = (float)f.route.edge_state;
    channels[49] = (float)f.route.edge_confirmed;
    channels[50] = f.route.edge_last_near_mm;
    channels[51] = f.route.edge_first_far_mm;
    channels[52] = f.route.edge_anchor_mm;
    channels[53] = f.route.edge_uncertainty_mm;
    channels[54] = (float)f.route.aux_sensor;
    channels[55] = (float)f.route.aux_measured_mm;
    channels[56] = (float)f.route.aux_target_mm;
    channels[57] = f.route.aux_remaining_mm;
    channels[58] = (float)f.route.aux_confidence;
    channels[59] = (float)f.route.aux_reject_reason;
    channels[60] = (float)f.route.aux_good_frames;
    channels[61] = (float)f.route.aux_bad_frames;
    channels[62] = (float)f.route.control_source;
    channels[63] = f.route.edge_speed_cap_mps;

    /* 显式输出小端字节，不依赖结构体填充或发送端的整数端序。 */
    for (channel = 0U; channel < ROUTE_VOFA_CHANNEL_COUNT; ++channel) {
        memcpy(&bits, &channels[channel], sizeof(bits));
        bytes[channel * 4U] = (uint8_t)bits;
        bytes[channel * 4U + 1U] = (uint8_t)(bits >> 8);
        bytes[channel * 4U + 2U] = (uint8_t)(bits >> 16);
        bytes[channel * 4U + 3U] = (uint8_t)(bits >> 24);
    }
    bytes[ROUTE_VOFA_FRAME_BYTES - 4U] = 0x00U;
    bytes[ROUTE_VOFA_FRAME_BYTES - 3U] = 0x00U;
    bytes[ROUTE_VOFA_FRAME_BYTES - 2U] = 0x80U;
    bytes[ROUTE_VOFA_FRAME_BYTES - 1U] = 0x7FU;
    return (uint16_t)ROUTE_VOFA_FRAME_BYTES;
}
