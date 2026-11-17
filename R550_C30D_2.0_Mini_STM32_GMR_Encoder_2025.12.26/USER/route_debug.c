#include "route_debug.h"
#include "route_config.h"
#include "vision_align.h"
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
    Camera_Data camera;
    VisionAlign_Diagnostics align;
    float channels[ROUTE_VOFA_CHANNEL_COUNT];
    uint32_t bits;
    uint16_t channel;
    if (bytes == 0 || capacity < ROUTE_VOFA_FRAME_BYTES) return 0U;
    /* Visual telemetry remains live before route start and after a route fault. */
    Camera_GetData(&camera);
    channels[0] = (float)camera.pole;
    channels[1] = (float)camera.offset;
    channels[2] = (float)camera.green;
    channels[3] = (float)camera.region;
    channels[4] = (float)camera.bytes;
    channels[5] = (float)camera.frames;
    channels[6] = (float)camera.bad;
    channels[7] = (float)camera.uart_errors;
    VisionAlign_GetDiagnostics(&align);
    channels[8] = (float)align.phase;
    channels[9] = (float)align.locked;
    channels[10] = (float)align.ly;
    channels[11] = (float)align.rx;
    channels[12] = (float)align.stick_active;
    channels[13] = (float)align.ready;
    channels[14] = align.vx_mps;
    channels[15] = align.vy_mps;
    channels[16] = align.wz_radps;
    channels[17] = align.rc_velocity_mmps;
    channels[18] = (float)align.control_flags;
    channels[19] = (float)align.blocked;
    for (channel = 0U; channel < 4U; ++channel) {
        channels[20U + channel] = align.wheel_target[channel];
        channels[24U + channel] = align.wheel_feedback[channel];
        channels[28U + channel] = align.wheel_pwm[channel];
    }

    /* Emit little-endian float32 without depending on structure padding. */
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
