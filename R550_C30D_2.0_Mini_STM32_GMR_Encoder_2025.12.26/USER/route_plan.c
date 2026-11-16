#include "route_plan.h"
#include "route_config.h"
#include <stddef.h>

/* 每行描述“驶向 Pxx”的动作。P01～P14 遍历七沟，P15～P17 出场停车。
 * WALL 由粗里程或连续近距接近打开窗口。EDGE 由完整垄端事件和独立围栏参考共同到点。
 * 各点注释的前后左右均指当前车体；侧墙参数只修正横向位置。 */
/* PLAN 宏保留原场地动作方向，通过朝向开关映射到实际车体。
 * 下方各点中文说明按当前反向车体（ROUTE_CHASSIS_REVERSED=1）描述。 */
#if ROUTE_CHASSIS_REVERSED
#define PLAN_BACK ROUTE_FORWARD
#define PLAN_FORWARD ROUTE_BACK
#define PLAN_LEFT ROUTE_RIGHT
#define PLAN_RIGHT ROUTE_LEFT
#define PLAN_LIDAR_FRONT ROUTE_LIDAR_BEHIND
#define PLAN_LIDAR_BEHIND ROUTE_LIDAR_FRONT
#define PLAN_LIDAR_LEFT ROUTE_LIDAR_RIGHT
#define PLAN_LIDAR_RIGHT ROUTE_LIDAR_LEFT
#else
#define PLAN_BACK ROUTE_BACK
#define PLAN_FORWARD ROUTE_FORWARD
#define PLAN_LEFT ROUTE_LEFT
#define PLAN_RIGHT ROUTE_RIGHT
#define PLAN_LIDAR_FRONT ROUTE_LIDAR_FRONT
#define PLAN_LIDAR_BEHIND ROUTE_LIDAR_BEHIND
#define PLAN_LIDAR_LEFT ROUTE_LIDAR_LEFT
#define PLAN_LIDAR_RIGHT ROUTE_LIDAR_RIGHT
#endif

static const RouteSegment segments[ROUTE_POINT_COUNT] = {
    /* P01：前进入场，用前激光定位；到点并停稳后进入 P02。 */
    {PLAN_BACK, ROUTE_REF_WALL, PLAN_LIDAR_BEHIND, 0, 1, ROUTE_ENTRY_LENGTH_MM,
     Front_Lidar_1, PLAN_LIDAR_RIGHT, 120, 0},
    /* P02：第一沟向车体右侧横移，用右激光定位。 */
    {PLAN_LEFT, ROUTE_REF_WALL, PLAN_LIDAR_LEFT, 0, 1, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_2, ROUTE_LIDAR_COUNT, 0, 0},
    /* P03：前进换到第二沟，用后激光定位，距离随前进增大。 */
    {PLAN_BACK, ROUTE_REF_WALL, PLAN_LIDAR_FRONT, 1, 2, ROUTE_LANE_PITCH_MM,
     Behind_Lidar_3, PLAN_LIDAR_LEFT, 120, 0},
    /* P04：第二沟向车体左侧横移，用左激光定位。 */
    {PLAN_RIGHT, ROUTE_REF_WALL, PLAN_LIDAR_RIGHT, 0, 2, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_4, ROUTE_LIDAR_COUNT, 0, 0},
    /* P05：前进换到第三沟；右激光检测普通垄端，前激光独立复核围栏距离。 */
    {PLAN_BACK, ROUTE_REF_EDGE, PLAN_LIDAR_LEFT, 0, 3, ROUTE_LANE_PITCH_MM,
     {ROUTE_LEFT_RIDGE_MM, 150, 15}, PLAN_LIDAR_RIGHT, 120, ROUTE_LEFT_EDGE_TO_CENTER_MM},
    /* P06：第三沟向车体右侧横移，用右激光定位。 */
    {PLAN_LEFT, ROUTE_REF_WALL, PLAN_LIDAR_LEFT, 0, 3, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_6, ROUTE_LIDAR_COUNT, 0, 0},
    /* P07：前进换到第四沟；左激光检测普通垄端，后激光独立复核围栏距离。 */
    {PLAN_BACK, ROUTE_REF_EDGE, PLAN_LIDAR_RIGHT, 0, 4, ROUTE_LANE_PITCH_MM,
     {ROUTE_RIGHT_RIDGE_MM, 150, 15}, PLAN_LIDAR_LEFT, 120, ROUTE_RIGHT_EDGE_TO_CENTER_MM},
    /* P08：第四沟向车体左侧横移，用左激光定位。 */
    {PLAN_RIGHT, ROUTE_REF_WALL, PLAN_LIDAR_RIGHT, 0, 4, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_8, ROUTE_LIDAR_COUNT, 0, 0},
    /* P09：前进换到第五沟；右激光检测缩进 200 mm 的短垄端，前激光独立复核。 */
    {PLAN_BACK, ROUTE_REF_EDGE, PLAN_LIDAR_LEFT, 0, 5, ROUTE_LANE_PITCH_MM,
     {ROUTE_LEFT_SHORT_RIDGE_MM, 150, 15}, PLAN_LIDAR_RIGHT, 120, ROUTE_LEFT_EDGE_TO_CENTER_MM},
    /* P10：第五沟向车体右侧横移，用右激光定位。 */
    {PLAN_LEFT, ROUTE_REF_WALL, PLAN_LIDAR_LEFT, 0, 5, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_10, ROUTE_LIDAR_COUNT, 0, 0},
    /* P11：前进换到第六沟，用前激光定位。 */
    {PLAN_BACK, ROUTE_REF_WALL, PLAN_LIDAR_BEHIND, 0, 6, ROUTE_LANE_PITCH_MM,
     Front_Lidar_11, PLAN_LIDAR_LEFT, 120, 0},
    /* P12：第六沟向车体左侧横移，用左激光定位。 */
    {PLAN_RIGHT, ROUTE_REF_WALL, PLAN_LIDAR_RIGHT, 0, 6, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_12, ROUTE_LIDAR_COUNT, 0, 0},
    /* P13：前进换到第七沟，用前激光定位。 */
    {PLAN_BACK, ROUTE_REF_WALL, PLAN_LIDAR_BEHIND, 0, 7, ROUTE_LANE_PITCH_MM,
     Front_Lidar_13, PLAN_LIDAR_RIGHT, 120, 0},
    /* P14：第七沟向车体右侧横移，用右激光定位，随后返程。 */
    {PLAN_LEFT, ROUTE_REF_WALL, PLAN_LIDAR_LEFT, 0, 7, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_14, ROUTE_LIDAR_COUNT, 0, 0},
    /* P15：倒车返回第一沟，用后激光定位。 */
    {PLAN_FORWARD, ROUTE_REF_WALL, PLAN_LIDAR_FRONT, 0, 1, ROUTE_RETURN_LENGTH_MM,
     Behind_Lidar_15, PLAN_LIDAR_LEFT, 120, 0},
    /* P16：第一沟向车体左侧横移回入口，用左激光定位。 */
    {PLAN_RIGHT, ROUTE_REF_WALL, PLAN_LIDAR_RIGHT, 0, 1, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_16, ROUTE_LIDAR_COUNT, 0, 0},
    /* P17：从 P16 到点位置开始，按里程倒车出场并停车。 */
    {PLAN_FORWARD, ROUTE_REF_ODOM, ROUTE_LIDAR_COUNT, 0, 0, ROUTE_EXIT_LENGTH_MM,
     {0, 150, 15}, PLAN_LIDAR_RIGHT, 120, 0}
};

/* 三组读数由用户确认在正确沟中心、对准正确围栏实测。 */
static const RouteAuxReference auxiliary[] = {
    {5U, PLAN_LIDAR_BEHIND, 0U, Front_Lidar_5},
    {7U, PLAN_LIDAR_FRONT, 1U, Behind_Lidar_7},
    {9U, PLAN_LIDAR_BEHIND, 0U, Front_Lidar_9}
};

const RouteAuxReference *RoutePlan_AuxReference(uint8_t point)
{
    unsigned int i;
    for (i = 0U; i < sizeof(auxiliary) / sizeof(auxiliary[0]); ++i)
        if (auxiliary[i].point == point) return &auxiliary[i];
    return NULL;
}

const RouteSegment *RoutePlan_Get(uint8_t point)
{
    return point >= 1U && point <= ROUTE_POINT_COUNT ? &segments[point - 1U] : NULL;
}

/* 仅放宽 P01 的远围栏量程，其他路径点沿用原有效范围。 */
uint16_t RoutePlan_ReferenceMax(const RouteSegment *segment)
{
    return segment == &segments[0] ? ROUTE_ENTRY_REFERENCE_MAX_MM : ROUTE_REFERENCE_MAX_MM;
}

uint8_t RoutePlan_Calibrated(void)
{
    uint8_t i;
    uint8_t valid = 1U;
    if (!ROUTE_CALIBRATION_READY || ROUTE_ENTRY_LENGTH_MM <= 0.0f ||
        ROUTE_EXIT_LENGTH_MM <= 0.0f || Front_Lidar_Center == 0U ||
        Behind_Lidar_Center == 0U || ROUTE_LEFT_EDGE_TO_CENTER_MM == 0U ||
        ROUTE_RIGHT_EDGE_TO_CENTER_MM == 0U || ROUTE_LASER_GATE_RATIO <= 0.0f ||
        ROUTE_LASER_GATE_RATIO >= 1.0f) valid = 0U;
    for (i = 0U; i < ROUTE_POINT_COUNT; ++i) {
        const RouteSegment *s = &segments[i];
        if (RoutePlan_ReferenceMax(s) > ROUTE_LIDAR_MAX_MM) valid = 0U;
        if (s->length_mm <= 0.0f || s->mark.turn_band_mm >= s->mark.slow_band_mm) valid = 0U;
        if (s->reference != ROUTE_REF_ODOM &&
            (s->mark.target_mm < ROUTE_LIDAR_MIN_MM || s->mark.target_mm > RoutePlan_ReferenceMax(s)))
            valid = 0U;
        if (s->reference == ROUTE_REF_EDGE) {
            const RouteAuxReference *a = RoutePlan_AuxReference((uint8_t)(i + 1U));
            if (!a || a->sensor >= ROUTE_LIDAR_COUNT || a->sensor == s->trigger ||
                a->mark.target_mm < ROUTE_LIDAR_MIN_MM || a->mark.target_mm > ROUTE_LIDAR_MAX_MM ||
                a->mark.turn_band_mm >= a->mark.slow_band_mm ||
                a->mark.turn_band_mm > ROUTE_EDGE_ARRIVAL_VERIFY_MM) valid = 0U;
        }
    }
    return valid && ROUTE_EDGE_NEAR_FRAMES >= 1U && ROUTE_EDGE_NEAR_FRAMES <= 255U &&
           ROUTE_EDGE_FAR_FRAMES >= 1U && ROUTE_EDGE_FAR_FRAMES <= 255U &&
           ROUTE_EDGE_NEAR_MIN_MS < ROUTE_SENSOR_WAIT_MS &&
           ROUTE_EDGE_AUX_WINDOW_MM > ROUTE_EDGE_AUX_REQUIRED_MM &&
           ROUTE_EDGE_AUX_REQUIRED_MM > ROUTE_RECOVER_MAX_MM &&
           ROUTE_EDGE_AUX_PREDICT_MS >= ROUTE_PERIOD_MS &&
           ROUTE_EDGE_AUX_PREDICT_MS < ROUTE_LIDAR_TTL_MS &&
           ROUTE_EDGE_AUX_HISTORY_FRAMES >= 2U && ROUTE_EDGE_AUX_HISTORY_FRAMES <= 255U &&
           ROUTE_EDGE_AUX_RECHECK_SPREAD_MM > 0.0f &&
           ROUTE_EDGE_AUX_RECHECK_SPREAD_MM <= ROUTE_WALL_INNOVATION_MM &&
           ROUTE_EDGE_AUX_RECHECK_MS >= ROUTE_PERIOD_MS &&
           ROUTE_EDGE_AUX_RECHECK_MS < ROUTE_SENSOR_WAIT_MS &&
           ROUTE_EDGE_AUX_RECHECK_MPS >= ROUTE_APPROACH_MIN_MPS &&
           ROUTE_EDGE_AUX_RECHECK_MPS <= ROUTE_SLOW_MPS &&
           ROUTE_EDGE_ARRIVAL_VERIFY_MM > 0.0f && ROUTE_EDGE_ARRIVAL_VERIFY_MM < ROUTE_RECOVER_MAX_MM &&
           ROUTE_EDGE_SETTLE_MS < ROUTE_SENSOR_WAIT_MS && ROUTE_EDGE_SEARCH_EXTRA_MM >= 0.0f &&
           ROUTE_CONFIRM_FRAMES >= 1U && ROUTE_CONFIRM_FRAMES <= 255U &&
           ROUTE_CAPTURE_FRAMES >= 1U && ROUTE_CAPTURE_FRAMES <= 255U &&
           ROUTE_CRUISE_MPS >= ROUTE_SLOW_MPS && ROUTE_SLOW_MPS > 0.0f &&
           ROUTE_APPROACH_KP > 0.0f && ROUTE_APPROACH_MIN_MPS > 0.0f &&
           ROUTE_APPROACH_MIN_MPS <= ROUTE_SLOW_MPS &&
           ROUTE_APPROACH_MIN_MPS <= ROUTE_RECOVER_MPS &&
           ROUTE_RECOVER_MAX_MM > ROUTE_ODOM_TOL_MM && ROUTE_RECOVER_TIMEOUT_MS > 0U &&
           ROUTE_LASER_ODOM_ERROR_MM >= 0.0f && ROUTE_WALL_INNOVATION_MM > 0.0f &&
           ROUTE_WALL_STEP_NOISE_MM > 0.0f && ROUTE_WALL_CAPTURE_PROGRESS_MM > 0.0f &&
           ROUTE_WALL_PROGRESS_STEP_MM > 0.0f &&
           ROUTE_WALL_SEARCH_TIMEOUT_MS > ROUTE_WALL_PREDICT_MS &&
           ROUTE_WALL_NO_PROGRESS_MS > ROUTE_LIDAR_TTL_MS &&
           ROUTE_EDGE_ODOM_ERROR_MM >= 0.0f &&
           ROUTE_WALL_MAX_EXTRA_MM >= 0.0f &&
           ROUTE_WALL_PREDICT_MS >= ROUTE_PERIOD_MS && ROUTE_WALL_PREDICT_MS < ROUTE_LIDAR_TTL_MS;
}

void RoutePlan_Target(uint8_t point, float *x_mm, float *y_mm)
{
    uint8_t i;
    *x_mm = 0.0f;
    *y_mm = 0.0f;
    for (i = 1U; i <= point && i <= ROUTE_POINT_COUNT; ++i) {
        const RouteSegment *segment = &segments[i - 1U];
        if (segment->motion == ROUTE_BACK) *x_mm -= segment->length_mm;
        else if (segment->motion == ROUTE_FORWARD) *x_mm += segment->length_mm;
        else if (segment->motion == ROUTE_LEFT) *y_mm += segment->length_mm;
        else *y_mm -= segment->length_mm;
    }
}

const char *RoutePlan_MotionName(RouteMotion motion)
{
    static const char *const names[] = {"BACK", "LEFT", "RIGHT", "FWD"};
    return motion <= ROUTE_FORWARD ? names[motion] : "STOP";
}
