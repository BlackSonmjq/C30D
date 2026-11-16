#include "route_plan.h"
#include "route_config.h"
#include <stddef.h>

/* 每行描述“驶向 Pxx”的动作。P01～P14 遍历七沟，P15～P17 出场停车。
 * WALL 由粗里程或连续近距接近打开窗口。EDGE 由完整垄端事件和独立围栏参考共同到点。
 * 左/右均指车身；车左横移是地图向右。侧墙参数只修正横向位置。 */
static const RouteSegment segments[ROUTE_POINT_COUNT] = {
    /* P01 固定摆位倒车到第一沟左入口，前方入口缺口不作为测距终点。 */
    {ROUTE_BACK, ROUTE_REF_ODOM, ROUTE_LIDAR_COUNT, 0, 1, ROUTE_ENTRY_LENGTH_MM,
     {0, 150, 15}, ROUTE_LIDAR_RIGHT, 120, 0},
    /* P02 第一沟左横移，左探头看右围栏。 */
    {ROUTE_LEFT, ROUTE_REF_WALL, ROUTE_LIDAR_LEFT, 0, 1, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_2, ROUTE_LIDAR_COUNT, 0, 0},
    /* P03 右端倒车到第二沟；此处前探头能稳定看下围栏。 */
    {ROUTE_BACK, ROUTE_REF_WALL, ROUTE_LIDAR_FRONT, 1, 2, ROUTE_LANE_PITCH_MM,
     Front_Lidar_3, ROUTE_LIDAR_LEFT, 120, 0},
    /* P04 第二沟右横移到左端。 */
    {ROUTE_RIGHT, ROUTE_REF_WALL, ROUTE_LIDAR_RIGHT, 0, 2, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_4, ROUTE_LIDAR_COUNT, 0, 0},
    /* P05 左端换第三沟，左探头朝场内观察第二条普通垄的端面。 */
    {ROUTE_BACK, ROUTE_REF_EDGE, ROUTE_LIDAR_LEFT, 0, 3, ROUTE_LANE_PITCH_MM,
     {ROUTE_LEFT_RIDGE_MM, 150, 15}, ROUTE_LIDAR_RIGHT, 120, ROUTE_LEFT_EDGE_TO_CENTER_MM},
    /* P06 第三沟左横移到右端。 */
    {ROUTE_LEFT, ROUTE_REF_WALL, ROUTE_LIDAR_LEFT, 0, 3, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_6, ROUTE_LIDAR_COUNT, 0, 0},
    /* P07 右端换第四沟，右探头观察第三条普通垄，另由前探头独立复核围栏距离。 */
    {ROUTE_BACK, ROUTE_REF_EDGE, ROUTE_LIDAR_RIGHT, 0, 4, ROUTE_LANE_PITCH_MM,
     {ROUTE_RIGHT_RIDGE_MM, 150, 15}, ROUTE_LIDAR_LEFT, 120, ROUTE_RIGHT_EDGE_TO_CENTER_MM},
    /* P08 第四沟右横移到左端。 */
    {ROUTE_RIGHT, ROUTE_REF_WALL, ROUTE_LIDAR_RIGHT, 0, 4, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_8, ROUTE_LIDAR_COUNT, 0, 0},
    /* P09 左端换第五沟，第四条短垄端面缩进 200 mm，单独标定近回波。 */
    {ROUTE_BACK, ROUTE_REF_EDGE, ROUTE_LIDAR_LEFT, 0, 5, ROUTE_LANE_PITCH_MM,
     {ROUTE_LEFT_SHORT_RIDGE_MM, 150, 15}, ROUTE_LIDAR_RIGHT, 120, ROUTE_LEFT_EDGE_TO_CENTER_MM},
    /* P10 第五沟左横移到右端。 */
    {ROUTE_LEFT, ROUTE_REF_WALL, ROUTE_LIDAR_LEFT, 0, 5, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_10, ROUTE_LIDAR_COUNT, 0, 0},
    /* P11 换第六沟，后探头看上围栏，避免用下方远墙判点。 */
    {ROUTE_BACK, ROUTE_REF_WALL, ROUTE_LIDAR_BEHIND, 0, 6, ROUTE_LANE_PITCH_MM,
     Behind_Lidar_11, ROUTE_LIDAR_LEFT, 120, 0},
    /* P12 第六沟右横移到左端。 */
    {ROUTE_RIGHT, ROUTE_REF_WALL, ROUTE_LIDAR_RIGHT, 0, 6, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_12, ROUTE_LIDAR_COUNT, 0, 0},
    /* P13 换第七沟，继续使用后探头看上围栏。 */
    {ROUTE_BACK, ROUTE_REF_WALL, ROUTE_LIDAR_BEHIND, 0, 7, ROUTE_LANE_PITCH_MM,
     Behind_Lidar_13, ROUTE_LIDAR_RIGHT, 120, 0},
    /* P14 第七沟左横移到右端，随后继续执行返程。 */
    {ROUTE_LEFT, ROUTE_REF_WALL, ROUTE_LIDAR_LEFT, 0, 7, ROUTE_LANE_LENGTH_MM,
     Left_Lidar_14, ROUTE_LIDAR_COUNT, 0, 0},
    /* P15 车头仍朝下，沿右端前进返回第一沟右端。 */
    {ROUTE_FORWARD, ROUTE_REF_WALL, ROUTE_LIDAR_FRONT, 0, 1, ROUTE_RETURN_LENGTH_MM,
     Front_Lidar_15, ROUTE_LIDAR_LEFT, 120, 0},
    /* P16 第一沟右横移回入口。P17 从这个已确认点开始计短程出场里程。 */
    {ROUTE_RIGHT, ROUTE_REF_WALL, ROUTE_LIDAR_RIGHT, 0, 1, ROUTE_LANE_LENGTH_MM,
     Right_Lidar_16, ROUTE_LIDAR_COUNT, 0, 0},
    {ROUTE_FORWARD, ROUTE_REF_ODOM, ROUTE_LIDAR_COUNT, 0, 0, ROUTE_EXIT_LENGTH_MM,
     {0, 150, 15}, ROUTE_LIDAR_RIGHT, 120, 0}
};

/* 三组读数由用户确认在正确沟中心、对准正确围栏实测。 */
static const RouteAuxReference auxiliary[] = {
    {5U, ROUTE_LIDAR_BEHIND, 0U, Behind_Lidar_5},
    {7U, ROUTE_LIDAR_FRONT, 1U, Front_Lidar_7},
    {9U, ROUTE_LIDAR_BEHIND, 0U, Behind_Lidar_9}
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
        if (s->length_mm <= 0.0f || s->mark.turn_band_mm >= s->mark.slow_band_mm) valid = 0U;
        if (s->reference != ROUTE_REF_ODOM &&
            (s->mark.target_mm < ROUTE_LIDAR_MIN_MM || s->mark.target_mm > ROUTE_REFERENCE_MAX_MM))
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
