#include "nav_plan.h"
#include "nav_config.h"

/* 全部是名义尺寸示例，绝不是实测值；NAV_CALIBRATED 默认 0。
 * 先把整车摆正，逐沟测量侧道 F/B、沟内居中 F/B。
 * 左侧 B 穿出入口，忽略它；side_back 由同一沟右侧实测。
 * 短垄在图示第4/5沟之间：第4沟前壁、第5沟后壁区间较短。
 * 若实物不同，只修改本表，不修改导航算法。
 */
const NavCalibration nav_calibration = {
    2800.0f, 100.0f, 100.0f,
    {40.0f, 40.0f, 40.0f, 40.0f},
    {
        /* 侧道F, 侧道B, 沟内F, 沟内B, 前壁开始L/结束L, 后壁开始L/结束L */
        {4320,  100, 60, 60, 450,2420, 450,2420}, /* 沟1 */
        {3590,  770, 60, 60, 450,2420, 450,2420}, /* 沟2 */
        {2880, 1450, 60, 60, 450,2420, 450,2420}, /* 沟3 */
        {2200, 2200, 60, 60, 630,2210, 630,2210}, /* 沟4：前侧短垄 */
        {1500, 2850, 60, 60, 630,2210, 630,2210}, /* 沟5：后侧短垄 */
        { 800, 3600, 60, 60, 450,2420, 450,2420}, /* 沟6 */
        { 100, 4300, 60, 60, 450,2420, 450,2420}  /* 沟7 */
    }
};

const NavPoint nav_plan[17] = {
    {0,NAV_SIDE_LEFT, NAV_MOVE_ENTRY},  /* P01 */
    {0,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P02 */
    {1,NAV_SIDE_RIGHT,NAV_MOVE_SIDE},   /* P03 */
    {1,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P04 */
    {2,NAV_SIDE_LEFT, NAV_MOVE_SIDE},   /* P05 */
    {2,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P06 */
    {3,NAV_SIDE_RIGHT,NAV_MOVE_SIDE},   /* P07 */
    {3,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P08 */
    {4,NAV_SIDE_LEFT, NAV_MOVE_SIDE},   /* P09 */
    {4,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P10 */
    {5,NAV_SIDE_RIGHT,NAV_MOVE_SIDE},   /* P11 */
    {5,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P12 */
    {6,NAV_SIDE_LEFT, NAV_MOVE_SIDE},   /* P13 */
    {6,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P14 */
    {0,NAV_SIDE_RIGHT,NAV_MOVE_RETURN},/* P15：同 P02 位置，返程阶段 */
    {0,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P16：同 P01 位置，返程阶段 */
    {0,NAV_SIDE_EXIT, NAV_MOVE_EXIT}    /* P17：起跑前距 */
};

static int range_ok(float v) { return v == v && v >= 1 && v <= NAV_LIDAR_MAX_MM; }
int NavCalibration_Valid(const NavCalibration *c)
{
    unsigned i;
    if (!c || !range_ok(c->lr_sum_mm) || !range_ok(c->left_stop_mm) ||
        !range_ok(c->right_stop_mm) || c->left_stop_mm+c->right_stop_mm >= c->lr_sum_mm)
        return 0;
    for (i=0;i<4;i++) if (!range_ok(c->min_distance_mm[i])) return 0;
    if (c->left_stop_mm <= c->min_distance_mm[NAV_LEFT]+NAV_POINT_TOL_MM ||
        c->right_stop_mm <= c->min_distance_mm[NAV_RIGHT]+NAV_POINT_TOL_MM) return 0;
    for (i=0;i<7;i++) {
        const NavLaneCalibration *l=&c->lane[i];
        if (!range_ok(l->side_front_mm) || !range_ok(l->side_back_mm) ||
            !range_ok(l->center_front_mm) || !range_ok(l->center_back_mm)) return 0;
        if (l->center_front_mm <= c->min_distance_mm[NAV_FRONT]+NAV_CENTER_DEADBAND_MM ||
            l->center_back_mm <= c->min_distance_mm[NAV_BACK]+NAV_CENTER_DEADBAND_MM) return 0;
        if (l->side_front_mm <= c->min_distance_mm[NAV_FRONT]+NAV_POINT_TOL_MM ||
            l->side_back_mm <= c->min_distance_mm[NAV_BACK]+NAV_POINT_TOL_MM) return 0;
        if (!(l->front_begin_left_mm > c->left_stop_mm+NAV_SIDE_FIELD_BAND_MM &&
              l->front_end_left_mm < c->lr_sum_mm-c->right_stop_mm-NAV_SIDE_FIELD_BAND_MM &&
              l->front_begin_left_mm < l->front_end_left_mm &&
              l->back_begin_left_mm > c->left_stop_mm+NAV_SIDE_FIELD_BAND_MM &&
              l->back_end_left_mm < c->lr_sum_mm-c->right_stop_mm-NAV_SIDE_FIELD_BAND_MM &&
              l->back_begin_left_mm < l->back_end_left_mm)) return 0;
        if (i && !(c->lane[i-1].side_front_mm > l->side_front_mm &&
                   c->lane[i-1].side_back_mm < l->side_back_mm)) return 0;
    }
    return 1;
}
