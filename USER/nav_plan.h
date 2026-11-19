#ifndef MANGO_NAV_PLAN_H
#define MANGO_NAV_PLAN_H
#include <stdint.h>

enum { NAV_FRONT, NAV_BACK, NAV_LEFT, NAV_RIGHT, NAV_DIRECTIONS };
enum { NAV_SIDE_LEFT, NAV_SIDE_RIGHT, NAV_SIDE_EXIT };
enum { NAV_MOVE_ENTRY, NAV_MOVE_CROSS, NAV_MOVE_SIDE, NAV_MOVE_RETURN, NAV_MOVE_EXIT };
typedef struct {
    float side_front_mm, side_back_mm;
    float center_front_mm, center_back_mm;
    /* 近垄壁确实存在的横移区间，坐标为左激光到左围栏的距离。 */
    float front_begin_left_mm, front_end_left_mm;
    float back_begin_left_mm, back_end_left_mm;
} NavLaneCalibration;
typedef struct {
    float lr_sum_mm; /* 同时照到左右围栏时 L+R，不是场地总宽 */
    float left_stop_mm, right_stop_mm;
    float min_distance_mm[NAV_DIRECTIONS]; /* 对应整车含机构的最近允许探头读数 */
    NavLaneCalibration lane[7];
} NavCalibration;
typedef struct { uint8_t lane, side, motion; } NavPoint;
extern const NavCalibration nav_calibration;
extern const NavPoint nav_plan[17];
int NavCalibration_Valid(const NavCalibration *cal);
#endif
