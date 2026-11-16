#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H
#include "route_io.h"

typedef struct {
    uint16_t target_mm, slow_band_mm, turn_band_mm;
} RouteMark;
typedef enum { ROUTE_BACK, ROUTE_LEFT, ROUTE_RIGHT, ROUTE_FORWARD, ROUTE_STOP } RouteMotion;
typedef enum { ROUTE_REF_ODOM, ROUTE_REF_WALL, ROUTE_REF_EDGE } RouteReference;

typedef struct {
    RouteMotion motion;
    RouteReference reference;
    RouteLidarDirection trigger;
    uint8_t distance_increases;
    uint8_t lane;
    float length_mm;
    RouteMark mark;
    RouteLidarDirection side_wall;
    uint16_t side_target_mm;
    uint16_t edge_to_center_mm;
} RouteSegment;

/* 垄端段的独立围栏参考；与扫垄端的左右探头分别验证。 */
typedef struct {
    uint8_t point;
    RouteLidarDirection sensor;
    uint8_t distance_increases;
    RouteMark mark;
} RouteAuxReference;

#define ROUTE_POINT_COUNT 17U
const RouteSegment *RoutePlan_Get(uint8_t point);
const RouteAuxReference *RoutePlan_AuxReference(uint8_t point);
uint8_t RoutePlan_Calibrated(void);
void RoutePlan_Target(uint8_t point, float *x_mm, float *y_mm);
const char *RoutePlan_MotionName(RouteMotion motion);
#endif
