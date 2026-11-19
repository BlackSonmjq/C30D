#ifndef MY_CHASSIS_TASK_H
#define MY_CHASSIS_TASK_H
#include "route_nav.h"

/* 0 等起跑，1～17 是当前目标，18 为完成；故障看 Route_ReadStatus。 */
extern volatile uint8_t Route_Stage;
extern volatile uint8_t Route_TaskCreated;
void my_chassis_task(void *parameters);
void Route_ReadStatus(RouteStatus *status);
typedef struct {
    uint8_t state, reason, target, confirmed;
    float main_error_mm, cross_error_mm, heading_error_rad, start_front_mm;
} RouteNavigationStatus;
void Route_ReadNavigationStatus(RouteNavigationStatus *status);
#endif
