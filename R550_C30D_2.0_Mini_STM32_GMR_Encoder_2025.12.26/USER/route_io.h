#ifndef ROUTE_IO_H
#define ROUTE_IO_H
#include <stdint.h>
#include "nav_can.h"

typedef enum {
    ROUTE_LIDAR_FRONT, ROUTE_LIDAR_BEHIND, ROUTE_LIDAR_LEFT,
    ROUTE_LIDAR_RIGHT, ROUTE_LIDAR_COUNT
} RouteLidarDirection;
typedef struct {
    uint16_t mm[ROUTE_LIDAR_COUNT];
    uint32_t sequence; /* 每收到一份新 CAN 数据递增，0 表示尚未收到。 */
    uint8_t fresh;     /* 整帧通信新鲜度；单路为 0 时该路测距仍无效。 */
} RouteLidarFrame;
typedef struct {
    float vx_mps, vy_mps;
    uint16_t rear_mm;
    uint8_t can_fresh, command_fresh;
} RouteIODiagnostics;

void RouteIO_PublishLidar(uint16_t front, uint16_t behind, uint16_t left, uint16_t right);
void RouteIO_Tick1ms(void);
uint32_t RouteIO_Millis(void);
void RouteIO_ReceiveCan(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t len, const uint8_t *data);
void RouteIO_ReadLidar(RouteLidarFrame *frame);
void RouteIO_ReadNavigationLidar(NavLidar *frame);
uint8_t RouteIO_BaseReady(void);
uint8_t RouteIO_Ready(void);
uint8_t RouteIO_ControlOwned(void);
void RouteIO_RequestStart(void);
uint8_t RouteIO_TakeStartRequest(void);
void RouteIO_RequestManual(void);
uint8_t RouteIO_TakeStopRequest(void);
void RouteIO_SetRunning(uint8_t running);
void RouteIO_SetHeading(float yaw_rad);
uint8_t RouteIO_StartRequested(void);
uint8_t RouteIO_ReverseTestRunning(void);
void RouteIO_SetVelocity(float vx_mps, float vy_mps, uint8_t quick_stop, float probe_assist_pwm);
void RouteIO_ReadDiagnostics(RouteIODiagnostics *diagnostics);
float RouteIO_ApplyDrive(void);
uint8_t RouteIO_ChassisStopped(void);
/* 中断只锁存一个疏果请求；任务层停车后再调用机构，完成/失败均有反馈。 */
uint8_t RouteIO_QueueWorkFromISR(uint8_t mango_id);
uint8_t RouteIO_ReserveServoDemo(void);
uint8_t RouteIO_WorkReserved(void);
uint8_t RouteIO_TakeWorkRequest(void);
void RouteIO_AllowWork(uint8_t allowed);
void RouteIO_CompleteWork(void);
#endif
