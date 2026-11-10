#ifndef __CAN_TEST_H
#define __CAN_TEST_H

#include "sys.h"

#define CAN_TEST_TIMEOUT_MS 500U

extern volatile uint32_t g_can_test_rx_count;
extern volatile uint32_t g_can_test_last_counter;
extern volatile uint32_t g_can_test_lost_count;
extern volatile uint32_t g_can_test_error_count;
extern volatile uint8_t g_can_test_online;

/* Wire order: D1=Left, D2=Front, D3=Right, D4=Behind. */
typedef enum {
    CAN_DISTANCE_LEFT = 0,
    CAN_DISTANCE_FRONT = 1,
    CAN_DISTANCE_RIGHT = 2,
    CAN_DISTANCE_BEHIND = 3,
    CAN_DISTANCE_COUNT = 4
} CAN_DistanceDirection;
extern volatile uint16_t g_can_distance_mm[CAN_DISTANCE_COUNT];

void CAN_Test_Init(void);
void CAN_Test_Parse(u8 ide, u8 rtr, u8 len, const u8 *data);
void CAN_Test_Process(void);
void CAN_TestTask(void *pvParameters);

#endif
