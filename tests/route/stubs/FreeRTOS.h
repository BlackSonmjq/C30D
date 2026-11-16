#ifndef ROUTE_TEST_FREERTOS_H
#define ROUTE_TEST_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(mask) ((void)(mask))
#endif
