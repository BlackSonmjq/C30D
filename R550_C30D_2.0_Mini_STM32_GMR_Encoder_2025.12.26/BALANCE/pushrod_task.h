#ifndef __PUSHROD_TASK_H
#define __PUSHROD_TASK_H
#include "system.h"

#define PUSHROD_TASK_PRIO 1
#define PUSHROD_STK_SIZE 128

void pushrod_task(void *pvParameters);

#endif
