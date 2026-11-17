#ifndef WORKFLOW_TASK_PRIORITY_H
#define WORKFLOW_TASK_PRIORITY_H

/* Configurable native FreeRTOS base priorities: idle is 0.
 * Input, IMU and chassis share one default group and rotate through ready
 * tasks with configUSE_TIME_SLICING; periodic work delays/blocks each pass.
 * These are allocation defaults, not a mandatory ordering or scheduler lock.
 * USB uses native xTaskCreate, not CMSIS osPriorityNormal (24).
 */
#ifndef TASK_PRIORITY_CONTROL_GROUP
#define TASK_PRIORITY_CONTROL_GROUP 12U
#endif
#ifndef TASK_PRIORITY_CHASSIS
#define TASK_PRIORITY_CHASSIS      TASK_PRIORITY_CONTROL_GROUP
#endif
#ifndef TASK_PRIORITY_INPUT
#define TASK_PRIORITY_INPUT        TASK_PRIORITY_CONTROL_GROUP
#endif
#ifndef TASK_PRIORITY_IMU
#define TASK_PRIORITY_IMU          TASK_PRIORITY_CONTROL_GROUP
#endif
#ifndef TASK_PRIORITY_VISION
#define TASK_PRIORITY_VISION       11U
#endif
#ifndef TASK_PRIORITY_ROUTE
#define TASK_PRIORITY_ROUTE         8U
#endif
#ifndef TASK_PRIORITY_DISTANCE
#define TASK_PRIORITY_DISTANCE      7U
#endif
#ifndef TASK_PRIORITY_STEPPER
#define TASK_PRIORITY_STEPPER       6U
#endif
#ifndef TASK_PRIORITY_SERVO
#define TASK_PRIORITY_SERVO         5U
#endif
#ifndef TASK_PRIORITY_COMMS
#define TASK_PRIORITY_COMMS         3U
#endif
#ifndef TASK_PRIORITY_DISPLAY
#define TASK_PRIORITY_DISPLAY       2U
#endif
#ifndef TASK_PRIORITY_STATUS
#define TASK_PRIORITY_STATUS        1U
#endif

/* NVIC uses the opposite order: smaller values run first. USART3 must
 * never call RTOS APIs. Telemetry TX uses only registers and a byte buffer.
 */
#ifndef IRQ_PRIORITY_VISION
#define IRQ_PRIORITY_VISION         0U
#endif
#ifndef IRQ_PRIORITY_TELEMETRY
#define IRQ_PRIORITY_TELEMETRY      6U
#endif

#endif
