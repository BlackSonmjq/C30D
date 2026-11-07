#ifndef PUSHROD_H
#define PUSHROD_H

#include "stm32f4xx.h"

#define PUSHROD_COUNT 4U

typedef enum
{
    PUSHROD_OK = 0,
    PUSHROD_INVALID_ARGUMENT,
    PUSHROD_NOT_INITIALIZED,
    PUSHROD_BUSY
} Pushrod_Result;

/* Configure PC0-PC3 as low outputs; connect each actuator's other input to GND. */
void Pushrod_Init(void);

/* id is 1-4. Start drives the corresponding pin high; Stop drives it low. */
Pushrod_Result Pushrod_Start(uint8_t id);
Pushrod_Result Pushrod_Stop(uint8_t id);

/* One timed output at a time; 1..6553 ms, stopped by TIM6 IRQ independently
 * of task scheduling. Start resets completion; Stop cancels it. */
Pushrod_Result Pushrod_StartTimed(uint8_t id, uint16_t duration_ms);
uint8_t Pushrod_TimedDone(uint8_t id);

#endif
