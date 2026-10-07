#ifndef PUSHROD_H
#define PUSHROD_H

#include "stm32f4xx.h"

#define PUSHROD_COUNT 4U

typedef enum
{
    PUSHROD_OK = 0,
    PUSHROD_INVALID_ARGUMENT,
    PUSHROD_NOT_INITIALIZED
} Pushrod_Result;

/* Configure PC0-PC3 as low outputs; connect each actuator's other input to GND. */
void Pushrod_Init(void);

/* id is 1-4. Start drives the corresponding pin high; Stop drives it low. */
Pushrod_Result Pushrod_Start(uint8_t id);
Pushrod_Result Pushrod_Stop(uint8_t id);

#endif
