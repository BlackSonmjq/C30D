#ifndef MOCK_PUSHROD_H
#define MOCK_PUSHROD_H
#include <stdint.h>
#define PUSHROD_COUNT 4U
typedef enum { PUSHROD_OK, PUSHROD_INVALID_ARGUMENT, PUSHROD_NOT_INITIALIZED } Pushrod_Result;
Pushrod_Result Pushrod_Start(uint8_t id);
Pushrod_Result Pushrod_Stop(uint8_t id);
Pushrod_Result Pushrod_StartTimed(uint8_t id, uint16_t duration_ms);
uint8_t Pushrod_TimedDone(uint8_t id);
#endif
