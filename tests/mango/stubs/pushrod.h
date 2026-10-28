#ifndef MOCK_PUSHROD_H
#define MOCK_PUSHROD_H
#include <stdint.h>
#define PUSHROD_COUNT 4U
typedef enum { PUSHROD_OK, PUSHROD_INVALID_ARGUMENT, PUSHROD_NOT_INITIALIZED } Pushrod_Result;
Pushrod_Result Pushrod_Start(uint8_t id);
Pushrod_Result Pushrod_Stop(uint8_t id);
#endif
