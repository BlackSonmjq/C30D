#ifndef MOCK_SERVO_H
#define MOCK_SERVO_H
#include <stdint.h>
#define SERVO_COUNT 4U
#define SERVO_HOME_PULSE 500U
#define SERVO_HALF_TURN_PULSE 1833U
#define SERVO_2_DELAY_MS 1500U /* Timed outward and return waits; no position checks. */
#define SERVO_2_POSITION_TOLERANCE 90U
#define SERVO_MOVE_MS 500U
#define SERVO_FRAME_CAPACITY 64U
typedef enum { SERVO_OK, SERVO_NOT_INITIALIZED, SERVO_TX_TIMEOUT, SERVO_NO_FRAME } Servo_Result;
Servo_Result Servo_SendCommand(const char *command);
Servo_Result Servo_MovePulse(uint8_t id, uint16_t pulse, uint16_t duration_ms);
Servo_Result Servo_ReadPosition(uint8_t id);
Servo_Result Servo_GetFrame(char *buffer, uint16_t capacity, uint16_t *length);
Servo_Result Servo_Stop(uint8_t id);
void Servo_ClearRx(void);
#endif

#ifndef SERVO_POSITION_TOLERANCE
#define SERVO_POSITION_TOLERANCE 100U
#endif
