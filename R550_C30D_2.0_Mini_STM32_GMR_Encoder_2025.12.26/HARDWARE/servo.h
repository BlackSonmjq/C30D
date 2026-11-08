#ifndef ZP_SERVO_H
#define ZP_SERVO_H

#include "stm32f4xx.h"

/* ZP10S: UART4, PC10 (single-wire half duplex), 115200 8N1.
 * IDs 1..4 match the former pushrod IDs. No broadcast motion API.
 * Mode 1: 270 degrees, P0500 = 0 degrees, P1833 ~= 180 degrees.
 */
#define SERVO_COUNT             4U
#define SERVO_HOME_PULSE        500U
#define SERVO_HALF_TURN_PULSE   1833U
#define SERVO_MOVE_MS           1500U
#define SERVO_FRAME_CAPACITY    64U

typedef enum {
    SERVO_OK = 0,
    SERVO_INVALID_ARGUMENT,
    SERVO_NOT_INITIALIZED,
    SERVO_BUSY,
    SERVO_TX_TIMEOUT,
    SERVO_NO_FRAME,
    SERVO_BUFFER_TOO_SMALL
} Servo_Result;

/* Configure transport only: does not change IDs/modes or move any servo.
 * Call once at startup. Send APIs are task-context only and non-reentrant:
 * concurrent send returns BUSY. Application owns request/reply sequencing.
 * OK means bytes sent, NOT acknowledged or physical movement completed.
 */
void Servo_Init(void);
Servo_Result Servo_SendCommand(const char *command);
/* Explicit persistent setup; allow servo processing/ACK before next command.
 * SetMode accepts position modes 1..4 only. Home/Twist assume mode 1.
 * SetID accepts old ID 0..254, new ID 1..4: connect ONE servo when assigning.
 */
Servo_Result Servo_SetID(uint8_t old_id, uint8_t new_id);
Servo_Result Servo_SetMode(uint8_t id, uint8_t mode);
Servo_Result Servo_MovePulse(uint8_t id, uint16_t pulse, uint16_t time_ms);
Servo_Result Servo_MoveAngle(uint8_t id, uint16_t degrees, uint16_t time_ms);
Servo_Result Servo_Home(uint8_t id, uint16_t time_ms);
Servo_Result Servo_Twist(uint8_t id, uint16_t time_ms);
Servo_Result Servo_Stop(uint8_t id);
Servo_Result Servo_ReadPosition(uint8_t id);

/* Single-frame mailbox, including terminating NUL. Supports #OK! and #id...!.
 * On BUFFER_TOO_SMALL the frame is retained and *length gives required bytes
 * excluding NUL. Later complete frames are dropped until this one is consumed.
 * No motion-complete inference is made from a reply.
 */
Servo_Result Servo_GetFrame(char *buffer, uint16_t capacity, uint16_t *length);
void Servo_ClearRx(void);
void Servo_RxByteHandler(uint8_t byte);

#endif
