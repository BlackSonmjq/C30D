#ifndef ZP_SERVO_H
#define ZP_SERVO_H

#include "stm32f4xx.h"

/* ?????????????? ID????????????????????
 * ?????ID 1/2 ??? 2?ID 3/4 ??? 1???????? 500 ms?
 * ?????? MCU ?????????????
 * ????????? M1/M2/M3/M4 ????????? 2/2/1/1???????
 */
/* ZP10S: UART4, PC10 (single-wire half duplex), 115200 8N1.
 * IDs 1..4 match the former pushrod IDs. No broadcast motion API.
 * Mode 1: 270 degrees, P0500 = 0 degrees, P1833 ~= 180 degrees.
 */
#define SERVO_COUNT             4U
#define SERVO_HOME_PULSE        500U
#define SERVO_HALF_TURN_PULSE   1833U
#define SERVO_MOVE_MS           500U
#define SERVO_POSITION_TOLERANCE 100U /* All IDs and targets: +/-100 feedback units. */
#define SERVO_FRAME_CAPACITY    64U

/* RX interrupts are armed after a successful send or an explicit RX probe.
 * Set to 0 to disable reply reception for hardware diagnosis.
 */
#ifndef SERVO_RX_IRQ_ENABLED
#define SERVO_RX_IRQ_ENABLED    1
#endif

typedef enum {
    SERVO_OK = 0,
    SERVO_INVALID_ARGUMENT,
    SERVO_NOT_INITIALIZED,
    SERVO_BUSY,
    SERVO_TX_TIMEOUT,
    SERVO_NO_FRAME,
    SERVO_BUFFER_TOO_SMALL,
    SERVO_RX_DISABLED
} Servo_Result;

typedef struct {
    uint32_t irq_count;
    uint32_t byte_count;
    uint32_t error_count;
    uint32_t error_flags;
    uint32_t last_status;
    uint8_t receiving;
    uint8_t pc10_high;
} Servo_Diagnostics;

/* Receive-only diagnosis: transmits no bytes and resets diagnostic counters.
 * Returns BUSY during a send, receive or while a reply is unread.
 * Caller must end the probe with ClearRx, including after RX stops on error.
 * All sends return BUSY until then. GetDiagnostics takes an atomic snapshot.
 */
Servo_Result Servo_StartRxProbe(void);
void Servo_GetDiagnostics(Servo_Diagnostics *data);

/* Configure transport only: does not change IDs/modes or move any servo.
 * Call once at startup. Send APIs are task-context only and non-reentrant:
 * concurrent send returns BUSY. Application owns request/reply sequencing.
 * Whole-frame TX suspends task scheduling when the scheduler is started;
 * IRQs remain enabled except during short RX/TX direction transitions.
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
/* Fault-stop latches out motion until MCU restart; it never writes ID/mode.
 * A sent stop is not a guarantee of physical stopping on a faulty link. */
Servo_Result Servo_FaultStop(uint8_t id);
uint8_t Servo_IsFaulted(void);
Servo_Result Servo_ReadPosition(uint8_t id);

/* Single-frame mailbox, including terminating NUL. Generic commands/probes
 * accept #OK! and #id...!. PRAD/PMOD queries wait for a valid same-ID position
 * or mode reply, ignoring ACKs, echoes and unrelated/malformed frames.
 * On BUFFER_TOO_SMALL the frame is retained and *length gives required bytes
 * excluding NUL. RX stops on a matching frame, UART error or byte budget limit.
 * Consume/clear the previous reply before sending the next command.
 * On application reply timeout call ClearRx to cancel reception.
 * No motion-complete inference is made from a reply.
 */
Servo_Result Servo_GetFrame(char *buffer, uint16_t capacity, uint16_t *length);
void Servo_ClearRx(void);
void Servo_RxByteHandler(uint8_t byte);

#endif
