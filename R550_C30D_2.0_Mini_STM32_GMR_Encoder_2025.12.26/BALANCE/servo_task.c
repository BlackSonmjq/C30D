#include "servo_task.h"
#include "usartx.h"
#include "usart.h"

/* All IDs: feedback-confirmed home, half turn, home.
 * Existing 270-degree position modes are configured externally. */
static volatile ServoTask_Data actuator = {0, SERVO_TASK_IDLE};
static uint8_t start_pending, phase, stage, stable, failed, moved;
static TickType_t move_started, query_started;

uint8_t ServoTask_IsBusy(void)
{
    return actuator.state == SERVO_TASK_RUNNING;
}

uint8_t ServoTask_Start(uint8_t id)
{
    uint8_t accepted = 0;
    if (id < 1U || id > SERVO_COUNT || Servo_IsFaulted()) return 0;
    taskENTER_CRITICAL();
    if (actuator.state != SERVO_TASK_RUNNING && !USART_ServoControlBusy()) {
        actuator.id = id;
        actuator.state = SERVO_TASK_RUNNING;
        moved = 0;
        start_pending = 1;
        accepted = 1;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void ServoTask_Cancel(void)
{
    uint8_t id, stop;
    taskENTER_CRITICAL();
    id = actuator.id;
    stop = actuator.state == SERVO_TASK_RUNNING && moved;
    start_pending = phase = stage = stable = failed = moved = 0;
    actuator.id = 0;
    actuator.state = SERVO_TASK_IDLE;
    taskEXIT_CRITICAL();
    if (stop) {
        Servo_ClearRx();
        Servo_Stop(id);
        Servo_ClearRx();
    }
}

void ServoTask_GetData(ServoTask_Data *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = actuator;
    taskEXIT_CRITICAL();
}

static uint8_t position_matches(const char *frame, uint16_t length, uint16_t target)
{
    unsigned i, id = 0, position = 0;
    int error, tolerance = SERVO_POSITION_TOLERANCE;
    if (length != 10U || frame[0] != '#' || frame[4] != 'P' || frame[9] != '!') return 0;
    for (i = 1; i <= 8; ++i) {
        if (i == 4U) continue;
        if (frame[i] < '0' || frame[i] > '9') return 0;
        if (i < 4U) id = id * 10U + (unsigned)(frame[i] - '0');
        else position = position * 10U + (unsigned)(frame[i] - '0');
    }
    if (id != actuator.id || position < 500U || position > 2500U) return 0;
    if (actuator.state==SERVO_TASK_RUNNING && position>SERVO_HALF_TURN_PULSE+SERVO_POSITION_TOLERANCE) {
        Servo_FaultStop(actuator.id);
        failed=1;
        actuator.state=SERVO_TASK_ERROR;
        any_printf(USART1,"SERVO %u unexpected position=%u; STOP AND LOCK\r\n",(unsigned)id,position);
        return 0;
    }
    any_printf(USART1, "SERVO %u phase=%u pos=%u target=%u\r\n",
               (unsigned)actuator.id, (unsigned)phase, (unsigned)position, (unsigned)target);
    error = (int)position - (int)target;
    return error >= -tolerance && error <= tolerance;
}

static void phase_failed(void)
{
    failed = 1;
    any_printf(USART1, "SERVO %u phase=%u ERROR\r\n", (unsigned)actuator.id, (unsigned)phase);
    Servo_ClearRx();
    Servo_FaultStop(actuator.id);
    Servo_ClearRx();
    actuator.state = SERVO_TASK_ERROR;
}

void servo_task(void *pvParameters)
{
    TickType_t now;
    uint16_t target, length;
    char frame[SERVO_FRAME_CAPACITY];
    (void)pvParameters;
    for (;;) {
        if (start_pending) {
            start_pending = 0;
            phase = stage = stable = failed = moved = 0;

        }
        if (actuator.state == SERVO_TASK_RUNNING) {
            target = phase == 1U ? SERVO_HALF_TURN_PULSE : SERVO_HOME_PULSE;
            now = xTaskGetTickCount();
            if (stage == 0U) {
                Servo_ClearRx();
                moved = 1;
                any_printf(USART1, "SERVO %u phase=%u target=%u time=%lu\r\n",
                           (unsigned)actuator.id, (unsigned)phase, (unsigned)target, (unsigned long)SERVO_MOVE_MS);
                if (Servo_MovePulse(actuator.id, target, SERVO_MOVE_MS) != SERVO_OK) {
                    phase_failed();
                } else {
                    move_started = xTaskGetTickCount();
                    stable = 0;
                    stage = 1;
                }
            } else if ((TickType_t)(now - move_started) >= pdMS_TO_TICKS(MANGO_SERVO_PHASE_TIMEOUT_MS)) {
                phase_failed();
            } else if (stage == 1U &&
                       (TickType_t)(now - move_started) >= pdMS_TO_TICKS(SERVO_MOVE_MS)) {
                Servo_ClearRx();
                if (Servo_ReadPosition(actuator.id) != SERVO_OK) {
                    phase_failed();
                } else {
                    query_started = xTaskGetTickCount();
                    stage = 2;
                }
            } else if (stage == 2U) {
                if (Servo_GetFrame(frame, sizeof(frame), &length) == SERVO_OK) {
                    stable = position_matches(frame, length, target) ? stable + 1U : 0U;
                    if (stable >= 2U) {
                        Servo_ClearRx();
                        if (phase < 2U) { ++phase; stage = 0; }
                        else {
                            actuator.state = failed ? SERVO_TASK_ERROR : SERVO_TASK_DONE;
                            any_printf(USART1, "SERVO %u finished error=%u\r\n",
                                       (unsigned)actuator.id, (unsigned)failed);
                        }
                    }
                }
                if (stage == 2U && actuator.state == SERVO_TASK_RUNNING &&
                    (TickType_t)(now - query_started) >= pdMS_TO_TICKS(MANGO_SERVO_QUERY_MS)) {
                    /* A missing query does not invalidate the last valid sample. */
                    stage = 1;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
