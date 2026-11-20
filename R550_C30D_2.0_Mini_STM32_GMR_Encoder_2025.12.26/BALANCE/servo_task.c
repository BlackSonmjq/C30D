#include "servo_task.h"
#include "usartx.h"
#include "usart.h"
#include <stdio.h>

/* All IDs: feedback-confirmed home, half turn, home.
 * Existing 270-degree position modes are configured externally. */
static volatile ServoTask_Data actuator = {0, SERVO_TASK_IDLE};
static uint8_t start_pending, phase, stage, stable, failed, moved;
static uint8_t motion_retries, stalled_samples, position_seen;
static uint16_t last_position;
static uint16_t single_target;
static volatile uint16_t retry_count;
enum { BOOT_IDLE, BOOT_POWER_WAIT, BOOT_PROBE_SEND, BOOT_PROBE_WAIT, BOOT_MOVING };
static volatile uint8_t boot_state;
static uint8_t boot_id, boot_probes;
static volatile uint8_t boot_failed;
static uint8_t boot_ready;
static TickType_t boot_tick;
static TickType_t last_motion_sent;
static TickType_t move_started, query_started;

uint16_t ServoTask_GetRetryCount(void) { return retry_count; }
void ServoTask_ResetRetryCount(void) { retry_count = 0U; }

uint8_t ServoTask_IsBusy(void)
{
    return boot_state != BOOT_IDLE || actuator.state == SERVO_TASK_RUNNING;
}

static uint8_t start_move(uint8_t id, uint16_t target)
{
    uint8_t accepted = 0;
    if (id < 1U || id > SERVO_COUNT) return 0;
    taskENTER_CRITICAL();
    if (actuator.state != SERVO_TASK_RUNNING && !USART_ServoControlBusy()) {
        actuator.id = id;
        actuator.state = SERVO_TASK_RUNNING;
        moved = 0;
        single_target = target;
        start_pending = 1;
        accepted = 1;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint8_t ServoTask_Start(uint8_t id)
{
    return ServoTask_CanStrike(id) && start_move(id, 0U);
}

uint8_t ServoTask_MoveTo(uint8_t id, uint16_t pulse)
{
    if (!ServoTask_CanStrike(id) || pulse < 500U || pulse > 2500U) return 0;
    return start_move(id, pulse);
}

uint8_t ServoTask_CanStrike(uint8_t id)
{
    return id >= 1U && id <= SERVO_COUNT && boot_state == BOOT_IDLE &&
           !(boot_failed & (1U << (id - 1U)));
}

void ServoTask_InitRest(void)
{
    ServoTask_ResetRetryCount();
    boot_id = 2U;
    boot_ready = boot_failed = boot_probes = 0U;
    boot_tick = 0U; /* RTOS tick starts at zero after system initialization. */
    boot_state = BOOT_POWER_WAIT;
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

static uint8_t read_position(const char *frame, uint16_t length, uint8_t expected_id,
                             uint16_t *value)
{
    unsigned i, id = 0, position = 0;
    if (length != 10U || frame[0] != '#' || frame[4] != 'P' || frame[9] != '!') return 0;
    for (i = 1; i <= 8; ++i) {
        if (i == 4U) continue;
        if (frame[i] < '0' || frame[i] > '9') return 0;
        if (i < 4U) id = id * 10U + (unsigned)(frame[i] - '0');
        else position = position * 10U + (unsigned)(frame[i] - '0');
    }
    if (id != expected_id || position < 500U || position > 2500U) return 0;
    *value = (uint16_t)position;
    return 1;
}

static uint8_t position_matches(const char *frame, uint16_t length, uint16_t target)
{
    uint16_t position;
    int error, tolerance = SERVO_POSITION_TOLERANCE;
    if (!read_position(frame, length, actuator.id, &position)) return 0;
    any_printf(USART1, "SERVO %u phase=%u pos=%u target=%u\r\n",
               (unsigned)actuator.id, (unsigned)phase, (unsigned)position, (unsigned)target);
    error = (int)position - (int)target;
    if (position_seen && (error < -tolerance || error > tolerance) &&
        (int)position-(int)last_position >= -5 && (int)position-(int)last_position <= 5)
        ++stalled_samples;
    else stalled_samples=0;
    position_seen=1;
    last_position=(uint16_t)position;
    return error >= -tolerance && error <= tolerance;
}

static void phase_failed(void)
{
    failed = 1;
    any_printf(USART1, "SERVO %u phase=%u ERROR\r\n", (unsigned)actuator.id, (unsigned)phase);
    Servo_ClearRx();
    if (!single_target && phase == 1U) {
        phase = 2;
        stage = 0;
    } else {
        Servo_Stop(actuator.id);
        Servo_ClearRx();
        actuator.state = SERVO_TASK_ERROR;
    }
}

static void boot_finish_id(uint8_t success)
{
    uint8_t bit = (uint8_t)(1U << (boot_id - 1U));
    if (success) boot_ready |= bit;
    else boot_failed |= bit;
    actuator.id = boot_id;
    actuator.state = success ? SERVO_TASK_DONE : SERVO_TASK_ERROR;
    any_printf(USART1, "boot: servo %u rest %s\r\n", (unsigned)boot_id,
               success ? "OK" : "FAILED");
    if (boot_id == 2U) {
        boot_id = 3U;
        boot_probes = 0U;
        boot_state = BOOT_PROBE_SEND;
    } else {
        any_printf(USART1, "boot: servo ready=0x%02X failed=0x%02X\r\n",
                   (unsigned)boot_ready, (unsigned)boot_failed);
        boot_state = BOOT_IDLE;
    }
}

/* Read-only rate discovery; no EEPROM or mode writes during startup. */
static void boot_process(TickType_t now)
{
    char frame[SERVO_FRAME_CAPACITY];
    uint16_t length, position;
    uint32_t baud = (boot_probes & 1U) ? 9600UL : 115200UL;
    if (boot_state == BOOT_POWER_WAIT) {
        if ((TickType_t)(now - boot_tick) >= pdMS_TO_TICKS(500U))
            boot_state = BOOT_PROBE_SEND;
    } else if (boot_state == BOOT_PROBE_SEND) {
        Servo_ClearRx();
        if (Servo_SetBusBaud(baud) != SERVO_OK) { boot_finish_id(0U); return; }
        snprintf(frame, sizeof(frame), "#%03uPRAD!", (unsigned)boot_id);
        if (Servo_SendCommand(frame) != SERVO_OK) { boot_finish_id(0U); return; }
        boot_tick = now;
        boot_state = BOOT_PROBE_WAIT;
    } else if (boot_state == BOOT_PROBE_WAIT) {
        if (Servo_GetFrame(frame, sizeof(frame), &length) == SERVO_OK &&
            read_position(frame, length, boot_id, &position)) {
            Servo_ClearRx();
            Servo_SetDeviceBaud(boot_id, baud);
            if (start_move(boot_id, SERVO_REST_PULSE)) boot_state = BOOT_MOVING;
            else boot_finish_id(0U);
        } else if ((TickType_t)(now - boot_tick) >= pdMS_TO_TICKS(MANGO_SERVO_QUERY_MS)) {
            Servo_ClearRx();
            if (++boot_probes >= 6U) boot_finish_id(0U);
            else boot_state = BOOT_PROBE_SEND;
        }
    } else if (boot_state == BOOT_MOVING && actuator.state != SERVO_TASK_RUNNING) {
        boot_finish_id(actuator.state == SERVO_TASK_DONE);
    }
}

void servo_task(void *pvParameters)
{
    TickType_t now;
    uint16_t target, length;
    char frame[SERVO_FRAME_CAPACITY];
    (void)pvParameters;
    for (;;) {
        if (boot_state != BOOT_IDLE) boot_process(xTaskGetTickCount());
        if (start_pending) {
            start_pending = 0;
            phase = stage = stable = failed = moved = 0;

        }
        if (actuator.state == SERVO_TASK_RUNNING) {
            target = single_target ? single_target :
                (phase == 1U ? SERVO_HALF_TURN_PULSE : SERVO_HOME_PULSE);
            now = xTaskGetTickCount();
            if (stage == 0U) {
                Servo_ClearRx();
                moved = 1;
                any_printf(USART1, "SERVO %u phase=%u target=%u time=%lu\r\n",
                           (unsigned)actuator.id, (unsigned)phase, (unsigned)target, (unsigned long)SERVO_MOVE_MS);
                if (Servo_MovePulse(actuator.id, target, SERVO_MOVE_MS) != SERVO_OK) {
                    phase_failed();
                } else {
                    move_started = last_motion_sent = xTaskGetTickCount();
                    motion_retries=stalled_samples=position_seen=0;
                    stable = 0;
                    stage = 1;
                }
            } else if ((TickType_t)(now - move_started) >= pdMS_TO_TICKS(MANGO_SERVO_PHASE_TIMEOUT_MS)) {
                phase_failed();
            } else if (stage == 1U &&
                       (TickType_t)(now - last_motion_sent) >= pdMS_TO_TICKS(SERVO_MOVE_MS)) {
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
                        if (!single_target && phase < 2U) { ++phase; stage = 0; }
                        else {
                            actuator.state = failed ? SERVO_TASK_ERROR : SERVO_TASK_DONE;
                            any_printf(USART1, "SERVO %u finished error=%u\r\n",
                                       (unsigned)actuator.id, (unsigned)failed);
                        }
                    }
                }
                if (stage == 2U && actuator.state == SERVO_TASK_RUNNING) {
                /* Retry a stalled absolute target within the original
                 * deadline, using the same feedback policy as manual S1..S4. */
                if (!stable && motion_retries<2U &&
                    ((position_seen &&
                      ((int)last_position-(int)target < -(int)SERVO_POSITION_TOLERANCE ||
                       (int)last_position-(int)target > (int)SERVO_POSITION_TOLERANCE)) ||
                     (!position_seen && boot_state == BOOT_MOVING)) &&
                    (stalled_samples>=1U || (TickType_t)(now-last_motion_sent)>=pdMS_TO_TICKS(1000U)) &&
                    (TickType_t)(now-move_started)+pdMS_TO_TICKS(SERVO_MOVE_MS+300U)<pdMS_TO_TICKS(MANGO_SERVO_PHASE_TIMEOUT_MS)) {
                    Servo_ClearRx();
                    ++motion_retries;
                    ++retry_count;
                    stalled_samples=0;
                    if (Servo_MovePulse(actuator.id,target,SERVO_MOVE_MS)!=SERVO_OK) phase_failed();
                    else { last_motion_sent=xTaskGetTickCount(); stage=1; }
                    any_printf(USART1,"SERVO %u motion retry=%u target=%u\r\n",
                               (unsigned)actuator.id,(unsigned)motion_retries,(unsigned)target);
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
