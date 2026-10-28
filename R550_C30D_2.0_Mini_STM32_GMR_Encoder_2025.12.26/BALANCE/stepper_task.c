#include "stepper_task.h"
#include "distance_task.h"
#include "FreeRTOS.h"
#include "task.h"

/* Left motor: negative pulse direction (DIR low) is forward. */
#define STEPPER_DISTANCE_MOTOR A4988_MOTOR_Left
#define STEPPER_FORWARD 0U
#define STEPPER_TARGET_MM 130U
#define STEPPER_DISTANCE_ARR 99U
#define STEPPER_CHECK_MS 10U
#define STEPPER_MAX_PULSES 70000UL

static volatile u8 start_requested = 0;
static volatile u8 return_requested = 0;
static volatile StepperTask_Data motion = {0, STEPPER_WAITING_COMMAND};

void StepperTask_RequestStartFromISR(void)
{
    /* USART1 IRQ priority 3 cannot call FreeRTOS FromISR APIs. */
    if (motion.state == STEPPER_WAITING_COMMAND)
        start_requested = 1;
}

void StepperTask_RequestReturnFromISR(void)
{
    if (motion.state == STEPPER_TARGET_REACHED || motion.state == STEPPER_LIMIT_REACHED)
        return_requested = 1;
}

u8 StepperTask_IsPositionReadyFromISR(void)
{
    return motion.state == STEPPER_TARGET_REACHED;
}

void StepperTask_GetData(StepperTask_Data *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = motion;
    taskEXIT_CRITICAL();
}

static void publish(StepperTask_State state, u32 count)
{

    taskENTER_CRITICAL();
    motion.pulses = count;
    motion.state = state;
    taskEXIT_CRITICAL();
}

void stepper_Task(void *pvParameters)
{
    DistanceTask_Data distance;
    u32 outward_pulses = 0, returned_pulses = 0, remaining, chunk, completed;
    StepperTask_State state = STEPPER_WAITING_COMMAND;
    (void)pvParameters;

    StepperMotor_Disable(STEPPER_DISTANCE_MOTOR);
    StepperMotor_SetSpeed(STEPPER_DISTANCE_MOTOR, STEPPER_DISTANCE_ARR);

    /* Task stays alive; reaching the target latches the stopped state. */
    for (;;) {
        if (state == STEPPER_WAITING_COMMAND && start_requested) {
            start_requested = 0;
            state = STEPPER_WAITING_DISTANCE;
        }
        if ((state == STEPPER_TARGET_REACHED || state == STEPPER_LIMIT_REACHED) && return_requested) {
            return_requested = 0;
            outward_pulses = StepperMotor_GetCompletedPulses(STEPPER_DISTANCE_MOTOR);
            state = STEPPER_RETURNING;
            publish(state, 0);
            remaining = outward_pulses;
            returned_pulses = 0;
            /* MovePulses takes s32; split a full u32 count without overflow. */
            while (remaining != 0) {
                chunk = remaining > 0x7fffffffUL ? 0x7fffffffUL : remaining;
                completed = StepperMotor_MovePulses(STEPPER_DISTANCE_MOTOR,
                                                   (s32)chunk);
                returned_pulses += completed;
                if (completed != chunk) break;
                remaining -= completed;
            }
            state = (returned_pulses == outward_pulses) ?
                    STEPPER_RETURNED : STEPPER_RETURN_STOP;
        }
        DistanceTask_GetData(&distance);

        /* Wait for a valid measurement before starting the motor. */
        if (state == STEPPER_WAITING_DISTANCE && distance.valid) {
            if (distance.distance_mm <= STEPPER_TARGET_MM) {
                state = STEPPER_TARGET_REACHED;
            } else if (StepperMotor_StartMove(STEPPER_DISTANCE_MOTOR,
                         STEPPER_FORWARD ? (s32)STEPPER_MAX_PULSES : -(s32)STEPPER_MAX_PULSES)) {
                state = STEPPER_MOVING;
            } else {
                state = STEPPER_SENSOR_STOP;
            }
        }

        if (state == STEPPER_MOVING) {
            if (StepperMotor_GetCompletedPulses(STEPPER_DISTANCE_MOTOR) >= STEPPER_MAX_PULSES) {
                StepperMotor_Disable(STEPPER_DISTANCE_MOTOR);
                state = STEPPER_LIMIT_REACHED;
                return_requested = 1;
            } else if (!distance.valid) {
                StepperMotor_Disable(STEPPER_DISTANCE_MOTOR);
                state = STEPPER_SENSOR_STOP;
            } else if (distance.distance_mm <= STEPPER_TARGET_MM) {
                StepperMotor_Disable(STEPPER_DISTANCE_MOTOR);
                state = STEPPER_TARGET_REACHED;
            }
        }

        /* data_task sends USART1 ready and the final count once on success. */
        publish(state, (state == STEPPER_RETURNED || state == STEPPER_RETURN_STOP) ?
                returned_pulses : StepperMotor_GetCompletedPulses(STEPPER_DISTANCE_MOTOR));
        /* Command 2 returns after target; reaching the pulse limit returns automatically. */
        vTaskDelay(pdMS_TO_TICKS(STEPPER_CHECK_MS));
    }
}
