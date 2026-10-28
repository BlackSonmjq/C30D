/* Execute the production coordinator and timed pushrod task in ARM emulation.
 * RTOS delay/timers/sensor/GPIO are deterministic mocks, not physical hardware.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#define DistanceTask_SelectLaser Mock_SelectLaser
#define DistanceTask_GetData Mock_GetDistance
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/pushrod_task.c"
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/stepper_task.c"

static void test_distance_flow(void);
static void distance_delay(TickType_t ticks);
static u8 in_distance, stop_return;
static jmp_buf job_exit, actuator_yield, distance_exit;
static TickType_t tick, high_at, last_high_duration;
static unsigned int checks, completed_jobs, hits, starts, case_id, stop_partial_edge;
static u8 in_actuator, sensor, active_motor, output_id, forced_invalid, initial_close;
static u8 inject_busy, inject_pulse_limit, fail_return, fail_strike, freeze_motion, jump_clock;
static u32 count[3], target[3];
static u8 running[3];
static s32 last_move;
static Mango_Result expected_result;

#define CHECK(expr) do { ++checks; if (!(expr)) { \
    printf("FAIL case %u line %u: %s\n", case_id, (unsigned int)__LINE__, #expr); exit(1); } } while (0)

TickType_t xTaskGetTickCount(void) { return tick; }
void any_printf(void *port, char *format, ...) { (void)port; (void)format; }
u8 StepperMotor_SetSpeed(u8 motor, u16 arr) { CHECK(arr == 199); return motor <= 2; }
u32 StepperMotor_GetCompletedPulses(u8 motor) { return count[motor]; }
u8 StepperMotor_IsRunning(u8 motor) { return running[motor]; }
void StepperMotor_Disable(u8 motor) {
    if (running[motor] && stop_partial_edge && last_move < 0) {
        ++count[motor]; /* Edge emitted after the last polling snapshot. */
        stop_partial_edge = 0;
    }
    running[motor] = 0;
}
void StepperMotor_ResetCompletedPulses(u8 motor) { StepperMotor_Disable(motor); count[motor] = 0; }
u8 StepperMotor_StartMove(u8 motor, s32 pulses) {
    CHECK(motor == active_motor);
    CHECK(output_id == 0); /* No motion while a pushrod output is high. */
    if (fail_return && pulses > 0) return 0;
    if (pulses < 0) CHECK((u32)-pulses == MANGO_MAX_PULSES);
    else CHECK((u32)pulses == count[motor]);
    last_move = pulses;
    count[motor] = 0;
    target[motor] = pulses < 0 ? (u32)-pulses : (u32)pulses;
    running[motor] = 1;
    ++starts;
    return 1;
}
void DistanceTask_SelectLaser(uint8_t id) { sensor = id; }
void DistanceTask_GetData(DistanceTask_Data *data) {
    memset(data, 0, sizeof(*data));
    data->laser_id = sensor;
    data->valid = sensor && !forced_invalid;
    data->status = data->valid ? VL53L0X_OK : VL53L0X_NOT_READY;
    data->distance_mm = initial_close || (last_move < 0 && count[active_motor] >= 37) ? 150 : 151;
    if (inject_pulse_limit) data->distance_mm = 151;
}
Pushrod_Result Pushrod_Start(uint8_t id) {
    CHECK(id == motion.mango_id);
    CHECK(!running[active_motor]);
    CHECK(sensor == id);
    if (fail_strike) return PUSHROD_NOT_INITIALIZED;
    CHECK(output_id == 0);
    output_id = id;
    high_at = tick;
    ++hits;
    return PUSHROD_OK;
}
Pushrod_Result Pushrod_Stop(uint8_t id) {
    if (output_id == id) {
        last_high_duration = tick - high_at;
        CHECK(last_high_duration >= MANGO_STRIKE_MS);
        CHECK(last_high_duration <= MANGO_STRIKE_MS + 10);
        output_id = 0;
    }
    return PUSHROD_OK;
}
void vTaskDelay(TickType_t ticks) {
    u32 delta;
    if (in_distance) { distance_delay(ticks); return; }
    if (in_actuator) longjmp(actuator_yield, 1);
    CHECK(ticks == 10);
    tick += ticks;
    if (jump_clock && motion.state == STEPPER_MOVING) {
        tick += MANGO_MOTION_TIMEOUT_MS;
        jump_clock = 0;
    }
    if (motion.state == STEPPER_MOVING && inject_busy) {
        CHECK(!StepperTask_RequestMangoFromISR(4));
        CHECK(sensor == motion.mango_id);
        inject_busy = 0;
    }
    if (motion.state == STEPPER_MOVING && expected_result == MANGO_SENSOR_LOST)
        forced_invalid = 1;
    if (running[active_motor] && !freeze_motion) {
        delta = inject_pulse_limit && last_move < 0 ? MANGO_MAX_PULSES : 7;
        if (delta > target[active_motor] - count[active_motor]) delta = target[active_motor] - count[active_motor];
        count[active_motor] += delta;
        if (count[active_motor] == target[active_motor]) running[active_motor] = 0;
    }
    if (motion.state == STEPPER_RETURNING && stop_return) {
        running[active_motor] = 0;
        stop_return = 0;
    }
    if (motion.state == STEPPER_RETURNING && freeze_motion) {
        tick += MANGO_MOTION_TIMEOUT_MS;
        running[active_motor] = 0;
    }
    in_actuator = 1;
    if (!setjmp(actuator_yield)) pushrod_task(NULL);
    in_actuator = 0;
    if (motion.state == STEPPER_FAULT) {
        CHECK(expected_result == MANGO_RETURN_FAILED);
        CHECK(!StepperTask_RequestMangoFromISR(1));
        CHECK(!output_id && !running[active_motor] && !sensor);
        longjmp(job_exit, 1);
    }
    if (motion.state == STEPPER_RETURNED) {
        CHECK(motion.result == expected_result);
        CHECK(motion.pulses == motion.outward_pulses);
        CHECK(!output_id && !running[active_motor] && !sensor);
        CHECK(!job_busy);
        ++completed_jobs;
        longjmp(job_exit, 1);
    }
    CHECK(tick < 2000000U || (tick > 0xffff0000U));
}
static void run_job(u8 id, Mango_Result result) {
    active_motor = id <= 2 ? 1 : 2;
    expected_result = result;
    CHECK(StepperTask_RequestMangoFromISR(id));
    CHECK(!StepperTask_RequestMangoFromISR(id)); /* Reserve even before task starts. */
    if (!setjmp(job_exit)) stepper_Task(NULL);
    CHECK(motion.motor == active_motor && motion.mango_id == id);
}
static void reset(void) {
    stop_return = 0;
    tick = 0; job_busy = pending_mango = rejected_command = 0;
    actuator.state = PUSHROD_TASK_IDLE; actuator.id = start_pending = 0;
    memset(&motion, 0, sizeof(motion));
    memset(count, 0, sizeof(count)); memset(running, 0, sizeof(running));
    hits = starts = stop_partial_edge = 0; last_move = 0;
    forced_invalid = initial_close = inject_busy = inject_pulse_limit = 0;
    fail_return = fail_strike = freeze_motion = jump_clock = output_id = sensor = 0;
    last_high_duration = 0;
    ++case_id;
}
int main(void) {
    u8 id;
    for (id = 1; id <= 4; ++id) {
        reset(); inject_busy = 1; stop_partial_edge = 1;
        run_job(id, MANGO_OK);
        CHECK(hits == 1 && starts == 2 && motion.outward_pulses == 43);
        /* Same ID again, initially at threshold: do not return stale pulses. */
        initial_close = 1;
        run_job(id, MANGO_OK);
        CHECK(hits == 2 && motion.outward_pulses == 0 && starts == 2);
    }
    reset(); forced_invalid = 1; run_job(2, MANGO_RANGE_TIMEOUT);
    CHECK(!hits && !starts);
    reset(); run_job(3, MANGO_SENSOR_LOST); CHECK(!hits && starts == 2);
    reset(); inject_pulse_limit = 1; run_job(1, MANGO_PULSE_LIMIT);
    CHECK(!hits && motion.outward_pulses == MANGO_MAX_PULSES);
    reset(); fail_strike = 1; run_job(4, MANGO_PUSHROD_FAILED); CHECK(!hits && starts == 2);
    reset(); jump_clock = 1; run_job(1, MANGO_MOTION_TIMEOUT); CHECK(!hits && starts == 2);
    reset(); fail_return = 1; run_job(3, MANGO_RETURN_FAILED); CHECK(hits == 1);
    reset(); stop_return = 1; run_job(1, MANGO_RETURN_FAILED); CHECK(hits == 1);
    reset(); tick = 0xfffffff0U; run_job(1, MANGO_OK); CHECK(hits == 1);
    reset(); CHECK(!StepperTask_RequestMangoFromISR(0)); CHECK(!StepperTask_RequestMangoFromISR(5));
    test_distance_flow();
    printf("PASS: %u scenarios, %u completed jobs, %u checks\n", case_id, completed_jobs, checks);
    return 0;
}


/* Now execute the real four-channel distance task, independently of the
 * coordinator's sensor mock above. */
#undef DistanceTask_SelectLaser
#undef DistanceTask_GetData
#define publish publish_distance
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/distance_task.c"
#undef publish

static uint16_t gpio_levels;
static u8 initialized_laser, measured_laser, gpio_ready;
static unsigned int range_reads, distance_loops;
static const uint16_t expected_shut[4] = {1U << 2, 1U << 3, 1U << 6, 1U << 7};
void GPIO_ResetBits(void *port, uint16_t mask) {
    CHECK(port == GPIOA && mask == LASER_SHUT_PINS);
    gpio_levels &= ~mask;
}
void GPIO_SetBits(void *port, uint16_t mask) {
    CHECK(port == GPIOA && gpio_ready);
    CHECK(gpio_levels == 0); /* Every selection must first shut down ALL. */
    CHECK(requested_laser >= 1 && requested_laser <= 4);
    CHECK(mask == expected_shut[requested_laser - 1]);
    gpio_levels |= mask;
}
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio) {
    CHECK(port == GPIOA && gpio->GPIO_Pin == LASER_SHUT_PINS);
    CHECK(!gpio_levels && gpio->GPIO_Mode == GPIO_Mode_OUT && gpio->GPIO_PuPd == GPIO_PuPd_DOWN);
    gpio_ready = 1;
}
void RCC_AHB1PeriphClockCmd(unsigned int peripheral, unsigned int enabled) {
    CHECK(peripheral == RCC_AHB1Periph_GPIOA && enabled == ENABLE);
}
VL53L0X_Status VL53L0X_Init(void) {
    CHECK(gpio_levels == expected_shut[requested_laser - 1]);
    CHECK(requested_laser == initialized_laser + 1);
    initialized_laser = requested_laser;
    return VL53L0X_OK;
}
VL53L0X_Status VL53L0X_ReadSingle(VL53L0X_Result *result) {
    CHECK(active_laser == initialized_laser && gpio_levels == expected_shut[active_laser - 1]);
    measured_laser = active_laser;
    ++range_reads;
    result->distance_mm = 123;
    return VL53L0X_OK;
}
static void distance_delay(TickType_t ticks) {
    DistanceTask_Data snapshot;
    tick += ticks;
    CHECK(++distance_loops < 1000);
    DistanceTask_GetData(&snapshot);
    if (!initialized_laser && ticks == 10 && active_laser == 0) {
        CHECK(!gpio_levels && !snapshot.valid);
        DistanceTask_SelectLaser(1);
    } else if (measured_laser == active_laser && measured_laser && snapshot.valid) {
        CHECK(snapshot.laser_id == measured_laser && snapshot.distance_mm == 123);
        DistanceTask_SelectLaser(measured_laser == 4 ? 0 : measured_laser + 1);
        DistanceTask_GetData(&snapshot);
        CHECK(!snapshot.valid); /* Old channel invalid immediately on request. */
    } else if (initialized_laser == 4 && !requested_laser && !active_laser && ticks == 10) {
        CHECK(!gpio_levels && !snapshot.valid && snapshot.laser_id == 0);
        longjmp(distance_exit, 1);
    }
}
static void test_distance_flow(void) {
    DistanceTask_Data snapshot;
    ++case_id;
    in_distance = 1;
    tick = 0;
    DistanceTask_InitShutdownPins();
    CHECK(gpio_ready && !gpio_levels);
    DistanceTask_SelectLaser(5);
    CHECK(!requested_laser);
    if (!setjmp(distance_exit)) distance_task(NULL);
    in_distance = 0;
    CHECK(initialized_laser == 4 && range_reads == 4);
    distance_data.valid = 1;
    distance_data.laser_id = requested_laser = 1;
    distance_data.update_tick = tick;
    tick += 500;
    DistanceTask_GetData(&snapshot);
    CHECK(!snapshot.valid && snapshot.status == VL53L0X_ERROR_TIMEOUT);
}
