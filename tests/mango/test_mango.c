/* Execute the production coordinator and servo actuator task in ARM emulation.
 * RTOS delay/timers/sensor/GPIO are deterministic mocks, not physical hardware.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#define DistanceTask_SelectLaser Mock_SelectLaser
#define DistanceTask_GetData Mock_GetDistance
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/servo_task.c"
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/stepper_task.c"

static void test_distance_flow(void);
static void distance_delay(TickType_t ticks);
static u8 in_distance, stop_return;
static jmp_buf job_exit, actuator_yield, distance_exit;
static TickType_t tick, high_at;
static uint8_t reply_pending, servo_id, manual_busy, bad_reply, no_reply;
static uint8_t fail_home, fail_query, home_after_twist;
static uint16_t servo_target;
static uint16_t physical_position;
static uint8_t lossy_test, drop_outward, drop_return;
static uint8_t sparse_lossy, stalled_replies;
static unsigned int move_count, query_count, stops;
static unsigned int checks, completed_jobs, hits, starts, case_id, stop_partial_edge;
static u8 in_actuator, sensor, active_motor, output_id, forced_invalid, initial_close;
static u8 inject_busy, inject_pulse_limit, fail_return, fail_strike, freeze_motion, jump_clock;
static u32 count[3], target[3];
static u8 running[3];
static s32 last_move;
static Mango_Result expected_result;
/* Independent expected values: do not derive boundary inputs from production macros. */
static const u16 expected_targets[4] = {108, 60, 45, 35};
/* Hardware direction: left approaches low, right approaches high. */
static int outward(s32 pulses) { return active_motor == 1 ? pulses < 0 : pulses > 0; }

#define CHECK(expr) do { ++checks; if (!(expr)) { \
    printf("FAIL case %u line %u: %s\n", case_id, (unsigned int)__LINE__, #expr); exit(1); } } while (0)

TickType_t xTaskGetTickCount(void) { return tick; }
void any_printf(void *port, char *format, ...) { (void)port; (void)format; }
u8 StepperMotor_SetSpeed(u8 motor, u16 arr) { CHECK(arr == 199); return motor <= 2; }
u32 StepperMotor_GetCompletedPulses(u8 motor) { return count[motor]; }
u8 StepperMotor_IsRunning(u8 motor) { return running[motor]; }
void StepperMotor_Disable(u8 motor) {
    if (running[motor] && stop_partial_edge && outward(last_move)) {
        ++count[motor]; /* Edge emitted after the last polling snapshot. */
        stop_partial_edge = 0;
    }
    running[motor] = 0;
}
void StepperMotor_ResetCompletedPulses(u8 motor) { StepperMotor_Disable(motor); count[motor] = 0; }
u8 StepperMotor_StartMove(u8 motor, s32 pulses) {
    CHECK(motor == active_motor);
    CHECK(output_id == 0); /* No motion while a pushrod output is high. */
    if (fail_return && !outward(pulses)) return 0;
    if (outward(pulses)) CHECK((u32)(pulses < 0 ? -pulses : pulses) == MANGO_MAX_PULSES);
    else CHECK((u32)(pulses < 0 ? -pulses : pulses) == count[motor]);
    last_move = pulses;
    count[motor] = 0;
    target[motor] = pulses < 0 ? (u32)-pulses : (u32)pulses;
    running[motor] = 1;
    ++starts;
    return 1;
}
void DistanceTask_SelectLaser(uint8_t id) { sensor = id; }
void DistanceTask_GetData(DistanceTask_Data *data) {
    u16 threshold = sensor >= 1U && sensor <= 4U ? expected_targets[sensor - 1U] : 60U;
    memset(data, 0, sizeof(*data));
    data->laser_id = sensor;
    data->valid = sensor && !forced_invalid;
    data->status = data->valid ? VL53L0X_OK : VL53L0X_NOT_READY;
    /* Equality keeps approaching; one millimeter below permits the servo. */
    data->distance_mm = initial_close || (outward(last_move) && count[active_motor] >= 37) ? threshold - 1U : threshold;
    if (inject_pulse_limit)
        data->distance_mm = inject_pulse_limit == 2U && count[active_motor] >= MANGO_MAX_PULSES ? threshold - 1U : threshold;
}
u8 USART_ServoControlBusy(void) { return manual_busy; }
void Servo_ClearRx(void) { reply_pending = 0; }
Servo_Result Servo_MovePulse(uint8_t id, uint16_t pulse, uint16_t duration_ms) {
    unsigned int index = move_count % 3U;
    CHECK(id == motion.mango_id && !running[active_motor] && sensor == id);
    CHECK(duration_ms == SERVO_MOVE_MS);
    if (fail_strike && pulse == SERVO_HALF_TURN_PULSE) return SERVO_TX_TIMEOUT;
    if (fail_home && hits && pulse == SERVO_HOME_PULSE) return SERVO_TX_TIMEOUT;
    CHECK(lossy_test || fail_strike || fail_query || bad_reply || no_reply ||
          (index == 0U ? pulse == SERVO_HOME_PULSE :
          index == 1U ? pulse == SERVO_HALF_TURN_PULSE : pulse == SERVO_HOME_PULSE));
    ++move_count;
    servo_id = id; servo_target = pulse;
    if (pulse==SERVO_HALF_TURN_PULSE && drop_outward) --drop_outward;
    else if (pulse==SERVO_HOME_PULSE && hits && drop_return) --drop_return;
    else physical_position=pulse==500U ? 545U : pulse;
    high_at = tick; output_id = id;
    if (pulse == SERVO_HALF_TURN_PULSE) ++hits;
    else if (hits) home_after_twist = 1;
    return SERVO_OK;
}
Servo_Result Servo_ReadPosition(uint8_t id) {
    CHECK(id == servo_id && (TickType_t)(tick-high_at) >= SERVO_MOVE_MS);
    /* All IDs now require feedback confirmation. */
    ++query_count;
    if (fail_query && servo_target == SERVO_HALF_TURN_PULSE) return SERVO_TX_TIMEOUT;
    reply_pending = 1;
    return SERVO_OK;
}
Servo_Result Servo_GetFrame(char *buffer, uint16_t capacity, uint16_t *length) {
    CHECK(capacity >= 11);
    if (!reply_pending || (no_reply && servo_target == SERVO_HALF_TURN_PULSE)) return SERVO_NO_FRAME;
    reply_pending = 0;
    if (sparse_lossy && abs((int)physical_position-(int)servo_target)>100 && stalled_replies++>0U)
        return SERVO_NO_FRAME;
    if (bad_reply && servo_target == SERVO_HALF_TURN_PULSE) {
        /* ACK, wrong ID, echo, malformed frame and off-target feedback. */
        switch (query_count % 5U) {
        case 0: strcpy(buffer,"#OK!"); break;
        case 1: sprintf(buffer,"#%03uP1833!", (unsigned)(servo_id % 4U + 1U)); break;
        case 2: sprintf(buffer,"#%03uP1833T0500!",(unsigned)servo_id); break;
        case 3: sprintf(buffer,"#%03uP18x3!",(unsigned)servo_id); break;
        default: sprintf(buffer,"#%03uP1200!",(unsigned)servo_id); break;
        }
    } else {
        sprintf(buffer,"#%03uP%04u!",(unsigned)servo_id,(unsigned)physical_position);
        if (phase == 2U && stable == 1U && servo_target == SERVO_HOME_PULSE) output_id = 0;
    }
    *length = (uint16_t)strlen(buffer);
    return SERVO_OK;
}
Servo_Result Servo_Stop(uint8_t id) {
    CHECK(id == servo_id || fail_strike);
    output_id = 0; ++stops;
    return SERVO_OK;
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
        delta = inject_pulse_limit && outward(last_move) ? MANGO_MAX_PULSES : 7;
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
    if (!setjmp(actuator_yield)) servo_task(NULL);
    in_actuator = 0;
    if (servo_id == 2U && (TickType_t)(tick-high_at) >= SERVO_MOVE_MS) output_id = 0;
    if (motion.state == STEPPER_FAULT) {
        CHECK(expected_result == MANGO_RETURN_FAILED);
        CHECK(StepperTask_IsBusy());
        CHECK(!StepperTask_RequestMangoFromISR(1));
        CHECK(!output_id && !running[active_motor] && !sensor);
        longjmp(job_exit, 1);
    }
    if (motion.state == STEPPER_RETURNED) {
        CHECK(motion.result == expected_result);
        CHECK(motion.pulses == motion.outward_pulses);
        CHECK(!output_id && !running[active_motor] && !sensor);
        CHECK(!job_busy && !StepperTask_IsBusy());
        ++completed_jobs;
        longjmp(job_exit, 1);
    }
    CHECK(tick < 2000000U || (tick > 0xffff0000U));
}
static void run_job(u8 id, Mango_Result result) {
    active_motor = id <= 2 ? 1 : 2;
    expected_result = result;
    CHECK(StepperTask_RequestMangoFromISR(id));
    CHECK(StepperTask_IsBusy());
    CHECK(!StepperTask_RequestMangoFromISR(id)); /* Reserve even before task starts. */
    if (!setjmp(job_exit)) stepper_Task(NULL);
    CHECK(motion.motor == active_motor && motion.mango_id == id);
}
static void reset(void) {
    stop_return = 0;
    tick = 0; job_busy = pending_mango = rejected_command = 0;
    actuator.state = SERVO_TASK_IDLE; actuator.id = start_pending = 0;
    phase = stage = stable = failed = moved = 0;
    memset(&motion, 0, sizeof(motion));
    memset(count, 0, sizeof(count)); memset(running, 0, sizeof(running));
    hits = starts = stop_partial_edge = 0; last_move = 0;
    forced_invalid = initial_close = inject_busy = inject_pulse_limit = 0;
    fail_return = fail_strike = freeze_motion = jump_clock = output_id = sensor = 0;
    reply_pending = servo_id = manual_busy = bad_reply = no_reply = 0;
    fail_home = fail_query = home_after_twist = 0;
    move_count = query_count = stops = 0; servo_target = 0;
    lossy_test=drop_outward=drop_return=0; physical_position=545;
    sparse_lossy=stalled_replies=0;
    ++case_id;
}
int main(void) {
    u8 id;
    CHECK(MANGO_MAX_PULSES == 26000UL);
    actuator.id = 2;
    CHECK(position_matches("#002P0600!", 10, 500));
    CHECK(!position_matches("#002P0601!", 10, 500));
    CHECK(position_matches("#002P1733!", 10, 1833));
    CHECK(position_matches("#002P1933!", 10, 1833));
    CHECK(!position_matches("#002P1732!", 10, 1833));
    CHECK(!position_matches("#002P1934!", 10, 1833));
    for (id = 1; id <= 4; ++id) {
        char reply[16];

        actuator.id = id;
        sprintf(reply, "#%03uP0600!", (unsigned)id); CHECK(position_matches(reply, 10, 500));
        sprintf(reply, "#%03uP0601!", (unsigned)id); CHECK(!position_matches(reply, 10, 500));
        sprintf(reply, "#%03uP1933!", (unsigned)id); CHECK(position_matches(reply, 10, 1833));
        sprintf(reply, "#%03uP1934!", (unsigned)id); CHECK(!position_matches(reply, 10, 1833));
    }
    for (id = 1; id <= 4; ++id) {
        reset(); inject_busy = 1; stop_partial_edge = 1;
        run_job(id, MANGO_OK);
        CHECK(hits == 1 && starts == 2 && motion.outward_pulses == 43);
        CHECK(move_count == 3U && query_count == 6U && home_after_twist);
        /* Same ID again, initially 1 mm below threshold: no stale pulses. */
        initial_close = 1;
        run_job(id, MANGO_OK);
        CHECK(hits == 2 && motion.outward_pulses == 0 && starts == 2);
    }
    reset(); forced_invalid = 1; run_job(2, MANGO_RANGE_TIMEOUT);
    CHECK(!hits && !starts);
    reset(); run_job(3, MANGO_SENSOR_LOST); CHECK(!hits && starts == 2);
    reset(); inject_pulse_limit = 1; run_job(1, MANGO_PULSE_LIMIT);
    CHECK(!hits && motion.outward_pulses == MANGO_MAX_PULSES);
    /* All four IDs: reaching 26000 pulses and a distance below threshold must return,
     * never start the servo. Return the same pulse count on both motors. */
    for (id = 1; id <= 4; ++id) {
        reset(); inject_pulse_limit = 2; run_job(id, MANGO_PULSE_LIMIT);
        CHECK(!hits && !move_count && starts == 2);
        CHECK(motion.outward_pulses == 26000UL && motion.pulses == 26000UL);
    }
    reset(); fail_strike = 1; run_job(4, MANGO_PUSHROD_FAILED); CHECK(!hits && starts == 2);
    reset(); jump_clock = 1; run_job(1, MANGO_MOTION_TIMEOUT); CHECK(!hits && starts == 2);
    reset(); fail_return = 1; run_job(3, MANGO_RETURN_FAILED); CHECK(hits == 1);
    reset(); stop_return = 1; run_job(1, MANGO_RETURN_FAILED); CHECK(hits == 1);
    reset(); tick = 0xfffffff0U; run_job(1, MANGO_OK); CHECK(hits == 1);
    reset(); CHECK(!StepperTask_RequestMangoFromISR(0)); CHECK(!StepperTask_RequestMangoFromISR(5));
    reset(); bad_reply = 1; run_job(2, MANGO_PUSHROD_FAILED); CHECK(home_after_twist && starts == 2);
    reset(); no_reply = 1; run_job(2, MANGO_PUSHROD_FAILED); CHECK(home_after_twist && query_count);
    reset(); fail_strike = 1; run_job(2, MANGO_PUSHROD_FAILED); CHECK(!hits && starts == 2);
    reset(); fail_home = 1; run_job(2, MANGO_PUSHROD_FAILED); CHECK(stops && starts == 2);
    reset(); tick = 0xfffffff0U; initial_close = 1; run_job(2, MANGO_OK); CHECK(home_after_twist && query_count);
    reset(); no_reply = 1; run_job(3, MANGO_PUSHROD_FAILED); CHECK(home_after_twist && starts == 2);
    reset(); fail_query = 1; run_job(1, MANGO_PUSHROD_FAILED); CHECK(home_after_twist && starts == 2);
    reset(); fail_home = 1; run_job(4, MANGO_PUSHROD_FAILED); CHECK(stops && starts == 2);
    reset(); manual_busy = 1; run_job(2, MANGO_PUSHROD_FAILED); CHECK(!hits && !move_count);
    reset(); lossy_test=drop_outward=1; run_job(2,MANGO_OK);
    CHECK(move_count==4 && home_after_twist);
    reset(); lossy_test=drop_return=1; run_job(2,MANGO_OK);
    CHECK(move_count==4 && home_after_twist);
    reset(); sparse_lossy=lossy_test=drop_outward=1; run_job(2,MANGO_OK);
    CHECK(move_count==4 && home_after_twist);
    reset(); sparse_lossy=lossy_test=drop_return=1; run_job(2,MANGO_OK);
    CHECK(move_count==4 && home_after_twist);
    reset(); CHECK(!ServoTask_Start(0) && !ServoTask_Start(5));
    CHECK(ServoTask_Start(1) && !ServoTask_Start(2));
    ServoTask_Cancel(); CHECK(!ServoTask_IsBusy() && !move_count && !stops);
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
static const uint16_t expected_shut[4] = {1U << 2, 1U << 3, 1U << 6, 1U << 11};
void GPIO_ResetBits(void *port, uint16_t mask) {
    CHECK((port == GPIOA && mask == LASER_SHUT_PINS) || (port == GPIOC && mask == GPIO_Pin_11));
    gpio_levels &= ~mask;
}
void GPIO_SetBits(void *port, uint16_t mask) {
    CHECK(port == (requested_laser == 4 ? GPIOC : GPIOA) && gpio_ready);
    CHECK(gpio_levels == 0); /* Every selection must first shut down ALL. */
    CHECK(requested_laser >= 1 && requested_laser <= 4);
    CHECK(mask == expected_shut[requested_laser - 1]);
    gpio_levels |= mask;
}
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio) {
    CHECK((port == GPIOA && gpio->GPIO_Pin == LASER_SHUT_PINS) || (port == GPIOC && gpio->GPIO_Pin == GPIO_Pin_11));
    CHECK(!gpio_levels && gpio->GPIO_Mode == GPIO_Mode_OUT && gpio->GPIO_PuPd == GPIO_PuPd_DOWN);
    gpio_ready = 1;
}
void RCC_AHB1PeriphClockCmd(unsigned int peripheral, unsigned int enabled) {
    CHECK(peripheral == (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC) && enabled == ENABLE);
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
