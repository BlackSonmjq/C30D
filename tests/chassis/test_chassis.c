/* Prepended to extracted production functions by run_tests.py. */
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include "sys.h"
#include "../../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/heading_control.h"

typedef struct { float Encoder, Target; int Motor_Pwm; } Motor;
static Motor MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
static struct { float VX, VY, VZ; } smooth_control;
static struct { u32 Time_count; } SysVal;
static struct { struct { short z; } gyro; } imu;
enum { Mec_Car, Mec_Car_V550, Omni_Car, Akm_Car, Diff_Car,
       FourWheel_Car, FourWheel_Car_V550, Tank_Car };
static u8 Car_Mode, Allow_Recharge, Get_Charging_HardWare, nav_walk, RED_STATE;
static u8 APP_ON_Flag, Remote_ON_Flag, PS2_ON_Flag, CAN_ON_Flag;
static u8 Usart1_ON_Flag, Usart5_ON_Flag, Flag_Stop, EN = 1, FlashWriteFlag;
static u8 start_check_flag, wait_clear_times, start_clear, clear_done_once, clear_state;
static unsigned int clear_again_times;
static int Voltage = 24, Buzzer_count, charger_check;
static float Move_X, Move_Y, Move_Z;
static float Recharge_UP_Move_X, Recharge_UP_Move_Z;
static float Recharge_Red_Move_X, Recharge_Red_Move_Y, Recharge_Red_Move_Z;
static float Velocity_KP = 200, Velocity_KI = 10;
#define CONTROL_DELAY 1000
#define RATE_100_HZ 100
#define F2T(value) (value)
static unsigned int frame, frames, scenario, checks, drive_calls, pwm_calls;
static u8 upper_busy;
static float sent_z;
static jmp_buf done;
#define CHECK(expr) do { ++checks; if(!(expr)) { printf("FAIL scenario %u frame %u line %u: %s\n", scenario, frame, __LINE__, #expr); exit(1); } } while(0)
void Balance_task(void *unused);
int Incremental_PI_A(float encoder, float target);
int Incremental_PI_B(float encoder, float target);
int Incremental_PI_C(float encoder, float target);
int Incremental_PI_D(float encoder, float target);
static u32 getSysTickCnt(void) { return 0; }
static void Get_Velocity_Form_Encoder(void) {}
static void Key(void) {}
static void Find_Charging_HardWare(void) {}
static u8 StepperTask_IsBusy(void) { return upper_busy; }
static void Get_RC(void) {}
static void Remote_Control(void) {}
static void PS2_control(void) {}
static u8 FlashParam_Save(u8 *flag) { return 0; }
static u8 Turn_Off(int voltage) { return voltage < 10 || !EN || Flag_Stop; }
static void Limit_Pwm(int limit) {}
static void auto_pwm_clear(void) {}
static void Set_Pwm(int a, int b, int c, int d) {
    ++pwm_calls;
    if(scenario == 4) CHECK(a == 0 && b == 0 && c == 0 && d == 0);
}
static void Drive_Motor(float x, float y, float z) {
    ++drive_calls;
    sent_z = z;
    MOTOR_A.Target = x + z; MOTOR_B.Target = x - z;
    MOTOR_C.Target = y + z; MOTOR_D.Target = y - z;
}
static void verify_frame(void) {
    CHECK(pwm_calls == 1);
    CHECK(drive_calls == 1); /* Actuator disturbance never blocks chassis commands. */
    if(scenario == 1) {
        /* Correct while driving, park through 50s of vibration, then resume. */
        if(frame > 1 && frame <= 100) CHECK(sent_z < 0);
        else CHECK(sent_z == 0);
        if(frame >= 101 && frame <= 5100)
            CHECK(MOTOR_A.Target == 0 && MOTOR_B.Target == 0);
        if(frame > 5100) CHECK(MOTOR_A.Target == 0.2f && MOTOR_B.Target == 0.2f);
    } else {
        /* Normal parked heading hold becomes active after the 10s startup. */
        if(scenario == 0 && frame <= CONTROL_DELAY) CHECK(sent_z == 0);
        if(scenario == 0 && frame > CONTROL_DELAY) CHECK(sent_z < 0);
        if(scenario == 2) CHECK(sent_z == 0.3f);
        if(scenario == 3 && frame == 1) CHECK(sent_z == 0);
        if(scenario == 3 && frame > 1) CHECK(sent_z < 0);
        if(scenario == 4) CHECK(sent_z == 0);
        if(scenario == 5 && frame == 1) CHECK(sent_z == 0);
        if(scenario == 5 && frame > 1) CHECK(sent_z < 0);
    }
}
static void vTaskDelayUntil(u32 *last, u32 interval) {
    if(frame) verify_frame();
    if(++frame > frames) longjmp(done, 1);
    drive_calls = pwm_calls = 0;
    upper_busy = scenario == 5 || (scenario == 1 && frame >= 101 && frame <= 5100);
    Move_X = (scenario == 3 || scenario == 4 || scenario == 5 ||
              (scenario == 1 && (frame <= 100 || frame > 5100))) ? 0.2f : 0;
    Move_Y = 0;
    Move_Z = scenario == 2 ? 0.3f : 0;
    imu.gyro.z = scenario == 1 && frame > 5100 ? 0 : 1000;
    EN = scenario != 4;
}
int main(void) {
    Car_Mode = Mec_Car;
    for(scenario = 0; scenario < 6; ++scenario) {
        SysVal.Time_count = scenario == 0 ? 0 : CONTROL_DELAY;
        HeadingControl_Reset();
        frame = 0;
        frames = scenario == 1 ? 5104 : (scenario == 0 ? 1100 : 1000);
        if(!setjmp(done)) Balance_task(NULL);
    }
    printf("PASS: 6 chassis scenarios, %u checks\n", checks);
    return 0;
}
