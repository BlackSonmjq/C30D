#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef struct { u32 period; } TIM_TypeDef;
static TIM_TypeDef timers[2];
#define TIM12 (&timers[0])
#define ENABLE 1
static u32 stepper_completed[2],stepper_count[2],stepper_ramp[2];
static u32 stepper_period[2],stepper_target_period_us[2];
static u8 stepper_continuous[2];
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)) { printf("FAIL line %u: %s\n",__LINE__,#x);exit(1); } } while(0)
static TIM_TypeDef *StepperMotor_Timer(u8 motor) { return &timers[motor-1U]; }
void TIM_SetAutoreload(TIM_TypeDef *t,u32 value) { t->period=value+1U; }
void TIM_SetCompare2(TIM_TypeDef *t,u32 v) { CHECK(v==t->period/2U); }
void TIM_SetCompare3(TIM_TypeDef *t,u32 v) { CHECK(v==t->period/2U); }
void TIM_SetCounter(TIM_TypeDef *t,u32 v) { (void)t;CHECK(v==0); }
void TIM_Cmd(TIM_TypeDef *t,unsigned v) { (void)t;CHECK(v==1); }
#include ".build/stepper_ramp.inc"
int main(void)
{
    unsigned motor,n;
    for(motor=1;motor<=2;++motor) {
        u32 total=0,accel=0,previous=STEPPER_START_PERIOD_US;
        unsigned index=motor-1U;
        stepper_count[index]=26000;stepper_ramp[index]=200;
        stepper_target_period_us[index]=89;stepper_period[index]=previous;
        for(n=0;n<26000;++n) {
            u32 period;
            stepper_completed[index]=n;StepperMotor_StartNext((u8)motor);
            period=timers[index].period;
            CHECK(period>=89U && period<=5000U);
            CHECK(period<=previous+150U && previous<=period+150U);
            if(n<200U) { CHECK(period<=previous);accel+=period; }
            if(n>=25800U) CHECK(period>=previous);
            previous=period;total+=period;
        }
        CHECK(accel<=600000U && total<3400000U);
        printf("motor %u: accel=%lu us, 26000 pulses=%lu us\n",motor,(unsigned long)accel,(unsigned long)total);
        stepper_target_period_us[index]=10000;stepper_period[index]=10000;
        stepper_completed[index]=0;StepperMotor_StartNext((u8)motor);
        CHECK(timers[index].period==10000U);
    }
    printf("PASS: bounded pulse ramp, monotonic acceleration/deceleration and slow-speed requests (%u checks)\n",checks);
    return 0;
}
