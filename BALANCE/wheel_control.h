#ifndef MANGO_WHEEL_CONTROL_H
#define MANGO_WHEEL_CONTROL_H
#include <stdint.h>
typedef struct { float pwm, last_error; } WheelPI;
typedef struct {
    WheelPI pi;
    int8_t direction;
    uint8_t started, stall_ticks;
} WheelDrive;

void WheelPI_Reset(WheelPI *s);
float WheelPI_Update(WheelPI *s,float actual,float target,float kp,float ki,float pwm_limit);
void WheelDrive_Reset(WheelDrive *wheel);
float WheelDrive_Update(WheelDrive *wheel,float actual,float target,float kp,float ki,float start_pwm,int translating);
float WheelRamp(float current,float target,float max_delta);
#endif
