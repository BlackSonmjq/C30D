#include "wheel_control.h"
void WheelPI_Reset(WheelPI *s) { s->pwm=0; s->last_error=0; }
float WheelPI_Update(WheelPI *s,float actual,float target,float kp,float ki,float maximum)
{
    float e=target-actual;
    s->pwm+=kp*(e-s->last_error)+ki*e;
    if(s->pwm>maximum) s->pwm=maximum;
    if(s->pwm< -maximum) s->pwm= -maximum;
    s->last_error=e;
    return s->pwm;
}
float WheelRamp(float current,float target,float delta)
{
    float e=target-current;
    if(e>delta) return current+delta;
    if(e< -delta) return current-delta;
    return target;
}
