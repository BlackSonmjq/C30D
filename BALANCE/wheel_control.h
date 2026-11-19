#ifndef MANGO_WHEEL_CONTROL_H
#define MANGO_WHEEL_CONTROL_H
typedef struct { float pwm, last_error; } WheelPI;
void WheelPI_Reset(WheelPI *s);
float WheelPI_Update(WheelPI *s,float actual,float target,float kp,float ki,float pwm_limit);
float WheelRamp(float current,float target,float max_delta);
#endif
