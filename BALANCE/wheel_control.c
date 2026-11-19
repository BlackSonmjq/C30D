#include "wheel_control.h"
#include "nav_config.h"
#include <math.h>

#define WHEEL_ASSIST_MIN_MPS  0.005f
#define WHEEL_ASSIST_FULL_MPS 0.03f
#define WHEEL_START_EXIT_MPS  0.04f
#define WHEEL_STILL_MPS       0.005f
#define WHEEL_STALL_TICKS     20U   /* 200 ms at 100 Hz */

static float clamp(float value,float lower,float upper)
{
    if(value<lower) return lower;
    if(value>upper) return upper;
    return value;
}

static float next_pi(WheelPI *s,float error,float kp,float ki)
{
    float increment=kp*(error-s->last_error)+ki*error;
    s->last_error=error;
    return s->pwm+increment;
}

void WheelPI_Reset(WheelPI *s) { s->pwm=0; s->last_error=0; }

float WheelPI_Update(WheelPI *s,float actual,float target,float kp,float ki,float maximum)
{
    s->pwm=clamp(next_pi(s,target-actual,kp,ki),-maximum,maximum);
    return s->pwm;
}

void WheelDrive_Reset(WheelDrive *wheel)
{
    WheelPI_Reset(&wheel->pi);
    wheel->direction=0;
    wheel->started=0;
    wheel->stall_ticks=0;
}

static void update_startup(WheelDrive *wheel,int direction,float feedback,float exit_speed)
{
    if(wheel->direction!=direction) {
        wheel->direction=(int8_t)direction;
        wheel->started=0;
        wheel->stall_ticks=0;
    }
    if(feedback>=exit_speed) {
        wheel->started=1;
        wheel->stall_ticks=0;
    } else if(wheel->started && fabsf(feedback)<=WHEEL_STILL_MPS) {
        if(++wheel->stall_ticks>=WHEEL_STALL_TICKS) {
            wheel->started=0;
            wheel->stall_ticks=0;
        }
    } else {
        wheel->stall_ticks=0;
    }
}

float WheelDrive_Update(WheelDrive *wheel,float actual,float target,float kp,float ki,float start_pwm,int translating)
{
    float speed=fabsf(target);
    float feedback,exit_speed,pwm,bias=0.0f;
    int direction;

    if(speed<=WHEEL_ASSIST_MIN_MPS) {
        wheel->direction=0;
        return WheelPI_Update(&wheel->pi,actual,target,kp,ki,NAV_PWM_LIMIT);
    }

    direction=target>0.0f?1:-1;
    feedback=actual*direction;
    exit_speed=speed<WHEEL_START_EXIT_MPS?speed:WHEEL_START_EXIT_MPS;
    update_startup(wheel,direction,feedback,exit_speed);

    pwm=next_pi(&wheel->pi,target-actual,kp,ki);
    /* Discard old opposite effort at rest. Preserve braking on a moving wheel. */
    if(fabsf(actual)<=WHEEL_STILL_MPS && pwm*direction<0.0f) pwm=0.0f;
    if(pwm*direction>=0.0f) {
        float scale=clamp((speed-WHEEL_ASSIST_MIN_MPS)/
                          (WHEEL_ASSIST_FULL_MPS-WHEEL_ASSIST_MIN_MPS),0.0f,1.0f);
        float overspeed=clamp((feedback-speed-WHEEL_STILL_MPS)/WHEEL_ASSIST_FULL_MPS,0.0f,1.0f);
        float deficit=clamp((speed-feedback)/WHEEL_ASSIST_FULL_MPS,0.0f,1.0f);
        float feedforward=(NAV_WHEEL_KS_PWM+NAV_WHEEL_KV_PWM*speed)*(1.0f-overspeed);
        float startup=0.0f;
        if(!wheel->started) {
            /* Keep full assist through the first 37.5% of the exit speed. */
            float fade=clamp((exit_speed-feedback)/(0.625f*exit_speed),0.0f,1.0f);
            startup=start_pwm*fade*fade;
            if(!translating && speed<WHEEL_START_EXIT_MPS) {
                float turn_scale=speed/WHEEL_START_EXIT_MPS;
                startup*=turn_scale*turn_scale;
            }
        }
        bias=direction*scale*(feedforward+NAV_WHEEL_LOAD_PWM*deficit+startup);
    }

    /* Reserve headroom in the stored PI output before adding compensation. */
    wheel->pi.pwm=clamp(pwm,-NAV_PWM_LIMIT-bias,NAV_PWM_LIMIT-bias);
    return clamp(wheel->pi.pwm+bias,-NAV_PWM_LIMIT,NAV_PWM_LIMIT);
}

float WheelRamp(float current,float target,float delta)
{
    float e=target-current;
    if(e>delta) return current+delta;
    if(e< -delta) return current-delta;
    return target;
}
