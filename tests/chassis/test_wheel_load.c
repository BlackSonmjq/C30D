#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_output.h"
enum { Mec_Car, Mec_Car_V550, Diff_Car, Akm_Car, Tank_Car, Omni_Car };
static int Car_Mode=Mec_Car,start_clear,clear_state,clear_done_once,wheel_output_active;
static float Velocity_KP=300,Velocity_KI=300;
static float Axle_spacing=.2f,Wheel_spacing=.3f;
typedef struct { float Encoder,Target; int Motor_Pwm; } TestMotor;
static TestMotor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
static TestMotor *motors[]={&MOTOR_A,&MOTOR_B,&MOTOR_C,&MOTOR_D};
static float target_limit_float(float x,float low,float high) { return x<low?low:x>high?high:x; }
static void Limit_Pwm(int limit)
{
    unsigned w;
    for(w=0;w<4;++w) motors[w]->Motor_Pwm=(int)target_limit_float((float)motors[w]->Motor_Pwm,(float)-limit,(float)limit);
}
#include "production.inc"
#undef assert
#define assert(c) do { if (!(c)) { printf("FAIL wheel load line %u: %s\n",__LINE__,#c); return 1; } } while (0)

int main(void)
{
    unsigned w,n,m;
    const float commands[][3]={
        {.04f,0,0},{-.04f,0,0},{0,.04f,0},{0,-.04f,0},
        {0,0,.08f},{0,0,-.08f},{.04f,.04f,0},{.04f,-.04f,0},
        {-.04f,.04f,0},{-.04f,-.04f,0}
    };
    const float physical_ks[]={2600,1200,1100,2200};
    const float physical_kv[]={6000,4500,4000,5500};
    int (*pi[])(float,float)={Incremental_PI_A,Incremental_PI_B,Incremental_PI_C,Incremental_PI_D};
    assert(clear_done_once==0);
    for(w=0;w<4;++w) {
        assert(WheelOutput_Feedforward(w,0,1)==0);
        assert(WheelOutput_Feedforward(w,.005f,1)==0);
        assert(WheelOutput_Feedforward(w,.04f,0)==0);
        assert(WheelOutput_Feedforward(w,.04f,1)==(w==0||w==3?2200:1160));
        assert(WheelOutput_Feedforward(w,-.04f,1)==(w==0||w==3?-2200:-1160));
        assert(WheelOutput_Feedforward(w,10,1)==6000);
        assert(WheelOutput_Feedforward(w,-10,1)==-6000);
        assert(WheelOutput_Feedforward(w,.006f,1)>0);
        assert(WheelOutput_Feedforward(w,.006f,1)<WheelOutput_Feedforward(w,.02f,1));
        assert(WheelOutput_Apply(w,-1000,0,.04f,1)==-1000);
        assert(WheelOutput_Apply(w,1000,.04f,0,0)==1000);
        assert(fabsf(WheelOutput_LoadAssist(w,1000,.04f,0,1)-(w==0||w==3?1500:500))<.001f);
        assert(fabsf(WheelOutput_LoadAssist(w,-1000,-.04f,0,1)-(w==0||w==3?-1500:-500))<.001f);
        assert(WheelOutput_LoadAssist(w,1000,.08f,0,1)==(w==0||w==3?1500:500));
        assert(WheelOutput_LoadAssist(w,-1000,.04f,0,1)==0);
        assert(WheelOutput_LoadAssist(w,1000,.04f,.04f,1)==0);
        assert(WheelOutput_LoadAssist(w,1000,.04f,.05f,1)==0);
        assert(WheelOutput_LoadAssist(w,1000,0,0,1)==0);
        /* Light-wheel overspeed must shed driving feedforward immediately,
         * for both signs, without changing underspeed help or PI braking. */
        assert(WheelOutput_TrackingFeedforward(w,.04f,.04f,1)==WheelOutput_Feedforward(w,.04f,1));
        assert(WheelOutput_TrackingFeedforward(w,.04f,0,1)==WheelOutput_Feedforward(w,.04f,1));
        assert(WheelOutput_TrackingFeedforward(w,.04f,-.1f,1)==WheelOutput_Feedforward(w,.04f,1));
        assert(WheelOutput_TrackingFeedforward(w,.04f,.06f,1)>0);
        assert(WheelOutput_TrackingFeedforward(w,.04f,.06f,1)<WheelOutput_Feedforward(w,.04f,1));
        assert(WheelOutput_TrackingFeedforward(w,.04f,.10f,1)==0);
        assert(WheelOutput_TrackingFeedforward(w,-.04f,-.10f,1)==0);
        assert(WheelOutput_Apply(w,-1000,.04f,.10f,1)==-1000);
        assert(WheelOutput_Apply(w,1000,-.04f,-.10f,1)==1000);
        assert(WheelOutput_Apply(w,1000,.04f,.08f,0)==1000);
        assert(WheelOutput_Apply(w,1000,.04f,.08f,1)<WheelOutput_Apply(w,1000,.04f,0,1));
        assert(WheelOutput_Apply(w,-1000,0,.08f,1)==-1000);
        assert(WheelOutput_LoadAssist(w,1000,.04f,.025f,1)<WheelOutput_LoadAssist(w,1000,.04f,0,1));
        WheelOutput_SetMotion(.04f,0);
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,1)==0);
        WheelOutput_SetMotion(0,.04f);
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,1)==(w==0||w==3?2000:1000));
        assert(WheelOutput_StartupAssist(w,1000,.04f,.02f,1)==(w==0||w==3?1000:500));
        assert(WheelOutput_StartupAssist(w,1000,.04f,.04f,1)==0);
        assert(WheelOutput_StartupAssist(w,-1000,.04f,0,1)==0);
        assert(WheelOutput_StartupAssist(w,1000,0,0,1)==0);
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,0)==0);
        WheelOutput_SetMotion(0,-.04f);
        assert(WheelOutput_StartupAssist(w,-1000,-.04f,0,1)==(w==0||w==3?-2000:-1000));
        WheelOutput_SetMotion(.04f,.04f);
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,1)==0);
        WheelOutput_SetMotion(0,0);
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,1)==0);
    }
    assert(WheelOutput_Feedforward(4,.1f,1)==0);
    assert(WheelOutput_LoadAssist(4,1000,.1f,0,1)==0);

    /* Actual body kinematics + actual final output block. An ideal tracking
     * wheel has zero PI error: each wheel must receive exactly its own model,
     * across forward/reverse/strafe/rotation/diagonal commands. */
    for(m=0;m<sizeof(commands)/sizeof(commands[0]);++m) {
        production_targets(commands[m][0],commands[m][1],commands[m][2]);
        for(w=0;w<4;++w) motors[w]->Encoder=motors[w]->Target;
        production_pwm();
        for(w=0;w<4;++w) {
            float speed=fabsf(motors[w]->Target);
            int expected=speed==0?0:(w==0||w==3?2000:1000)+(int)((w==0||w==3?5000:4000)*speed);
            if(motors[w]->Target<0) expected=-expected;
            assert(motors[w]->Motor_Pwm==expected);
        }
    }
    /* Clearing can finish inside PI_D. Final output must retain the disabled
     * compensation decision taken before the four PI calls for this cycle. */
    production_targets(.04f,0,0);
    for(w=0;w<4;++w) motors[w]->Encoder=motors[w]->Target;
    start_clear=1;production_pwm();
    assert(start_clear==0 && clear_done_once==1);
    for(w=0;w<4;++w) assert(motors[w]->Motor_Pwm==0);
    production_targets(.04f,0,0);
    for(w=0;w<4;++w) { assert(motors[w]->Target==.04f); motors[w]->Encoder=0; }
    production_pwm();
    for(w=0;w<4;++w) assert(abs(motors[w]->Motor_Pwm-(w==0||w==3?3724:1684))<=1);
    production_targets(0,.04f,0);
    for(w=0;w<4;++w) motors[w]->Encoder=0;
    production_pwm();
    for(w=0;w<4;++w) {
        const int expected_start[]={5736,-2672,2696,-5712};
        assert(fabsf(motors[w]->Target)==.04f);
        assert(motors[w]->Motor_Pwm==expected_start[w]);
    }

    /* Unequal simulated wheel resistance differs from the feedforward table.
     * Independent real PI loops must still converge toward the shared body
     * command; this validates control behavior, not physical calibration. */
    for(m=0;m<6;++m) {
        for(n=0;n<600;++n) {
            production_targets(commands[m][0]*2,commands[m][1]*2,commands[m][2]*2);
            production_pwm();
            for(w=0;w<4;++w) {
                float output=(float)motors[w]->Motor_Pwm;
                float speed=fabsf(output)>physical_ks[w]?(fabsf(output)-physical_ks[w])/physical_kv[w]:0;
                if(output<0) speed=-speed;
                motors[w]->Encoder+=(speed-motors[w]->Encoder)*.1f;
                assert(abs(motors[w]->Motor_Pwm)<=16700);
            }
        }
        for(w=0;w<4;++w) assert(fabsf(motors[w]->Encoder-motors[w]->Target)<.01f);
    }
    /* Much lighter B/C: their actual friction is below the configured Ks.
     * Run real PI/output for both strafe directions, not only a bias helper. */
    for(m=2;m<4;++m) {
        for(n=0;n<600;++n) {
            production_targets(commands[m][0],commands[m][1],commands[m][2]);
            production_pwm();
            for(w=0;w<4;++w) {
                float output=(float)motors[w]->Motor_Pwm;
                float friction=(w==1||w==2)?200.0f:physical_ks[w];
                float drag=(w==1||w==2)?4000.0f:physical_kv[w];
                float speed=fabsf(output)>friction?(fabsf(output)-friction)/drag:0;
                if(output<0) speed=-speed;
                motors[w]->Encoder+=(speed-motors[w]->Encoder)*.1f;
                assert(abs(motors[w]->Motor_Pwm)<=16700);
                if(n>300 && fabsf(motors[w]->Encoder-motors[w]->Target)>=.01f)
                    printf("Light strafe mode %u tick %u wheel %u: target=%f feedback=%f pwm=%d\n",m,n,w,motors[w]->Target,motors[w]->Encoder,motors[w]->Motor_Pwm);
                if(n>300) assert(fabsf(motors[w]->Encoder-motors[w]->Target)<.01f);
            }
        }
    }
    for(w=0;w<4;++w) {
        int pwm;
        for(n=0;n<2000;++n) pwm=pi[w](0,.5f);
        assert(pwm<=WheelOutput_ClampPi(w,1e6f,.5f,0,1));
        assert(WheelOutput_Apply(w,pwm,.5f,0,1)==16700);
        for(n=0;n<2000;++n) pwm=pi[w](0,-.5f);
        assert(pwm>=WheelOutput_ClampPi(w,-1e6f,-.5f,0,1));
        assert(WheelOutput_Apply(w,pwm,-.5f,0,1)==-16700);
        assert(WheelOutput_Apply(w,30000,.5f,0,1)==16700);
        assert(WheelOutput_Apply(w,-30000,-.5f,0,1)==-16700);
    }
    puts("PASS: global per-wheel output, 10 motion patterns, unequal-load and very-light-wheel strafe simulation, overspeed feedforward fade, braking and full-bias anti-windup");
    return 0;
}
