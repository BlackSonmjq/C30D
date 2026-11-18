#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_output.h"
enum { Mec_Car, Mec_Car_V550, Diff_Car, Akm_Car, Tank_Car, Omni_Car };
static int Car_Mode=Mec_Car,start_clear,clear_state,clear_done_once,wheel_output_active;
static float Velocity_KP=300,Velocity_KI=300;
static int vision_park_braking;
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
    WheelOutput_ResetStartup();
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
        const float start=w==0||w==3?8000:5000;
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,1)==start);
        assert(fabsf(WheelOutput_StartupAssist(w,1000,.04f,.02f,1)-start*.64f)<.01f);
        assert(WheelOutput_StartupAssist(w,1000,.04f,.04f,1)==0);
        assert(WheelOutput_StartupAssist(w,-1000,.04f,0,1)==0);
        assert(WheelOutput_StartupAssist(w,1000,0,0,1)==0);
        assert(WheelOutput_StartupAssist(w,1000,.04f,0,0)==0);
        assert(WheelOutput_StartupAssist(w,-1000,-.04f,0,1)==-start);
    }
    for(w=0;w<4;++w) {
        float peak=w==0||w==3?8000:5000;
        WheelOutput_UpdateStartup(w,.04f,.04f,1);
        assert(WheelOutput_StartupAssist(w,0,.04f,0,1)==0);
        for(n=1;n<WHEEL_START_STALL_TICKS;++n) {
            WheelOutput_UpdateStartup(w,.04f,0,1);
            assert(WheelOutput_StartupAssist(w,0,.04f,0,1)==0);
        }
        WheelOutput_UpdateStartup(w,.04f,0,1);
        assert(WheelOutput_StartupAssist(w,0,.04f,0,1)==peak);
        WheelOutput_UpdateStartup(w,.04f,.04f,1);
        for(n=0;n<100;++n) WheelOutput_UpdateStartup(w,.04f,.01f,1);
        assert(WheelOutput_StartupAssist(w,0,.04f,.01f,1)==0);
        /* A brief stop followed by motion must not accumulate a stall. */
        for(n=0;n<19;++n) WheelOutput_UpdateStartup(w,.04f,0,1);
        WheelOutput_UpdateStartup(w,.04f,.01f,1);
        for(n=0;n<19;++n) WheelOutput_UpdateStartup(w,.04f,0,1);
        assert(WheelOutput_StartupAssist(w,0,.04f,0,1)==0);
        WheelOutput_UpdateStartup(w,0,0,1);
        WheelOutput_UpdateStartup(w,.04f,0,1);
        assert(WheelOutput_StartupAssist(w,0,.04f,0,1)==peak);
        WheelOutput_UpdateStartup(w,-.04f,0,1);
        assert(WheelOutput_StartupAssist(w,0,-.04f,0,1)==-peak);
        WheelOutput_UpdateStartup(w,-.04f,0,0);
        assert(!wheel_start_complete[w] && !wheel_start_direction[w]);
    }
    WheelOutput_ResetStartup();
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
    for(w=0;w<4;++w) assert(abs(motors[w]->Motor_Pwm-(w==0||w==3?11724:6684))<=1);
    production_targets(0,.04f,0);
    for(w=0;w<4;++w) motors[w]->Encoder=0;
    production_pwm();
    for(w=0;w<4;++w) {
        const int expected_start[]={11736,-6672,6696,-11712};
        assert(fabsf(motors[w]->Target)==.04f);
        assert(motors[w]->Motor_Pwm==expected_start[w]);
    }

    /* All control sources use the same signed per-wheel startup rule. */
    production_pwm();
    for(w=0;w<4;++w) {
        float t=motors[w]->Target;
        float peak=(t>0?1:-1)*(w==0||w==3?8000:5000);
        assert(WheelOutput_StartupAssist(w,0,t,0,1)==peak);
        assert(WheelOutput_StartupAssist(w,0,t,t*.25f,1)==peak);
        assert(fabsf(WheelOutput_StartupAssist(w,0,t,t*.5f,1)-peak*.64f)<.01f);
        assert(WheelOutput_StartupAssist(w,0,t,-t,1)==peak);
        /* The first smoothed 0.01 m/s target must get full breakaway PWM. */
        assert(WheelOutput_StartupAssist(w,0,t*.25f,0,1)==peak);
        assert(WheelOutput_StartupAssist(w,0,t*.25f,t*.25f,1)==0);
        assert(WheelOutput_StartupAssist(w,0,t*.125f,0,1)==0);
        assert(WheelOutput_StartupAssist(w,t>0?-1000:1000,t,0,1)==0);
        /* PI headroom includes the full new static compensation. */
        {
            float target=t>0?.01f:-.01f;
            float pi=WheelOutput_ClampPi(w,target>0?50000:-50000,target,0,1);
            int output=WheelOutput_Apply(w,(int)pi,target,0,1);
            assert(abs(output)<=WHEEL_OUTPUT_PWM_LIMIT);
            assert(abs(output)>=WHEEL_OUTPUT_PWM_LIMIT-2);
        }
        assert(WheelOutput_StartupAssist(w,0,t,t,1)==0);
    }
    /* Low pure-yaw targets must not apply a full 8000/5000 PWM kick. */
    for(w=0;w<4;++w) {
        float full=w==0||w==3?8000.0f:5000.0f;
        WheelOutput_UpdateStartup(w,0,0,1);
        WheelOutput_UpdateStartup(w,.01f,0,1);
        wheel_output_translation_active=0;
        assert(fabsf(WheelOutput_StartupAssist(w,0,.01f,0,1)-full*.0625f)<.01f);
        assert(fabsf(WheelOutput_StartupAssist(w,0,-.01f,0,1)+full*.0625f)<.01f);
        wheel_output_translation_active=1;
        assert(WheelOutput_StartupAssist(w,0,.01f,0,1)==full);
    }
    production_targets(0,0,0);
    production_pwm();
    for(w=0;w<4;++w) assert(WheelOutput_StartupAssist(w,0,motors[w]->Target,0,1)==0);
    production_targets(0,.04f,0);
    production_pwm();
    assert(WheelOutput_StartupAssist(0,0,.04f,0,1)==8000);

    /* Exercise start assist through real targets for all ten motion patterns,
     * including pure IMU yaw correction and zero-wheel diagonal targets. */
    for(m=0;m<sizeof(commands)/sizeof(commands[0]);++m) {
        production_targets(commands[m][0],commands[m][1],commands[m][2]);
        for(w=0;w<4;++w) {
            float t=motors[w]->Target;
            float peak=t==0 ? 0 : (t>0?1:-1)*(w==0||w==3?8000:5000);
            assert(WheelOutput_StartupAssist(w,0,t,0,1)==peak);
            motors[w]->Encoder=0;
        }
        production_pwm();
        for(w=0;w<4;++w) assert(abs(motors[w]->Motor_Pwm)<=16700);
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
    /* Replay the log failure: old opposite PI at zero speed must no longer
     * suppress a new small yaw target. Verify both signs on all four real PIs. */
    for(w=0;w<4;++w) {
        int positive,negative;
        WheelOutput_ResetStartup();
        for(n=0;n<300;++n) pi[w](0,-.5f);
        WheelOutput_UpdateStartup(w,.0117f,0,1);
        positive=pi[w](0,.0117f);
        assert(positive>=0);
        assert(WheelOutput_Apply(w,positive,.0117f,0,1)>0);
        for(n=0;n<300;++n) pi[w](0,.5f);
        WheelOutput_UpdateStartup(w,-.0117f,0,1);
        negative=pi[w](0,-.0117f);
        assert(negative<=0);
        assert(WheelOutput_Apply(w,negative,-.0117f,0,1)<0);
        /* Genuine overspeed braking and zero-target damping remain available. */
        assert(WheelOutput_ClampPi(w,-1000,.04f,.10f,1)==-1000);
        assert(WheelOutput_ClampPi(w,1000,-.04f,-.10f,1)==1000);
        assert(WheelOutput_ClampPi(w,-1000,0,0,1)==-1000);
        assert(WheelOutput_ClampPi(w,-1000,.04f,0,0)==-1000);
    }
    /* Run a completed small yaw start, then the log's four-wheel standstill
     * through the real production dispatch. Same-direction stalls must rearm. */
    production_targets(0,0,.0234f);
    for(w=0;w<4;++w) motors[w]->Encoder=motors[w]->Target;
    production_pwm();
    for(w=0;w<4;++w) assert(wheel_start_complete[w]);
    for(w=0;w<4;++w) motors[w]->Encoder=0;
    for(n=1;n<WHEEL_START_STALL_TICKS;++n) production_pwm();
    for(w=0;w<4;++w) assert(wheel_start_complete[w]);
    production_pwm();
    for(w=0;w<4;++w) {
        assert(!wheel_start_complete[w]);
        assert(motors[w]->Motor_Pwm*motors[w]->Target>0);
        assert(abs(motors[w]->Motor_Pwm)>=(w==0||w==3?600:350));
        assert(abs(motors[w]->Motor_Pwm)<(w==0||w==3?8000:5000));
        assert(abs(motors[w]->Motor_Pwm)<=16700);
    }

    /* Stop translation while keeping yaw hold: clear old drive PI only once. */
    production_targets(.2f,0,0);
    for(w=0;w<4;++w) motors[w]->Encoder=0;
    for(n=0;n<300;++n) production_pwm();
    production_targets(0,0,.08f);
    assert(wheel_output_reset_pi);
    for(w=0;w<4;++w) motors[w]->Encoder=motors[w]->Target;
    production_pwm();
    assert(!wheel_output_reset_pi);
    for(w=0;w<4;++w)
        assert(motors[w]->Motor_Pwm==(int)WheelOutput_Feedforward(w,motors[w]->Target,1));

    /* A tiny residual heading command must not bypass manual parking damping. */
    production_targets(0,0,.0001f);
    vision_park_braking=0;
    for(w=0;w<4;++w) motors[w]->Encoder=-.08f;
    production_pwm();
    assert(wheel_output_stopping);
    for(w=0;w<4;++w) {
        assert(motors[w]->Motor_Pwm>=(w==0||w==3?1200:600));
        assert(abs(motors[w]->Motor_Pwm)<=16700);
    }
    for(w=0;w<4;++w) motors[w]->Encoder=0;
    production_pwm();
    for(w=0;w<4;++w) assert(motors[w]->Motor_Pwm==0);

    /* After saturation, parking must brake a moving wheel and remove all
     * drive at standstill, including on the first ordinary zero command. */
    production_targets(0,0,0);
    vision_park_braking=1;
    for(m=0;m<2;++m) {
        for(w=0;w<4;++w) motors[w]->Encoder=m ? -.1f : .1f;
        production_pwm();
        for(w=0;w<4;++w) {
            int braking=(w==0 || w==3) ? 1230 : 630;
            assert(motors[w]->Motor_Pwm==(m ? braking : -braking));
        }
    }
    assert(WheelOutput_StopAssist(0,.005f,1)==0);
    assert(fabsf(WheelOutput_StopAssist(3,.025f,1)+400)<.01f);
    assert(WheelOutput_StopAssist(3,.1f,0)==0);
    for(w=0;w<4;++w) motors[w]->Encoder=0;
    production_pwm();
    for(w=0;w<4;++w) assert(motors[w]->Motor_Pwm==0);
    vision_park_braking=0;
    production_pwm();
    for(w=0;w<4;++w) assert(motors[w]->Motor_Pwm==0);
    assert(WheelOutput_StopAssist(0,-.1f,1)==1200);
    production_targets(.04f,-.04f,0);
    production_pwm();
    assert(MOTOR_A.Target==0 && WheelOutput_StopAssist(0,.1f,1)==0);
    puts("PASS: 10 motion patterns, repeated stall restart, stale PI removal, unequal loads, braking and anti-windup");
    return 0;
}
