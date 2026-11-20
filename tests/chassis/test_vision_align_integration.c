#ifndef VISION_PICK_ENABLE
#define VISION_PICK_ENABLE 0
#endif
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "heading_control.c"
#include "vision_align.h"
#include "vision_strike.h"
#include "route_io.h"
typedef struct { float yaw_rad; uint32_t tick_ms; u8 valid; float x_mm,y_mm; } ChassisOdomFrame;
#define ROUTE_ODOM_MAX_GAP_MS 100U
#define CONTROL_DELAY 1000U
#define MANUAL_RC_MAX_MMPS 480.0f
#define MANUAL_RC_TARGET_MMPS 330.0f
static float target_limit_float(float v,float lo,float hi){return v<lo?lo:v>hi?hi:v;}
enum { Mec_Car=1, Mec_Car_V550=2 };
static int EN=1,Flag_Stop,Check,Car_Mode=Mec_Car,RC_Velocity=500;
static int tool_busy,servo_busy,debug_busy,route_owned,reserved;
static unsigned strikes,submitted_id;
enum { STEPPER_RETURNED=6, STEPPER_FAULT=7, MANGO_OK=0 };
typedef struct { unsigned state,result,mango_id; } StepperTask_Data;
static StepperTask_Data tool;
static void StepperTask_GetData(StepperTask_Data *data) { *data=tool; }
static float Voltage=12,Move_X,Move_Y,Move_Z,dx,dy,dz;
static struct { float VX,VY,VZ; } smooth_control;
static struct { unsigned Time_count; } SysVal={2000};
static struct { struct { short z; } gyro; } imu;
static uint32_t now=10000;
static ChassisOdomFrame pose={0,10000,1};
#if !VISION_PICK_ENABLE
static RouteLidarFrame manual_lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *data) { *data=manual_lidar; }
#endif
static Camera_Receiver camera;
static uint8_t camera_green=1;
static struct { float Target,Encoder; int Motor_Pwm; }
    MOTOR_A={.125f,.0625f,1000},MOTOR_B={-.125f,-.0625f,-1000},
    MOTOR_C={.25f,.125f,2000},MOTOR_D={-.25f,-.125f,-2000};
static u8 manual_heading_locked;
static u8 manual_heading_pole;
static u8 vision_park_braking;
static float wheel_output_probe_assist_pwm;
static float manual_heading_target;
static VisionAlign_State vision_align;
static uint32_t vision_manual_frames, vision_manual_seen;
static VisionStrike_State vision_pick;
static VisionPickPhase vision_pick_phase;
static uint8_t vision_pick_id;
static uint8_t vision_pick_attempts;
static uint8_t vision_boot_attempts;
static uint8_t vision_boot_alignments;
static uint8_t vision_detection_timed_out;
static uint8_t vision_rearm_clear_active;
static uint32_t vision_rearm_clear_since;
static uint8_t vision_green_lost_active;
static uint32_t vision_green_lost_since;
static uint32_t vision_green_window_since, vision_green_window_frames, vision_green_window_good;
static VisionPickRange_State vision_pick_range;
static uint8_t vision_target_region;
static uint8_t vision_resume_pending;
static uint32_t vision_fruit_seen;
static uint8_t vision_hit_region;
static uint32_t vision_hit_count;
static uint32_t vision_lock_count;
static uint32_t vision_pick_phase_started;
volatile uint8_t vision_hit_report;
static int chassis_stopped=1;
static uint8_t vision_align_ready;
static uint16_t vision_align_blocked;
static uint8_t vision_stop_reason, vision_release_reason;
static int PS2_ON_Flag,Remote_ON_Flag,APP_ON_Flag;
static int CAN_ON_Flag,Usart1_ON_Flag,Usart5_ON_Flag,Allow_Recharge;
static uint32_t __get_PRIMASK(void) { return 0; }
#define __disable_irq() ((void)0)
static void __set_PRIMASK(uint32_t mask) { (void)mask; }
static int get_key(uint8_t id) { (void)id; return 0; }
static struct { int LY,RX; int (*getKeyState)(uint8_t); } gamepad={127,127,get_key},*GamePadInterface=&gamepad;
static void ChassisOdom_Read(ChassisOdomFrame *p) { *p=pose; }
static uint32_t xTaskGetTickCount(void) { return now; }
void Camera_GetData(Camera_Data *data) { *data=camera.data; }
uint8_t RouteIO_ChassisStopped(void) { return chassis_stopped; }
uint8_t RouteIO_ControlOwned(void) { return route_owned; }
/* Route-specific fixtures provide the real command application below. */
#ifndef NAVIGATION_MERGE_TEST
float RouteIO_ApplyDrive(void) { dx=dy=dz=0; return 0; }
#endif
static uint8_t route_stop_requested;
void RouteIO_RequestManual(void) { route_owned=0; route_stop_requested=1; }
static void Get_RC(void) {}
static void Remote_Control(void) {}
static int StepperTask_IsBusy(void) { return tool_busy; }
static int ServoTask_IsBusy(void) { return servo_busy; }
static int USART_ServoControlBusy(void) { return debug_busy; }
uint8_t RouteIO_WorkReserved(void) { return reserved; }
static int StepperTask_RequestMango(uint8_t id) { submitted_id=id; ++strikes;tool_busy=1; return 1; }
#ifndef NAVIGATION_MERGE_TEST
uint8_t RouteIO_QueueWorkFromISR(uint8_t id) { return StepperTask_RequestMango(id); }
#endif
uint8_t RouteIO_Ready(void) { return EN && !Flag_Stop && Voltage>=10 && !tool_busy; }
static void Drive_Motor(float x,float y,float z) { dx=x;dy=y;dz=z; }
#include "production_logic.inc"
static unsigned checks;
#define REQUIRE(c) do { ++checks; if (!(c)) { printf("FAIL line %u: %s\n", __LINE__, #c); return 1; } } while (0)
static void poll(void) { pose.tick_ms=now;production_cycle();VisionStrike_Process(); }
static void frame(int pole, int offset)
{
    unsigned i;
    uint8_t data[6]={0x63,0,0,0,0,0x66};
    data[1]=(uint8_t)pole;data[3]=camera_green;
    data[2]=(uint8_t)offset;data[4]=pole && camera_green ? 2 : 0;
    for(i=0;i<6;++i) Camera_Push(&camera,data[i]);
    now+=50;poll();
}
/* Explicitly model stable absence when a test starts a separate fruit episode. */
static void clear_fruit(void)
{
    unsigned k;
    camera_green=0;
    for(k=0;k<2U+(VISION_PICK_GREEN_GAP_MS+VISION_PICK_REARM_CLEAR_MS)/50U;++k) frame(0,0);
}
static void release_done(void) { unsigned k; imu.gyro.z=0; for(k=0;k<20;++k) frame(1,0); }
static void restart(void)
{
    EN=0;poll();EN=1;poll();
}
int main(void)
{
    unsigned i;
    VisionAlign_Diagnostics diag;
    REQUIRE(VISION_STRIKE_ENABLE==0);
    /* A trunk alone must leave a held manual command in control. */
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;camera_green=0;
    for(i=0;i<5;++i) frame(1,60);
    VisionAlign_GetDiagnostics(&diag);
    REQUIRE(dx>.019f && dy==0 && VisionAlign_GetPhase()==VISION_ALIGN_WAIT && !diag.locked);
    camera_green=1;frame(1,60);frame(1,60);
    REQUIRE(dx>.019f && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    camera_green=0;frame(1,60);
    camera_green=1;frame(1,60);frame(1,60);
    REQUIRE(dx>.019f && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    frame(1,60);REQUIRE(dx==0 && dy>0 && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    camera_green=0;frame(1,0);
    REQUIRE(dx>.019f && dy==0 && dz==0 && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    camera_green=1;frame(1,60);frame(1,60);frame(1,60);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0);
    camera_green=0;frame(1,0);VisionAlign_GetDiagnostics(&diag);
    REQUIRE(dx>.019f && dy==0 && VisionAlign_GetPhase()==VISION_ALIGN_WAIT && !diag.locked);
    VisionAlign_ReleaseControl();camera_green=1;PS2_ON_Flag=0;gamepad.LY=127;
    GamePadInterface=NULL;
    SysVal.Time_count=0;
    frame(1,60);frame(1,60);frame(1,60);
    VisionAlign_GetDiagnostics(&diag);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_IDLE && !diag.ready && diag.blocked==32 && dx==0 && dy==0);
    SysVal.Time_count=CONTROL_DELAY;poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_WAIT && dx==0 && dy==0); /* Cached startup B cannot start motion. */
    frame(0,0);
    REQUIRE(dx==0 && dy==0 && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    VisionAlign_GetDiagnostics(&diag); REQUIRE(diag.ready && !diag.blocked && diag.control_flags==16);
    /* Standalone WAIT must keep its yaw reference without a camera or gamepad. */
    pose.yaw_rad=.1f;
    for(i=0;i<5;++i) poll();
    VisionAlign_GetDiagnostics(&diag);
    REQUIRE(dx==0 && dy==0 && dz<0 && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    REQUIRE(diag.heading_locked && diag.yaw_target_deg==0 && diag.yaw_error_deg < -5);
    pose.yaw_rad=0;restart();
    frame(1,60); frame(1,60);
    REQUIRE(dx==0 && dy==0);
    frame(1,60); REQUIRE(dx==0 && dy==.24f && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;gamepad.RX=0;poll();
    REQUIRE(dx==0 && dy==.24f && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    VisionAlign_GetDiagnostics(&diag);
    REQUIRE(diag.locked && diag.ly==254 && diag.rx==0 && diag.stick_active && diag.ready && diag.blocked==0);
    REQUIRE(diag.control_flags==81 && diag.vx_mps==0 && diag.vy_mps==.24f);
    REQUIRE(diag.wheel_target[0]==.125f && diag.wheel_feedback[1]==-.0625f && diag.wheel_pwm[2]==2000 && diag.wheel_pwm[3]==-2000);
    imu.gyro.z=751;poll(); REQUIRE(dy>0 && dz<0); /* IMU damping remains active while translating. */
    GamePadInterface=NULL;PS2_ON_Flag=0;pose.yaw_rad=.1f;poll();
    REQUIRE(dy>0 && dz<0 && manual_heading_target==0);
    VisionAlign_GetDiagnostics(&diag);
    REQUIRE(diag.heading_locked && diag.imu_fresh && diag.yaw_target_deg==0);
    REQUIRE(fabsf(diag.yaw_deg-5.729578f)<.001f && fabsf(diag.yaw_error_deg+5.729578f)<.001f);
    REQUIRE(fabsf(diag.gyro_z_dps-751.0f/65.5f)<.001f);
    REQUIRE(diag.heading_i_radps==HeadingControl_GetIntegralOutput());
    {
        float saved_target=manual_heading_target;
        manual_heading_target=3.13f;pose.yaw_rad=-3.13f;
        VisionAlign_GetDiagnostics(&diag);
        REQUIRE(diag.yaw_error_deg<0 && diag.yaw_error_deg>-2);
        pose.valid=0;VisionAlign_GetDiagnostics(&diag);REQUIRE(!diag.imu_fresh);
        pose.valid=1;pose.tick_ms=now-101;VisionAlign_GetDiagnostics(&diag);REQUIRE(!diag.imu_fresh);
        pose.tick_ms=now;pose.yaw_rad=.1f;manual_heading_target=saved_target;
    } /* Standalone cycle preserves yaw reference. */
    frame(1,-20); REQUIRE(dx==0 && dy<0);
    pose.yaw_rad=manual_heading_target; /* Simulate the chassis returning to its locked yaw. */
    smooth_control.VX=.1f;smooth_control.VY=.1f;smooth_control.VZ=.1f;
    frame(1,0); REQUIRE(dx==0 && dy==0 && dz<0);
    REQUIRE(smooth_control.VX==0 && smooth_control.VY==0 && smooth_control.VZ==dz);
    imu.gyro.z=0;
    for(i=0;i<10;++i) poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    frame(1,0); frame(1,0);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0 &&
            dz==HeadingControl_GetIntegralOutput());
    REQUIRE(manual_heading_locked && manual_heading_target==0);release_done();
    gamepad.LY=gamepad.RX=127;
    for(i=0;i<10;++i) frame(1,i%2?1:-1);
    /* After the visual hold, retain the locked yaw and bounded I effort. */
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0 &&
            dz==HeadingControl_GetIntegralOutput() && strikes==0);release_done();
    frame(1,-60);frame(1,-60);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0 &&
            dz==HeadingControl_GetIntegralOutput());
    frame(1,-60);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy<0);
    VisionAlign_GetDiagnostics(&diag); REQUIRE(diag.locked && diag.control_flags==16 && diag.vy_mps<0);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0 &&
            dz==HeadingControl_GetIntegralOutput() && strikes==0);release_done();
    imu.gyro.z=0;GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.RX=0;poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dy>.19f);
    gamepad.RX=127; poll(); frame(1,60);frame(1,60);frame(1,60);
    REQUIRE(dy>0 && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    /* Fully automatic tracking ignores sticks, even with manual speed zero. */
    RC_Velocity=0;gamepad.RX=254;poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy>0);
    RC_Velocity=500;gamepad.RX=127;frame(1,60);frame(1,60);frame(1,60);
    frame(0,0); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy>0);
    now+=250;poll(); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_LOST && dx==0 && dy==0 &&
                             dz==HeadingControl_GetIntegralOutput());
    for(i=0;i<6;++i) frame(1,60);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dy>0);
    restart();frame(1,60);frame(1,60);frame(1,60);
    now+=300;poll(); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_LOST && dx==0 && dy==0 && dz==0);
    restart();frame(1,60);frame(1,60);frame(1,60);
    gamepad.getKeyState=NULL;poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dy>0);
    GamePadInterface=NULL;PS2_ON_Flag=0;poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dy>0);
    gamepad.getKeyState=get_key;
    tool_busy=1;poll(); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_IDLE && dx==0 && dy==0 && dz==0);
    VisionAlign_GetDiagnostics(&diag); REQUIRE(!diag.ready && diag.blocked==512 && !diag.locked);
    tool_busy=0;for(i=0;i<5;++i) frame(1,60);
    REQUIRE(dy>0 && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    now+=101;production_cycle(); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_IDLE && dx==0 && dy==0 && dz==0);
    frame(1,60);frame(1,60);frame(1,60);
    Voltage=9;poll(); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_IDLE && dx==0 && dy==0 && dz==0);
    VisionAlign_GetDiagnostics(&diag); REQUIRE(diag.blocked==16 && !diag.ready);
    REQUIRE(diag.wheel_pwm[0]==0 && diag.wheel_pwm[1]==0 && diag.wheel_pwm[2]==0 && diag.wheel_pwm[3]==0);
    Voltage=12;frame(1,60);frame(1,60);frame(1,60);
    REQUIRE(dy>0 && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    VisionAlign_ReleaseControl(); REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_IDLE);
    frame(1,60); REQUIRE(dx==0 && dy==0 && strikes==0);
    /* Held stick remains manual for two NEW target frames, then auto owns. */
    VisionAlign_ReleaseControl();GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;gamepad.RX=127;
    frame(0,0); REQUIRE(dx>.019f && dy==0);
    frame(1,60); REQUIRE(dx>.019f && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    for(i=0;i<10;++i) poll(); REQUIRE(dx>.019f && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    frame(1,60); REQUIRE(dx>.019f && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
    frame(1,60); REQUIRE(dx==0 && dy==.24f && VisionAlign_GetPhase()==VISION_ALIGN_TRACK);
    frame(1,0);frame(1,0);frame(1,0);
    { uint32_t stopped=now;
      REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0 && dz==0);
      now=stopped+999;poll();VisionAlign_GetDiagnostics(&diag);
      REQUIRE(dx==0 && dy==0 && dz==0 && diag.locked);
      now=stopped+1000;poll();VisionAlign_GetDiagnostics(&diag);
      REQUIRE(dx>.019f && dy==0 && !diag.locked);
    }
#if !VISION_PICK_ENABLE
    PS2_ON_Flag=0;GamePadInterface=NULL;camera_green=1;imu.gyro.z=0;pose.yaw_rad=0;
    manual_lidar.mm[ROUTE_LIDAR_BEHIND]=120;restart();
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx<0 && dy==0);
    manual_lidar.mm[ROUTE_LIDAR_BEHIND]=VISION_ALIGN_REAR_MM;
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_DONE && dx==0 && dy==0 && dz==0);
    manual_lidar.mm[ROUTE_LIDAR_BEHIND]=35;manual_lidar.mm[ROUTE_LIDAR_FRONT]=40;restart();
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy==0);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=1000;poll();REQUIRE(dx>0);
    manual_lidar.fresh=0;poll();
    REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_LOST && dx==0 && dy==0 && dz==0);
#endif
    printf("PASS: standalone no-controller dispatch, DX alignment, IMU and automatic strike disabled (%u checks)\n",checks);
    return 0;
}
