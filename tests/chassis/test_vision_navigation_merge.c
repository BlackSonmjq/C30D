#define VISION_PICK_ENABLE 1
#define NAVIGATION_MERGE_TEST 1
#define main legacy_alignment_main
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#include "align_fixture.inc"
#undef main
#include "nav_core.c"
#include "nav_button.c"
#include "nav_can.c"
#include "nav_calibration.c"

typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
#define ROUTE_COMMAND_TTL_MS 120U
typedef struct { float vx_mps,vy_mps,probe_assist_pwm; TickType_t tick; uint8_t quick_stop; } RouteCommand;
static RouteCommand command;
static volatile NavLidar lidar;
static uint8_t start_pending,command_seen,work_allowed;
static float heading_target;
#define start_requested route_owned
#define stop_pending route_stop_requested
#define clock_ms now
#define work_reserved reserved
#include "navigation_io.inc"
#undef start_requested
#undef stop_pending
#undef clock_ms
#undef work_reserved

uint8_t RouteIO_BaseReady(void) { return EN && !Flag_Stop && Voltage>=10 && SysVal.Time_count>=CONTROL_DELAY; }
uint8_t RouteIO_TakeWorkRequest(void) { return 0; }
void RouteIO_CompleteWork(void) { reserved=0; }
void RouteIO_AllowWork(uint8_t allowed) { (void)allowed; }
static void publish_status(void) {}
static NavCore navigator;
static ChassisOdomFrame previous_pose;
static uint32_t previous_ms;
static uint16_t navigation_rear_mm=45;
static uint8_t work_id,work_started,demo_id,demo_started,demo_cancelled,demo_used;
static const float half_cross_mm=1300.0f;
enum { SERVO_TASK_IDLE, SERVO_TASK_RUNNING, SERVO_TASK_DONE };
typedef struct { uint8_t id,state; } ServoTask_Data;
static ServoTask_Data servo;
static unsigned demo_starts;
#define USART3 3
static void any_printf(int port,const char *format,...) { (void)port; (void)format; }
static void ServoTask_GetData(ServoTask_Data *data) { *data=servo; }
static uint8_t ServoTask_Start(uint8_t id)
{
    servo.id=id; servo.state=SERVO_TASK_RUNNING; servo_busy=1; ++demo_starts;
    return 1;
}
static void ServoTask_RequestCancel(void)
{
    servo.state=SERVO_TASK_IDLE; servo_busy=0;
}
static NavButton button;
#define KEY 1
static void navigation_step(void)
{
    NavInput in;
    NavLidar lidar;
    ChassisOdomFrame pose;
    StepperTask_Data tool;
#include "navigation_control.inc"
}
static void next_navigation_frame(void)
{
    pose.tick_ms=now;
    RouteIO_PublishLidar(5000,navigation_rear_mm,100,2700);
    navigation_step();
}
static void observed_frame(int pole, int offset)
{
    next_navigation_frame();
    frame(pole,offset);
}
int main(void)
{
    unsigned i;
    uint8_t target;
    RouteLidarFrame ranges;
    uint8_t bytes[8]={0,100,19,136,10,140,0,45}; /* L100 F5000 R2700 B45 */
    GamePadInterface=&gamepad; PS2_ON_Flag=1;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    RouteIO_ReceiveCan(0x301,0,0,7,bytes);
    RouteIO_ReadLidar(&ranges); REQUIRE(!ranges.fresh);
    RouteIO_ReceiveCan(0x301,0,0,8,bytes);
    RouteIO_ReadLidar(&ranges);
    REQUIRE(ranges.fresh && ranges.mm[ROUTE_LIDAR_FRONT]==5000 && ranges.mm[ROUTE_LIDAR_BEHIND]==45);
    now+=299; RouteIO_ReadLidar(&ranges); REQUIRE(ranges.fresh);
    ++now; RouteIO_ReadLidar(&ranges); REQUIRE(!ranges.fresh && !ranges.mm[ROUTE_LIDAR_FRONT]);
    RouteIO_PublishLidar(65535,65535,100,2700);
    RouteIO_ReadLidar(&ranges);
    REQUIRE(ranges.fresh && !ranges.mm[ROUTE_LIDAR_FRONT] && !ranges.mm[ROUTE_LIDAR_BEHIND]);
    Nav_Init(&navigator,&nav_calibration,NAV_CALIBRATED);
    previous_ms=now;
    previous_pose=pose;
    next_navigation_frame();
    start_pending=1; now+=20; next_navigation_frame(); poll();
    REQUIRE(route_owned && Nav_IsActive(&navigator) && dx>0 && dy==0);
    target=navigator.target;
    start_pending=1; now+=20; next_navigation_frame(); poll();
    REQUIRE(route_owned && navigator.target==target && navigator.state==NAV_RUN);
    /* A recently published command cannot extend the CAN 300 ms deadline. */
    now+=300; pose.tick_ms=now;
    RouteIO_SetVelocity(.28f,0,0,0); poll();
    REQUIRE(dx==0 && dy==0);
    next_navigation_frame(); poll();
    /* REBUILD can locate P03 from the back laser while the optional front beam is lost. */
    {
        NavCore saved = navigator;
        navigator.state=NAV_RUN; navigator.target=3; navigator.confirmed=2;
        navigator.position_axes=0; navigator.lidar.sequence=0;
        memset(navigator.filter,0,sizeof(navigator.filter));
        now+=20; pose.tick_ms=now;
        RouteIO_PublishLidar(65535,577,2700,100);
        navigation_step(); poll();
        REQUIRE(navigator.state==NAV_ALIGN && navigator.reason==NAV_OK);
        REQUIRE(navigator.target==3 && navigator.confirmed==2 && navigator.output.vx>.119f && dx>.119f);
        for(i=0;i<10;++i) {
            now+=20; pose.tick_ms=now;
            RouteIO_PublishLidar(0,577,2700,100);
            navigation_step(); poll();
            REQUIRE(dx>.119f && navigator.target==3 && navigator.confirmed==2);
        }
        for(i=0;i<8;++i) {
            now+=20; pose.tick_ms=now;
            RouteIO_PublishLidar(3783,577,2700,100);
            navigation_step(); poll();
        }
        REQUIRE(navigator.output.vx>0 && dx>0 && navigator.target==3);
        now+=20; pose.tick_ms=now;
        RouteIO_PublishLidar(45,577,2700,100);
        navigation_step(); poll();
        REQUIRE(dx>.119f && navigator.target==3);
        now+=20; pose.tick_ms=now;
        RouteIO_PublishLidar(40,577,2700,100);
        navigation_step(); poll();
        REQUIRE(navigator.state==NAV_WAIT_LIDAR && navigator.reason==NAV_TOO_CLOSE && dx==0);
        navigator=saved;
        smooth_control.VX=.28f; smooth_control.VY=.48f;
        for(i=0;i<110;++i) {
            now+=20; pose.tick_ms=now;
            RouteIO_PublishLidar(5576,577,100,2700);
            navigation_step(); poll();
        }
        REQUIRE(navigator.state==NAV_WAIT_LIDAR && navigator.target==1 && navigator.confirmed==0);
        REQUIRE(smooth_control.VX==0 && smooth_control.VY==0 && command.quick_stop);
        navigator=saved;
        next_navigation_frame(); poll();
    }
    observed_frame(1,20); observed_frame(1,20); observed_frame(1,20);
    REQUIRE(VisionAlign_ControlOwned() && dx==0 && dy>0 && route_owned);
    next_navigation_frame();
    REQUIRE(navigator.pause_latched && navigator.target==target && navigator.output.vx==0);
    /* Navigation's paused zero must not erase the active visual owner's ramp
       every chassis tick; otherwise 0.24m/s stays at the first 0.01m/s step. */
    smooth_control.VX=0; smooth_control.VY=.12f;
    for(i=0;i<5;++i) {
        observed_frame(1,20);
        REQUIRE(command.quick_stop && VisionAlign_ControlOwned());
        REQUIRE(smooth_control.VY>.119f && dy>.19f);
    }
    camera_green=0; observed_frame(0,0);
    REQUIRE(VisionAlign_ControlOwned());
    for(i=0;i<40;++i) { next_navigation_frame(); observed_frame(0,0); }
    REQUIRE(!VisionAlign_ControlOwned());
    for(i=0;i<5;++i) { now+=20; next_navigation_frame(); poll(); }
    REQUIRE(route_owned && navigator.target==target && !navigator.pause_latched && dx>0);
    for(i=0;i<7;++i) { next_navigation_frame(); observed_frame(0,0); }
    camera_green=1; observed_frame(1,20); observed_frame(1,20); observed_frame(1,20);
    next_navigation_frame();
    for(i=0;i<15;++i) { next_navigation_frame(); observed_frame(1,0); }
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && dx==0 && dy==0 && dz==0);
    REQUIRE(navigator.pause_latched && navigator.target==target);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2;
    poll(); camera_green=0; observed_frame(0,0);
    for(i=0;i<5;++i) { now+=20; next_navigation_frame(); poll(); }
    REQUIRE(route_owned && navigator.target==target && !navigator.pause_latched && dx>0);
    gamepad.LY=254; poll();
    REQUIRE(!route_owned && route_stop_requested && dx>.47f);
    next_navigation_frame();
    REQUIRE(navigator.state==NAV_STOPPED && navigator.reason==NAV_USER_STOP);
    for(i=0;i<5;++i) { now+=20; next_navigation_frame(); poll(); }
    REQUIRE(!route_owned && dx>.47f && navigator.target==target);
    /* A work request not yet dispatched must not survive manual cancellation. */
    work_id=3; work_started=0; reserved=1;
    next_navigation_frame();
    REQUIRE(work_id==0 && !reserved && !route_owned);
    /* A long visual pause preserves P17's retreat deadline and remaining travel. */
    gamepad.LY=gamepad.RX=127;
    Nav_Init(&navigator,&nav_calibration,NAV_CALIBRATED);
    navigator.state=NAV_RUN; navigator.target=17; navigator.confirmed=16;
    navigator.exit_phase=EXIT_REVERSE; navigator.exit_remaining_mm=400;
    navigator.exit_started_ms=now;
    navigation_rear_mm=1000; /* Retreat requires rear clearance above 45 mm. */
    vision_align.phase=VISION_ALIGN_TRACK;
    for(i=0;i<600;++i) { now+=20; next_navigation_frame(); }
    REQUIRE(navigator.pause_latched && navigator.state==NAV_PAUSED && navigator.target==17);
    VisionAlign_Cancel(&vision_align);
    for(i=0;i<5;++i) { now+=20; next_navigation_frame(); }
    REQUIRE(!navigator.pause_latched && navigator.state==NAV_RUN && navigator.target==17);
    REQUIRE(navigator.exit_remaining_mm==400 && navigator.reason==NAV_OK);
    navigation_rear_mm=0;
    now+=20; next_navigation_frame();
    REQUIRE(navigator.state==NAV_RUN && navigator.output.vx==-NAV_EXIT_SPEED_MPS);
    /* Fixed midpoint stops: exercise production dispatch and same-point resume. */
    for(i=1;i<=2;++i) {
        uint16_t front, rear;
        Nav_Init(&navigator,&nav_calibration,NAV_CALIBRATED);
        navigator.state=NAV_RUN; navigator.reason=NAV_OK; navigator.target=(uint8_t)(4U*i);
        navigator.confirmed=(uint8_t)(navigator.target-1U);
        navigator.output.vy=0.18f;
        navigator.main_error_mm=-(half_cross_mm-5.0f);
        route_owned=1; route_stop_requested=0;
        now+=20; pose.tick_ms=now;
        front=(uint16_t)nav_calibration.lane[nav_plan[navigator.target-1U].lane].center_front_mm;
        rear=(uint16_t)nav_calibration.lane[nav_plan[navigator.target-1U].lane].center_back_mm;
        RouteIO_PublishLidar(front,rear,1395,1405);
        MOTOR_A.Encoder=0.08f;
        navigation_step();
        REQUIRE(navigator.state==NAV_PAUSING && reserved && !work_allowed && !servo_busy);
        MOTOR_A.Encoder=0;
        now+=20; pose.tick_ms=now;
        navigation_step();
        REQUIRE(navigator.state==NAV_PAUSED && servo.id==i && servo_busy && demo_starts==i);
        if(i==2) {
            route_stop_requested=1;
            navigation_step();
            REQUIRE(navigator.state==NAV_STOPPED && !servo_busy);
            navigation_step();
            REQUIRE(!reserved && demo_id==0);
        } else {
            servo.state=SERVO_TASK_DONE; servo_busy=0;
            for(unsigned j=0;j<5;++j) {
                now+=20; pose.tick_ms=now;
                RouteIO_PublishLidar(front,rear,1395,1405);
                navigation_step();
            }
            REQUIRE(navigator.state==NAV_RUN && !reserved && navigator.target==4);
            REQUIRE(demo_starts==1);
        }
    }
    puts("PASS: P04 servo 1, P08 servo 2, stop before motion, resume once and cancel");
    puts("PASS: actual CAN age, route-to-vision handover, same-target resume, tool pause and stick cancellation");
    return 0;
}
