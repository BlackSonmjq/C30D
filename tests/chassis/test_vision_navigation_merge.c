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
typedef struct { float vx_mps,vy_mps; TickType_t tick; uint8_t quick_stop; } RouteCommand;
static RouteCommand command;
static volatile NavLidar lidar;
static uint8_t start_pending,command_seen;
static float heading_target;
#define start_requested route_owned
#define stop_pending route_stop_requested
#define clock_ms now
#include "navigation_io.inc"
#undef start_requested
#undef stop_pending
#undef clock_ms

uint8_t RouteIO_BaseReady(void) { return EN && !Flag_Stop && Voltage>=10 && SysVal.Time_count>=CONTROL_DELAY; }
uint8_t RouteIO_TakeWorkRequest(void) { return 0; }
void RouteIO_CompleteWork(void) { reserved=0; }
void RouteIO_AllowWork(uint8_t allowed) { (void)allowed; }
static void publish_status(void) {}
static NavCore navigator;
static ChassisOdomFrame previous_pose;
static uint32_t previous_ms;
static uint16_t navigation_rear_mm=45;
static uint8_t work_id,work_started;
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
    RouteIO_SetVelocity(.28f,0,0); poll();
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
    camera_green=0; observed_frame(0,0);
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
    puts("PASS: actual CAN age, route-to-vision handover, same-target resume, tool pause and stick cancellation");
    return 0;
}
