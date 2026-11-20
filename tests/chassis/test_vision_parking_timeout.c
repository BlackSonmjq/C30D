#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main

static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

int main(void)
{
    unsigned i;
    uint32_t parked_at;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=.02f;
    MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && VisionAlign_ControlOwned());
    parked_at=vision_pick_wait_started;
    while((uint32_t)(now-parked_at)<4950U) {
        frame(1,0);
        REQUIRE(!vision_detection_timed_out && VisionAlign_ControlOwned() && !strikes);
    }
    MOTOR_A.Encoder=0; /* Becoming ready exactly at expiry must not start work. */
    frame(1,0);
    REQUIRE((uint32_t)(now-parked_at)==5000U);
    REQUIRE(vision_detection_timed_out && !VisionAlign_ControlOwned() && !strikes);
    REQUIRE(!vision_target_region && !vision_align.dx_hold_speed && dx==0 && dy==0);
    /* Existing input resumes without an extra stick edge or recentring. */
    poll(); REQUIRE(dy>.29f);
    for(i=0;i<20;++i) frame(1,0);
    REQUIRE(vision_detection_timed_out && !VisionAlign_ControlOwned() && dy>.29f);

    /* A fresh absent-fruit observation begins a separate episode. */
    camera_green=0; frame(0,0); camera_green=1;
    MOTOR_A.Encoder=.02f;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT);
    parked_at=vision_pick_wait_started;
    while((uint32_t)(now-parked_at)<5000U) {
        uint32_t elapsed=(uint32_t)(now-parked_at);
        frame(1,(elapsed%800U)<400U?8:0);
        REQUIRE(!strikes);
    }
    REQUIRE(vision_detection_timed_out && !VisionAlign_ControlOwned());
    REQUIRE(!vision_target_region && !vision_align.dx_hold_speed);
    poll(); REQUIRE(dy>.29f);

    /* The released input may be the route simulation with no gamepad. */
    GamePadInterface=NULL; PS2_ON_Flag=0;
    RC_Velocity=330; /* Existing visual slowdown must not obscure handoff. */
    Chassis_NavigationDrive(.28f,0,0);
    REQUIRE(dx==.28f && dy==0 && !VisionAlign_ControlOwned());

    /* Accepted work is exempt even after the parking deadline expires. */
    camera_green=0; frame(0,0); camera_green=1;
    MOTOR_A.Encoder=0;
    for(i=0;i<20 && !strikes;++i) frame(1,0);
    REQUIRE(strikes==1 && vision_pick_phase==PICK_WORK && tool_busy);
    for(i=0;i<120;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && VisionAlign_ControlOwned() && dx==0 && dy==0);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2;
    poll(); REQUIRE(vision_pick_phase==PICK_REPORT && vision_hit_count==1);
    camera_green=0; frame(0,0);
    REQUIRE(!VisionAlign_ControlOwned());
    puts("PASS: parking expires at 5s across repeated adjustments, input resumes, accepted work keeps safe-return lock");
    return 0;
}
