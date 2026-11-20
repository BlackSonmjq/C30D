#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

int main(void)
{
    unsigned i;
    uint32_t started, lost;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    for(i=0;i<130;++i) frame(1,20);
    started=vision_align.started_ms;
    camera_green=0; frame(0,0); lost=now;
    REQUIRE(VisionAlign_ControlOwned() && vision_target_region==2);
    REQUIRE(dx==0 && dy==0 && !vision_align.dx_hold_speed && !strikes);
    for(i=0;i<9;++i) {
        frame(0,0);
        REQUIRE(VisionAlign_ControlOwned() && dx==0 && dy==0 && !strikes);
    }
    camera_green=1; frame(1,20);
    REQUIRE(vision_align.started_ms==started); /* Green flicker cannot reset the attempt deadline. */
    for(i=0;i<6;++i) frame(1,20);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>0 && !vision_detection_timed_out);

    /* A receive outage still releases at 300 ms, even during the green grace. */
    camera_green=0; frame(0,0);
    now+=300; poll();
    REQUIRE(!VisionAlign_ControlOwned() && vision_detection_timed_out && !vision_target_region);
    clear_fruit();
    camera_green=1;
    MOTOR_A.Encoder=.02f;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && VisionAlign_ControlOwned());
    camera_green=0; frame(0,0); lost=now;
    for(i=0;i<40 && VisionAlign_ControlOwned();++i) {
        frame(0,0);
        REQUIRE(!strikes);
    }
    REQUIRE(now-lost<=2000 && !VisionAlign_ControlOwned() && !vision_target_region);
    REQUIRE(!strikes && !vision_align.dx_hold_speed);

    /* A recovered parked target gets a fresh stationary qualification. */
    clear_fruit(); camera_green=1; MOTOR_A.Encoder=0;
    for(i=0;i<5;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && !strikes);
    camera_green=0; frame(0,0);
    for(i=0;i<20;++i) frame(0,0);
    REQUIRE(VisionAlign_ControlOwned() && !strikes);
    camera_green=1; frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && !strikes);
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2;
    poll(); REQUIRE(vision_pick_phase==PICK_REPORT);
    camera_green=0; frame(0,0);
    REQUIRE(!VisionAlign_ControlOwned()); /* No grace after a completed hit. */
    puts("PASS: two-second green-loss parking grace, recovery, receive timeout and post-hit release");
    return 0;
}
