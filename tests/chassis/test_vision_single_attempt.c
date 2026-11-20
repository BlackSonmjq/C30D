#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,45,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    frame(1,0); frame(1,0); frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    for(i=0;i<3;++i) { frame(1,0); REQUIRE(!strikes); }
    frame(1,0);
    REQUIRE(strikes==1 && vision_pick_phase==PICK_WORK);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2;
    poll(); frame(1,0);
    REQUIRE(vision_detection_timed_out && !VisionAlign_ControlOwned());
    vision_hit_report=0;
    camera_green=0; frame(0,0); frame(0,0); camera_green=1;
    for(i=0;i<50;++i) frame(1,0);
    REQUIRE(strikes==1 && vision_detection_timed_out);
    clear_fruit(); camera_green=1;
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(strikes==2 && vision_pick_phase==PICK_WORK);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=1; tool.mango_id=2;
    poll();
    camera_green=0; frame(0,0); camera_green=1;
    for(i=0;i<70;++i) frame(1,0);
    REQUIRE(strikes==2 && vision_detection_timed_out && !VisionAlign_ControlOwned());
    puts("PASS: stationary 200ms, one attempt per episode, success/failure and short G gaps never retry");
    return 0;
}
