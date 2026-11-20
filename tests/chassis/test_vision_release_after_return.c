#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar; frame->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1);
    /* Only the matching safe return permits a stale-camera release. */
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2; poll();
    REQUIRE(vision_pick_phase==PICK_REPORT);
    now+=299; poll(); REQUIRE(vision_pick_phase==PICK_REPORT && dx==0);
    now+=1; poll(); REQUIRE(vision_detection_timed_out && vision_pick_phase==PICK_IDLE && dx==0);
    poll(); REQUIRE(dx>.29f && !vision_align.dx_hold_speed);
    clear_fruit();
    camera_green=1; vision_hit_report=0;
    frame(1,20); frame(1,20); frame(1,20);
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==2);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2; poll();
    /* Quota exhausted with a still-visible target releases on the next fresh frame. */
    vision_pick_attempts=2;
    frame(1,0);
    REQUIRE(vision_detection_timed_out && vision_pick_phase==PICK_IDLE && dx==0);
    poll(); REQUIRE(dx>.29f && strikes==2);
    puts("PASS: stale camera and exhausted quota release only after matching safe return");
    return 0;
}
