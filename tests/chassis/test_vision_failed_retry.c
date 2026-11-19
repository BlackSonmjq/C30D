#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20);frame(1,20);frame(1,20);
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && vision_pick_attempts==1 && strikes==1);
    tool_busy=0;tool.state=STEPPER_FAULT;tool.mango_id=2;tool.result=7;poll();
    REQUIRE(vision_pick_phase==PICK_WORK && dx==0); /* Fault is never rearmed. */
    tool.state=STEPPER_RETURNED;tool.result=1;poll();
    REQUIRE(vision_pick_phase==PICK_COOLDOWN && vision_pick.latched);
    for(i=0;i<40;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_IDLE && vision_align.phase==VISION_ALIGN_ACQUIRE);
    REQUIRE(!vision_pick.latched && !vision_detection_timed_out && strikes==1 && dx==0);
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==2 && vision_pick_attempts==2 && vision_lock_count==1);
    tool_busy=0;tool.state=STEPPER_RETURNED;tool.result=1;poll();
    for(i=0;i<40;++i) frame(1,0);
    REQUIRE(vision_detection_timed_out && vision_pick_phase==PICK_IDLE && dx==0);
    REQUIRE(vision_align.phase==VISION_ALIGN_WAIT && vision_hit_count==0);
    for(i=0;i<30;++i) frame(1,0);
    REQUIRE(strikes==2 && dx>.32f && !vision_hit_report);
    camera_green=0;frame(0,0);
    REQUIRE(!vision_detection_timed_out && vision_pick_attempts==0);
    printf("PASS: safely returned failure rearms once, reconfirms alignment, then quota exhaustion releases; fault lock preserved (%u checks)\n",checks);
    return 0;
}
