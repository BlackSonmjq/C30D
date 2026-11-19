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
    tool_busy=1;
    for(i=0;i<60;++i) frame(1,20);
    REQUIRE(vision_lock_count==0 && vision_target_region==0 && strikes==0);
    tool_busy=0;frame(1,20);
    REQUIRE(vision_lock_count==1 && vision_target_region==2);
    frame(1,20);frame(1,20);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    frame(1,3);frame(1,3);
    REQUIRE(strikes==0 && dy==0);
    for(i=0;i<20;++i) poll();
    REQUIRE(dy==0 && strikes==0); /* Cached snapshots cannot rearm. */
    frame(1,3);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>.044f && dy<.046f && strikes==0);
    /* Slight remaining error during rear approach must also be corrected. */
    vision_align.approaching=1;vision_align.restart_frames=0;vision_align.restart_sign=0;
    frame(1,-4);frame(1,-4);frame(1,-4);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy==0 && strikes==0);
    frame(1,-4);
    REQUIRE(dy<-.059f && strikes==0);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && vision_lock_count==1);
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && submitted_id==2);
    for(i=0;i<20;++i) frame(1,0);
    REQUIRE(vision_target_region==2 && vision_lock_count==1); /* Own WORK retains its lock. */
    printf("PASS: external busy blocks lock count, persistent DX3/4 resumes original correction, cached frames do not count, stopped locked target submits once (%u checks)\n",checks);
    return 0;
}
