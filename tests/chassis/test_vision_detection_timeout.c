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
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK);
    /* G remains 1 while the trunk disappears: LOST must also time out. */
    frame(0,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>0);
    now+=250;poll();
    REQUIRE(vision_align.phase==VISION_ALIGN_LOST);
    while(!vision_detection_timed_out) {
        frame(0,0);
        REQUIRE(dx==0 && dy==0 && strikes==0);
        REQUIRE(now<22000);
    }
    REQUIRE(vision_align.phase==VISION_ALIGN_WAIT && vision_pick_phase==PICK_IDLE);
    for(i=0;i<20;++i) frame(1,20);
    REQUIRE(dx>.29f && strikes==0 && vision_detection_timed_out);
    camera_green=0;frame(0,0);
    REQUIRE(!vision_detection_timed_out);
    camera_green=1;frame(1,20);frame(1,20);frame(1,20);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK);
    /* Continuous unsuccessful TRACK must likewise release after its timeout. */
    while(!vision_detection_timed_out) {
        frame(1,20);
        REQUIRE(strikes==0 && now<34000);
    }
    REQUIRE(dx==0 && dy==0 && vision_align.phase==VISION_ALIGN_WAIT);
    poll();REQUIRE(dx>.29f && vision_pick_phase==PICK_IDLE);
    printf("PASS: trunk-lost and alignment timeouts stop then release, same green cannot reacquire (%u checks)\n",checks);
    return 0;
}
