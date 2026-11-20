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
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && vision_pick_phase==PICK_HOLD);
    /* A rolling wheel blocks a hit until the condition wait expires. */
    MOTOR_A.Encoder=.006f;
    while((uint32_t)(now-vision_pick_wait_started)<VISION_PICK_WAIT_MS) {
        frame(1,0);
        REQUIRE(dx==0 && dy==0 && dz==0 && strikes==0);
    }
    REQUIRE(vision_align.phase==VISION_ALIGN_WAIT && vision_pick_phase==PICK_IDLE && vision_detection_timed_out);
    REQUIRE(dx==0 && dy==0 && dz==0);
    MOTOR_A.Encoder=0;
    /* Same green target cannot reacquire; a fresh G=0 rearms a new episode. */
    for(i=0;i<35;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_IDLE && strikes==0 && dx>.29f);
    camera_green=0;frame(0,0);
    REQUIRE(!vision_detection_timed_out);
    camera_green=1;
    frame(1,20);frame(1,20);frame(1,20);
    frame(1,0);frame(1,0);frame(1,0);
    MOTOR_A.Encoder=.006f;
    for(i=0;i<80;++i) frame(1,0); /* Recover before the new 5-second deadline. */
    REQUIRE(vision_pick_phase==PICK_WAIT && !vision_detection_timed_out && strikes==0);
    MOTOR_A.Encoder=0;
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && tool_busy);
    REQUIRE(dx==0 && dy==0 && !vision_hit_report);
    printf("PASS: timed-out pick wait stops once, releases manual control and waits for G=0 to rearm (%u checks)\n",checks);
    return 0;
}
