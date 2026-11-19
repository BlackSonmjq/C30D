#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
int main(void)
{
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;RC_Velocity=480;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    camera_green=0;frame(0,0);REQUIRE(fabsf(dx-.48f)<.00001f);
    camera_green=1;
    smooth_control.VX=.48f;smooth_control.VY=-.48f;
    frame(1,20);
    REQUIRE(fabsf(dx-.33f)<.00001f);
    REQUIRE(smooth_control.VX<=.33f && smooth_control.VY>=-.33f);
    /* The slowdown still applies after automatic control has been released. */
    vision_detection_timed_out=1;
    gamepad.LY=190;frame(1,20);
    REQUIRE(fabsf(dx-63.0f*.33f/127.0f)<.00001f);
    gamepad.LY=127;gamepad.RX=0;frame(1,20);
    REQUIRE(dx==0 && fabsf(dy-.33f)<.00001f);
    RC_Velocity=200;frame(1,20);
    REQUIRE(fabsf(dy-.2f)<.00001f);
    RC_Velocity=480;frame(0,0); /* G remains 1 but trunk is absent. */
    REQUIRE(fabsf(dy-.33f)<.00001f);
    frame(1,20);REQUIRE(fabsf(dy-.33f)<.00001f);
    now+=300;poll();REQUIRE(fabsf(dy-.48f)<.00001f); /* Stale vision cannot cap speed. */
    camera_green=0;frame(1,0);REQUIRE(fabsf(dy-.33f)<.00001f);
    frame(0,0);REQUIRE(fabsf(dy-.48f)<.00001f);
    REQUIRE(RC_Velocity==480); /* Neither Flash nor the user's speed is overwritten. */
    printf("PASS: fresh B=1 or G=1 scales manual axes to 330mm/s, keeps low speed and restores 480 for missing/stale targets (%u checks)\n",checks);
    return 0;
}
