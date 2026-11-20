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
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>.19f);
    frame(0,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>.239f && dy<=.24001f && dx==0 && strikes==0);
    frame(0,0);frame(0,0);frame(0,0);frame(0,0);
    REQUIRE(dy>.19f && vision_target_region==2 && vision_lock_count==1);
    frame(0,0); /* Exactly 300ms since the last true DX. */
    REQUIRE(vision_align.phase==VISION_ALIGN_LOST && dx==0 && dy==0 && strikes==0);
    for(i=0;i<6;++i) frame(1,20);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>.19f);
    frame(0,0);frame(1,-20);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy<-.19f && strikes==0);
    frame(0,0);REQUIRE(dy<-.239f && dy>=-.24001f);
    camera_green=0;frame(0,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_LOST && vision_target_region==2 && dy==0 && !vision_align.dx_hold_speed);
    clear_fruit();camera_green=1;frame(1,20);frame(1,20);frame(1,20);
    frame(0,0);lidar.fresh=0;poll();
    REQUIRE(dx==0 && dy==0 && vision_align.phase==VISION_ALIGN_LOST);
    lidar.fresh=1;for(i=0;i<6;++i) frame(1,20);
    frame(1,0); /* Never coast on the old direction after a real centered DX. */
    frame(0,0);REQUIRE(dx==0 && dy==0 && strikes==0);
    clear_fruit();camera_green=1;
    frame(1,8);frame(1,8);frame(1,8);
    REQUIRE(dy>.119f && dy<.121f);
    frame(0,0);REQUIRE(dy>.179f && dy<.181f);
    clear_fruit();camera_green=1;
    frame(1,20);frame(1,20);frame(1,20);
    frame(0,0);EN=0;poll();REQUIRE(dx==0 && dy==0);
    EN=1;clear_fruit();camera_green=1;
    frame(1,20);frame(1,20);frame(1,20);
    lidar.mm[ROUTE_LIDAR_BEHIND]=70;
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(vision_align.approaching && dx<0);
    frame(0,0);
    REQUIRE(dx==0 && dy==0 && strikes==0); /* Do not extend longitudinal approach without DX. */
    printf("PASS: DX loss uses 1.5x lateral effort capped at 0.24m/s for at most300ms; new DX and stop protections retained (%u checks)\n",checks);
    return 0;
}
