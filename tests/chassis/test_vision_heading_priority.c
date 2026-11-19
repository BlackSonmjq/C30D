#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    pose.yaw_rad=.08f;
    for(i=0;i<5;++i) {
        frame(1,20);
        REQUIRE(dx==0 && dy>0 && dz<0 && !strikes);
    }
    frame(0,0); REQUIRE(dy>0 && dz<0); /* Short DX gap uses existing memory. */
    camera_green=0; frame(0,0);
    REQUIRE(!vision_target_region && !vision_align.dx_hold_speed && dx==0 && dy==0);
    poll(); REQUIRE(dy>.29f && dz<0);
    camera_green=1; frame(1,20); frame(1,20); frame(1,20);
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1);
    puts("PASS: global heading during alignment, no separate yaw strike gate, green loss releases");
    return 0;
}
