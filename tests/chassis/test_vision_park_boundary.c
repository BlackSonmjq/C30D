#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,45,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    uint32_t started;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.LY=254;
    MOTOR_A.Encoder=.006f;
    MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20);frame(1,20);frame(1,20);
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && !strikes);
    MOTOR_A.Encoder=0;
    frame(1,2);
    REQUIRE(vision_pick_phase==PICK_HOLD && !strikes);
    frame(1,2);frame(1,2);
    frame(1,3);
    REQUIRE(vision_pick_phase==PICK_WAIT && !strikes);
    frame(1,2);started=now;
    REQUIRE(vision_pick_phase==PICK_HOLD && !strikes);
    for(i=0;i<3;++i) {frame(1,2);REQUIRE(!strikes);}
    REQUIRE(now-started==150);
    frame(1,2);
    REQUIRE(now-started==200 && strikes==1 && vision_pick_phase==PICK_WORK);
    puts("PASS: DX outside tolerance resets the continuous 200ms visual/range/wheel hold");
    return 0;
}
