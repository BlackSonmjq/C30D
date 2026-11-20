#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,50,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20);frame(1,20);frame(1,20);
    for(i=0;i<8;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dx<0 && strikes==0);
    lidar.mm[ROUTE_LIDAR_BEHIND]=45;
    for(i=0;i<5;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && dx==0 && dy==0 && strikes==0);
    lidar.mm[ROUTE_LIDAR_BEHIND]=44;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && dx==0 && strikes==0);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(dx==0 && strikes==0);
    frame(1,0);
    REQUIRE(dx>0 && strikes==0); /* Move away from the rear safety boundary. */
    lidar.mm[ROUTE_LIDAR_BEHIND]=45;
    for(i=0;i<12;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && dx==0 && dy==0 && dz==0);
    puts("PASS: rear50 approaches, rear45 stops and permits a hit, rear44 blocks hit");
    return 0;
}
