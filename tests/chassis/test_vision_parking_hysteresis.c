#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,45,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=.02f; /* Keep waiting for settling, without dispatching a tool. */
    MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && vision_pick_phase==PICK_WAIT);
    for(i=0;i<6;++i) {
        frame(1,3); frame(1,3); frame(1,3);
        REQUIRE(vision_align.phase==VISION_ALIGN_DONE && dx==0 && dy==0 && !strikes);
        frame(1,0);
    }
    frame(1,3); frame(1,3);
    for(i=0;i<30;++i) poll();
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && dx==0 && dy==0);
    frame(1,3); frame(1,3);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && !dy);
    frame(1,3);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>0 && dx==0);
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE);
    lidar.mm[ROUTE_LIDAR_BEHIND]=49;
    for(i=0;i<4;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && dx==0 && !strikes);
    lidar.mm[ROUTE_LIDAR_BEHIND]=45;
    for(i=0;i<3;++i) frame(1,0);
    lidar.mm[ROUTE_LIDAR_BEHIND]=49;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dx<0 && dy==0);
    lidar.mm[ROUTE_LIDAR_BEHIND]=45;
    for(i=0;i<6;++i) frame(1,0);
    frame(1,10); frame(1,10); frame(1,10);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>0); /* Large error keeps prompt response. */
    REQUIRE(!strikes);
    puts("PASS: parked small errors need 200ms plus new observations, cache/jitter cannot restart motion");
    return 0;
}
