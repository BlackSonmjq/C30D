#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
int main(void)
{
    unsigned i;
    VisionPickRange_State filter={0};
    REQUIRE(VisionPickRange_Update(&filter,50,1,1)==50);
    REQUIRE(VisionPickRange_Update(&filter,50,2,1)==50);
    REQUIRE(VisionPickRange_Update(&filter,54,3,1)==50);
    for(i=0;i<20;++i) REQUIRE(VisionPickRange_Update(&filter,54,3,1)==50);
    REQUIRE(filter.count==3 && filter.next==0); /* Repeated snapshot adds no samples. */
    REQUIRE(VisionPickRange_Update(&filter,54,4,1)==54); /* Persistent changes pass. */
    REQUIRE(VisionPickRange_Update(&filter,50,5,0)==0 && filter.count==0);
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20);frame(1,20);frame(1,20);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    lidar.mm[ROUTE_LIDAR_BEHIND]=49;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && strikes==0); /* Isolated range noise. */
    lidar.mm[ROUTE_LIDAR_BEHIND]=44;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && strikes==0 && dx==0); /* Raw safety wins. */
    lidar.mm[ROUTE_LIDAR_BEHIND]=VISION_ALIGN_REAR_MM;MOTOR_A.Encoder=.006f;
    frame(1,0);frame(1,0);
    frame(1,3);
    REQUIRE(vision_pick_phase==PICK_WAIT && dy==0 && strikes==0);
    for(i=0;i<20;++i) poll();
    REQUIRE(dy==0 && strikes==0);
    frame(1,-3);frame(1,3);frame(1,3);
    REQUIRE(vision_pick_phase==PICK_WAIT && dy==0); /* Alternating signs cannot restart. */
    frame(1,3);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && dy==0);
    frame(1,3); frame(1,3);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && dy>.044f && dy<.046f && strikes==0);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && dx==0 && dy==0 && strikes==0);
    MOTOR_A.Encoder=0;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    for(i=0;i<5;++i) frame(1,0);
    REQUIRE(strikes==0);
    frame(1,0);
    REQUIRE(strikes==1 && vision_pick_phase==PICK_WORK && submitted_id==2);
    printf("PASS: median rejects isolated range noise, raw44 blocks strike, persistent DX3 resumes correction then waits300ms (%u checks)\n",checks);
    return 0;
}
