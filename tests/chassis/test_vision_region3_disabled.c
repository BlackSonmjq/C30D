#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,45,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar;v->sequence=now; }
static void observation(unsigned region,int offset)
{
    unsigned i;
    uint8_t bytes[6]={0x63,1,(uint8_t)offset,1,(uint8_t)region,0x66};
    for(i=0;i<6;++i) Camera_Push(&camera,bytes[i]);
    now+=50;poll();
}
int main(void)
{
    unsigned i;
    VisionStrike_State isolated={0};Camera_Data data={0};
    data.pole=data.green=1;data.region=3;
    for(i=1;i<=8;++i) {data.frames=i;REQUIRE(!VisionStrike_Update(&isolated,&data,i*20,0));}
    VisionStrike_Accept(&isolated);
    for(i=9;i<=16;++i) {data.frames=i;REQUIRE(!VisionStrike_Update(&isolated,&data,i*20,0) && isolated.latched);}
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<30;++i) observation(3,0);
    REQUIRE(!strikes && !vision_boot_attempts && !vision_target_region && !vision_lock_count);
    REQUIRE(!VisionAlign_ControlOwned() && dx>.3f && camera.data.region==3);
    observation(1,20);REQUIRE(vision_target_region==1);
    for(i=0;i<12;++i) observation(3,0); /* A jump cannot replace an existing allowed lock. */
    REQUIRE(strikes==1 && submitted_id==1 && vision_boot_attempts==1);
    REQUIRE(camera.data.region==3);
    puts("PASS: REGION3 never locks/aligns/strikes or consumes quota; allowed lock and raw telemetry preserved");
    return 0;
}
