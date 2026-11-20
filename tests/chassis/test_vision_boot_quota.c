#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,45,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar;v->sequence=now; }
int main(void)
{
    unsigned n,i;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(n=0;n<3;++n) {
        clear_fruit();camera_green=1;vision_hit_report=0;
        for(i=0;i<18;++i) frame(1,0);
        REQUIRE(strikes==n+1 && vision_pick_phase==PICK_WORK);
        camera_green=0;frame(0,0);
        REQUIRE(vision_pick_phase==PICK_WORK && tool_busy && dx==0);
        tool_busy=0;tool.state=STEPPER_RETURNED;tool.mango_id=2;
        tool.result=n==1?1:MANGO_OK;poll();
        for(i=0;i<50;++i) frame(0,0);
    }
    REQUIRE(strikes==3 && vision_hit_count==2 && vision_boot_alignments==3);
    vision_hit_report=0;clear_fruit();camera_green=1;
    EN=0;poll();EN=1;
    for(i=0;i<50;++i) frame(1,0);
    REQUIRE(strikes==3 && !VisionAlign_ControlOwned() && dx>.3f);
    REQUIRE(vision_release_reason==VISION_RELEASE_QUOTA && !vision_target_region);
    puts("PASS: three visual trees per boot, failures count, third returns safely, absence/mode changes never reset quota");
    return 0;
}
