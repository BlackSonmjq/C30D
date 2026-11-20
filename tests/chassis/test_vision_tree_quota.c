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
    /* Observations without a trunk cannot consume a tree. */
    { uint8_t bytes[6]={0x63,0,20,1,2,0x66};
      for(i=0;i<6;++i) Camera_Push(&camera,bytes[i]);
      now+=50;poll(); }
    REQUIRE(!VisionAlign_ControlOwned() && !vision_boot_alignments);
    for(n=0;n<3;++n) {
        clear_fruit();camera_green=1;
        for(i=0;i<6;++i) frame(1,20);
        REQUIRE(VisionAlign_ControlOwned() && vision_boot_alignments==n+1);
        frame(0,20);
        for(i=0;i<8;++i) frame(1,20);
        REQUIRE(VisionAlign_ControlOwned() && vision_boot_alignments==n+1);
        if(n==1) {
            /* An alignment timeout counts even when no mechanism started. */
            for(i=0;i<220 && VisionAlign_ControlOwned();++i) frame(1,20);
            REQUIRE(!VisionAlign_ControlOwned() && vision_detection_timed_out && strikes==1);
        } else {
            for(i=0;i<20;++i) frame(1,0);
            REQUIRE(vision_pick_phase==PICK_WORK && strikes==(n==0?1:2));
            camera_green=0;frame(0,0);
            REQUIRE(tool_busy && vision_pick_phase==PICK_WORK && dx==0);
            tool_busy=0;tool.state=STEPPER_RETURNED;tool.mango_id=2;tool.result=MANGO_OK;
            poll();REQUIRE(vision_pick_phase==PICK_REPORT);
            vision_hit_report=0;
        }
    }
    clear_fruit();camera_green=1;
    EN=0;poll();EN=1;
    for(i=0;i<50;++i) frame(1,20);
    REQUIRE(vision_boot_alignments==3 && vision_boot_attempts==2 && strikes==2);
    REQUIRE(!VisionAlign_ControlOwned() && !vision_target_region && dx>.3f);
    REQUIRE(vision_release_reason==VISION_RELEASE_QUOTA);
    puts("PASS: three alignment entries per boot, timeout counts, same-tree recovery is free, third returns safely");
    return 0;
}
