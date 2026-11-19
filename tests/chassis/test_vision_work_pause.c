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
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && vision_target_region==2);
    for(i=0;i<30;++i) {
        camera_green=i%2;frame(i%2,i%2?60:0);
        REQUIRE(vision_pick_phase==PICK_WORK && vision_target_region==2);
        REQUIRE(vision_pick_attempts==1 && vision_lock_count==1 && dx==0 && dy==0 && dz==0);
    }
    tool_busy=0;tool.state=STEPPER_FAULT;tool.result=7;tool.mango_id=2;poll();
    REQUIRE(vision_pick_phase==PICK_WORK && dx==0);
    tool.state=STEPPER_RETURNED;tool.result=1;camera_green=0;frame(0,0);
    REQUIRE(vision_pick_phase==PICK_COOLDOWN);
    now+=250;poll(); /* Cached G=0 from WORK must not release or clear the target. */
    REQUIRE(vision_pick_phase==PICK_COOLDOWN && vision_target_region==2);
    REQUIRE(!vision_detection_timed_out && vision_pick_attempts==1 && dx==0 && dy==0);
    camera_green=1;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_COOLDOWN);
    now+=1800;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_IDLE && vision_align.phase==VISION_ALIGN_ACQUIRE);
    REQUIRE(vision_target_region==2 && vision_lock_count==1);
    for(i=0;i<15;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==2);
    camera_green=0;frame(0,0);
    tool_busy=0;tool.state=STEPPER_RETURNED;tool.result=MANGO_OK;tool.mango_id=2;poll();
    REQUIRE(vision_pick_phase==PICK_REPORT && vision_hit_count==1);
    now+=250;poll();
    REQUIRE(vision_pick_phase==PICK_REPORT && dx==0 && dy==0);
    frame(0,0);
    REQUIRE(vision_pick_phase==PICK_IDLE && vision_align.phase==VISION_ALIGN_WAIT);
    REQUIRE(vision_align.dx_hold_speed==0 && !vision_target_region);
    gamepad.LY=gamepad.RX=127;camera_green=1;
    for(i=0;i<10;++i) {
        frame(0,0);
        REQUIRE(dx==0 && dy==0 && vision_align.dx_hold_speed==0);
        REQUIRE(vision_pick_phase==PICK_IDLE && vision_align.phase==VISION_ALIGN_WAIT);
    }
    REQUIRE(strikes==2 && vision_lock_count==1);
    printf("PASS: full WORK ignores visual changes and G=0, fault stays parked, success/failure resume only on new post-return frames, released control never uses old DX (%u checks)\n",checks);
    return 0;
}
