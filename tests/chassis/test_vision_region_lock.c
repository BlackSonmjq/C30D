#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
static void observation(unsigned b,unsigned g,unsigned id,int offset)
{
    unsigned j;
    uint8_t data[6]={0x63,(uint8_t)b,(uint8_t)offset,(uint8_t)g,(uint8_t)id,0x66};
    for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
    now+=50;poll();
}
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    observation(0,1,0,0);
    REQUIRE(!vision_target_region && dx>.32f && dy==0);
    observation(1,1,3,20);REQUIRE(vision_target_region==3 && strikes==0 && vision_lock_count==1);
    { VisionAlign_Diagnostics diag;VisionAlign_GetDiagnostics(&diag);REQUIRE(diag.locked_region==3 && diag.lock_count==1); }
    observation(1,1,2,20);observation(1,1,0,20);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK && vision_target_region==3 && vision_lock_count==1);
    observation(1,1,1,3);REQUIRE(fabsf(dy-.045f)<.00001f);
    observation(1,1,4,4);REQUIRE(fabsf(dy-.06f)<.00001f);
    observation(1,1,0,0);observation(1,1,0,0);observation(1,1,0,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && vision_target_region==3);
    for(i=0;i<6;++i) observation(1,1,i%2?0:1,0);
    REQUIRE(strikes==1 && submitted_id==3 && vision_pick_phase==PICK_WORK && vision_lock_count==1);
    REQUIRE(camera.data.region==0); /* Raw telemetry is never overwritten. */
    observation(0,0,0,0);
    REQUIRE(vision_pick_phase==PICK_WORK && vision_target_region==3 && dx==0);
    tool_busy=0;tool.state=STEPPER_RETURNED;tool.result=MANGO_OK;tool.mango_id=2;poll();
    REQUIRE(vision_pick_phase==PICK_WORK);
    tool.mango_id=3;poll();
    REQUIRE(vision_pick_phase==PICK_REPORT && vision_target_region==0 && vision_hit_region==3);
    observation(1,1,4,20);
    REQUIRE(vision_target_region==4 && vision_align.phase==VISION_ALIGN_ACQUIRE && vision_lock_count==2);
    vision_hit_report=0;
    for(i=0;i<20;++i) observation(1,1,i%2?0:2,0);
    REQUIRE(strikes==2 && submitted_id==4 && vision_pick_attempts==2);
    tool_busy=0;tool.state=STEPPER_RETURNED;tool.result=1;tool.mango_id=4;poll();
    for(i=0;i<40;++i) observation(1,1,0,0);
    REQUIRE(vision_detection_timed_out && vision_pick_phase==PICK_IDLE);
    poll();REQUIRE(!vision_target_region);
    for(i=0;i<7;++i) observation(0,0,0,0);
    REQUIRE(!vision_detection_timed_out && !vision_pick_attempts);
    observation(1,1,1,20);REQUIRE(vision_target_region==1);
    now+=300;poll();REQUIRE(!vision_target_region && strikes==2);
    observation(1,1,2,20);REQUIRE(vision_target_region==2);
    observation(0,0,0,0);REQUIRE(!vision_target_region && vision_lock_count==4);
    printf("PASS: first REGION locks through zeros/jumps until release/return, old DX force restored, safe work/quota/stale protections retained (%u checks)\n",checks);
    return 0;
}
