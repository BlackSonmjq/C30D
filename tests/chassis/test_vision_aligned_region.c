#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
static void observation(unsigned b,unsigned g,unsigned region,int offset)
{
    unsigned j;
    uint8_t data[6]={0x63,(uint8_t)b,(uint8_t)offset,(uint8_t)g,(uint8_t)region,0x66};
    for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
    now+=50;poll();
}
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    /* Green fruit alone must never stop a moving manual chassis. */
    for(i=0;i<60;++i) {
        observation(0,1,0,0);
        REQUIRE(dx>.29f && dy==0 && vision_pick_phase==PICK_IDLE);
        REQUIRE(!vision_detection_timed_out && strikes==0);
    }
    /* Without a numbered observation, trunk/green alone cannot start alignment. */
    observation(1,1,0,20);observation(1,1,0,20);observation(1,1,0,20);
    REQUIRE(!vision_target_region && dx>.32f && dy==0 && vision_pick_phase==PICK_IDLE);
    observation(0,0,0,0);
    /* A single earlier valid number now stays locked through zeros and jumps. */
    observation(1,1,2,20);observation(1,1,3,20);observation(1,1,0,20);
    observation(1,1,0,0);observation(1,1,0,0);observation(1,1,0,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && vision_target_region==2 && !vision_detection_timed_out);
    observation(1,1,4,0);
    REQUIRE(vision_target_region==2 && vision_pick_phase==PICK_HOLD && strikes==0);
    observation(0,0,0,0);
    REQUIRE(vision_target_region==2 && vision_pick_phase==PICK_WAIT && strikes==0);
    for(i=0;i<40;++i) observation(0,0,0,0);
    REQUIRE(!vision_target_region && vision_pick_phase==PICK_IDLE && strikes==0);
    printf("PASS: green alone stays manual, no numbered observation stays manual, first valid REGION locks through later zeros/jumps (%u checks)\n",checks);
    return 0;
}
