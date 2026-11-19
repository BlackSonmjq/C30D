#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    pose.yaw_rad=.08f;
    frame(0,0); now+=250; poll();
    REQUIRE(vision_align.phase==VISION_ALIGN_LOST && dx==0 && dy==0 && dz<0);
    for(i=0;i<6;++i) frame(1,0);
    for(i=0;i<4;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && dx==0 && dy==0 && dz<0);
    for(i=0;i<10 && vision_pick_phase!=PICK_WORK;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && dx==0 && dy==0 && dz==0);
    pose.yaw_rad=.15f; poll();
    REQUIRE(vision_pick_phase==PICK_WORK && dx==0 && dy==0 && dz==0);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2;
    poll(); REQUIRE(vision_pick_phase==PICK_REPORT && dz==0);
    poll(); REQUIRE(vision_pick_phase==PICK_REPORT && dx==0 && dy==0 && dz<0);
    puts("PASS: yaw hold during visual stop and lost tracking, held manual blocked and tool work stopped");
    return 0;
}
