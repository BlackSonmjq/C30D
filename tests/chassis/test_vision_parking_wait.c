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
    MOTOR_A.Encoder=.02f;
    MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && VisionAlign_ControlOwned());
    /* Valid target may wait for wheel settling beyond the removed 5s limit. */
    for(i=0;i<120;++i) {
        frame(1,0);
        REQUIRE(!vision_detection_timed_out && VisionAlign_ControlOwned() && !strikes);
    }
    MOTOR_A.Encoder=0;
    for(i=0;i<7;++i) frame(1,0);
    REQUIRE(strikes==1 && vision_pick_phase==PICK_WORK && tool_busy);
    camera_green=0;
    for(i=0;i<120;++i) frame(0,0);
    REQUIRE(vision_pick_phase==PICK_WORK && VisionAlign_ControlOwned() && dx==0 && dy==0);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2;
    poll(); REQUIRE(vision_pick_phase==PICK_REPORT && vision_hit_count==1);
    frame(0,0); REQUIRE(!VisionAlign_ControlOwned());
    poll(); REQUIRE(dy>.29f);
    puts("PASS: no 5s parking deadline, valid target can settle later, accepted work preserves safe return");
    return 0;
}
