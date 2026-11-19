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
    REQUIRE(dy>.19f);
    frame(1,7); REQUIRE(dy>0 && dy<=.06001f);
    for(i=0;i<10;++i) {
        frame(1,-7); REQUIRE(dy==0);
        frame(1,-7); REQUIRE(dy==0);
        frame(1,7); REQUIRE(dy>0 && dy<=.06001f);
    }
    frame(1,-7); frame(1,-7);
    poll(); REQUIRE(dy==0); /* Cached snapshots cannot confirm a reversal. */
    frame(1,-7); REQUIRE(dy<0 && dy>=-.06001f);
    frame(1,0); REQUIRE(dy==0 && !vision_align.dx_hold_speed);
    frame(0,0); REQUIRE(dy==0);
    puts("PASS: near-centre speed capped, short alternating errors cannot reverse, true centre stops");
    return 0;
}
