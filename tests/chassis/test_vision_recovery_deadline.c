#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar; frame->sequence=now; }
int main(void)
{
    unsigned i;
    uint32_t start;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20);frame(1,20);frame(1,20);
    start=vision_align.started_ms;
    for(i=0;i<300 && !vision_detection_timed_out;++i) {
        frame((i%12)>=6,(i%12)>=6?20:0);
    }
    REQUIRE(vision_detection_timed_out && now-start>=VISION_ALIGN_TIMEOUT_MS &&
            now-start<=VISION_ALIGN_TIMEOUT_MS+100U);
    REQUIRE(dx==0 && dy==0 && !vision_align.dx_hold_speed && strikes==0);
    poll(); REQUIRE(dy>.29f);
    puts("PASS: repeated lost/recovered camera frames cannot reset the alignment deadline");
    return 0;
}
