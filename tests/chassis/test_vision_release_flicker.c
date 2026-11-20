#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,90,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

int main(void)
{
    unsigned i,j;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<3;++i) frame(1,-80);
    REQUIRE(VisionAlign_ControlOwned() && dy<0 && !strikes);
    camera_green=0; frame(1,-40);
    REQUIRE(!VisionAlign_ControlOwned() && !vision_target_region);
    poll(); REQUIRE(dy>.29f);
    /* A one-frame green loss must not let the same fruit steal the input again. */
    camera_green=1;
    for(i=0;i<8;++i) frame(1,-80);
    REQUIRE(!VisionAlign_ControlOwned() && dy>.29f && !vision_target_region);
    for(i=0;i<8;++i) {
        camera_green=0; frame(1,-40); frame(1,-40);
        camera_green=1;
        for(j=0;j<4;++j) frame(1,-80);
        REQUIRE(!VisionAlign_ControlOwned() && dy>.29f && !strikes);
    }
    /* Continuous real absence, rather than a stick action, rearms automation. */
    camera_green=0;
    for(i=0;i<7;++i) frame(1,-40);
    REQUIRE(!vision_detection_timed_out && !VisionAlign_ControlOwned());
    camera_green=1;
    for(i=0;i<3;++i) frame(1,-80);
    REQUIRE(VisionAlign_ControlOwned() && dy<0);

    /* A stale G=0 cache must not count as continuous absence. */
    camera_green=0; frame(1,-40);
    now+=500; poll();
    camera_green=1;
    for(i=0;i<6;++i) frame(1,-80);
    REQUIRE(!VisionAlign_ControlOwned() && dy>.29f);
    camera_green=0;
    for(i=0;i<7;++i) frame(1,-40);
    camera_green=1;
    for(i=0;i<3;++i) frame(1,-80);
    REQUIRE(VisionAlign_ControlOwned());
    puts("PASS: green flicker releases immediately without reacquiring; only 300ms fresh absence rearms, no stick gesture required");
    return 0;
}
