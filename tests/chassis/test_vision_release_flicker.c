#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,90,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

int main(void)
{
    unsigned i,j;
    uint32_t started;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<3;++i) frame(1,-80);
    REQUIRE(VisionAlign_ControlOwned() && dy<0 && !strikes);
    started=vision_align.started_ms;
    camera_green=0; frame(1,-40);
    REQUIRE(VisionAlign_ControlOwned() && vision_target_region==2 && dx==0 && dy==0);
    /* Flicker parks the existing attempt without restoring manual input or relocking. */
    for(i=0;i<8;++i) {
        camera_green=1;
        for(j=0;j<8;++j) frame(1,-80);
        REQUIRE(VisionAlign_ControlOwned() && dy<0 && vision_lock_count==1);
        camera_green=0; frame(1,-40); frame(1,-40);
        REQUIRE(VisionAlign_ControlOwned() && dx==0 && dy==0 && !strikes);
    }
    /* Repeated grace periods must not make an unfinished alignment unbounded. */
    for(i=0;i<200 && !vision_detection_timed_out;++i) {
        camera_green=(i%8)>=2;
        frame(1,-40);
    }
    REQUIRE(vision_detection_timed_out && !VisionAlign_ControlOwned());
    REQUIRE(now-started>=VISION_ALIGN_TIMEOUT_MS && now-started<=VISION_ALIGN_TIMEOUT_MS+50U);
    camera_green=1;
    for(i=0;i<8;++i) frame(1,-80);
    REQUIRE(!VisionAlign_ControlOwned() && dy>.29f && !vision_target_region);
    /* Only 300 ms of real absence rearms after release. */
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
    clear_fruit(); camera_green=1;
    for(i=0;i<3;++i) frame(1,-80);
    REQUIRE(VisionAlign_ControlOwned());
    puts("PASS: green flicker parks the same target, preserves deadline, and rearming needs fresh absence");
    return 0;
}
