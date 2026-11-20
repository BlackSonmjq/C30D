#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

int main(void)
{
    unsigned i;
    uint32_t started;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=.02f; /* Keep the tool waiting for the chassis to settle. */
    MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && vision_pick_phase==PICK_WAIT);
    started=vision_align.started_ms;
    for(i=0;i<220;++i) frame(1,0);
    REQUIRE(now-started>VISION_ALIGN_TIMEOUT_MS && !vision_detection_timed_out);

    /* Three persistent DX observations restart alignment even after a long park. */
    frame(1,20); frame(1,20);
    REQUIRE(vision_pick_phase==PICK_WAIT && dx==0 && dy==0);
    frame(1,20);
    REQUIRE(!vision_detection_timed_out && vision_align.phase==VISION_ALIGN_TRACK);
    REQUIRE(vision_pick_phase==PICK_IDLE && VisionAlign_ControlOwned() && dy>0 && dx==0);
    REQUIRE(vision_align.started_ms==now && strikes==0);
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_DONE && vision_pick_phase==PICK_WAIT);
    for(i=0;i<220;++i) frame(1,0);

    /* Rear distance drift must also restart the approach, not release to held sticks. */
    lidar.mm[ROUTE_LIDAR_BEHIND]=60;
    for(i=0;i<5;++i) frame(1,0);
    REQUIRE(!vision_detection_timed_out && vision_align.phase==VISION_ALIGN_TRACK);
    REQUIRE(vision_align.approaching && vision_pick_phase==PICK_IDLE && dx<0 && dy==0);
    REQUIRE(VisionAlign_ControlOwned() && strikes==0);

    lidar.mm[ROUTE_LIDAR_BEHIND]=VISION_ALIGN_REAR_MM;
    MOTOR_A.Encoder=0;
    for(i=0;i<16 && !strikes;++i) frame(1,0);
    REQUIRE(strikes==1 && submitted_id==2 && vision_pick_phase==PICK_WORK);
    puts("PASS: parked DX/rear drift starts a new alignment budget and retains ownership until settled");
    return 0;
}
