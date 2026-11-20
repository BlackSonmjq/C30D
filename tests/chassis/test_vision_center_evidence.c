#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,85,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

int main(void)
{
    unsigned i;
    uint32_t started;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    started=vision_align.started_ms;
    frame(1,2); frame(0,0);
    REQUIRE(VisionAlign_ControlOwned() && dx==0 && dy==0);
    frame(1,3); frame(1,1); frame(1,4);
    REQUIRE(!vision_align.approaching && dx==0);
    for(i=0;i<5;++i) poll();
    REQUIRE(!vision_align.approaching && dx==0); /* Cache cannot add votes. */
    frame(1,-1);
    REQUIRE(vision_align.approaching && dx<0 && dy==0 && !strikes);
    REQUIRE(vision_align.started_ms==started);

    /* A missing trunk pauses approach immediately, then fresh DX resumes it. */
    frame(0,0); REQUIRE(dx==0 && dy==0 && VisionAlign_ControlOwned());
    frame(1,1); REQUIRE(vision_align.approaching && dx<0 && dy==0);
    for(i=0;i<5;++i) frame(0,0);
    frame(1,1); /* A new packet at 300 ms cannot hide the preceding long DX gap. */
    REQUIRE(vision_align.phase==VISION_ALIGN_LOST && dx==0 && dy==0);
    for(i=0;i<6;++i) frame(1,20);
    REQUIRE(!vision_align.approaching && dx==0 && dy>0);

    /* Large deviation erases old centre evidence. */
    frame(1,0); frame(1,0); frame(1,8); frame(1,0);
    REQUIRE(!vision_align.approaching && dx==0);
    frame(1,3); frame(1,0);
    REQUIRE(!vision_align.approaching && dx==0);
    frame(1,0);
    REQUIRE(vision_align.approaching && dx<0);
    for(i=0;i<3;++i) frame(1,4);
    REQUIRE(!vision_align.approaching && dx==0 && dy>0); /* Persistent error realigns first. */
    /* Evidence older than 300 ms cannot combine into an approach. */
    frame(1,0); frame(1,0);
    for(i=0;i<5;++i) frame(0,0);
    frame(1,0);
    REQUIRE(vision_align.phase==VISION_ALIGN_LOST && !vision_align.approaching && dx==0 && dy==0);
    for(i=0;i<7;++i) frame(1,0);
    REQUIRE(vision_align.approaching && dx<0 && !strikes);
    lidar.mm[ROUTE_LIDAR_BEHIND]=45;
    for(i=0;i<16 && !strikes;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && submitted_id==2);
    puts("PASS: bounded 3-of-5 DX evidence, short trunk-gap pause/resume, large-error reset and safe rear approach");
    return 0;
}
