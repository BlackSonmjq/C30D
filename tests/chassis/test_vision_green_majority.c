#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,90,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }

/* Deliver several packets between chassis cycles; cached snapshots cannot vote. */
static void packets(unsigned good,unsigned missing)
{
    uint8_t yes[6]={0x63,1,20,1,2,0x66}, no[6]={0x63,0,0,0,0,0x66};
    unsigned i,j;
    for(i=0;i<missing;++i) for(j=0;j<6;++j) Camera_Push(&camera,no[j]);
    for(i=0;i<good;++i) for(j=0;j<6;++j) Camera_Push(&camera,yes[j]);
    now+=50; poll();
}
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20); frame(1,20); frame(1,20);
    for(i=0;i<39;++i) packets(1,3);
    REQUIRE(!VisionAlign_ControlOwned() && vision_detection_timed_out && !vision_target_region);
    clear_fruit(); camera_green=1;
    frame(1,20); frame(1,20); frame(1,20);
    packets(0,2); /* Balance the two confirmation frames after the first lock. */
    for(i=0;i<38;++i) packets(1,1);
    REQUIRE(!VisionAlign_ControlOwned() && vision_detection_timed_out); /* Exactly half fails. */
    clear_fruit(); camera_green=1;
    frame(1,20); frame(1,20); frame(1,20);
    for(i=0;i<39;++i) packets(3,1);
    REQUIRE(VisionAlign_ControlOwned() && !vision_detection_timed_out && vision_target_region==2);
    /* G=1 counts despite missing pole/number, and old votes expire at each window. */
    for(i=0;i<40;++i) {
        uint8_t yes[6]={0x63,0,0,1,0,0x66}; unsigned j;
        for(j=0;j<6;++j) Camera_Push(&camera,yes[j]);
        now+=50; poll();
        /* The separate real-number timeout remains active at two seconds. */
        if(i<38) REQUIRE(VisionAlign_ControlOwned());
    }
    REQUIRE(!VisionAlign_ControlOwned());
    REQUIRE(!strikes);
    puts("PASS: actual packet majority, exactly-half rejection, G-only evidence and independent number timeout");
    return 0;
}
