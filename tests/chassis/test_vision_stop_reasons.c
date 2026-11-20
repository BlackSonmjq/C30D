#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{44,40,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *v) { *v=lidar; v->sequence=now; }
int main(void)
{
    unsigned i;
    VisionAlign_Diagnostics d;
    GamePadInterface=&gamepad; PS2_ON_Flag=1; gamepad.RX=0;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(i=0;i<8;++i) frame(1,0);
    VisionAlign_GetDiagnostics(&d);
    REQUIRE(d.stop_reason==VISION_STOP_FRONT_LIMIT && dx==0 && VisionAlign_ControlOwned());
    lidar.mm[ROUTE_LIDAR_FRONT]=100;
    frame(1,0);
    REQUIRE(dx>0 && vision_stop_reason==VISION_STOP_NONE);
    frame(0,0);
    REQUIRE(vision_stop_reason==VISION_STOP_POLE_WAIT && dx==0);
    camera_green=0; frame(0,0);
    REQUIRE(vision_stop_reason==VISION_STOP_GREEN_WAIT && dx==0);
    now+=300; poll();
    VisionAlign_GetDiagnostics(&d);
    REQUIRE(!VisionAlign_ControlOwned() && d.release_reason==VISION_RELEASE_CAMERA_TIMEOUT);
    poll(); REQUIRE(vision_release_reason==VISION_RELEASE_CAMERA_TIMEOUT);
    clear_fruit(); camera_green=1; lidar.mm[ROUTE_LIDAR_BEHIND]=45;
    MOTOR_A.Encoder=.02f;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_release_reason==VISION_RELEASE_NONE && vision_stop_reason==VISION_STOP_WHEELS);
    for(i=0;i<39;++i) {
        uint8_t no[6]={0x63,0,0,0,0,0x66},yes[6]={0x63,1,0,1,2,0x66}; unsigned j,k;
        for(k=0;k<3;++k) for(j=0;j<6;++j) Camera_Push(&camera,no[j]);
        for(j=0;j<6;++j) Camera_Push(&camera,yes[j]);
        now+=50; poll();
    }
    REQUIRE(!VisionAlign_ControlOwned() && vision_release_reason==VISION_RELEASE_GREEN_RATIO);
    clear_fruit(); camera_green=1; MOTOR_A.Encoder=0;
    for(i=0;i<16 && !strikes;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && vision_release_reason==VISION_RELEASE_NONE);
    poll();
    REQUIRE(vision_stop_reason==VISION_STOP_WORK && vision_release_reason==VISION_RELEASE_NONE);
    puts("PASS: front safety, missing pole/green, receive timeout, settling and majority release diagnostics");
    return 0;
}
