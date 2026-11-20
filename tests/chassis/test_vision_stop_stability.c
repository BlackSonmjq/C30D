#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main
static RouteLidarFrame lidar={{1000,90,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
int main(void)
{
    unsigned i;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,20);frame(1,20);frame(1,20);
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(dx<-.11f && fabsf(dx)<=.12001f && dy==0);
    lidar.mm[ROUTE_LIDAR_BEHIND]=54;
    frame(1,0);frame(1,0);frame(1,0);
    REQUIRE(dx<-.035f && dx>=-.0361f);
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=-.06f;
    lidar.mm[ROUTE_LIDAR_BEHIND]=35;
    frame(1,0);
    REQUIRE(dx==0 && dy==0 && dz==0 && vision_park_braking);
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    frame(1,0);
    REQUIRE(dx>0 && fabsf(dx-.04f)<.00001f); /* Resume after residual motion stops. */
    lidar.mm[ROUTE_LIDAR_BEHIND]=VISION_ALIGN_REAR_MM;
    MOTOR_A.Encoder=.006f;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WAIT && strikes==0);
    MOTOR_A.Encoder=0;frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && strikes==0);
    REQUIRE(vision_pick.confirmed>=VISION_STRIKE_CONFIRM_FRAMES);
    frame(1,3);
    REQUIRE(vision_pick_phase==PICK_WAIT && strikes==0 && dx==0 && dy==0);
    REQUIRE(vision_pick.confirmed>=VISION_STRIKE_CONFIRM_FRAMES); /* Same raw number stays confirmed. */
    frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD && strikes==0);
    for(i=0;i<3;++i) {
        frame(1,0);
        REQUIRE(vision_pick_phase==PICK_HOLD && strikes==0);
    }
    frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1 && submitted_id==2);
    printf("PASS: rear approach slows, reverse waits for braking, DX=3 resets the continuous 200ms qualification (%u checks)\n",checks);
    return 0;
}
