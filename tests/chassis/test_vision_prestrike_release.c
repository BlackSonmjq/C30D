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
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    camera_green=1;
    for(i=0;i<30;++i) {
        frame(0,0);
        REQUIRE(vision_pick_phase==PICK_WAIT && vision_target_region==2);
        REQUIRE(dx==0 && dy==0 && dz==0 && !strikes);
    }
    camera_green=1; lidar.mm[ROUTE_LIDAR_BEHIND]=59;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_align.approaching && dx<0 && dy==0 && !strikes);
    lidar.mm[ROUTE_LIDAR_BEHIND]=VISION_ALIGN_REAR_MM;
    for(i=0;i<12;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==1);
    pose.yaw_rad=.05f; poll();
    REQUIRE(manual_heading_locked && manual_heading_target==0 && dz==0);
    tool_busy=0; tool.state=STEPPER_RETURNED; tool.result=MANGO_OK; tool.mango_id=2; poll();
    camera_green=0; frame(0,0);
    REQUIRE(vision_pick_phase==PICK_IDLE && vision_align.dx_hold_speed==0);
    pose.yaw_rad=0; vision_hit_report=0;
    clear_fruit();
    camera_green=1;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    camera_green=0;
    frame(0,0);
    REQUIRE(vision_target_region==2 && VisionAlign_ControlOwned() && !dy && strikes==1);
    for(i=0;i<40;++i) frame(0,0);
    REQUIRE(!vision_target_region && vision_align.phase==VISION_ALIGN_WAIT && strikes==1);
    poll(); REQUIRE(dy>.29f && !vision_align.dx_hold_speed);
    clear_fruit(); camera_green=1;
    for(i=0;i<3;++i) frame(1,20);
    REQUIRE(vision_align.phase==VISION_ALIGN_TRACK);
    /* Keep sending real DX without a fruit number almost to the deadline. */
    for(i=0;i<39;++i) {
        ++camera.data.frames; ++camera.data.green_frames; camera.data.region=0; now+=50; poll();
    }
    REQUIRE(vision_target_region==2 && dy>0);
    frame(0,0);
    REQUIRE(vision_detection_timed_out && !vision_target_region && !vision_align.dx_hold_speed);
    REQUIRE(dx==0 && dy==0 && dz==0 && strikes==1);
    poll(); REQUIRE(dy>.29f && vision_align.phase==VISION_ALIGN_WAIT);
    clear_fruit(); camera_green=1;
    for(i=0;i<6;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_HOLD);
    now+=300; poll();
    REQUIRE(vision_detection_timed_out && !vision_target_region && dx==0 && dy==0);
    REQUIRE(!vision_align.dx_hold_speed && strikes==1);
    puts("PASS: prestrike grace, rear correction, work heading and fruit expiry with remembered DX");
    return 0;
}
