#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main

static RouteLidarFrame lidar={{1000,VISION_ALIGN_REAR_MM,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar;frame->sequence=now; }
static void selected_frame(unsigned id, int dx_value)
{
    unsigned j;
    uint8_t data[6]={0x63,1,(uint8_t)dx_value,1,(uint8_t)id,0x66};
    for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
    now+=50;poll();
}


int main(void)
{
    unsigned id,i,before,job;
    static const unsigned regions[3]={1,2,4};
    VisionAlign_Diagnostics diag;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(job=0;job<3;++job) {
        id=regions[job];
        /* New target absence while idle rearms the one-fruit latch. */
        camera_green=0;for(i=0;i<25;++i) frame(0,0);camera_green=1;vision_hit_report=0;
        selected_frame(id,20);selected_frame(id,20);selected_frame(id,20);
        REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy>0);
        /* Build the selected REGION with the real camera parser. */
        for(i=0;i<3;++i) {
            uint8_t data[6]={0x63,1,(uint8_t)(i%2?2:-2),1,(uint8_t)id,0x66};unsigned j;
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
        }
        REQUIRE((vision_pick_phase==PICK_HOLD) && vision_park_braking && dx==0 && dy==0 && dz==0 && !tool_busy);
        before=strikes;
        MOTOR_A.Encoder=.006f;
        for(i=0;i<24;++i) {
            uint8_t data[6]={0x63,1,(uint8_t)(i%2?-1:2),1,(uint8_t)id,0x66};unsigned j;
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
        }
        REQUIRE(strikes==before && (vision_pick_phase==PICK_WAIT) && dx==0);
        MOTOR_A.Encoder=0;
        { uint8_t data[6]={0x63,1,3,1,(uint8_t)id,0x66};unsigned j;
          for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
          now+=50;poll(); REQUIRE(strikes==before && (vision_pick_phase==PICK_WAIT));
          data[2]=2;
          for(i=0;i<2;++i) {
              for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
              now+=50;poll(); REQUIRE(strikes==before);
          }
          { const uint16_t rear_mm[]={0,39,40,80,81,79,VISION_ALIGN_REAR_MM};
            for(i=0;i<sizeof(rear_mm)/sizeof(rear_mm[0]);++i) {
                lidar.mm[ROUTE_LIDAR_BEHIND]=rear_mm[i];
                lidar.fresh=(i!=5);
                for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
                now+=50;poll();
                if(i<6) REQUIRE(strikes==before && vision_pick_phase!=PICK_WORK);
            }
          } }
        MOTOR_A.Encoder=.006f;
        for(i=0;i<12;++i) selected_frame(id,2);
        MOTOR_A.Encoder=0;selected_frame(id,2);
        REQUIRE(vision_pick_phase==PICK_HOLD && strikes==before);
        /* A rolling wheel breaks the consecutive stationary interval. */
        now+=100;MOTOR_A.Encoder=.006f;poll();
        REQUIRE(vision_pick_phase==PICK_WAIT && !vision_hit_report);
        MOTOR_A.Encoder=0;
        for(i=0;i<=VISION_PICK_STOP_HOLD_MS/50;++i) {
            unsigned j;uint8_t data[6]={0x63,1,0,1,(uint8_t)id,0x66};
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
            if(i<VISION_PICK_STOP_HOLD_MS/50) REQUIRE(!vision_hit_report && vision_pick_phase==PICK_HOLD);
        }
        REQUIRE(!vision_hit_report && vision_pick_phase==PICK_WORK && strikes==before+1 && tool_busy && submitted_id==id);
        ++before;
        /* Moving, returning and a latched actuator fault all keep the chassis parked. */
        for(i=0;i<50;++i) {
            now+=50;poll();
            REQUIRE(vision_pick_phase==PICK_WORK && dx==0 && dy==0 && dz==0 && !vision_hit_report);
        }
        camera_green=0;frame(0,0); /* Target loss cannot move a working mechanism. */
        REQUIRE(vision_pick_phase==PICK_WORK && dx==0 && dy==0);
        tool.state=STEPPER_FAULT;tool.result=7;tool_busy=0;poll();
        REQUIRE(vision_pick_phase==PICK_WORK && !vision_hit_report && dx==0);
        tool.state=STEPPER_RETURNED;tool.result=MANGO_OK;tool.mango_id=id==4?1:id+1;poll();
        REQUIRE(vision_pick_phase==PICK_WORK && !vision_hit_report);
        camera_green=1;frame(1,0);
        tool.mango_id=id;poll();
        REQUIRE(vision_hit_report==id && vision_pick_phase==PICK_REPORT && strikes==before && !tool_busy);
        VisionAlign_GetDiagnostics(&diag); REQUIRE(diag.locked);
        REQUIRE(diag.pick_phase==PICK_REPORT && diag.hit_region==id && diag.hit_count==job+1);
        poll();REQUIRE(vision_hit_report==id && vision_park_braking);
        poll();REQUIRE(vision_pick_phase==PICK_REPORT);
        REQUIRE(vision_pick_attempts==1);
        for(i=0;i<120;++i) {
            unsigned j;uint8_t data[6]={0x63,1,0,1,(uint8_t)(i%4+1),0x66};
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
            REQUIRE(vision_pick_phase==PICK_IDLE && vision_detection_timed_out && strikes==before);
            REQUIRE(vision_pick_attempts==1);
        }
        clear_fruit();
        REQUIRE(vision_pick_phase==PICK_IDLE && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
        REQUIRE(dx>.29f && dy==0 && strikes==before && vision_hit_report==id);
        REQUIRE(vision_pick_attempts==(job==2?1U:0U));
        vision_hit_report=0;poll();
        REQUIRE(dx>.29f && !tool_busy);
        VisionAlign_GetDiagnostics(&diag);REQUIRE(!diag.locked);
        frame(0,0);frame(0,0);if(job<2) REQUIRE(!vision_pick.latched);
    }

    /* The boot-wide tree quota rejects a fresh fourth alignment. */
    camera_green=0;for(i=0;i<25;++i) frame(0,0);camera_green=1;
    before=strikes;
    for(i=0;i<65;++i) frame(1,0);
    REQUIRE(strikes==before && vision_boot_attempts==3 && vision_hit_count==3 && vision_boot_alignments==3);
    REQUIRE(vision_pick_phase==PICK_IDLE && !VisionAlign_ControlOwned());
    /* No G/REGION: never align or start a mechanism; retain manual control. */
    for(i=0;i<25;++i) frame(0,0);
    before=strikes;
    for(i=0;i<110;++i) {
        unsigned j;uint8_t data[6]={0x63,1,20,0,0,0x66};
        for(j=0;j<6;++j) Camera_Push(&camera,data[j]);now+=50;poll();
        REQUIRE(vision_pick_phase==PICK_IDLE && VisionAlign_GetPhase()==VISION_ALIGN_WAIT && dx>.019f);
    }
    REQUIRE(strikes==before && !vision_hit_report);
    printf("PASS: REGION 1/2/4 jobs limited to one attempt per green episode; align first, raw region check then stationary 200ms, safe return, fresh G=0 immediate exit and fault lock (%u checks)\n",checks);
    return 0;
}
