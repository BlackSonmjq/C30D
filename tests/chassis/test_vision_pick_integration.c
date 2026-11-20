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
    unsigned id,i,before;
    VisionAlign_Diagnostics diag;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    MOTOR_A.Encoder=MOTOR_B.Encoder=MOTOR_C.Encoder=MOTOR_D.Encoder=0;
    for(id=1;id<=4;++id) {
        /* New target absence while idle rearms the one-fruit latch. */
        if(id==1 || id==3) {camera_green=0;for(i=0;i<25;++i) frame(0,0);camera_green=1;}
        selected_frame(id,20);selected_frame(id,20);selected_frame(id,20);
        REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy>0);
        /* Build the selected REGION with the real camera parser. */
        for(i=0;i<3;++i) {
            uint8_t data[6]={0x63,1,(uint8_t)(i%2?2:-2),1,(uint8_t)id,0x66};unsigned j;
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
        }
        REQUIRE((vision_pick_phase==(id==2?PICK_WAIT:PICK_HOLD)) && vision_park_braking && dx==0 && dy==0 && dz==0 && !tool_busy);
        before=strikes;
        if(id==2) {
            for(i=0;i<5;++i) frame(1,0);
            REQUIRE(strikes==before && vision_pick_phase==PICK_WAIT && vision_hit_report==1);
            vision_hit_report=0;
        }
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
        if(id!=4) {camera_green=1;frame(1,0);}
        tool.mango_id=id;poll();
        REQUIRE(vision_hit_report==id && vision_pick_phase==PICK_REPORT && strikes==before && !tool_busy);
        VisionAlign_GetDiagnostics(&diag); REQUIRE(diag.locked);
        REQUIRE(diag.pick_phase==PICK_REPORT && diag.hit_region==id && diag.hit_count==id);
        poll();REQUIRE(vision_hit_report==id && vision_park_braking);
        if(id==1 || id==3) {
            selected_frame(id+1,20); /* New target is locked before alignment. */
            REQUIRE(vision_pick_phase==PICK_IDLE && VisionAlign_GetPhase()==VISION_ALIGN_ACQUIRE);
            REQUIRE(dx==0 && dy==0 && vision_park_braking && !vision_pick.latched);
            for(i=0;i<10;++i) poll();
            REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_ACQUIRE && strikes==before);
            if(id!=1) vision_hit_report=0;
        } else {
            poll();REQUIRE(vision_pick_phase==PICK_REPORT); /* Cached G=0 cannot exit. */
            REQUIRE(vision_pick_attempts==2);
            camera_green=1;
            for(i=0;i<120;++i) {
                unsigned j;uint8_t data[6]={0x63,1,0,1,(uint8_t)(i%4+1),0x66};
                for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
                now+=50;poll();
                REQUIRE(vision_pick_phase==PICK_IDLE && vision_detection_timed_out && strikes==before);
                REQUIRE(vision_pick_attempts==2);
            }
            clear_fruit();
            REQUIRE(vision_pick_phase==PICK_IDLE && VisionAlign_GetPhase()==VISION_ALIGN_WAIT);
            REQUIRE(dx>.29f && dy==0 && strikes==before && vision_hit_report==id);
            REQUIRE(vision_pick_attempts==0);
            vision_hit_report=0;poll();
            REQUIRE(dx>.29f && !tool_busy);
            VisionAlign_GetDiagnostics(&diag);REQUIRE(!diag.locked);
            frame(0,0);frame(0,0);REQUIRE(!vision_pick.latched);
        }
    }

    /* A failed job that returned safely releases after cooldown without success. */
    camera_green=0;for(i=0;i<25;++i) frame(0,0);camera_green=1;
    frame(1,20);frame(1,20);frame(1,20);
    before=strikes;
    for(i=0;i<65;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==before+1 && submitted_id==2);
    tool.state=STEPPER_RETURNED;tool.mango_id=2;tool.result=1;tool_busy=0;poll();
    REQUIRE(vision_pick_phase==PICK_COOLDOWN && !vision_hit_report && vision_hit_count==4);
    camera_green=0;for(i=0;i<40;++i) frame(0,0);
    REQUIRE(vision_detection_timed_out && vision_pick_phase==PICK_IDLE && dx==0);
    poll();
    REQUIRE(vision_pick_phase==PICK_IDLE && dx>.019f);
    /* No G/REGION: never align or start a mechanism; retain manual control. */
    for(i=0;i<25;++i) frame(0,0);
    before=strikes;
    for(i=0;i<110;++i) {
        unsigned j;uint8_t data[6]={0x63,1,20,0,0,0x66};
        for(j=0;j<6;++j) Camera_Push(&camera,data[j]);now+=50;poll();
        REQUIRE(vision_pick_phase==PICK_IDLE && VisionAlign_GetPhase()==VISION_ALIGN_WAIT && dx>.019f);
    }
    REQUIRE(strikes==before && !vision_hit_report);
    printf("PASS: REGION 1-4 jobs limited to one repeat per green episode; align first, raw region check then stationary 300ms, safe return, fresh G=0 immediate exit and fault lock (%u checks)\n",checks);
    return 0;
}
