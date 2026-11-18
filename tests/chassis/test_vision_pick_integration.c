#define VISION_PICK_ENABLE 1
#define main legacy_alignment_main
#include "align_fixture.inc"
#undef main

static RouteLidarFrame lidar={{1000,50,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=lidar; }

int main(void)
{
    unsigned id,i,before;
    VisionAlign_Diagnostics diag;
    GamePadInterface=&gamepad;PS2_ON_Flag=1;gamepad.LY=254;
    for(id=1;id<=4;++id) {
        /* New target absence while idle rearms the one-fruit latch. */
        for(i=0;i<25;++i) frame(0,0);
        frame(1,20);frame(1,20);frame(1,20);
        REQUIRE(VisionAlign_GetPhase()==VISION_ALIGN_TRACK && dx==0 && dy>0);
        /* Build the selected REGION with the real camera parser. */
        for(i=0;i<3;++i) {
            uint8_t data[6]={0x63,1,(uint8_t)(i%2?2:-2),1,(uint8_t)id,0x66};unsigned j;
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
        }
        REQUIRE((vision_pick_phase==PICK_WAIT) && vision_park_braking && dx==0 && dy==0 && dz==0 && !tool_busy);
        before=strikes;
        chassis_stopped=0;
        for(i=0;i<24;++i) {
            uint8_t data[6]={0x63,1,(uint8_t)(i%2?-1:2),1,(uint8_t)id,0x66};unsigned j;
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
        }
        REQUIRE(strikes==before && (vision_pick_phase==PICK_WAIT) && dx==0);
        chassis_stopped=1;
        { uint8_t data[6]={0x63,1,3,1,(uint8_t)id,0x66};unsigned j;
          for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
          now+=50;poll(); REQUIRE(strikes==before && (vision_pick_phase==PICK_WAIT));
          data[2]=2;
          for(i=0;i<2;++i) {
              for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
              now+=50;poll(); REQUIRE(strikes==before);
          }
          { const uint16_t rear_mm[]={0,39,40,80,81,79,50};
            for(i=0;i<sizeof(rear_mm)/sizeof(rear_mm[0]);++i) {
                lidar.mm[ROUTE_LIDAR_BEHIND]=rear_mm[i];
                lidar.fresh=(i!=5);
                for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
                now+=50;poll();
                if(i<6) REQUIRE(strikes==before && (vision_pick_phase==PICK_WAIT) && vision_park_braking);
            }
          } }
        REQUIRE(vision_pick_phase==PICK_HOLD && strikes==before);
        /* A rolling wheel breaks the consecutive stationary interval. */
        now+=100;chassis_stopped=0;poll();
        REQUIRE(vision_pick_phase==PICK_WAIT && !vision_hit_report);
        chassis_stopped=1;
        for(i=0;i<=20;++i) {
            unsigned j;uint8_t data[6]={0x63,1,0,1,(uint8_t)id,0x66};
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);
            now+=50;poll();
            if(i<20) REQUIRE(!vision_hit_report && vision_pick_phase==PICK_HOLD);
        }
        REQUIRE(!vision_hit_report && vision_pick_phase==PICK_WORK && strikes==before+1 && tool_busy && submitted_id==id);
        ++before;
        /* Moving, returning and a latched actuator fault all keep the chassis parked. */
        for(i=0;i<50;++i) {
            now+=50;poll();
            REQUIRE(vision_pick_phase==PICK_WORK && dx==0 && dy==0 && dz==0 && !vision_hit_report);
        }
        tool.state=STEPPER_FAULT;tool.result=7;poll();
        REQUIRE(vision_pick_phase==PICK_WORK && !vision_hit_report);
        tool.state=STEPPER_RETURNED;tool.result=MANGO_OK;tool.mango_id=id;
        tool_busy=0;poll();
        REQUIRE(vision_hit_report==id && vision_pick_phase==PICK_REPORT && strikes==before && !tool_busy);
        VisionAlign_GetDiagnostics(&diag); REQUIRE(diag.locked);
        REQUIRE(diag.pick_phase==PICK_REPORT && diag.hit_region==id && diag.hit_count==id);
        poll();REQUIRE(vision_hit_report==id && vision_park_braking);
        vision_hit_report=0;poll(); /* DATA_task has sent the line. */
        REQUIRE(vision_pick_phase==PICK_COOLDOWN);
        for(i=0;i<39;++i) frame(0,0);
        REQUIRE(vision_pick_phase==PICK_COOLDOWN && dx>.019f && vision_pick.latched);
        frame(1,20);REQUIRE(vision_pick_phase==PICK_IDLE && dx>.019f);
        poll();REQUIRE(!vision_hit_report && strikes==before && vision_pick.latched);
        for(i=0;i<5;++i) {
            uint8_t data[6]={0x63,1,0,1,(uint8_t)id,0x66};unsigned j;
            for(j=0;j<6;++j) Camera_Push(&camera,data[j]);now+=50;poll();
        }
        REQUIRE(strikes==before && !vision_hit_report);
    }
    /* A failed job that returned safely releases after cooldown without success. */
    for(i=0;i<25;++i) frame(0,0);
    frame(1,20);frame(1,20);frame(1,20);
    before=strikes;
    for(i=0;i<35;++i) frame(1,0);
    REQUIRE(vision_pick_phase==PICK_WORK && strikes==before+1 && submitted_id==2);
    tool.state=STEPPER_RETURNED;tool.mango_id=2;tool.result=1;tool_busy=0;poll();
    REQUIRE(vision_pick_phase==PICK_COOLDOWN && !vision_hit_report && vision_hit_count==4);
    for(i=0;i<40;++i) frame(0,0);
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
    printf("PASS: held-stick alignment -> 1s stop -> REGION 1-4 physical job -> successful return -> report -> 2s camera cooldown -> manual release (%u checks)\n",checks);
    return 0;
}
