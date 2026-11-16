#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include "balance.h"
#include "route_nav.c"
#include "route_io.c"
#include "my_chassis_task.c"
#include "stm32f4xx_usart.h"

TestSystem SysVal={CONTROL_DELAY};
TestSmooth smooth_control;
TestMotor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
TestImu imu;
uint8_t Car_Mode=Mec_Car_V550,EN=1,Flag_Stop;
float Voltage=12.0f;
volatile uint8_t Route_TaskCreated=1;
#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition); exit(1); } } while (0)
static uint32_t clock_ms;
static uint8_t tool_busy;
static float driven_x,driven_y;
static uint32_t integration_at,tool_end;
static uint8_t injected,work_starts;
static jmp_buf integration_done;
static StepperTask_Data simulated_tool;
USART_TypeDef route_test_usart1,route_test_usart2,route_test_usart3;
static uint8_t tx_interrupt, tx_ready=1, rx_pending;
static uint8_t transmitted[ROUTE_VOFA_FRAME_BYTES * 2U];
static unsigned transmitted_count;
void USART_ITConfig(USART_TypeDef *uart,uint16_t interrupt,int enabled){
    assert(uart==USART2&&interrupt==USART_IT_TXE);tx_interrupt=(uint8_t)enabled;
}
int USART_GetITStatus(USART_TypeDef *uart,uint16_t interrupt){
    assert(uart==USART2);
    if(interrupt==USART_IT_RXNE)return rx_pending;
    assert(interrupt==USART_IT_TXE);return tx_interrupt&&tx_ready;
}
void USART_SendData(USART_TypeDef *uart,uint16_t byte){
    assert(uart==USART2&&transmitted_count<sizeof(transmitted));
    transmitted[transmitted_count++]=(uint8_t)byte;
}
/* 实际 USART2 中断/文本发送函数由 runner 提取，外围 APP 回调仅记录是否被调用。 */
static unsigned legacy_rx_calls;
static uint8_t PS2_ON_Flag,Remote_ON_Flag,APP_ON_Flag,CAN_ON_Flag,Usart1_ON_Flag,Usart5_ON_Flag;
static uint8_t Turn_Flag,Flag_Direction,Flag_Left,Flag_Right,PID_Send,FlashWriteFlag,Allow_Recharge;
static int Check,RC_Velocity,Velocity_KP,Velocity_KI,LineDiffParam;
static struct { uint8_t Usart2_check_dir; } check_parameter;
static uint8_t ATCommandFeedBack_JDY33(int value){(void)value;return 0;}
static uint8_t ATCommandFeedBack_BT04A(int value){(void)value;return 0;}
static void _System_Reset_(int value){(void)value;++legacy_rx_calls;}
static void RobotControl_SetDebugLevel(int value){(void)value;}
#include "legacy_usart2.c"
TickType_t xTaskGetTickCount(void){return clock_ms;}
uint8_t StepperTask_IsBusy(void){return tool_busy;}
void Drive_Motor(float x,float y,float z){(void)z;driven_x=x;driven_y=y;}
uint8_t StepperTask_RequestMango(uint8_t id){
    assert(id==1&&!tool_busy);
    ++work_starts;tool_busy=1;tool_end=clock_ms+200;
    simulated_tool.state=1;simulated_tool.result=0;
    return 1;
}
void StepperTask_GetData(StepperTask_Data *data){*data=simulated_tool;}
void vTaskDelayUntil(TickType_t *wake,TickType_t period){
    for(uint32_t step=0;step<period;step+=10){
        clock_ms+=10;
        ChassisOdom_UpdateDelta(driven_x*10.0f,driven_y*10.0f,0,clock_ms,1);
        RouteIO_ApplyDrive();
        if((clock_ms-integration_at)%100==0)RouteIO_PublishLidar(80,75,120,120);
        if(!injected&&clock_ms-integration_at>=300){
            assert(RouteIO_QueueWorkFromISR(1));
            injected=1;EN=0;
        }
        if(clock_ms-integration_at>=600)EN=1;
        if(tool_busy&&clock_ms>=tool_end){
            tool_busy=0;simulated_tool.state=STEPPER_RETURNED;simulated_tool.result=0;
        }
    }
    *wake=clock_ms;
    if(clock_ms-integration_at>=1500)longjmp(integration_done,1);
}


static void position(RouteController *c,RouteInputs *in,float progress)
{
    RouteMotion motion=RoutePlan_Get(c->status.target_point)->motion;
    in->odom.x_mm=c->start_x;
    in->odom.y_mm=c->start_y;
    if(motion==ROUTE_BACK)in->odom.x_mm-=progress;
    else if(motion==ROUTE_FORWARD)in->odom.x_mm+=progress;
    else if(motion==ROUTE_LEFT)in->odom.y_mm+=progress;
    else in->odom.y_mm-=progress;
}
static void fixture(RouteController *c,RouteInputs *in,uint8_t point)
{
    memset(in,0,sizeof(*in));
    in->now_ms=10000;
    in->odom.valid=in->ready=in->chassis_stopped=1;
    in->odom.tick_ms=in->now_ms;
    in->lidar.fresh=1;
    RouteNav_Init(c);
    RouteNav_Start(c,in);
    c->status.target_point=point;
    begin_segment(c,in);
}
static void frame_with_aux(RouteController *c,RouteInputs *in,RouteOutput *out,
                           float progress,uint16_t mm,int32_t aux_override)
{
    const RouteSegment *s=RoutePlan_Get(c->status.target_point);
    const RouteAuxReference *a=RoutePlan_AuxReference(c->status.target_point);
    in->now_ms+=100;
    in->odom.tick_ms=in->now_ms;
    position(c,in,progress);
    in->lidar.mm[0]=Front_Lidar_Center;
    in->lidar.mm[1]=Behind_Lidar_Center;
    in->lidar.mm[2]=in->lidar.mm[3]=120;
    if(s->trigger<ROUTE_LIDAR_COUNT)in->lidar.mm[s->trigger]=mm;
    if(a){
        float distance=a->distance_increases?a->mark.target_mm-(s->length_mm-progress):
                                              a->mark.target_mm+(s->length_mm-progress);
        if(distance<1)distance=1;
        in->lidar.mm[a->sensor]=aux_override>=0?(uint16_t)aux_override:(uint16_t)lroundf(distance);
    }
    ++in->lidar.sequence;
    in->lidar.fresh=1;
    RouteNav_Step(c,in,out);
}
static void frame(RouteController *c,RouteInputs *in,RouteOutput *out,float progress,uint16_t mm)
{
    frame_with_aux(c,in,out,progress,mm,-1);
}
/* Build a physically continuous approach, rather than teleporting the laser onto a target. */
static void prime_wall(RouteController *c,RouteInputs *in,RouteOutput *out,float progress,uint16_t mm)
{
    const RouteSegment *s=RoutePlan_Get(c->status.target_point);
    /* Two median warmup frames consume 10 mm of the initial approach. */
    int start=(int)(ceilf(ROUTE_WALL_CAPTURE_PROGRESS_MM/10.0f)*10.0f)+30;
    for(int offset=start;offset>=0;offset-=10)
        frame(c,in,out,progress-offset,(uint16_t)(s->distance_increases?mm-offset:mm+offset));
    for(unsigned n=0;n<ROUTE_CAPTURE_FRAMES;n++)frame(c,in,out,progress,mm);
    if(!c->status.wall_locked||c->status.phase==ROUTE_FAULT)
        printf("Prime P%u at %.0f range %u: %s/%s C%u reason%u\n",c->status.target_point,
               progress,mm,RouteNav_PhaseName(c->status.phase),RouteNav_ErrorName(c->status.reason),
               c->status.capture_confirmed,c->status.reject_reason);
    assert(c->status.wall_locked&&c->status.phase!=ROUTE_FAULT);
}
static void test_frames_and_spikes(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,2);
    for(int n=0;n<4;n++)frame(&c,&in,&out,1000,2300);
    assert(c.status.target_point==2&&c.status.phase==ROUTE_ODOM_RUN&&out.vy_mps>0);
    assert(fabsf(c.status.gate_mm-1733.42f)<0.01f); /* Independent of the 500 mm capture offset. */
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2200,520);
    assert(c.status.wall_locked&&c.status.phase==ROUTE_LASER_NEAR);
    frame(&c,&in,&out,2200,120);
    assert(c.status.target_point==2&&c.status.confirmed==0&&out.vy_mps>0);
    frame(&c,&in,&out,2200,120);
    assert(c.status.target_point==2&&c.status.confirmed==0);
    for(int n=0;n<7;n++)frame(&c,&in,&out,2200,520);
    assert(c.status.target_point==2&&c.status.wall_locked&&c.status.phase==ROUTE_LASER_NEAR);

    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2550,170);
    for(int progress=2560;progress<=2590;progress+=10)
        frame(&c,&in,&out,(float)progress,(uint16_t)(2720-progress));
    frame(&c,&in,&out,2590,130);
    assert(c.status.confirmed==1&&c.status.target_point==2);
    for(int n=0;n<4;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;
        RouteNav_Step(&c,&in,&out);
        assert(c.status.confirmed==1&&c.status.target_point==2);
    }
    frame(&c,&in,&out,2590,130);
    assert(c.status.at_point==2&&c.status.target_point==3&&out.vx_mps==0&&out.vy_mps==0);

    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2580,140);
    for(int progress=2600;progress<=2700;progress+=20)
        frame(&c,&in,&out,(float)progress,(uint16_t)(2720-progress));
    for(unsigned n=0;n<4U&&c.status.phase!=ROUTE_FAULT;n++)
        frame(&c,&in,&out,2700,20); /* Stable 100 mm overrun exceeds the 80 mm recovery bound. */
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_OVERRUN);
    assert(c.status.stop_flags==ROUTE_STOP_RECOVER_LIMIT&&out.vx_mps==0&&out.vy_mps==0);
    puts("Fresh-frame capture/arrival, independent gate, double-spike rejection and large-overrun stop passed.");
}

static void test_adaptive_endpoint_and_recovery(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    /* P06 photo: endpoint offset about 462 mm. Crossing the old 2780 mm cap must keep moving. */
    fixture(&c,&in,6);
    prime_wall(&c,&in,&out,2000,1182);
    assert(c.status.wall_locked&&fabsf(c.status.estimated_end_mm-3062)<0.01f);
    for(int progress=2020;progress<=2780;progress+=20)
        frame(&c,&in,&out,(float)progress,(uint16_t)(120+3062-progress));
    frame(&c,&in,&out,2781,401);
    assert(c.status.phase==ROUTE_LASER_NEAR&&out.vy_mps>0&&c.status.target_point==6);
    assert(c.status.estimated_end_mm>3000&&c.status.control_remaining_mm>250);
    assert(c.status.odom_remaining_mm<0);
    for(int progress=2800;progress<=3040;progress+=20)
        frame(&c,&in,&out,(float)progress,(uint16_t)(120+3062-progress));
    for(int n=0;n<5&&c.status.target_point==6;n++)frame(&c,&in,&out,3052,130);
    assert(c.status.at_point==6&&c.status.target_point==7);

    /* A laser-confirmed endpoint before the nominal endpoint is valid after capture, too. */
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2150,270);
    for(int progress=2170;progress<=2290;progress+=20)
        frame(&c,&in,&out,(float)progress,(uint16_t)(120+2300-progress));
    for(int n=0;n<5&&c.status.target_point==2;n++)frame(&c,&in,&out,2290,130);
    assert(c.status.at_point==2&&c.status.target_point==3); /* Nominal remaining was 310 > old 150. */

    /* P03 photo: measured front=810, target=780, odometry=751. A bounded forward correction is needed. */
    fixture(&c,&in,3);
    prime_wall(&c,&in,&out,690,749);
    frame(&c,&in,&out,711,770);
    frame(&c,&in,&out,731,790);
    for(int n=0;n<4&&c.status.phase!=ROUTE_RECOVER;n++)frame(&c,&in,&out,751,810);
    assert(c.status.phase==ROUTE_RECOVER&&out.vx_mps>0&&out.vx_mps<=ROUTE_RECOVER_MPS);
    assert(c.status.motion==ROUTE_FORWARD&&c.status.target_point==3);
    frame(&c,&in,&out,746,805);
    for(int n=0;n<5&&c.status.target_point==3;n++)frame(&c,&in,&out,731,790);
    assert(c.status.at_point==3&&c.status.target_point==4&&out.vx_mps==0&&out.vy_mps==0);

    /* Late decreasing-distance endpoint uses the opposite lateral direction for recovery. */
    fixture(&c,&in,6);
    prime_wall(&c,&in,&out,2570,150);
    frame(&c,&in,&out,2590,130);
    frame(&c,&in,&out,2610,110);
    for(int n=0;n<4&&c.status.phase!=ROUTE_RECOVER;n++)frame(&c,&in,&out,2630,90);
    assert(c.status.phase==ROUTE_RECOVER&&out.vy_mps<0&&c.status.motion==ROUTE_RIGHT);
    in.now_ms+=ROUTE_RECOVER_TIMEOUT_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_RECOVER_TIME);
    assert(out.vx_mps==0&&out.vy_mps==0);

    fixture(&c,&in,6);
    for(int n=0;n<7;n++)frame(&c,&in,&out,ROUTE_LANE_LENGTH_MM+ROUTE_WALL_MAX_EXTRA_MM+1,400);
    assert(c.status.phase!=ROUTE_FAULT&&c.status.odom_untrusted&&out.vy_mps>0);
    assert(c.status.stop_flags==0&&out.vy_mps<=ROUTE_SLOW_MPS);
    fixture(&c,&in,1);
    frame(&c,&in,&out,ROUTE_ENTRY_LENGTH_MM+ROUTE_MAX_OVERRUN_MM+1,0);
    assert(c.status.phase==ROUTE_FAULT&&c.status.stop_flags==ROUTE_STOP_TRAVEL_LIMIT);
    puts("Adaptive endpoints, recovery in both directions, timeout, wall travel warning and odometry-only hard bound passed.");
}
static void test_closed_loop_wall_bias(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    const float scales[]={1.2f,0.4f,3.0f,0.0f,-1.0f};
    for(unsigned mode=0;mode<sizeof(scales)/sizeof(scales[0]);++mode){
      float actual_progress=0.0f;
      fixture(&c,&in,6);
      for(unsigned n=0;n<3500&&c.status.target_point==6&&c.status.phase!=ROUTE_FAULT;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;
        if(n%5U==0U){
            float range=120.0f+2600.0f-actual_progress;
            ++in.lidar.sequence;
            in.lidar.mm[0]=Front_Lidar_Center;in.lidar.mm[1]=Behind_Lidar_Center;
            in.lidar.mm[2]=(uint16_t)lroundf(range);in.lidar.mm[3]=120;
        }
        RouteNav_Step(&c,&in,&out);
        actual_progress+=out.vy_mps*20.0f;
        in.odom.x_mm+=out.vx_mps*20.0f;
        float scale=scales[mode]<0?(actual_progress<1300?2.5f:0.3f):scales[mode];
        in.odom.y_mm+=out.vy_mps*20.0f*scale;
        if(out.correct_axis==0)in.odom.x_mm=out.correct_position_mm;
        if(out.correct_axis==1)in.odom.y_mm=out.correct_position_mm;
      }
      if(c.status.phase==ROUTE_FAULT)printf("Slip mode %u failed: %s at actual %.1f / encoder %.1f\n",mode,RouteNav_ErrorName(c.status.reason),actual_progress,c.status.progress_mm);
      assert(c.status.at_point==6&&c.status.target_point==7);
      assert(fabsf(actual_progress-2600.0f)<=16.0f);
      /* A confirmed landmark starts the next segment at zero and clears its trust warning. */
      assert(c.status.progress_mm==0&&c.status.odom_untrusted==0);
    }

    /* The normal 20 ms control cycles between 100 ms CAN frames must not pulse cruise/slow. */
    fixture(&c,&in,6);
    prime_wall(&c,&in,&out,1800,1100);
    assert(out.vy_mps>ROUTE_SLOW_MPS);
    {
        float speed=out.vy_mps;
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;
        RouteNav_Step(&c,&in,&out);
        assert(fabsf(out.vy_mps-speed)<0.001f);
        in.now_ms+=ROUTE_WALL_PREDICT_MS/2U;in.odom.tick_ms=in.now_ms;
        RouteNav_Step(&c,&in,&out);
        assert(out.vy_mps>0&&out.vy_mps<=ROUTE_SLOW_MPS);
    }

    /* Without another laser frame, an encoder prediction alone may stop, but never switch or reverse. */
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2550,170);
    position(&c,&in,2595);in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_CONFIRM&&c.status.confirmed==0&&out.vy_mps==0);
    position(&c,&in,2620);in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.target_point==2&&c.status.phase==ROUTE_CONFIRM&&out.vy_mps==0);
    puts("20%/60%/200%, frozen and changing wheel slip converge by laser; segment reset and prediction-only guards passed.");
}

static void test_waits_and_edges(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2200,520);
    in.lidar.fresh=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase!=ROUTE_WAIT_LIDAR&&out.vy_mps>0); /* Bounded prediction first. */
    in.now_ms+=ROUTE_WALL_PREDICT_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_WAIT_LIDAR);
    assert(c.status.stop_flags==ROUTE_STOP_NO_LIDAR);
    in.now_ms+=ROUTE_SENSOR_WAIT_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_LIDAR);

    fixture(&c,&in,5);
    for(int n=0;n<4;n++)frame(&c,&in,&out,300+n*5,120);
    assert(c.edge_armed);
    for(int n=0;n<4;n++)frame(&c,&in,&out,350+n*48,120);
    for(int n=0;n<4;n++)frame(&c,&in,&out,510+n*6,1500);
    assert(c.edge_found&&c.edge_goal>690&&c.edge_goal<750);
    for(int progress=540;progress<690;progress+=20)frame(&c,&in,&out,(float)progress,1500);
    for(unsigned n=0;n<8U&&c.status.target_point==5;n++)frame(&c,&in,&out,690,1500);
    assert(c.status.at_point==5&&c.status.target_point==6);

    /* Two bad far samples followed by a recovered raw sample must not latch an edge. */
    fixture(&c,&in,5);
    for(int n=0;n<4;n++)frame(&c,&in,&out,300,120);
    frame(&c,&in,&out,480,1500);
    frame(&c,&in,&out,480,1500);
    frame(&c,&in,&out,480,120);
    assert(c.edge_armed&&!c.edge_found&&c.far_count==0);

    fixture(&c,&in,9);
    for(int n=0;n<4;n++)frame(&c,&in,&out,300,120);
    assert(!c.edge_armed);
    for(int n=0;n<4;n++)frame(&c,&in,&out,320,320);
    assert(c.edge_armed);

    fixture(&c,&in,2);
    in.now_ms+=ROUTE_WALL_NO_PROGRESS_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_STALL);
    puts("Bounded sensor waits, normal/short ridge references and no-progress detection passed.");
}
static void prime_edge(RouteController *c,RouteInputs *in,RouteOutput *out,uint8_t point)
{
    fixture(c,in,point);
    const RouteSegment *s=RoutePlan_Get(point);
    for(unsigned n=0;n<5U;n++)frame(c,in,out,300.0f+20.0f*n,s->mark.target_mm);
    for(unsigned n=0;n<4U;n++)frame(c,in,out,400.0f+30.0f*n,s->mark.target_mm);
    frame(c,in,out,520,1500);
    float anchor=c->edge_candidate;
    uint8_t far_count=c->far_count;
    for(unsigned n=0;n<4U;n++){
        in->now_ms+=20;in->odom.tick_ms=in->now_ms;RouteNav_Step(c,in,out);
        assert(c->far_count==far_count&&!c->edge_found);
    }
    frame(c,in,out,540,1500);frame(c,in,out,560,1500);
    assert(c->edge_found&&c->status.edge_state==2&&c->auxiliary.locked);
    assert(fabsf(anchor-505.0f)<.001f&&c->edge_candidate==anchor);
    assert(c->status.edge_last_near_mm==490&&c->status.edge_first_far_mm==520);
    assert(c->status.edge_uncertainty_mm==15&&c->edge_goal==anchor+s->edge_to_center_mm);
}

static void test_edge_auxiliary_guards(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    assert(RoutePlan_Get(5)->edge_to_center_mm==200&&RoutePlan_Get(7)->edge_to_center_mm==200&&
           RoutePlan_Get(9)->edge_to_center_mm==200); /* Geometric calibration is preserved. */
    for(uint8_t point=5;point<=9;point+=2){
        prime_edge(&c,&in,&out,point);
        const RouteAuxReference *a=RoutePlan_AuxReference(point);
        assert(c.status.control_source==ROUTE_CONTROL_EDGE_AUX&&c.status.aux_target_mm==a->mark.target_mm);
        float last_cap=c.status.edge_speed_cap_mps;
        in.chassis_stopped=0;
        for(unsigned progress=580;progress<=680;progress+=20){
            frame(&c,&in,&out,(float)progress,0); /* The departed ridge need not remain visible. */
            assert(c.status.edge_speed_cap_mps<=last_cap+.00001f);last_cap=c.status.edge_speed_cap_mps;
        }
        for(unsigned n=0;n<8U;n++){
            frame(&c,&in,&out,690,0);
            assert(c.status.target_point==point); /* Range match cannot switch while wheels are moving. */
        }
        assert(c.status.phase==ROUTE_WAIT_LIDAR&&!c.auxiliary.near_verified&&out.vx_mps==0&&out.vy_mps==0);
        in.chassis_stopped=1;
        for(unsigned n=0;n<7U&&c.status.target_point==point;n++)frame(&c,&in,&out,690,0);
        assert(c.status.at_point==point&&c.status.target_point==point+1U);
    }

    /* A candidate cannot span a communication gap. */
    fixture(&c,&in,5);
    for(unsigned n=0;n<7U;n++)frame(&c,&in,&out,300.0f+20.0f*n,120);
    frame(&c,&in,&out,440,1500);assert(c.edge_candidate_valid);
    in.now_ms+=ROUTE_LIDAR_TTL_MS;
    frame(&c,&in,&out,460,1500);
    assert(!c.edge_found&&!c.edge_armed&&!c.edge_candidate_valid);

    /* A near/far edge alone cannot complete the odometry compensation without the independent beam. */
    fixture(&c,&in,5);
    for(unsigned n=0;n<8U;n++)frame_with_aux(&c,&in,&out,300.0f+20.0f*n,120,65535);
    for(unsigned n=0;n<5U;n++)frame_with_aux(&c,&in,&out,460.0f+10.0f*n,1500,65535);
    for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT;n++){
        frame_with_aux(&c,&in,&out,700,1500,65535);
        assert(c.status.target_point==5&&out.vx_mps==0&&out.vy_mps==0);
    }
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_LIDAR);

    /* Frozen false-near auxiliary data must not become an arrival or a reverse command. */
    prime_edge(&c,&in,&out,5);
    const RouteAuxReference *a=RoutePlan_AuxReference(5);
    float goal=c.status.estimated_end_mm;uint32_t trusted=c.auxiliary.trusted_at;
    for(unsigned n=0;n<9U;n++){
        frame_with_aux(&c,&in,&out,560,1500,a->mark.target_mm-100U);
        assert(c.status.target_point==5&&!c.recovering&&!c.status.confirmed);
        assert(c.status.estimated_end_mm==goal&&c.auxiliary.trusted_at==trusted);
    }
    assert(c.status.phase==ROUTE_WAIT_LIDAR&&out.vx_mps==0);
    puts("All edge points backdate the event, require independent fresh range and standstill; gaps and false-near data cannot finish.");
}

static void test_edge_wait_and_pause(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,5);float actual=0;
    const RouteAuxReference *a=RoutePlan_AuxReference(5);
    for(unsigned n=0;n<1500U&&c.status.phase!=ROUTE_FAULT;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;position(&c,&in,actual);
        if(n%5U==0){
            ++in.lidar.sequence;
            in.lidar.mm[0]=Front_Lidar_Center;in.lidar.mm[1]=(uint16_t)lroundf(a->mark.target_mm+700-actual);
            in.lidar.mm[2]=1800;in.lidar.mm[3]=120; /* No near ridge event. */
        }
        RouteNav_Step(&c,&in,&out);actual+=-out.vx_mps*20;
        assert(c.status.target_point==5&&!c.edge_found);
    }
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);
    assert(actual>650&&actual<700&&out.vx_mps==0);

    prime_edge(&c,&in,&out,5);
    in.work_request=1;RouteNav_Step(&c,&in,&out);assert(c.status.phase==ROUTE_WORK_STOP);
    in.work_request=0;in.now_ms+=ROUTE_TOOL_SETTLE_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);assert(out.start_work==1);
    in.tool_busy=1;in.now_ms+=ROUTE_WALL_NO_PROGRESS_MS+1000U;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);assert(c.status.phase==ROUTE_WAIT_TOOL);
    in.tool_busy=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
    assert(out.work_finished);
    for(unsigned n=0;n<10U;n++){
        frame(&c,&in,&out,560,0);assert(c.status.phase!=ROUTE_FAULT);
    }
    assert(c.status.aux_confidence==ROUTE_CONF_TRUSTED&&c.status.phase==ROUTE_LASER_NEAR);

    /* A real auxiliary-range overrun retains the original four-second recovery bound. */
    in.chassis_stopped=0;
    for(unsigned progress=580;progress<=760;progress+=20)frame(&c,&in,&out,(float)progress,0);
    in.chassis_stopped=1;
    for(unsigned n=0;n<8U&&!c.recovering;n++)frame(&c,&in,&out,760,0);
    assert(c.recovering&&out.vx_mps>0&&out.vx_mps<=ROUTE_RECOVER_MPS);
    in.now_ms+=ROUTE_RECOVER_TIMEOUT_MS;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_RECOVER_TIME&&out.vx_mps==0);
    puts("Missing ridge stops at the auxiliary target; tool pauses do not consume optical progress; true recovery still times out.");
}

static uint16_t aux_at_remaining(uint8_t point,int remaining)
{
    const RouteAuxReference *a=RoutePlan_AuxReference(point);
    return (uint16_t)(a->distance_increases?a->mark.target_mm-remaining:a->mark.target_mm+remaining);
}

static void prime_edge_outage(RouteController *c,RouteInputs *in,RouteOutput *out,uint8_t point)
{
    prime_edge(c,in,out,point);
    in->chassis_stopped=0;
    for(unsigned n=0;n<5U;n++)frame_with_aux(c,in,out,560.0f+10*n,1500,65535);
    assert(c->status.phase==ROUTE_WAIT_LIDAR&&c->status.stop_flags==ROUTE_STOP_NO_LIDAR);
    assert(c->edge_found&&c->auxiliary.locked&&out->vx_mps==0);
    in->chassis_stopped=1;
}

static void test_edge_stationary_resume(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    for(uint8_t point=5;point<=9;point+=2){
        prime_edge_outage(&c,&in,&out,point);
        float goal=c.status.estimated_end_mm;uint32_t trusted=c.auxiliary.trusted_at;
        uint16_t near=aux_at_remaining(point,60);
        for(unsigned n=0;n<3U;n++){
            frame_with_aux(&c,&in,&out,660,1500,near);
            assert(c.auxiliary.trusted_at==trusted&&c.status.estimated_end_mm==goal&&out.vx_mps==0);
        }
        unsigned good=c.status.aux_good_frames;
        for(unsigned n=0;n<5U;n++){
            in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
            assert(c.status.aux_good_frames==good&&c.auxiliary.trusted_at==trusted&&out.vx_mps==0);
        }
        for(unsigned n=0;n<3U;n++)frame_with_aux(&c,&in,&out,660,1500,near);
        assert(c.status.aux_confidence==ROUTE_CONF_TRUSTED&&c.auxiliary.trusted_at>trusted);
        assert(c.status.control_remaining_mm==60&&c.status.target_point==point&&!c.status.confirmed);
        assert(out.vx_mps<0&&fabsf(out.vx_mps)<=ROUTE_EDGE_AUX_RECHECK_MPS);
    }

    /* Four rapid new packets alone are insufficient, and time with only repeats cannot accept. */
    prime_edge_outage(&c,&in,&out,5);
    uint32_t rapid_trusted=c.auxiliary.trusted_at;
    for(unsigned n=0;n<6U;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;position(&c,&in,660);
        in.lidar.mm[ROUTE_LIDAR_BEHIND]=aux_at_remaining(5,60);++in.lidar.sequence;
        RouteNav_Step(&c,&in,&out);
        assert(c.auxiliary.trusted_at==rapid_trusted&&out.vx_mps==0);
    }
    for(unsigned n=0;n<10U;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
        assert(c.auxiliary.trusted_at==rapid_trusted&&out.vx_mps==0);
    }
    in.now_ms+=20;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;RouteNav_Step(&c,&in,&out);
    assert(c.auxiliary.trusted_at>rapid_trusted&&out.vx_mps<0);

    /* A short EN pause cannot bridge the final pre-pause validation frame. */
    prime_edge_outage(&c,&in,&out,5);
    uint32_t before_pause=c.auxiliary.trusted_at;
    for(unsigned n=0;n<5U;n++)frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,60));
    assert(c.status.aux_good_frames==3&&c.auxiliary.trusted_at==before_pause);
    in.ready=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_WAIT_READY);
    in.ready=1;in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
    frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,60));
    assert(c.status.aux_good_frames==1&&c.auxiliary.trusted_at==before_pause&&out.vx_mps==0);
    for(unsigned n=0;n<3U;n++)frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,60));
    assert(c.status.aux_confidence==ROUTE_CONF_TRUSTED&&c.auxiliary.trusted_at>before_pause);

    /* Movement or a range wobble resets the separate standstill validation. */
    for(unsigned mode=0;mode<2U;mode++){
        prime_edge_outage(&c,&in,&out,5);
        uint32_t trusted=c.auxiliary.trusted_at;float goal=c.status.estimated_end_mm;
        in.chassis_stopped=(uint8_t)(mode!=0U);
        for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT;n++){
            uint16_t mm=aux_at_remaining(5,mode&&n%2U?70:40);
            frame_with_aux(&c,&in,&out,660,1500,mm);
            assert(c.auxiliary.trusted_at==trusted&&c.status.estimated_end_mm==goal);
            assert(c.status.target_point==5&&!c.recovering&&out.vx_mps==0);
        }
        assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);
    }

    /* A missing ridge and a reference never acquired before cannot use the shortcut. */
    for(unsigned mode=0;mode<2U;mode++){
        fixture(&c,&in,5);
        if(mode==0U){
            for(unsigned n=0;n<12U;n++)frame(&c,&in,&out,300.0f+20*n,120);
            assert(c.auxiliary.locked&&!c.edge_found);
            for(unsigned n=0;n<5U;n++)frame_with_aux(&c,&in,&out,560,120,65535);
        }
        uint32_t trusted=c.auxiliary.trusted_at;
        for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT;n++){
            frame_with_aux(&c,&in,&out,660,120,aux_at_remaining(5,60));
            assert(c.status.target_point==5&&c.auxiliary.trusted_at==trusted&&!c.recovering);
            assert(out.vx_mps==0);
        }
        assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);
    }

    /* A newly stable echo beyond the target cannot initiate reverse recovery. */
    prime_edge_outage(&c,&in,&out,5);
    uint32_t trusted=c.auxiliary.trusted_at;
    for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT;n++){
        frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,-50));
        assert(c.auxiliary.trusted_at==trusted&&!c.recovering&&!c.status.confirmed&&out.vx_mps==0);
    }
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);

    /* Revalidating an in-band reference still requires the final fresh arrival/settle handshake. */
    prime_edge_outage(&c,&in,&out,5);trusted=c.auxiliary.trusted_at;
    for(unsigned n=0;n<6U;n++)frame_with_aux(&c,&in,&out,700,1500,aux_at_remaining(5,0));
    assert(c.auxiliary.trusted_at>trusted&&c.status.phase==ROUTE_CONFIRM&&c.status.target_point==5);
    assert(c.status.confirmed==1&&out.vx_mps==0);
    for(unsigned n=0;n<8U;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
        assert(c.status.target_point==5&&c.status.confirmed==1);
    }
    in.now_ms+=20;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;RouteNav_Step(&c,&in,&out);
    assert(c.status.target_point==6&&c.status.at_point==5);
    puts("All three edge references resume only after stable fresh standstill data; missing context, moving wheels, oscillations, repeats and false overrun remain blocked.");
}

static void test_edge_closed_loop_slip(void)
{
    const float scales[]={1.2f,0.4f,0.0f};
    for(uint8_t point=5;point<=9;point+=2){
      for(unsigned mode=0;mode<3U;mode++){
        RouteController c;RouteInputs in;RouteOutput out;float actual=0;
        fixture(&c,&in,point);
        const RouteSegment *s=RoutePlan_Get(point);const RouteAuxReference *a=RoutePlan_AuxReference(point);
        float cap=ROUTE_CRUISE_MPS;
        for(unsigned n=0;n<3500U&&c.status.target_point==point&&c.status.phase!=ROUTE_FAULT;n++){
            in.now_ms+=20;in.odom.tick_ms=in.now_ms;
            position(&c,&in,actual*scales[mode]);
            in.chassis_stopped=1; /* Ideal zero-inertia test model. */
            if(n%5U==0){
                ++in.lidar.sequence;
                in.lidar.mm[0]=Front_Lidar_Center;in.lidar.mm[1]=Behind_Lidar_Center;
                in.lidar.mm[2]=in.lidar.mm[3]=120;
                in.lidar.mm[s->trigger]=(actual>=200&&actual<=500)?s->mark.target_mm:1800;
                float distance=a->distance_increases?a->mark.target_mm-(700-actual):a->mark.target_mm+(700-actual);
                in.lidar.mm[a->sensor]=(uint16_t)lroundf(distance);
            }
            RouteNav_Step(&c,&in,&out);
            actual+=-out.vx_mps*20;
            if(c.status.target_point==point){
                assert(c.status.edge_speed_cap_mps<=cap+.00001f);cap=c.status.edge_speed_cap_mps;
            }
        }
        if(c.status.phase==ROUTE_FAULT)printf("Edge slip P%u scale %.1f: %s actual %.1f S %.1f aux %.1f/%u\n",point,scales[mode],RouteNav_ErrorName(c.status.reason),actual,c.status.progress_mm,c.status.aux_remaining_mm,c.status.aux_reject_reason);
        assert(c.status.at_point==point&&c.status.target_point==point+1U&&fabsf(actual-700)<=ROUTE_EDGE_ARRIVAL_VERIFY_MM);
      }
    }
    puts("P05/P07/P09 converge by independent range with 20% overcount, 60% undercount and frozen odometry; no capture acceleration.");
}

static void prime_near_arrival(RouteController *c,RouteInputs *in,RouteOutput *out,uint8_t point,int remaining)
{
    prime_edge_outage(c,in,out,point);
    for(unsigned n=0;n<6U;n++)frame_with_aux(c,in,out,660,1500,aux_at_remaining(point,remaining));
    assert(c->auxiliary.near_verified&&c->status.phase==ROUTE_CONFIRM&&c->status.confirmed==1);
    assert(c->status.target_point==point&&out->vx_mps==0&&out->vy_mps==0&&!c->recovering);
}

static void test_near_arrival_guards(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    const int offsets[]={-40,-36,0,36,40};
    for(uint8_t point=5;point<=9;point+=2){
      for(unsigned mode=0;mode<sizeof(offsets)/sizeof(offsets[0]);mode++){
        prime_near_arrival(&c,&in,&out,point,offsets[mode]);
        for(unsigned n=0;n<8U;n++){
            in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
            assert(c.status.target_point==point&&c.status.confirmed==1&&out.vx_mps==0);
        }
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;RouteNav_Step(&c,&in,&out);
        assert(c.status.at_point==point&&c.status.target_point==point+1U);
        assert(!c.auxiliary.near_arrival&&!c.auxiliary.near_verified);
      }
    }

    /* Healthy approaches proactively stop; the trigger packet and moving wheels are not proof. */
    for(uint8_t point=5;point<=9;point+=2){
        prime_edge(&c,&in,&out,point);in.chassis_stopped=0;
        for(unsigned p=580;p<=640;p+=20)frame(&c,&in,&out,(float)p,1500);
        for(unsigned n=0;n<6U&&!c.auxiliary.near_arrival;n++)
            frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(point,36));
        assert(c.auxiliary.near_arrival&&c.status.phase==ROUTE_WAIT_LIDAR);
        assert(!c.auxiliary.recheck_frames&&!c.auxiliary.near_verified&&!c.status.confirmed);
        assert(out.vx_mps==0&&out.vy_mps==0);
        uint32_t wait_at=c.wait_at;
        for(unsigned n=0;n<2U;n++){
            frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(point,36));
            assert(!c.auxiliary.recheck_frames&&!c.auxiliary.near_verified&&c.wait_at==wait_at);
            assert(c.status.target_point==point&&out.vx_mps==0&&out.vy_mps==0);
        }
        in.chassis_stopped=1;
        for(unsigned n=0;n<3U;n++){
            frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(point,36));
            assert(c.auxiliary.recheck_frames==n+1U&&!c.auxiliary.near_verified);
            assert(c.status.target_point==point&&out.vx_mps==0);
        }
        for(unsigned n=0;n<5U;n++){
            in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
            assert(c.auxiliary.recheck_frames==3U&&!c.auxiliary.near_verified&&c.wait_at==wait_at);
        }
        frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(point,36));
        assert(c.auxiliary.near_verified&&c.status.phase==ROUTE_CONFIRM&&c.status.confirmed==1);
        for(unsigned n=0;n<2U;n++)frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(point,36));
        assert(c.status.at_point==point&&c.status.target_point==point+1U);
    }

    /* Neither an unqualified constant echo nor a missing full ridge event may trigger/finish. */
    fixture(&c,&in,5);
    for(unsigned n=0;n<15U&&c.status.phase!=ROUTE_FAULT;n++){
        frame_with_aux(&c,&in,&out,300+20.0f*n,n<7U?120:1500,aux_at_remaining(5,36));
        assert(!c.auxiliary.approach_qualified&&!c.auxiliary.near_arrival&&c.status.target_point==5);
    }
    assert(c.edge_found&&!c.status.confirmed);
    prime_edge(&c,&in,&out,5);c.edge_found=0;c.edge_armed=0;
    for(unsigned n=0;n<5U;n++){
        frame_with_aux(&c,&in,&out,660,0,aux_at_remaining(5,36));
        assert(!c.auxiliary.near_arrival&&!c.status.confirmed&&c.status.target_point==5);
    }

    /* A 41-mm negative candidate is outside the supplemental band and cannot confirm. */
    prime_edge_outage(&c,&in,&out,5);
    for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT;n++){
        frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,-41));
        assert(c.status.target_point==5&&!c.status.confirmed&&!c.recovering&&out.vx_mps==0);
    }
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);

    /* In-band chatter must not accumulate four stable samples. */
    prime_edge_outage(&c,&in,&out,5);
    for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT;n++){
        frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,n%2U?-36:-10));
        assert(c.status.target_point==5&&!c.auxiliary.near_verified&&!c.recovering&&out.vx_mps==0);
    }
    assert(c.status.phase==ROUTE_FAULT);

    /* Even if the navigation task did not run during a gap, a late packet cannot reuse verification. */
    prime_near_arrival(&c,&in,&out,5,-36);
    in.now_ms+=ROUTE_EDGE_AUX_PREDICT_MS+20U;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;
    RouteNav_Step(&c,&in,&out);
    assert(!c.auxiliary.near_verified&&!c.status.confirmed&&c.status.target_point==5&&out.vx_mps==0);
    for(unsigned n=0;n<3U;n++){
        frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,-36));
        assert(!c.auxiliary.near_verified&&c.status.target_point==5);
    }
    frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,-36));
    assert(c.auxiliary.near_verified&&c.status.confirmed==1&&c.status.target_point==5);

    /* Movement and an invalid new frame each revoke already-verified arrival evidence. */
    for(unsigned mode=0;mode<2U;mode++){
        prime_near_arrival(&c,&in,&out,5,-36);
        if(mode==0U){
            in.chassis_stopped=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;
            RouteNav_Step(&c,&in,&out);
        }else frame_with_aux(&c,&in,&out,660,1500,65535);
        assert(!c.auxiliary.near_verified&&!c.status.confirmed&&c.status.target_point==5);
        assert(out.vx_mps==0&&out.vy_mps==0);
        in.chassis_stopped=1;
        for(unsigned n=0;n<3U;n++){
            frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,-36));
            assert(!c.auxiliary.near_verified&&!c.status.confirmed&&c.status.target_point==5);
        }
        for(unsigned n=0;n<3U&&!c.auxiliary.near_verified;n++)
            frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,-36));
        assert(c.auxiliary.near_verified&&c.status.confirmed==1&&c.status.target_point==5);
    }

    /* A believable positive reference outside the near band can restore a bounded approach. */
    prime_near_arrival(&c,&in,&out,5,-36);
    for(unsigned n=0;n<10U&&out.vx_mps==0;n++)
        frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(5,60));
    assert(!c.auxiliary.near_arrival&&!c.auxiliary.near_verified&&out.vx_mps<0);
    assert(c.status.target_point==5&&!c.status.confirmed&&fabsf(out.vx_mps)<=ROUTE_EDGE_AUX_RECHECK_MPS);
    puts("Proactive and interrupted arrivals share the +/-40 mm bound and fresh standstill proof; trigger/repeated/moving frames, missing context, expiry and false overrun stay guarded.");
}

static void test_edge_first_capture_history(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    for(uint8_t point=5;point<=9;point+=2){
      for(unsigned mode=0;mode<3U;mode++){
        fixture(&c,&in,point);
        const RouteSegment *s=RoutePlan_Get(point);
        for(unsigned n=0;n<5U;n++)frame_with_aux(&c,&in,&out,300.0f+20*n,s->mark.target_mm,65535);
        /* Only three coherent median candidates precede the outage. They must also approach.
         * Mode 1 has constant range despite encoder movement; mode 2 approaches but has only two. */
        unsigned count=mode==2U?4U:5U;
        for(unsigned n=0;n<count;n++){
            int remaining=mode==1U?240:mode==2U?390-60*(int)n:330-30*(int)n;
            uint16_t ridge=n<3U?s->mark.target_mm:1500;
            frame_with_aux(&c,&in,&out,400.0f+20*n,ridge,aux_at_remaining(point,remaining));
            assert(!c.auxiliary.locked&&c.auxiliary.trusted_at==0&&c.status.estimated_end_mm==700);
        }
        assert(c.status.aux_good_frames==(mode==2U?2:3));
        for(unsigned n=0;n<5U;n++)frame_with_aux(&c,&in,&out,500.0f+20*n,1500,65535);
        assert(c.edge_found&&!c.auxiliary.locked&&c.status.phase==ROUTE_WAIT_LIDAR);
        for(unsigned n=0;n<20U&&c.status.phase!=ROUTE_FAULT&&!c.auxiliary.locked;n++){
            frame_with_aux(&c,&in,&out,660,1500,aux_at_remaining(point,60));
            if(!c.auxiliary.locked)assert(c.auxiliary.trusted_at==0&&out.vx_mps==0);
        }
        if(mode){
            assert(!c.auxiliary.locked&&c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);
        }else{
            assert(c.status.aux_confidence==ROUTE_CONF_TRUSTED&&c.status.control_source==ROUTE_CONTROL_EDGE_AUX);
            assert(c.status.control_remaining_mm==60&&out.vx_mps<0&&fabsf(out.vx_mps)<=ROUTE_EDGE_AUX_RECHECK_MPS);
            /* Only fresh in-band measurements may finish, and the next segment loses the history. */
            for(unsigned n=0;n<8U&&c.status.target_point==point;n++)
                frame_with_aux(&c,&in,&out,690,1500,aux_at_remaining(point,10));
            assert(c.status.target_point==point+1U&&!c.auxiliary.approach_qualified&&!c.auxiliary.history_frames);
        }
      }
    }
    puts("Pre-lock outages resume at all edge points only with enough real range approach; flat range and two-frame evidence remain blocked, and new segments clear history.");
}

#include "p05_trace.h"
#include "p05_resume_trace.h"
#include "p05_first_capture_trace.h"
#include "p07_arrival_trace.h"
#include "p07_proactive_trace.h"
static void test_recorded_p05(void)
{
    RouteController c;RouteInputs in;RouteOutput out;float cap=ROUTE_CRUISE_MPS;
    fixture(&c,&in,5);
    uint8_t finished=0,saw_bad_aux=0;
    for(unsigned n=0;n<sizeof(p05_trace)/sizeof(p05_trace[0]);n++){
        const P05TraceRow *r=&p05_trace[n];
        in.now_ms=10000U+r->tick-p05_trace[0].tick;in.odom.tick_ms=in.now_ms;
        position(&c,&in,r->progress);
        in.lidar.mm[0]=r->front;in.lidar.mm[1]=r->behind;
        in.lidar.mm[2]=r->left;in.lidar.mm[3]=r->right;in.lidar.sequence=r->sequence;
        float previous_goal=c.status.estimated_end_mm;uint32_t previous_trusted=c.auxiliary.trusted_at;
        RouteNav_Step(&c,&in,&out);
        assert(c.status.phase!=ROUTE_FAULT);
        if(r->behind>ROUTE_LIDAR_MAX_MM){
            saw_bad_aux=1;
            assert(c.status.estimated_end_mm==previous_goal&&c.auxiliary.trusted_at==previous_trusted);
        }
        if(c.status.target_point==5){
            assert(c.status.edge_speed_cap_mps<=cap+.00001f);cap=c.status.edge_speed_cap_mps;
            if(r->progress>700)assert(fabsf(out.vx_mps)<=ROUTE_APPROACH_MIN_MPS+.001f);
        }
        if(c.auxiliary.near_arrival){
            assert(out.vx_mps==0&&out.vy_mps==0&&!c.auxiliary.recheck_frames);
            assert(fabsf((float)r->behind-2850)<=ROUTE_EDGE_ARRIVAL_VERIFY_MM&&r->progress<760);
            printf("Recorded P05 proactive zero at tick %u rear=%u mm, S=%.1f; stop fixed-input replay before the old moving tail.\n",r->tick,r->behind,r->progress);
            /* The recorded vehicle kept moving with the old controller. This tail instead
             * models stable standstill after the changed zero command; it is not a field run. */
            for(unsigned k=0;k<12U&&c.status.target_point==5;k++){
                frame_with_aux(&c,&in,&out,r->progress,r->left,r->behind);
                assert(c.status.phase!=ROUTE_FAULT&&out.vx_mps==0&&out.vy_mps==0);
            }
            assert(c.status.at_point==5&&c.status.target_point==6);
            finished=1;break;
        }
    }
    if(!finished)printf("Recorded P05 pending: %s/%s aux %.1f good %u reject %u edge %u S %.1f\n",RouteNav_PhaseName(c.status.phase),RouteNav_ErrorName(c.status.reason),c.status.aux_remaining_mm,c.status.aux_good_frames,c.status.aux_reject_reason,c.status.edge_state,c.status.progress_mm);
    assert(finished&&saw_bad_aux);
}

static void replay_p05_resume(const P05ResumeRow *trace,unsigned count,
                              float expected_remaining,uint8_t expected_prior_lock)
{
    RouteController c;RouteInputs in;RouteOutput out;
    uint8_t saw_invalid=0,saw_wait=0,prior_lock=0;
    fixture(&c,&in,5);
    for(unsigned n=0;n<count;n++){
        const P05ResumeRow *r=&trace[n];
        in.now_ms=10000U+r->tick-trace[0].tick;in.odom.tick_ms=in.now_ms;
        position(&c,&in,r->progress);
        in.chassis_stopped=fabsf(r->vx)<=.005f&&fabsf(r->vy)<=.005f;
        in.lidar.mm[0]=r->front;in.lidar.mm[1]=r->behind;
        in.lidar.mm[2]=r->left;in.lidar.mm[3]=r->right;in.lidar.sequence=r->sequence;
        float previous_goal=c.status.estimated_end_mm;uint32_t previous_trusted=c.auxiliary.trusted_at;
        uint8_t previous_edge=c.edge_found;
        RouteNav_Step(&c,&in,&out);
        if(r->behind>ROUTE_LIDAR_MAX_MM){
            saw_invalid=1;
            assert(c.auxiliary.trusted_at==previous_trusted);
            /* The independent primary beam may confirm its first edge during an auxiliary outage. */
            if(previous_edge||!c.edge_found)assert(c.status.estimated_end_mm==previous_goal);
        }
        if(!saw_invalid&&c.auxiliary.locked)prior_lock=1;
        if(c.status.phase==ROUTE_WAIT_LIDAR)saw_wait=1;
        assert(c.status.target_point==5&&c.status.phase!=ROUTE_FAULT&&!c.recovering);
    }
    if(c.status.aux_confidence!=ROUTE_CONF_TRUSTED)
        printf("P05 resume remains blocked: %s/%s remaining %.1f trusted %.1f reject %u\n",
               RouteNav_PhaseName(c.status.phase),RouteNav_ErrorName(c.status.reason),
               c.status.aux_remaining_mm,c.status.control_remaining_mm,c.status.aux_reject_reason);
    assert(saw_invalid&&saw_wait&&c.edge_found&&c.status.aux_confidence==ROUTE_CONF_TRUSTED);
    assert(prior_lock==expected_prior_lock);
    assert(c.status.control_remaining_mm==expected_remaining&&out.vx_mps<0&&fabsf(out.vx_mps)<=.05f);
    assert(!c.status.confirmed&&c.status.estimated_end_mm<760);

    /* This continuation is a closed-loop model, not an unrecorded physical run. */
    float actual_remaining=expected_remaining,progress=c.status.progress_mm;
    const RouteAuxReference *a=RoutePlan_AuxReference(5);
    for(unsigned n=0;n<200U&&c.status.target_point==5;n++){
        actual_remaining+=out.vx_mps*100.0f;progress-=out.vx_mps*100.0f;
        frame_with_aux(&c,&in,&out,progress,2700,(int32_t)lroundf(a->mark.target_mm+actual_remaining));
        assert(c.status.phase!=ROUTE_FAULT);
        if(c.status.target_point==5)assert(fabsf(out.vx_mps)<=.05f);
    }
    assert(c.status.at_point==5&&c.status.target_point==6&&fabsf(actual_remaining)<=ROUTE_EDGE_ARRIVAL_VERIFY_MM);
}

static void test_recorded_p05_resume(void)
{
    replay_p05_resume(p05_resume_trace,sizeof(p05_resume_trace)/sizeof(p05_resume_trace[0]),110,1);
    replay_p05_resume(p05_first_capture_trace,
                      sizeof(p05_first_capture_trace)/sizeof(p05_first_capture_trace[0]),122,0);
    puts("Both captured P05 outages resume, before and after the first auxiliary lock; separate closed-loop continuations finish at bounded speed.");
}

static void test_recorded_p07_arrival(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    for(unsigned mode=0;mode<2U;mode++){
        const P07ArrivalRow *trace=mode?p07_far_surface_trace:p07_near_arrival_trace;
        unsigned count=mode?sizeof(p07_far_surface_trace)/sizeof(p07_far_surface_trace[0]):
                            sizeof(p07_near_arrival_trace)/sizeof(p07_near_arrival_trace[0]);
        uint8_t finished=0,saw_wait=0;
        unsigned first=0;
        if(mode){
            /* The old controller later resumed motion after an earlier in-band window.
             * Use its recorded bad suffix with a separate legitimate approach context;
             * do not claim that the changed zero command follows the old moving prefix. */
            prime_edge(&c,&in,&out,7);
            while(first<count&&trace[first].tick<63041U)++first;
            assert(first<count);
        }else fixture(&c,&in,7);
        uint32_t replay_at=in.now_ms;
        for(unsigned n=first;n<count;n++){
            const P07ArrivalRow *r=&trace[n];
            in.now_ms=replay_at+r->tick-trace[first].tick;in.odom.tick_ms=in.now_ms;
            position(&c,&in,r->progress);
            in.chassis_stopped=fabsf(r->vx)<=.005f&&fabsf(r->vy)<=.005f;
            in.lidar.mm[0]=r->front;in.lidar.mm[1]=r->behind;
            in.lidar.mm[2]=r->left;in.lidar.mm[3]=r->right;in.lidar.sequence=r->sequence;
            RouteNav_Step(&c,&in,&out);
            if(c.status.phase==ROUTE_WAIT_LIDAR)saw_wait=1;
            assert(!c.recovering&&out.vx_mps<=0);
            if(c.status.target_point==8){
                assert(!mode&&saw_wait&&in.chassis_stopped&&out.vx_mps==0&&out.vy_mps==0);
                assert(abs((int)r->front-2090)>15&&abs((int)r->front-2090)<=40);
                finished=1;break;
            }
            if(c.status.phase==ROUTE_FAULT)break;
        }
        if(!mode){
            if(!finished)printf("Recorded near P07 remains %s/%s aux %.1f reject %u\n",
                                RouteNav_PhaseName(c.status.phase),RouteNav_ErrorName(c.status.reason),
                                c.status.aux_remaining_mm,c.status.aux_reject_reason);
            assert(finished&&c.status.at_point==7);
        }else{
            /* A short synthetic stationary tail removes telemetry/control tick alignment ambiguity. */
            for(unsigned n=0;n<10U&&c.status.phase!=ROUTE_FAULT;n++){
                in.now_ms+=100;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;
                RouteNav_Step(&c,&in,&out);
            }
            assert(!finished&&c.status.target_point==7&&c.status.phase==ROUTE_FAULT);
            assert(c.status.reason==ROUTE_ERROR_REFERENCE&&c.status.aux_reject_reason==ROUTE_REJECT_FAR);
        }
    }
    puts("Recorded P07 interrupted 212x-mm near arrival confirms at rest; a separate approach-context replay rejects the recorded 343x-mm bad suffix.");
}

static void test_recorded_proactive_stop(void)
{
    RouteController c;RouteInputs in;RouteOutput out;uint8_t stopped=0;
    fixture(&c,&in,7);
    for(unsigned n=0;n<sizeof(p07_proactive_trace)/sizeof(p07_proactive_trace[0]);n++){
        const P07ArrivalRow *r=&p07_proactive_trace[n];
        in.now_ms=10000U+r->tick-p07_proactive_trace[0].tick;in.odom.tick_ms=in.now_ms;
        position(&c,&in,r->progress);
        in.chassis_stopped=fabsf(r->vx)<=.005f&&fabsf(r->vy)<=.005f;
        in.lidar.mm[0]=r->front;in.lidar.mm[1]=r->behind;
        in.lidar.mm[2]=r->left;in.lidar.mm[3]=r->right;in.lidar.sequence=r->sequence;
        RouteNav_Step(&c,&in,&out);
        assert(c.status.phase!=ROUTE_FAULT);
        if(c.edge_found&&c.auxiliary.approach_qualified&&
           abs((int)r->front-2090)<=40&&fabsf(c.status.aux_remaining_mm)<=40&&
           (c.status.aux_reject_reason==0||c.status.aux_reject_reason==ROUTE_REJECT_WARMUP)){
            assert(out.vx_mps==0&&out.vy_mps==0&&c.auxiliary.near_arrival);
            assert(c.auxiliary.recheck_frames==0&&!c.auxiliary.near_verified);
            assert(r->tick<88141U&&c.status.target_point==7); /* Before the captured surface jump. */
            printf("Captured P07 first proactive zero at tick %u, front %u mm; decision replay stops here.\n",r->tick,r->front);
            /* A separate stationary model represents the changed zero command, not the old moving tail. */
            in.chassis_stopped=1;
            for(unsigned k=0;k<10U&&c.status.target_point==7;k++){
                frame_with_aux(&c,&in,&out,r->progress,4000,r->front);
                assert(out.vx_mps==0&&out.vy_mps==0&&!c.recovering&&c.status.phase!=ROUTE_FAULT);
            }
            assert(c.status.at_point==7&&c.status.target_point==8);
            stopped=1;break;
        }
    }
    assert(stopped);
}

static void test_slip_reference_guards(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    /* Latest P08 photo: 1617 mm disagreement is a warning, then a bounded far-wall search.
     * The synthetic encoders run three times faster than the independently decreasing range. */
    fixture(&c,&in,8);
    for(unsigned n=0;n<240;n++){
        frame(&c,&in,&out,1822.0f+18.0f*n,(uint16_t)(2515U-6U*n));
        assert(c.status.phase!=ROUTE_FAULT&&c.status.target_point==8);
        assert(c.status.stop_flags!=ROUTE_STOP_POSITION_MISMATCH);
    }
    assert(c.status.wall_locked&&c.status.odom_untrusted&&c.status.progress_mm>3400);
    assert(out.vy_mps< -ROUTE_SLOW_MPS&&!c.status.speed_limited); /* Credible far data restores cruise. */

    /* P02 acquires an old reference, rejects a step, then reacquires stable near data.
     * The old endpoint must not permanently veto the newly validated laser. */
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,1800,1176);
    for(unsigned n=0;n<10;n++){
        frame(&c,&in,&out,1903.0f+6.0f*n,(uint16_t)(698U-6U*n));
        assert(c.status.phase!=ROUTE_FAULT&&c.status.target_point==2);
    }
    assert(!c.status.odom_untrusted&&c.status.estimated_end_mm<2510);
    assert(out.vy_mps>0&&out.vy_mps<=ROUTE_SLOW_MPS);

    /* The confirmed harness obstruction must not become an arrival or reverse command. */
    fixture(&c,&in,4);
    for(unsigned n=0;n<25&&c.status.phase!=ROUTE_FAULT;n++){
        frame(&c,&in,&out,1820,56);
        assert(out.vx_mps==0&&out.vy_mps==0&&!c.recovering&&c.status.target_point==4);
    }
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_REFERENCE);
    assert(c.status.stop_flags==ROUTE_STOP_REFERENCE_INVALID&&!c.status.wall_locked);

    /* Rotating wheels cannot keep a stationary laser alive indefinitely. */
    fixture(&c,&in,2);
    for(unsigned n=0;n<90&&c.status.phase!=ROUTE_FAULT;n++)
        frame(&c,&in,&out,1800.0f+50.0f*n,800);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_STALL);
    assert(out.vy_mps==0&&c.status.odom_untrusted);

    /* Periodic range spikes and reacquisition cannot reset a stationary target's timer. */
    fixture(&c,&in,2);
    for(unsigned n=0;n<90&&c.status.phase!=ROUTE_FAULT;n++)
        frame(&c,&in,&out,1800.0f+50.0f*n,(uint16_t)(n%10U==9U?1100U:800U));
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_STALL);
    assert(out.vy_mps==0);

    /* Fresh, slowly changing far data cannot extend the search forever. */
    fixture(&c,&in,8);
    for(unsigned n=0;n<350&&c.status.phase!=ROUTE_FAULT;n++)
        frame(&c,&in,&out,2000.0f+6.0f*n,(uint16_t)(5000U-2U*n));
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_SEARCH_TIME);
    assert(c.status.stop_flags==ROUTE_STOP_FAR_REFERENCE&&out.vy_mps==0);
    puts("P08 far search, P02 reacquisition, wire obstruction, spinning wheels and search timeout guards passed.");
}
static void test_confidence_and_speed_restore(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,1800,1100);
    assert(c.status.confidence==ROUTE_CONF_TRUSTED&&!c.status.odom_untrusted);
    {
        float end=c.status.estimated_end_mm;
        uint32_t trusted_at=c.wall_at;
        frame(&c,&in,&out,1800,50);
        assert(c.status.confidence==ROUTE_CONF_SUSPECT&&c.status.bad_frames==1);
        assert(c.status.estimated_end_mm==end&&c.wall_at==trusted_at);
        assert(!c.status.odom_untrusted&&!c.status.odom_warn_count&&!c.status.confirmed&&!c.recovering);
        for(unsigned n=0;n<4;n++){
            in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
            assert(c.status.bad_frames==1&&c.status.good_frames==0&&c.wall_at==trusted_at);
        }
        frame(&c,&in,&out,1800,50);
        assert(c.status.speed_limited&&c.status.bad_frames==ROUTE_CONF_BAD_FRAMES);
        assert(c.status.estimated_end_mm==end&&c.wall_at==trusted_at&&!c.status.odom_untrusted);
    }
    for(unsigned n=0;n<ROUTE_CAPTURE_FRAMES+ROUTE_CONF_RESTORE_FRAMES+4U;n++)
        frame(&c,&in,&out,1800,1100);
    assert(c.status.confidence==ROUTE_CONF_TRUSTED&&!c.status.speed_limited);
    assert(!c.status.odom_untrusted&&out.vy_mps>ROUTE_SLOW_MPS);

    /* !O persists as a warning; even a negative nominal remaining must not pin cruise. */
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,2200,1120);
    for(unsigned n=0;n<ROUTE_CONF_ODOM_FRAMES+ROUTE_CONF_RESTORE_FRAMES+2U;n++)
        frame(&c,&in,&out,2700,1120);
    assert(c.status.odom_untrusted&&c.status.odom_remaining_mm<0);
    assert(!c.status.speed_limited&&out.vy_mps>ROUTE_SLOW_MPS);

    /* Stable false-near 150 mm must not create a 30 mm control remaining and creep/reverse. */
    fixture(&c,&in,2);
    for(unsigned n=0;n<10U;n++){
        frame(&c,&in,&out,1800,150);
        if(n>=2U)assert(out.vx_mps==0&&out.vy_mps==0);
        assert(!c.status.wall_locked&&!c.recovering&&!c.status.confirmed);
    }
    assert(c.status.reject_reason==ROUTE_REJECT_NO_APPROACH&&c.wall_at==0);
    assert(c.status.estimated_end_mm==ROUTE_LANE_LENGTH_MM);

    /* A false early capture window is reversible when stable far data return before the gate. */
    fixture(&c,&in,2);
    for(unsigned n=0;n<5U;n++)frame(&c,&in,&out,1000,150);
    assert(c.wall_window_open&&c.status.phase==ROUTE_WAIT_LIDAR);
    for(unsigned n=0;n<ROUTE_CAPTURE_FRAMES+ROUTE_CONF_RESTORE_FRAMES+3U;n++)
        frame(&c,&in,&out,1000,2300);
    assert(!c.wall_window_open&&!c.status.wall_locked&&!c.status.speed_limited);
    assert(c.status.phase==ROUTE_ODOM_RUN&&out.vy_mps>ROUTE_SLOW_MPS);
    assert(c.wall_at==0&&c.status.estimated_end_mm==ROUTE_LANE_LENGTH_MM);

    /* Individually small changes accumulating in the wrong direction are rejected too. */
    fixture(&c,&in,2);
    prime_wall(&c,&in,&out,1800,700);
    {
        unsigned rejected=0;
        for(unsigned n=1;n<=10U;n++){
            float end=c.status.estimated_end_mm;uint32_t trusted_at=c.wall_at;
            frame(&c,&in,&out,1800.0f+20.0f*n,(uint16_t)(700U+20U*n));
            if(c.status.reject_reason==ROUTE_REJECT_DIRECTION){
                ++rejected;assert(c.wall_at==trusted_at&&c.status.estimated_end_mm==end);
                assert(c.status.confidence==ROUTE_CONF_SUSPECT);
            }
        }
        assert(rejected);
    }
    /* A reversal must be measured from the closest recent range, even after a long approach. */
    fixture(&c,&in,2);prime_wall(&c,&in,&out,1800,1100);
    for(unsigned n=1;n<=20U;n++)frame(&c,&in,&out,1800.0f+20.0f*n,(uint16_t)(1100U-20U*n));
    {
        unsigned rejected=0;
        for(unsigned n=1;n<=7U;n++){
            frame(&c,&in,&out,2200.0f+20.0f*n,(uint16_t)(700U+20U*n));
            if(c.status.reject_reason==ROUTE_REJECT_DIRECTION)++rejected;
        }
        assert(rejected);
    }
    puts("Rejected samples cannot change endpoint/trust/timestamps; stable references restore cruise and false windows close.");
}

static void test_trim_credibility(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,2);prime_wall(&c,&in,&out,2200,520);
    for(unsigned n=0;n<8U;n++){
        in.now_ms+=100;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;
        in.lidar.mm[ROUTE_LIDAR_FRONT]=Front_Lidar_Center+20U;
        RouteNav_Step(&c,&in,&out);
    }
    assert(out.vx_mps>0&&out.vy_mps>0);
    in.now_ms+=100;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;
    in.lidar.mm[ROUTE_LIDAR_FRONT]=65535U;RouteNav_Step(&c,&in,&out);
    assert(out.vx_mps==0&&out.vy_mps>0);
    for(unsigned n=0;n<4U;n++){
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;RouteNav_Step(&c,&in,&out);
        assert(c.laser[ROUTE_LIDAR_FRONT].trim_frames==0&&out.vx_mps==0);
    }
    for(unsigned n=0;n<8U;n++){
        in.now_ms+=100;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;
        in.lidar.mm[ROUTE_LIDAR_FRONT]=Front_Lidar_Center+20U;
        RouteNav_Step(&c,&in,&out);
        if(n<ROUTE_CAPTURE_FRAMES-1U)assert(out.vx_mps==0);
    }
    assert(out.vx_mps>0);
    /* In-range raw value outside the role's calibrated band also disables stale trim immediately. */
    in.now_ms+=100;in.odom.tick_ms=in.now_ms;++in.lidar.sequence;
    in.lidar.mm[ROUTE_LIDAR_FRONT]=Front_Lidar_Center+ROUTE_TRIM_VALID_MM+1U;
    RouteNav_Step(&c,&in,&out);assert(out.vx_mps==0&&out.vy_mps>0);
    puts("Trim uses consecutive valid samples in its calibrated band; invalid raw and repeated frames cannot enable trim.");
}

static void test_false_recovery_exit(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,6);prime_wall(&c,&in,&out,2570,150);
    frame(&c,&in,&out,2590,130);frame(&c,&in,&out,2610,110);
    for(unsigned n=0;n<4U&&c.status.phase!=ROUTE_RECOVER;n++)frame(&c,&in,&out,2630,90);
    assert(c.recovering&&out.vy_mps<0);
    {
        uint32_t old_recover=c.recover_at;
        unsigned far_accepted=0;
        for(unsigned n=0;n<ROUTE_CAPTURE_FRAMES+ROUTE_CONF_RESTORE_FRAMES+4U;n++){
            frame(&c,&in,&out,2630,1120);
            if(c.wall_at==in.now_ms)++far_accepted;
            if(far_accepted<ROUTE_CONF_RESTORE_FRAMES)assert(c.recovering&&c.recover_at==old_recover);
        }
        assert(!c.recovering&&!c.status.recovering&&!c.recover_at&&c.status.recovery_resets==1);
        assert(!c.status.speed_limited&&out.vy_mps>ROUTE_SLOW_MPS);
        for(unsigned n=1;n<=45U;n++)frame(&c,&in,&out,2630.0f+10.0f*n,(uint16_t)(1120U-10U*n));
        assert(in.now_ms-old_recover>ROUTE_RECOVER_TIMEOUT_MS&&c.status.phase!=ROUTE_FAULT);
        assert(c.status.target_point==6&&!c.recovering&&out.vy_mps>0);
    }
    /* A small positive correction is still part of a true recovery and keeps its original timer. */
    fixture(&c,&in,6);prime_wall(&c,&in,&out,2570,150);
    frame(&c,&in,&out,2590,130);frame(&c,&in,&out,2610,110);
    for(unsigned n=0;n<4U&&c.status.phase!=ROUTE_RECOVER;n++)frame(&c,&in,&out,2630,90);
    {
        uint32_t old_recover=c.recover_at;
        for(unsigned mm=110;mm<=170;mm+=20)frame(&c,&in,&out,2630,(uint16_t)mm);
        assert(c.recovering&&c.recover_at==old_recover&&!c.status.recovery_resets);
        in.now_ms=old_recover+ROUTE_RECOVER_TIMEOUT_MS;in.odom.tick_ms=in.now_ms;
        RouteNav_Step(&c,&in,&out);
        assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_RECOVER_TIME);
        assert(out.vx_mps==0&&out.vy_mps==0);
    }
    puts("False recovery exits on a confirmed far reference; real near-point recovery retains the four-second limit.");
}

static void test_work_pause(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    fixture(&c,&in,2);
    in.work_request=1;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_WORK_STOP&&out.start_work==0);
    in.work_request=0;in.now_ms+=ROUTE_TOOL_SETTLE_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_WAIT_TOOL&&out.start_work==1);
    in.tool_busy=1;in.ready=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_WAIT_READY);
    in.ready=1;in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_WAIT_TOOL);
    in.tool_busy=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_ODOM_RUN&&out.work_finished&&c.status.work_completed==1);
    /* A long tool stop must not consume the optical no-progress/search budget. */
    fixture(&c,&in,8);
    for(unsigned n=0;n<8;n++)frame(&c,&in,&out,1800+6.0f*n,(uint16_t)(2300U-6U*n));
    assert(c.searching);
    in.work_request=1;RouteNav_Step(&c,&in,&out);
    in.work_request=0;in.now_ms+=ROUTE_TOOL_SETTLE_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);assert(out.start_work==1);
    in.tool_busy=1;in.now_ms+=ROUTE_WALL_NO_PROGRESS_MS+1000U;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);assert(c.status.phase==ROUTE_WAIT_TOOL);
    in.tool_busy=0;in.now_ms+=20;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);assert(out.work_finished);
    frame(&c,&in,&out,1842,2258);
    assert(c.status.phase!=ROUTE_FAULT&&c.status.search_elapsed_ms<500U);
    fixture(&c,&in,2);
    in.work_request=1;RouteNav_Step(&c,&in,&out);
    in.work_request=0;in.now_ms+=ROUTE_TOOL_TIMEOUT_MS;in.odom.tick_ms=in.now_ms;
    RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==
           (ROUTE_TOOL_TIMEOUT_MS>=ROUTE_RUN_TIMEOUT_MS ? ROUTE_ERROR_RUN_TIME : ROUTE_ERROR_TOOL_TIME));
    fixture(&c,&in,2);
    in.tool_busy=in.tool_fault=1;RouteNav_Step(&c,&in,&out);
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_TOOL);
    fixture(&c,&in,14);
    prime_wall(&c,&in,&out,2550,170);
    for(int progress=2560;progress<=2590;progress+=10)
        frame(&c,&in,&out,(float)progress,(uint16_t)(2720-progress));
    for(unsigned n=0;n<3U&&c.status.target_point==14;n++)frame(&c,&in,&out,2590,130);
    if(ROUTE_DEBUG_MODE)assert(c.status.target_point==15);
    else assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_NO_WORK);
    puts("Tool stop, dispatch, readiness interruption and completion handshake passed.");
}
static void test_odometry_and_io(void)
{
    ChassisOdomFrame p;RouteIODiagnostics d;
    ChassisOdom_Reset(0);
    ChassisOdom_UpdateDelta(10,0,0,10,1);
    ChassisOdom_UpdateDelta(10,0,0,20,1);
    ChassisOdom_Read(&p);
    assert(p.valid&&fabsf(p.x_mm-10)<0.01f&&p.y_mm==0);
    ChassisOdom_CorrectAxis(1,99);
    ChassisOdom_Read(&p);
    assert(fabsf(p.x_mm-10)<0.01f&&p.y_mm==99);
    ChassisOdom_UpdateDelta(10,0,0,121,1);
    ChassisOdom_UpdateDelta(10,0,0,131,1);
    ChassisOdom_Read(&p);
    assert(!p.valid);
    ChassisOdom_Reset(UINT32_MAX-20);
    ChassisOdom_UpdateDelta(0,0,0,UINT32_MAX-10,1);
    ChassisOdom_UpdateDelta(10,0,0,0,1);
    ChassisOdom_Read(&p);
    assert(p.valid&&fabsf(p.x_mm-10)<0.01f&&p.sample_dt_ms==11);
    HeadingControl_Reset();
    assert(HeadingControl_HoldYaw(0,0.1f,0)<0);

    start_requested=command_seen=0;
    memset(&lidar,0,sizeof(lidar));
    clock_ms=1000;
    ChassisOdom_Reset(980);
    ChassisOdom_UpdateDelta(0,0,0,990,1);
    ChassisOdom_UpdateDelta(0,0,0,1000,1);
    RouteIO_RequestStart();assert(!start_requested);
    RouteIO_PublishLidar(80,75,120,120);
    Route_TaskCreated=0;RouteIO_RequestStart();assert(!start_requested);
    Route_TaskCreated=1;EN=0;RouteIO_RequestStart();assert(!start_requested);
    EN=1;RouteIO_RequestStart();assert(start_requested&&RouteIO_ControlOwned());
    ChassisOdom_UpdateDelta(0,0,0,1010,1);
    ChassisOdom_UpdateDelta(0,0,0,1020,1);
    clock_ms=1020;RouteIO_SetVelocity(-0.12f,0);RouteIO_ApplyDrive();
    assert(driven_x<0&&driven_y==0);
    tool_busy=1;RouteIO_ApplyDrive();assert(driven_x==0);
    tool_busy=0;clock_ms+=ROUTE_COMMAND_TTL_MS;RouteIO_ApplyDrive();assert(driven_x==0);
    RouteIO_ReadDiagnostics(&d);assert(!d.command_fresh);
    clock_ms=1400;RouteIO_ReadDiagnostics(&d);assert(!d.can_fresh);
    RouteIO_AllowWork(1);assert(RouteIO_QueueWorkFromISR(1));assert(!RouteIO_QueueWorkFromISR(2));
    assert(RouteIO_TakeWorkRequest()==1&&RouteIO_TakeWorkRequest()==0);
    RouteIO_CompleteWork();
    puts("Odometry, yaw correction, start guards, TTLs and work queue passed.");
}
static void test_encoder_delta_timing(void)
{
    ChassisOdomFrame p;
    const uint32_t gaps[]={10,20,50};
    for(unsigned i=0;i<sizeof(gaps)/sizeof(gaps[0]);++i){
        ChassisOdom_Reset(0);
        ChassisOdom_UpdateDelta(0,0,0,10,1);
        ChassisOdom_UpdateDelta(0,2.0f,0,10+gaps[i],1);
        ChassisOdom_Read(&p);
        assert(p.valid&&fabsf(p.y_mm-2.0f)<0.0001f);
        assert(p.sample_dt_ms==gaps[i]&&p.max_sample_dt_ms==gaps[i]);
        assert(fabsf(p.vy_mps-(2.0f/(float)gaps[i]))<0.0001f);
    }
    ChassisOdom_Reset(0);
    ChassisOdom_UpdateDelta(0,0,0,10,1);
    ChassisOdom_UpdateDelta(1.5f,0,0,10,1);
    ChassisOdom_Read(&p);
    assert(p.valid&&fabsf(p.x_mm-1.5f)<0.0001f&&p.sample_dt_ms==0);
    ChassisOdom_Reset(0);
    ChassisOdom_UpdateDelta(0,0,0,10,1);
    ChassisOdom_UpdateDelta(0,0,16384,30,1);
    ChassisOdom_Read(&p);
    assert(fabsf(p.yaw_rad-5.0f*3.14159265358979323846f/180.0f)<0.00001f);
    puts("Encoder displacement is unchanged at 10/20/50 ms; same-tick counts and actual-time gyro integration passed.");
}

static float packet_channel(const uint8_t *bytes,unsigned channel)
{
    uint32_t bits=(uint32_t)bytes[channel*4U]|
                  (uint32_t)bytes[channel*4U+1U]<<8|
                  (uint32_t)bytes[channel*4U+2U]<<16|
                  (uint32_t)bytes[channel*4U+3U]<<24;
    float result;memcpy(&result,&bits,sizeof(result));return result;
}

static void test_diagnostics_and_vofa(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    RouteDebugFrame saved,after;
    uint8_t expected[ROUTE_VOFA_FRAME_BYTES];
    RouteDebug_Reset();
    assert(!RouteDebug_Read(&saved)&&!RouteDebug_BuildPacket(expected,sizeof(expected)));
    fixture(&c,&in,4);
    in.odom.sample_dt_ms=20;in.odom.max_sample_dt_ms=50;
    for(unsigned n=0;n<90&&c.status.phase!=ROUTE_FAULT;++n){
        frame(&c,&in,&out,1876,2223);
        RouteDebug_Capture(&c,&in,out.vx_mps,out.vy_mps);
    }
    assert(c.status.phase==ROUTE_FAULT&&c.status.reason==ROUTE_ERROR_STALL);
    assert(c.status.stop_flags==0&&!c.status.odom_untrusted); /* Far reference was never accepted. */
    assert(RouteDebug_ReadFault(&saved)&&saved.frozen&&saved.route.target_point==4);
    assert(saved.route.odom_remaining_mm==724&&saved.route.laser_remaining_mm==2103);
    assert(saved.route.mismatch_mm==1379);
    assert(fabsf(saved.route.gate_mm-1733.42f)<0.01f);
    in.now_ms+=20;in.lidar.mm[ROUTE_LIDAR_RIGHT]=500;in.odom.y_mm-=1000;
    RouteDebug_Capture(&c,&in,0,0);
    assert(RouteDebug_Read(&after)&&after.tick_ms==saved.tick_ms);
    assert(after.raw_mm[ROUTE_LIDAR_RIGHT]==2223&&after.route.progress_mm==1876);
    assert(RouteDebug_BuildPacket(expected,sizeof(expected)-1)==0);
    assert(RouteDebug_BuildPacket(expected,sizeof(expected))==sizeof(expected));
    assert(expected[sizeof(expected)-4]==0&&expected[sizeof(expected)-3]==0&&
           expected[sizeof(expected)-2]==0x80&&expected[sizeof(expected)-1]==0x7f);
    assert(packet_channel(expected,1)==4&&packet_channel(expected,9)==2223);
    assert(packet_channel(expected,12)==724&&packet_channel(expected,14)==1379);
    assert(packet_channel(expected,15)==20&&packet_channel(expected,16)==50);
    assert(packet_channel(expected,22)==0&&packet_channel(expected,23)==1);
    assert(packet_channel(expected,32)==2600&&packet_channel(expected,35)==0);
    assert(packet_channel(expected,36)==0&&packet_channel(expected,37)==0);
    assert(packet_channel(expected,38)>0&&packet_channel(expected,39)>=ROUTE_WALL_NO_PROGRESS_MS);
    assert(ROUTE_VOFA_CHANNEL_COUNT==64U&&ROUTE_VOFA_FRAME_BYTES==260U);
    assert(packet_channel(expected,40)==ROUTE_CONF_CANDIDATE);
    assert(packet_channel(expected,41)==ROUTE_REJECT_FAR);
    assert(packet_channel(expected,42)==ROUTE_CONF_RESTORE_FRAMES&&packet_channel(expected,43)==0);
    for(unsigned n=44;n<48U;n++)assert(packet_channel(expected,n)==0);
    assert(packet_channel(expected,48)==0&&packet_channel(expected,54)==ROUTE_LIDAR_COUNT);
    assert(packet_channel(expected,62)==ROUTE_CONTROL_ODOM);

    clock_ms=1000000;transmitted_count=0;tx_interrupt=0;tx_ready=1;
    RouteVofa_Poll();
#if ROUTE_DEMO_ENABLE && ROUTE_DEBUG_MODE && ROUTE_VOFA_ENABLE
    assert(tx_interrupt&&transmitted_count==0); /* Poll only queues; it never sends/waits. */
    RouteVofa_Poll();assert(transmitted_count==0);
    tx_ready=0;route_test_usart2_irq();assert(transmitted_count==0);tx_ready=1;
    for(unsigned i=0;i<ROUTE_VOFA_FRAME_BYTES+1U;++i)route_test_usart2_irq();
    assert(!tx_interrupt&&transmitted_count==sizeof(expected));
    assert(memcmp(transmitted,expected,sizeof(expected))==0);
    clock_ms+=ROUTE_VOFA_PERIOD_MS-1U;RouteVofa_Poll();assert(!tx_interrupt);
    ++clock_ms;RouteVofa_Poll();assert(tx_interrupt);
    for(unsigned i=0;i<ROUTE_VOFA_FRAME_BYTES+1U;++i)route_test_usart2_irq();
    assert(transmitted_count==2U*sizeof(expected)&&!tx_interrupt);
    assert(memcmp(transmitted+sizeof(expected),expected,sizeof(expected))==0);
#else
    route_test_usart2_irq();assert(!tx_interrupt&&transmitted_count==0);
#endif
    RouteDebug_Reset();assert(!RouteDebug_ReadFault(&saved));
    puts("Trust diagnostics, immutable fault snapshot, 64-channel JustFloat bytes and nonblocking UART TX passed.");
}

static void test_bluetooth_port_isolation(void)
{
    const int active=ROUTE_DEMO_ENABLE&&ROUTE_DEBUG_MODE&&ROUTE_VOFA_ENABLE;
    assert(RouteVofa_IsActive()==active);
    route_test_init_uart2();assert(initialized_uart2_baud==(active?115200U:9600U));
    route_test_usart2.CR1=USART_CR1_UE|USART_CR1_TE;route_test_usart2.SR=0xc0;route_test_usart2.DR=0xdead;
    assert(route_test_fputc('p',NULL)=='p');
    assert(route_test_usart2.DR==0xdeadU); /* printf is suppressed in servo log mode. */
    route_test_usart2.DR=0xdead;route_test_any_printf(USART2,"x");
    assert(route_test_usart2.DR==(active?0xdeadU:(uint32_t)'x'));
    route_test_usart2.DR=0xdead;route_test_usart2_send('y');
    assert(route_test_usart2.DR==(active?0xdeadU:(uint32_t)'y'));
    route_test_usart1.CR1=USART_CR1_UE|USART_CR1_TE;route_test_usart1.SR=0xc0;route_test_usart1.DR=0xdead;
    route_test_any_printf(USART1,"SERVO z");assert(route_test_usart1.DR=='z');
    Check=0;APP_ON_Flag=0;legacy_rx_calls=0;rx_pending=1;
    route_test_usart2.DR='A';route_test_usart2_irq();
    route_test_usart2.DR='A';route_test_usart2_irq();rx_pending=0;
    assert(APP_ON_Flag==(active?0:1)&&legacy_rx_calls==(active?0U:2U));
    APP_ON_Flag=0;
    puts("USART2 baud selection, real TXE interrupt dispatch, text suppression and APP RX isolation/restoration passed.");
}

static void test_complete_route(void)
{
    RouteController c;RouteInputs in;RouteOutput out;
    uint8_t requested=0;
    uint32_t tool_until=0;
    fixture(&c,&in,1);
    for(unsigned n=0;n<17000&&c.status.phase!=ROUTE_DONE&&c.status.phase!=ROUTE_FAULT;n++){
        const RouteSegment *s=RoutePlan_Get(c.status.target_point);
        float progress=segment_progress(&c,&in,s->motion);
        float remaining=s->length_mm-progress;
        in.now_ms+=20;in.odom.tick_ms=in.now_ms;
        in.work_request=0;
        if(!requested&&c.status.target_point==2&&progress>500){
            requested=1;in.work_request=1;
        }
        if(tool_until)in.tool_busy=in.now_ms<tool_until;
        if(n%5==0){
            ++in.lidar.sequence;
            in.lidar.mm[0]=Front_Lidar_Center;in.lidar.mm[1]=Behind_Lidar_Center;
            in.lidar.mm[2]=in.lidar.mm[3]=120;
            if(s->trigger<ROUTE_LIDAR_COUNT){
                float mm=s->reference==ROUTE_REF_EDGE?
                    ((progress>=200&&progress<=500)?s->mark.target_mm:1800.0f):
                    (s->distance_increases?s->mark.target_mm-remaining:s->mark.target_mm+remaining);
                if(mm<1)mm=1;
                in.lidar.mm[s->trigger]=(uint16_t)lroundf(mm);
            }
            {
                const RouteAuxReference *a=RoutePlan_AuxReference(c.status.target_point);
                if(a)in.lidar.mm[a->sensor]=(uint16_t)lroundf(a->distance_increases?
                    a->mark.target_mm-remaining:a->mark.target_mm+remaining);
            }
        }
        RouteNav_Step(&c,&in,&out);
        if(out.start_work){tool_until=in.now_ms+5000;in.tool_busy=1;}
        in.odom.x_mm+=out.vx_mps*20;
        in.odom.y_mm+=out.vy_mps*20;
        if(out.correct_axis==0)in.odom.x_mm=out.correct_position_mm;
        if(out.correct_axis==1)in.odom.y_mm=out.correct_position_mm;
    }
    if(c.status.phase!=ROUTE_DONE)printf("Failed at point %u phase %s reason %s progress %.1f time %lu\n",
      c.status.target_point,RouteNav_PhaseName(c.status.phase),RouteNav_ErrorName(c.status.reason),
      c.status.progress_mm,(unsigned long)c.status.elapsed_ms);
    assert(c.status.phase==ROUTE_DONE&&c.status.at_point==17&&c.status.work_completed==1);
    printf("All 17 points, one work cycle and return parking passed in %.2f simulated seconds.\n",c.status.elapsed_ms/1000.0);
}
static void test_task_integration(void)
{
    RouteStatus status;
    clock_ms=10000;integration_at=clock_ms;
    injected=work_starts=tool_busy=0;
    start_requested=command_seen=pending_work=work_reserved=work_allowed=0;
    memset(&command,0,sizeof(command));memset(&lidar,0,sizeof(lidar));
    driven_x=driven_y=0;EN=1;Flag_Stop=0;
    ChassisOdom_Reset(clock_ms-20);
    ChassisOdom_UpdateDelta(0,0,0,clock_ms-10,1);
    ChassisOdom_UpdateDelta(0,0,0,clock_ms,1);
    RouteIO_PublishLidar(80,75,120,120);
    if(ROUTE_DEBUG_MODE)RouteIO_RequestStart();
    if(!setjmp(integration_done))my_chassis_task(NULL);
    Route_ReadStatus(&status);
    assert(start_requested&&status.target_point==1&&status.work_completed==1);
    assert(work_starts==1&&!pending_work&&!work_reserved);
    assert(status.phase==ROUTE_ODOM_RUN&&driven_x<0);
    puts("Production task auto/manual start and work request retained across EN pause passed.");
}

int main(void)
{
    setbuf(stdout,NULL);
    if(!ROUTE_CALIBRATION_READY){assert(!RoutePlan_Calibrated());puts("Uncalibrated start is blocked.");return 0;}
    assert(RoutePlan_Calibrated());
    test_frames_and_spikes();
    test_adaptive_endpoint_and_recovery();
    test_closed_loop_wall_bias();
    test_waits_and_edges();
    test_edge_auxiliary_guards();
    test_edge_wait_and_pause();
    test_edge_stationary_resume();
    test_edge_closed_loop_slip();
    test_near_arrival_guards();
    test_edge_first_capture_history();
    test_recorded_p05();
    test_recorded_p05_resume();
    test_recorded_p07_arrival();
    test_recorded_proactive_stop();
    test_slip_reference_guards();
    test_confidence_and_speed_restore();
    test_trim_credibility();
    test_false_recovery_exit();
    test_work_pause();
    test_odometry_and_io();
    test_encoder_delta_timing();
    test_diagnostics_and_vofa();
    test_bluetooth_port_isolation();
    test_complete_route();
    test_task_integration();
    return 0;
}
