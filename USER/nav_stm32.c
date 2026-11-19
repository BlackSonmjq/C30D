#include "system.h"
#include "nav_stm32.h"
#include "nav_button.h"
#include "nav_config.h"
#include "remote_control.h"
#include <stdio.h>
#include <string.h>

#if configTICK_RATE_HZ != 1000
#error NavBoard_Tick1ms requires the vendor 1000 Hz SysTick.
#endif

static volatile uint32_t clock_ms;
static volatile NavLidar can_mailbox;
static volatile struct { float yaw, yaw_rate; uint32_t stamp; uint8_t ready; } imu_mailbox;
static volatile uint8_t work_requests;
static volatile RemoteSample remote_mailbox;
static NavCore navigation;
static NavButton button;
static RemoteControl remote;
static float manual_heading_origin;
static uint8_t manual_heading_locked;
static NavStatus status;

static uint32_t lock_irq(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void unlock_irq(uint32_t p) { __set_PRIMASK(p); }
void NavBoard_Tick1ms(void) { clock_ms++; }
uint32_t NavBoard_Millis(void) { return clock_ms; }
void NavBoard_CanRx(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t dlc,const uint8_t *bytes)
{
    NavLidar next;
    if(!NavCan_Decode(id,ide,rtr,dlc,bytes,&next)) return;
    next.received_ms=clock_ms; next.sequence=can_mailbox.sequence+1U;
    if(!next.sequence) next.sequence=1;
    can_mailbox=next; /* 仅 CAN ISR 写；任务以 PRIMASK 保护完整拷贝。 */
}
void NavBoard_PublishImu(float yaw,float yaw_rate,int ready)
{
    uint32_t p=lock_irq();
    imu_mailbox.yaw=yaw; imu_mailbox.yaw_rate=yaw_rate;
    imu_mailbox.stamp=clock_ms; imu_mailbox.ready=(uint8_t)(ready!=0);
    unlock_irq(p);
}
void RemoteBoard_Publish(uint8_t ly,uint8_t rx,int mode_pressed,int nav_start_pressed)
{
    uint32_t p=lock_irq();
    uint32_t now=clock_ms;
    /* 控制任务读走重连标记之前，新报告不能将它覆盖。 */
    if (!remote_mailbox.valid || (uint32_t)(now-remote_mailbox.received_ms)>=REMOTE_TIMEOUT_MS)
        remote_mailbox.link_reset=1;
    remote_mailbox.ly=ly;
    remote_mailbox.rx=rx;
    remote_mailbox.mode_pressed=(uint8_t)(mode_pressed!=0);
    remote_mailbox.nav_start_pressed=(uint8_t)(nav_start_pressed!=0);
    remote_mailbox.received_ms=now;
    remote_mailbox.valid=1;
    unlock_irq(p);
}
void RemoteBoard_Disconnect(void)
{
    uint32_t p=lock_irq();
    remote_mailbox.valid=0;
    remote_mailbox.link_reset=1;
    unlock_irq(p);
}
void NavBoard_Init(void)
{
    Nav_Init(&navigation,&nav_calibration,NAV_CALIBRATED);
    NavButton_Init(&button);
    RemoteControl_Init(&remote);
    manual_heading_locked=0;
    Velocity_KP=NAV_WHEEL_KP; Velocity_KI=NAV_WHEEL_KI; LineDiffParam=50;
    Allow_Recharge=0; Check=0;
    memset(&status,0,sizeof(status));
}
void Nav_RequestWorkPause(void) { uint32_t p=lock_irq(); work_requests|=1U; unlock_irq(p); }
void Nav_RequestWorkContinue(void) { uint32_t p=lock_irq(); work_requests|=2U; unlock_irq(p); }
int Nav_WorkStopped(void) { return status.work_stopped!=0; }
void NavBoard_ReadStatus(NavStatus *s) { uint32_t p=lock_irq(); *s=status; unlock_irq(p); }
void NavBoard_PublishDrive(int motor_allowed)
{
    uint32_t p=lock_irq();
    status.motor_allowed=(uint8_t)(motor_allowed!=0);
    status.hardware_enable=(uint8_t)(EN!=0);
    status.software_stop=(uint8_t)(Flag_Stop!=0);
    status.battery_voltage=Voltage;
    status.command[0]=Move_X; status.command[1]=Move_Y; status.command[2]=Move_Z;
    status.wheel_target[0]=MOTOR_A.Target; status.wheel_target[1]=MOTOR_B.Target;
    status.wheel_target[2]=MOTOR_C.Target; status.wheel_target[3]=MOTOR_D.Target;
    status.wheel_actual[0]=MOTOR_A.Encoder; status.wheel_actual[1]=MOTOR_B.Encoder;
    status.wheel_actual[2]=MOTOR_C.                                                                                                                Encoder; status.wheel_actual[3]=MOTOR_D.Encoder;
    status.wheel_pwm[0]=MOTOR_A.Motor_Pwm; status.wheel_pwm[1]=MOTOR_B.Motor_Pwm;
    status.wheel_pwm[2]=MOTOR_C.Motor_Pwm; status.wheel_pwm[3]=MOTOR_D.Motor_Pwm;
    unlock_irq(p);
}

void NavBoard_ControlStep(NavOutput *out)
{
    NavInput in;
    NavLidar frame;
    RemoteSample remote_frame;
    NavOutput manual_output;
    float manual_heading_error=0.0f;
    uint32_t p,stamp;
    uint8_t requests,ready;
    unsigned event,i;
    int active,can_switch,can_start;
    RemoteEvent remote_event;
    float w[4]={MOTOR_A.Encoder,MOTOR_B.Encoder,MOTOR_C.Encoder,MOTOR_D.Encoder};
    memset(&in,0,sizeof(in));
    p=lock_irq(); frame=can_mailbox; in.yaw_rad=imu_mailbox.yaw;
    in.yaw_rate_rad_s=imu_mailbox.yaw_rate;
    stamp=imu_mailbox.stamp; ready=imu_mailbox.ready;
    remote_frame=remote_mailbox; remote_mailbox.link_reset=0;
    requests=work_requests; work_requests=0; unlock_irq(p);
    in.now_ms=clock_ms;
    in.imu_ready=(uint8_t)(ready && (uint32_t)(in.now_ms-stamp)<NAV_IMU_TTL_MS);
    in.enabled=(uint8_t)(EN && !Flag_Stop && Voltage>=10.0f);
    for(i=0;i<4;i++) if(float_abs(w[i])>in.wheel_max_mps) in.wheel_max_mps=float_abs(w[i]);
    in.forward_step_mm=(w[0]+w[1]+w[2]+w[3])*(250.0f/CONTROL_FREQUENCY);
    in.lateral_step_mm=(w[0]-w[1]+w[2]-w[3])*(250.0f/CONTROL_FREQUENCY);
    if(status.motor_allowed) {
        const float lateral_pwm=0.25f*(MOTOR_A.Motor_Pwm-MOTOR_B.Motor_Pwm+
                                       MOTOR_C.Motor_Pwm-MOTOR_D.Motor_Pwm);
        const float threshold=NAV_CROSS_STALL_PWM_RATIO*NAV_PWM_LIMIT;
        if(Move_Y>0 && lateral_pwm>=threshold) in.loaded_lateral=1;
        else if(Move_Y<0 && lateral_pwm<=-threshold) in.loaded_lateral=-1;
    }
    in.pause=(uint8_t)((requests&1U)!=0); in.resume=(uint8_t)((requests&2U)!=0);
    active=Nav_IsActive(&navigation);
    can_switch=!navigation.pause_latched && !(in.pause && active);
    can_start=navigation.calibrated && in.imu_ready && in.enabled && frame.sequence &&
        (uint32_t)(in.now_ms-frame.received_ms)<NAV_LIDAR_TTL_MS &&
        navigation.filter[NAV_FRONT].count==NAV_FILTER_FRAMES &&
        in.wheel_max_mps<=NAV_STOP_WHEEL_MPS;
    remote_event=RemoteControl_Step(&remote,&remote_frame,&in,can_switch,can_start && !active,&manual_output);
    if (remote_event==REMOTE_MODE_CHANGED) NavButton_Init(&button);
    if(remote.manual && manual_output.drive_enabled && in.imu_ready) {
        if(!manual_heading_locked) {
            manual_heading_origin=in.yaw_rad;
            manual_heading_locked=1;
        }
        manual_heading_error=Nav_HeadingError(manual_heading_origin,in.yaw_rad);
        manual_output.wz=Nav_HeadingCorrection(manual_heading_error,in.yaw_rate_rad_s);
    } else {
        manual_heading_locked=0;
    }
    if (remote.manual) {
        /* 继续更新导航观测。切回导航后需要新的启动按键。 */
        in.enabled=0;
    } else {
        event=NavButton_Update(&button,KEY==0,active,can_start,in.now_ms);
        in.start=(uint8_t)(event==NAV_KEY_START || remote_event==REMOTE_NAV_START);
        in.stop=(uint8_t)(event==NAV_KEY_STOP);
    }
    Nav_Tick(&navigation,&in,&frame);
    *out=remote.manual ? manual_output : navigation.output;
    if (remote_event==REMOTE_MODE_CHANGED) out->reset_pi=1;
    p=lock_irq();
    status.now_ms=in.now_ms; status.lidar_sequence=frame.sequence;
    status.lidar_age_ms=frame.sequence?(uint32_t)(in.now_ms-frame.received_ms):0xFFFFFFFFU;
    for(i=0;i<4;i++) status.raw_mm[i]=frame.mm[i];
    status.yaw_rad=in.yaw_rad; status.main_error_mm=navigation.main_error_mm;
    status.cross_error_mm=navigation.cross_error_mm; status.wheel_max_mps=in.wheel_max_mps;
    status.state=(uint8_t)navigation.state; status.reason=(uint8_t)navigation.reason;
    status.manual_mode=remote.manual;
    status.target=navigation.target; status.confirmed=navigation.confirmed;
    status.estimated_point=navigation.estimated_point;
    status.work_stopped=out->work_stopped; status.imu_ready=in.imu_ready;
    status.can_fresh=(uint8_t)(frame.sequence && status.lidar_age_ms<NAV_LIDAR_TTL_MS);
    status.valid_mask=0; status.reference_mask=navigation.reference_mask;
    for(i=0;i<4;i++) {
        if(status.can_fresh && frame.mm[i]>=NAV_LIDAR_MIN_MM && frame.mm[i]<=NAV_LIDAR_MAX_MM)
            status.valid_mask|=(uint8_t)(1U<<i);
        status.filtered_mm[i]=navigation.filter[i].count?navigation.filter[i].value:0;
        status.reference_target_mm[i]=(status.reference_mask&(1U<<i))?navigation.reference_target_mm[i]:0;
    }
    status.heading_error_rad=remote.manual?manual_heading_error:navigation.heading_error_rad;
    status.start_front_mm=navigation.start_front_mm;
    unlock_irq(p);
}
void NavBoard_Draw(void)
{
    NavStatus s;
    char text[32];
    NavBoard_ReadStatus(&s); OLED_Clear();
    if (s.manual_mode) snprintf(text,sizeof(text),"MANUAL");
    else snprintf(text,sizeof(text),"%s P%02u>%02u",Nav_StateName((NavState)s.state),s.confirmed,s.target);
    OLED_ShowString(0,0,(const u8 *)text);
    snprintf(text,sizeof(text),"L%4u R%4u",s.raw_mm[NAV_LEFT],s.raw_mm[NAV_RIGHT]);
    OLED_ShowString(0,16,(const u8 *)text);
    snprintf(text,sizeof(text),"F%4u B%4u",s.raw_mm[NAV_FRONT],s.raw_mm[NAV_BACK]);
    OLED_ShowString(0,32,(const u8 *)text);
    snprintf(text,sizeof(text),"%s %lums",s.manual_mode?"LIDAR":Nav_ReasonName((NavReason)s.reason),
             (unsigned long)(s.lidar_age_ms==0xFFFFFFFFU?9999U:s.lidar_age_ms));
    OLED_ShowString(0,48,(const u8 *)text); OLED_Refresh_Gram();
}
/* USART1 115200，10 Hz，只输出 CSV，不接受底盘运动命令。 */
void NavBoard_DebugSend(void)
{
    NavStatus s;
    char line[384];
    unsigned i,guard,used;
    NavBoard_ReadStatus(&s);
    used=(unsigned)snprintf(line,sizeof(line),"NAV,%lu,%s,%s,%u,%u,%u,%u,%u,%u,%u,%lu,%ld,%ld,%ld,%ld",
        (unsigned long)s.now_ms,Nav_StateName((NavState)s.state),Nav_ReasonName((NavReason)s.reason),
        s.target,s.confirmed,s.estimated_point,s.raw_mm[NAV_LEFT],s.raw_mm[NAV_FRONT],
        s.raw_mm[NAV_RIGHT],s.raw_mm[NAV_BACK],(unsigned long)s.lidar_age_ms,
        (long)s.main_error_mm,(long)s.cross_error_mm,(long)(s.yaw_rad*1000.0f),
        (long)(s.wheel_max_mps*1000.0f));
    for(i=0;i<3;i++) { if(used+16>=sizeof(line)) return;
        used+=(unsigned)snprintf(line+used,sizeof(line)-used,",%ld",(long)(s.command[i]*1000.0f)); }
    for(i=0;i<4;i++) { if(used+16>=sizeof(line)) return;
        used+=(unsigned)snprintf(line+used,sizeof(line)-used,",%ld",(long)(s.wheel_target[i]*1000.0f)); }
    for(i=0;i<4;i++) { if(used+16>=sizeof(line)) return;
        used+=(unsigned)snprintf(line+used,sizeof(line)-used,",%ld",(long)(s.wheel_actual[i]*1000.0f)); }
    for(i=0;i<4;i++) { if(used+16>=sizeof(line)) return;
        used+=(unsigned)snprintf(line+used,sizeof(line)-used,",%ld",(long)s.wheel_pwm[i]); }
    if(used+3>=sizeof(line)) return;
    snprintf(line+used,sizeof(line)-used,"\r\n");
    for(i=0;line[i];i++) {
        guard=100000U;
        while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET && guard) guard--;
        if(!guard) return;
        USART_SendData(USART1,(uint8_t)line[i]);
    }
}
