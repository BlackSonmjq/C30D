#include "nav_core.h"
#include "nav_config.h"
#include <math.h>
#include <string.h>

enum { REF_NONE, REF_FIELD, REF_NEAR };
enum { POSITION_X=1U, POSITION_Y=2U };
typedef struct {
    float x, ex, ey, main_error;
    uint8_t x_ok, y_ok, y_required, full, known[4];
} Observation;
static float absolute(float x) { return x<0?-x:x; }
static float limit(float x,float m) { return x>m?m:(x< -m?-m:x); }
static float wrap(float x) {
    while(x>3.14159265f) x-=6.28318531f;
    while(x< -3.14159265f) x+=6.28318531f;
    return x;
}
static int raw_valid(uint16_t x) { return x>=NAV_LIDAR_MIN_MM && x<=NAV_LIDAR_MAX_MM; }
static void clear_filters(NavCore *n) { memset(n->filter,0,sizeof(n->filter)); }
static float median(const float *values) {
    float sorted[NAV_FILTER_FRAMES],v;
    unsigned i,j;
    for(i=0;i<NAV_FILTER_FRAMES;i++) {
        v=values[i]; j=i;
        while(j && sorted[j-1U]>v) { sorted[j]=sorted[j-1U]; j--; }
        sorted[j]=v;
    }
    return sorted[NAV_FILTER_FRAMES/2U];
}
/* 每路独立按序号入窗。切换墙面、无效样本均清本路窗口。 */
static int reference(NavCore *n,unsigned d,unsigned role,float hint,float *v)
{
    NavFilter *f=&n->filter[d];
    float raw=(float)n->lidar.mm[d];
    if(f->role!=role) { memset(f,0,sizeof(*f)); f->role=(uint8_t)role; }
    if(!role || !raw_valid(n->lidar.mm[d]) ||
       (role==REF_NEAR && absolute(raw-hint)>NAV_SCENE_TOL_MM)) {
        f->count=0; f->next=0; return 0;
    }
    if(f->last_sequence!=n->lidar.sequence) {
        f->last_sequence=n->lidar.sequence;
        f->samples[f->next]=raw; f->next=(uint8_t)((f->next+1U)%NAV_FILTER_FRAMES);
        if(f->count<NAV_FILTER_FRAMES) f->count++;
        f->value=f->count==NAV_FILTER_FRAMES?median(f->samples):raw;
    }
    *v=f->value;
    return 1;
}
static int matches_position(float previous,float filtered,float raw)
{
    return absolute(filtered-previous)<=NAV_SCENE_TOL_MM &&
           absolute(raw-previous)<=NAV_SCENE_TOL_MM;
}
static float combine_position(float first,float second,int use_first,int use_second)
{
    if(use_first && use_second) return (first+second)*0.5f;
    return use_first?first:second;
}
static int read_x(NavCore *n,Observation *o)
{
    float left=0,right=0;
    int use_left=reference(n,NAV_LEFT,REF_FIELD,0,&left);
    int use_right=reference(n,NAV_RIGHT,REF_FIELD,0,&right);
    float right_x=n->cal->lr_sum_mm-right;
    float raw_left=n->lidar.mm[NAV_LEFT];
    float raw_right_x=n->cal->lr_sum_mm-n->lidar.mm[NAV_RIGHT];
    o->known[NAV_LEFT]=(uint8_t)use_left; o->known[NAV_RIGHT]=(uint8_t)use_right;
    if(!use_left && !use_right) return 0;
    if(use_left && use_right && (absolute(left-right_x)>NAV_SCENE_TOL_MM ||
       absolute(raw_left-raw_right_x)>NAV_SCENE_TOL_MM)) {
        if(!(n->position_axes&POSITION_X)) return 0;
        use_left=matches_position(n->last_x_mm,left,raw_left);
        use_right=matches_position(n->last_x_mm,right_x,raw_right_x);
        if(use_left==use_right) return 0;
    }
    o->x=combine_position(left,right_x,use_left,use_right);
    /* 连续性用原始位置，控制指令仍用滤波位置。 */
    n->last_x_mm=combine_position(raw_left,raw_right_x,use_left,use_right);
    n->position_axes|=POSITION_X;
    if(use_left) n->reference_mask|=1U<<NAV_LEFT;
    if(use_right) n->reference_mask|=1U<<NAV_RIGHT;
    if((use_left && n->filter[NAV_LEFT].count<NAV_FILTER_FRAMES) ||
       (use_right && n->filter[NAV_RIGHT].count<NAV_FILTER_FRAMES)) o->full=0;
    return 1;
}
static int read_y(NavCore *n,Observation *o,int side,int near_front,int near_back,int field)
{
    const NavPoint *p=&nav_plan[n->target-1U];
    const NavLaneCalibration *c=&n->cal->lane[p->lane];
    float front=0,back=0;
    float front_target=field?c->side_front_mm:c->center_front_mm;
    float back_target=field?c->side_back_mm:c->center_back_mm;
    unsigned rf=field?REF_FIELD:(near_front?REF_NEAR:REF_NONE);
    unsigned rb=field?(side==NAV_SIDE_RIGHT?REF_FIELD:REF_NONE):(near_back?REF_NEAR:REF_NONE);
    int use_front=reference(n,NAV_FRONT,rf,front_target,&front);
    int use_back=reference(n,NAV_BACK,rb,back_target,&back);
    float front_error=front-front_target,back_error=back_target-back;
    float raw_front_error=n->lidar.mm[NAV_FRONT]-front_target;
    float raw_back_error=back_target-n->lidar.mm[NAV_BACK];
    int raw_conflict=absolute(raw_front_error-raw_back_error)>NAV_SCENE_TOL_MM;
    int filtered_conflict=absolute(front_error-back_error)>NAV_SCENE_TOL_MM;
    if(field) {
        float gate=NAV_SCENE_TOL_MM;
        if(p->motion==NAV_MOVE_SIDE || p->motion==NAV_MOVE_RETURN) {
            const NavPoint *prev=&nav_plan[n->target-2U];
            gate+=absolute(n->cal->lane[prev->lane].side_front_mm-c->side_front_mm);
        }
        if(absolute(front_error)>gate || absolute(raw_front_error)>gate) use_front=0;
        if(absolute(back_error)>gate || absolute(raw_back_error)>gate) use_back=0;
    }
    n->reference_target_mm[NAV_FRONT]=front_target;
    n->reference_target_mm[NAV_BACK]=back_target;
    o->known[NAV_FRONT]=(uint8_t)use_front; o->known[NAV_BACK]=(uint8_t)use_back;
    if(!use_front && !use_back) return 0;
    if(use_front && use_back && (raw_conflict || filtered_conflict)) {
        float previous_error=n->last_front_mm-c->side_front_mm;
        int front_matches=(n->position_axes&POSITION_Y) && matches_position(previous_error,front_error,raw_front_error);
        int back_matches=(n->position_axes&POSITION_Y) && matches_position(previous_error,back_error,raw_back_error);
        if(front_matches!=back_matches) {
            use_front=front_matches; use_back=back_matches;
        } else if(filtered_conflict) return 0;
    }
    o->ey=combine_position(front_error,back_error,use_front,use_back);
    if(!raw_conflict || !use_front || !use_back) {
        /* 近垄壁误差也换算成场地前距，供后续换沟使用。 */
        n->last_front_mm=c->side_front_mm+combine_position(raw_front_error,raw_back_error,use_front,use_back);
        n->position_axes|=POSITION_Y;
    }
    if(use_front) n->reference_mask|=1U<<NAV_FRONT;
    if(use_back) n->reference_mask|=1U<<NAV_BACK;
    if((use_front && n->filter[NAV_FRONT].count<NAV_FILTER_FRAMES) ||
       (use_back && n->filter[NAV_BACK].count<NAV_FILTER_FRAMES)) o->full=0;
    return 1;
}
static Observation observe(NavCore *n)
{
    Observation o;
    const NavPoint *p=&nav_plan[n->target-1U];
    const NavLaneCalibration *c=&n->cal->lane[p->lane];
    float f=0,target_x;
    memset(&o,0,sizeof(o)); o.full=1; o.y_required=1;
    if(p->motion==NAV_MOVE_ENTRY || p->motion==NAV_MOVE_EXIT) {
        o.y_ok=(uint8_t)reference(n,NAV_FRONT,REF_FIELD,0,&f);
        reference(n,NAV_BACK,REF_NONE,0,&f);
        if(o.y_ok) {
            n->last_front_mm=n->lidar.mm[NAV_FRONT];
            n->position_axes|=POSITION_Y;
            n->reference_mask|=1U<<NAV_FRONT;
        }
        o.known[NAV_FRONT]=o.y_ok;
        o.ey=f-(p->motion==NAV_MOVE_ENTRY?c->side_front_mm:n->start_front_mm);
        n->reference_target_mm[NAV_FRONT]=p->motion==NAV_MOVE_ENTRY?c->side_front_mm:n->start_front_mm;
        o.main_error=o.ey;
        if(n->filter[NAV_FRONT].count<NAV_FILTER_FRAMES) o.full=0;
        if(p->motion==NAV_MOVE_ENTRY && o.y_ok && absolute(o.ey)<=NAV_ENTRY_CAPTURE_MM) {
            o.x_ok=(uint8_t)read_x(n,&o); o.ex=n->cal->left_stop_mm-o.x;
            n->reference_target_mm[NAV_LEFT]=n->cal->left_stop_mm;
            n->reference_target_mm[NAV_RIGHT]=n->cal->lr_sum_mm-n->cal->left_stop_mm;
        } else {
            o.x_ok=1; o.ex=0; /* 已人工对正；场外侧向距离不作参考 */
        }
        return o;
    }
    o.x_ok=(uint8_t)read_x(n,&o);
    if(!o.x_ok) return o;
    target_x=p->side==NAV_SIDE_LEFT?n->cal->left_stop_mm:n->cal->lr_sum_mm-n->cal->right_stop_mm;
    o.ex=target_x-o.x;
    n->reference_target_mm[NAV_LEFT]=target_x;
    n->reference_target_mm[NAV_RIGHT]=n->cal->lr_sum_mm-target_x;
    if(p->motion==NAV_MOVE_CROSS) {
        int left=o.x<=n->cal->left_stop_mm+NAV_SIDE_FIELD_BAND_MM;
        int right=o.x>=n->cal->lr_sum_mm-n->cal->right_stop_mm-NAV_SIDE_FIELD_BAND_MM;
        int nf=o.x>=c->front_begin_left_mm && o.x<=c->front_end_left_mm;
        int nb=o.x>=c->back_begin_left_mm && o.x<=c->back_end_left_mm;
        if(left||right) o.y_ok=(uint8_t)read_y(n,&o,right?NAV_SIDE_RIGHT:NAV_SIDE_LEFT,0,0,1);
        else if(nf||nb) o.y_ok=(uint8_t)read_y(n,&o,0,nf,nb,0);
        else {
            /* 有限的入口/出口过渡区：靠横移进度与航向通过，不追远墙居中。 */
            reference(n,NAV_FRONT,REF_NONE,0,&f); reference(n,NAV_BACK,REF_NONE,0,&f);
            o.y_required=0; o.y_ok=1; o.ey=0;
        }
        o.main_error=o.ex;
    } else {
        if(absolute(o.ex)>NAV_SIDE_FIELD_BAND_MM) return o;
        o.y_ok=(uint8_t)read_y(n,&o,p->side,0,0,1);
        o.main_error=o.ey;
    }
    return o;
}
void Nav_Init(NavCore *n,const NavCalibration *cal,int calibrated)
{
    memset(n,0,sizeof(*n)); n->cal=cal; n->state=NAV_BOOT;
    n->calibrated=(uint8_t)(calibrated && NavCalibration_Valid(cal));
    n->target=1; n->reason=n->calibrated?NAV_NEED_IMU:NAV_NEED_CALIBRATION;
}
int Nav_IsActive(const NavCore *n)
{
    return n->state==NAV_RUN || n->state==NAV_ALIGN || n->state==NAV_WAIT_LIDAR ||
           n->state==NAV_PAUSING || n->state==NAV_PAUSED;
}
static void stopped(NavCore *n,NavState state,NavReason reason)
{
    n->state=state; n->reason=reason; n->arrival_count=0; n->recovery_count=0;
    n->output.reset_pi=1; n->output.quick_stop=1; n->pause_latched=0;
}
static void wait_lidar(NavCore *n,NavReason reason,float wheel_speed)
{
    if(n->state!=NAV_WAIT_LIDAR) n->zero_reset_done=0;
    n->state=NAV_WAIT_LIDAR; n->reason=reason;
    n->arrival_count=0; n->recovery_count=0; n->output.quick_stop=1;
    if(wheel_speed<=NAV_STOP_WHEEL_MPS && !n->zero_reset_done) {
        n->output.reset_pi=1; n->zero_reset_done=1;
    }
}
static float speed(float e,float cap,float deadband)
{
    return absolute(e)<=deadband?0:limit(NAV_POSITION_KP*e,cap);
}
static float correction(float e)
{
    return absolute(e)<=NAV_CENTER_DEADBAND_MM?0:limit(NAV_CENTER_KP*e,NAV_SPEED_ALIGN);
}
static int blocked(NavCore *n,const Observation *o)
{
    unsigned i;
    float towards[4]={n->output.vx,-n->output.vx,n->output.vy,-n->output.vy};
    for(i=0;i<4;i++) if(o->known[i] && raw_valid(n->lidar.mm[i]) && towards[i]>0 &&
        (float)n->lidar.mm[i]<=n->cal->min_distance_mm[i]) return 1;
    return 0;
}
enum { CROSS_NUDGE_READY, CROSS_NUDGE_STALLING, CROSS_NUDGE_FORWARD,
       CROSS_NUDGE_BACK, CROSS_NUDGE_USED };

static int nudge_has_room(const NavCore *n,unsigned direction,float travel_mm)
{
    uint16_t distance=n->lidar.mm[direction];
    return raw_valid(distance) && distance>n->cal->min_distance_mm[direction]+travel_mm;
}

static int try_cross_nudge(NavCore *n,const NavInput *in)
{
    unsigned direction;
    uint32_t elapsed=in->now_ms-n->cross_nudge_since_ms;
    if(n->cross_nudge_state==CROSS_NUDGE_USED) return 0;

    if(n->cross_nudge_state==CROSS_NUDGE_FORWARD || n->cross_nudge_state==CROSS_NUDGE_BACK) {
        direction=n->cross_nudge_state==CROSS_NUDGE_FORWARD?NAV_FRONT:NAV_BACK;
        n->cross_nudge_remaining_mm-=direction==NAV_FRONT?in->forward_step_mm:-in->forward_step_mm;
        if(n->cross_nudge_remaining_mm<=0 || elapsed>=NAV_CROSS_NUDGE_MAX_MS || n->output.vy==0 ||
           !nudge_has_room(n,direction,n->cross_nudge_remaining_mm)) {
            n->cross_nudge_state=CROSS_NUDGE_USED;
            n->output.reset_pi=1;
            return 0;
        }
    } else {
        if(n->output.vy*in->loaded_lateral<=0 || in->wheel_max_mps>NAV_STOP_WHEEL_MPS) {
            n->cross_nudge_state=CROSS_NUDGE_READY;
            return 0;
        }
        if(n->cross_nudge_state==CROSS_NUDGE_READY) {
            n->cross_nudge_since_ms=in->now_ms;
            n->cross_nudge_state=CROSS_NUDGE_STALLING;
            return 0;
        }
        if(elapsed<NAV_CROSS_STALL_MS) return 0;
        /* 入口和出口过渡区也用原始前后距判断点动空间。 */
        if(nudge_has_room(n,NAV_FRONT,NAV_CROSS_NUDGE_MM)) {
            direction=NAV_FRONT;
            n->cross_nudge_state=CROSS_NUDGE_FORWARD;
        } else if(nudge_has_room(n,NAV_BACK,NAV_CROSS_NUDGE_MM)) {
            direction=NAV_BACK;
            n->cross_nudge_state=CROSS_NUDGE_BACK;
        } else return 0;
        n->cross_nudge_remaining_mm=NAV_CROSS_NUDGE_MM;
        n->cross_nudge_since_ms=in->now_ms;
        n->output.reset_pi=1;
    }
    n->output.vx=direction==NAV_FRONT?NAV_CROSS_NUDGE_MPS:-NAV_CROSS_NUDGE_MPS;
    n->output.vy=0;
    n->arrival_count=0;
    return 1;
}

void Nav_Tick(NavCore *n,const NavInput *in,const NavLidar *frame)
{
    int fresh,new_frame,forward_segment,approaching;
    float f=0,cap,slowdown_mm;
    Observation o;
    const NavPoint *p;
    memset(&n->output,0,sizeof(n->output));
    n->reference_mask=0;
    memset(n->reference_target_mm,0,sizeof(n->reference_target_mm));
    if(frame && frame->sequence!=n->lidar.sequence) {
        if(n->lidar.sequence && (uint32_t)(frame->received_ms-n->lidar.received_ms)>=NAV_LIDAR_TTL_MS) clear_filters(n);
        n->lidar=*frame;
    }
    fresh=n->lidar.sequence && (uint32_t)(in->now_ms-n->lidar.received_ms)<NAV_LIDAR_TTL_MS;
    new_frame=fresh && n->last_sequence!=n->lidar.sequence;
    if(new_frame) n->last_sequence=n->lidar.sequence;
    if(in->stop) { stopped(n,NAV_STOPPED,NAV_USER_STOP); return; }
    if(Nav_IsActive(n) && (!in->enabled || !in->imu_ready)) {
        stopped(n,NAV_STOPPED,in->enabled?NAV_NEED_IMU:NAV_DISABLED); return;
    }
    if(!Nav_IsActive(n)) {
        int front_ok;
        if(in->wheel_max_mps>NAV_STOP_WHEEL_MPS) clear_filters(n);
        front_ok=in->wheel_max_mps<=NAV_STOP_WHEEL_MPS && fresh &&
            reference(n,NAV_FRONT,REF_FIELD,0,&f) && n->filter[NAV_FRONT].count==NAV_FILTER_FRAMES;
        if(n->state==NAV_BOOT || n->state==NAV_READY) {
            n->reason=!n->calibrated?NAV_NEED_CALIBRATION:(!in->imu_ready?NAV_NEED_IMU:
                      (!in->enabled?NAV_DISABLED:(!front_ok?NAV_NEED_LIDAR:NAV_OK)));
            n->state=n->reason==NAV_OK?NAV_READY:NAV_BOOT;
        }
        if(in->start && n->calibrated && in->imu_ready && in->enabled && front_ok &&
           f>n->cal->lane[0].side_front_mm+NAV_POINT_TOL_MM) {
            n->start_front_mm=f; n->heading_origin_rad=in->yaw_rad;
            n->target=1; n->confirmed=0; n->estimated_point=0;
            n->last_x_mm=n->cal->left_stop_mm; n->position_axes=POSITION_X;
            n->arrival_count=0; n->recovery_count=0; n->pause_latched=0; n->zero_reset_done=0;
            n->cross_nudge_state=CROSS_NUDGE_READY;
            clear_filters(n); n->state=NAV_RUN; n->reason=NAV_OK; n->output.reset_pi=1;
        } else return;
    }
    n->output.drive_enabled=1;
    n->heading_error_rad=wrap(n->heading_origin_rad-in->yaw_rad);
    if(in->pause && !n->pause_latched) { n->pause_latched=1; n->state=NAV_PAUSING; n->arrival_count=0; }
    if(n->pause_latched) {
        n->output.quick_stop=1; n->reason=NAV_WORK_PAUSE;
        if(in->wheel_max_mps<=NAV_STOP_WHEEL_MPS) {
            if(n->arrival_count<NAV_FILTER_FRAMES) n->arrival_count++;
            if(n->arrival_count>=NAV_FILTER_FRAMES) { n->state=NAV_PAUSED; n->output.work_stopped=1; }
        } else { n->arrival_count=0; n->state=NAV_PAUSING; }
        if(!(in->resume && n->state==NAV_PAUSED)) return;
        n->pause_latched=0; n->state=NAV_WAIT_LIDAR; n->recovery_count=0; n->arrival_count=0;
        n->position_axes=0;
        clear_filters(n); /* 继续前必须重新取得控制参考，保留点号与起跑前距。 */
        return;
    }
    if(!fresh) { clear_filters(n); wait_lidar(n,NAV_NEED_LIDAR,in->wheel_max_mps); return; }
    o=observe(n); p=&nav_plan[n->target-1U];
    if(!o.x_ok || !o.y_ok) {
        NavReason why=!o.x_ok?NAV_NEED_X:NAV_NEED_Y;
        if((!o.x_ok && o.known[NAV_LEFT] && o.known[NAV_RIGHT]) ||
           (!o.y_ok && o.known[NAV_FRONT] && o.known[NAV_BACK])) why=NAV_BAD_SCENE;
        wait_lidar(n,why,in->wheel_max_mps); return;
    }
    if(n->state==NAV_WAIT_LIDAR) {
        n->output.quick_stop=1;
        if(!n->zero_reset_done && in->wheel_max_mps<=NAV_STOP_WHEEL_MPS) {
            n->output.reset_pi=1; n->zero_reset_done=1;
        }
        if(new_frame && n->recovery_count<NAV_FILTER_FRAMES) n->recovery_count++;
        if(n->recovery_count<NAV_FILTER_FRAMES || !o.full || in->wheel_max_mps>NAV_STOP_WHEEL_MPS) return;
        n->state=NAV_RUN; n->reason=NAV_OK; n->output.quick_stop=0; n->output.reset_pi=1;
        n->zero_reset_done=0;
        if(n->cross_nudge_state==CROSS_NUDGE_STALLING) n->cross_nudge_state=CROSS_NUDGE_READY;
        else if(n->cross_nudge_state!=CROSS_NUDGE_READY) n->cross_nudge_state=CROSS_NUDGE_USED;
    }
    /* 粗定位仅比较当前段端点；点号推进仍须停车后验收，返程不会进沟。 */
    n->estimated_point=n->confirmed;
    if(o.y_required && absolute(o.ex)<=NAV_SCENE_TOL_MM && absolute(o.ey)<=NAV_SCENE_TOL_MM)
        n->estimated_point=n->target;
    n->main_error_mm=o.main_error;
    n->cross_error_mm=p->motion==NAV_MOVE_CROSS?o.ey:o.ex;
    forward_segment=p->motion==NAV_MOVE_ENTRY || p->motion==NAV_MOVE_SIDE;
    slowdown_mm=NAV_SLOWDOWN_MM;
    if(p->motion==NAV_MOVE_CROSS) slowdown_mm=NAV_CROSS_SLOWDOWN_MM;
    else if(forward_segment) slowdown_mm=NAV_LONGITUDINAL_SLOWDOWN_MM;
    approaching=absolute(o.main_error)<=slowdown_mm;
    cap=approaching?NAV_SPEED_APPROACH:NAV_SPEED_CRUISE;
    /* 进场和侧道换垄的接近、越点回调共用纵向限速。 */
    if(forward_segment && (approaching || o.main_error<0)) cap=NAV_SPEED_LONGITUDINAL_APPROACH;
    if((p->motion==NAV_MOVE_EXIT || p->motion==NAV_MOVE_RETURN) && o.main_error>0) cap=NAV_SPEED_ALIGN;
    if(p->motion==NAV_MOVE_CROSS && n->target>1) {
        int expected_right=p->side==NAV_SIDE_RIGHT;
        if(approaching || (expected_right?o.ex<0:o.ex>0)) cap=NAV_SPEED_CROSS_APPROACH;
        else cap=NAV_SPEED_CROSS_CRUISE;
        n->output.vy=-speed(o.ex,cap,NAV_POINT_TOL_MM);
        n->output.vx=o.y_required?correction(o.ey):0;
    } else {
        n->output.vx=speed(o.ey,cap,NAV_POINT_TOL_MM);
        n->output.vy=-correction(o.ex);
    }
    n->output.wz=limit(NAV_HEADING_KP*n->heading_error_rad,NAV_HEADING_MAX_RAD_S);
    if(absolute(n->heading_error_rad)<=NAV_HEADING_TOL_RAD) n->output.wz=0;
    if(blocked(n,&o)) { memset(&n->output,0,sizeof(n->output)); n->output.drive_enabled=1;
        wait_lidar(n,NAV_TOO_CLOSE,in->wheel_max_mps); return; }
    n->state=approaching?NAV_ALIGN:NAV_RUN;
    n->reason=NAV_OK;
    if(p->motion==NAV_MOVE_CROSS && try_cross_nudge(n,in)) return;
    if(o.y_required && absolute(o.ex)<=NAV_POINT_TOL_MM && absolute(o.ey)<=NAV_POINT_TOL_MM &&
       absolute(n->heading_error_rad)<=NAV_HEADING_TOL_RAD) {
        n->output.vx=0; n->output.vy=0; n->output.wz=0;
        if(new_frame && o.full && in->wheel_max_mps<=NAV_STOP_WHEEL_MPS) n->arrival_count++;
        else if(in->wheel_max_mps>NAV_STOP_WHEEL_MPS) n->arrival_count=0;
        if(n->arrival_count>=NAV_FILTER_FRAMES) {
            n->confirmed=n->target; n->estimated_point=n->target; n->arrival_count=0;
            n->output.reset_pi=1;
            if(n->target==17) { stopped(n,NAV_DONE,NAV_OK); n->output.drive_enabled=0; }
            else {
                n->target++; n->state=NAV_RUN; n->reference_mask=0;
                n->cross_nudge_state=CROSS_NUDGE_READY;
            }
        }
    } else n->arrival_count=0;
}
const char *Nav_StateName(NavState s)
{
    static const char *const names[]={"BOOT","READY","RUN","ALIGN","WAIT","PAUSING","PAUSED","STOPPED","DONE"};
    return (unsigned)s<sizeof(names)/sizeof(names[0])?names[s]:"?";
}
const char *Nav_ReasonName(NavReason r)
{
    static const char *const names[]={"OK","CAL","IMU","LIDAR","X_REF","Y_REF","SCENE","CLEARANCE","DISABLED","KEY_STOP","WORK"};
    return (unsigned)r<sizeof(names)/sizeof(names[0])?names[r]:"?";
}
