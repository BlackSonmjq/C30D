#include "nav_core.h"
#include "nav_button.h"
#include "nav_config.h"
#include "wheel_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    NavCore nav;
    NavInput input;
    NavLidar frame;
    float x,y,yaw,sx,sy,sw;
    unsigned frame_number,visited,drop,bad_front,bad_back,bad_left;
    int conflicting_returns;
} Simulation;
static uint16_t distance_mm(float x) { return (uint16_t)(x<1?1:(x>6000?6000:x)); }
static void sensors(Simulation *s)
{
    int lane=(int)floorf((s->y+350.0f)/700.0f);
    float f=4300.0f-s->y,b=100.0f+s->y;
    if(lane>=0 && lane<7 && s->x>180 && s->x<2620) {
        const NavLaneCalibration *c=&nav_calibration.lane[lane];
        float dy=s->y-(float)lane*700.0f;
        if(s->x>=c->front_begin_left_mm && s->x<=c->front_end_left_mm) f=100.0f-dy;
        if(s->x>=c->back_begin_left_mm && s->x<=c->back_end_left_mm) b=100.0f+dy;
    }
    if(s->x<=180) b=777; /* Plausible outside return: must be ignored on the left. */
    s->frame.mm[NAV_FRONT]=s->bad_front?0:distance_mm(f);
    s->frame.mm[NAV_BACK]=s->bad_back?0:distance_mm(b);
    s->frame.mm[NAV_LEFT]=s->bad_left?0:distance_mm(s->x);
    s->frame.mm[NAV_RIGHT]=distance_mm(2800.0f-s->x);
    if(s->y< -100) { s->frame.mm[NAV_LEFT]=12; s->frame.mm[NAV_RIGHT]=1234; }
    else if(s->conflicting_returns) {
        if(s->x>2630) s->frame.mm[NAV_LEFT]=1200;
        if(s->x<175) s->frame.mm[NAV_RIGHT]=1600;
        if(s->nav.target==3 && s->y>200 && s->y<500) s->frame.mm[NAV_BACK]=distance_mm(b+160);
        if(s->nav.target==15 && s->y>1000 && s->y<3500) s->frame.mm[NAV_FRONT]=distance_mm(f-160);
    }
    s->frame.received_ms=s->input.now_ms;
    s->frame.sequence=++s->frame_number;
}
static void init_sim(Simulation *s)
{
    memset(s,0,sizeof(*s)); s->x=100; s->y=-500;
    s->input.enabled=1; s->input.imu_ready=1;
    Nav_Init(&s->nav,&nav_calibration,1);
}
static void step(Simulation *s)
{
    unsigned old=s->nav.confirmed;
    s->input.now_ms+=10;
    if(!s->drop && (!s->frame.sequence || s->input.now_ms%100==0)) sensors(s);
    s->input.yaw_rad=s->yaw;
    s->input.yaw_rate_rad_s=s->sw;
    s->input.wheel_max_mps=fabsf(s->sx)+fabsf(s->sy)+0.178f*fabsf(s->sw);
    Nav_Tick(&s->nav,&s->input,&s->frame);
    s->input.start=s->input.stop=s->input.pause=s->input.resume=0;
    if(s->nav.confirmed!=old) {
        assert(s->nav.confirmed==old+1); s->visited|=1U<<(s->nav.confirmed-1U);
    }
    if(s->nav.output.reset_pi) s->sx=s->sy=s->sw=0;
    if(!s->nav.output.drive_enabled || s->nav.output.quick_stop) s->sx=s->sy=s->sw=0;
    else {
        s->sx=WheelRamp(s->sx,s->nav.output.vx,NAV_ACCEL_MPS2*0.01f);
        s->sy=WheelRamp(s->sy,s->nav.output.vy,NAV_ACCEL_MPS2*0.01f);
        s->sw=WheelRamp(s->sw,s->nav.output.wz,NAV_ANGULAR_ACCEL_RAD_S2*0.01f);
    }
    s->x-=s->sy*10.0f; s->y+=s->sx*10.0f; s->yaw+=s->sw*0.01f;
}
static void start_sim(Simulation *s)
{
    unsigned i; init_sim(s);
    for(i=0;i<50;i++) step(s);
    assert(s->nav.state==NAV_READY);
    s->input.start=1; step(s); assert(Nav_IsActive(&s->nav));
    assert(fabsf(s->nav.start_front_mm-4800)<1);
}
static void until(Simulation *s,unsigned target,float x_min,float x_max)
{
    while(s->input.now_ms<300000U) {
        step(s);
        if(s->nav.target==target && s->x>x_min && s->x<x_max && s->nav.state!=NAV_WAIT_LIDAR) return;
    }
    printf("STALLED target=%u confirmed=%u state=%s reason=%s x=%.1f y=%.1f F=%u B=%u L=%u R=%u main=%.1f cross=%.1f\n",
        s->nav.target,s->nav.confirmed,Nav_StateName(s->nav.state),Nav_ReasonName(s->nav.reason),
        s->x,s->y,s->frame.mm[0],s->frame.mm[1],s->frame.mm[2],s->frame.mm[3],s->nav.main_error_mm,s->nav.cross_error_mm);
    fflush(stdout); assert(!"Did not reach requested segment");
}
static void test_can(void)
{
    const uint8_t p[8]={3,0xE8,7,0xD0,0x0B,0xB8,0x0F,0xA0};
    NavLidar a,b;
    memset(&a,0,sizeof(a)); assert(NavCan_Decode(0x301,0,0,8,p,&a));
    assert(a.mm[NAV_LEFT]==1000 && a.mm[NAV_FRONT]==2000 && a.mm[NAV_RIGHT]==3000 && a.mm[NAV_BACK]==4000);
    b=a;
    assert(!NavCan_Decode(0x302,0,0,8,p,&a)); assert(!memcmp(&a,&b,sizeof(a)));
    assert(!NavCan_Decode(0x301,4,0,8,p,&a)); assert(!NavCan_Decode(0x301,0,2,8,p,&a));
    assert(!NavCan_Decode(0x301,0,0,7,p,&a)); assert(!NavCan_Decode(0x301,0,0,8,0,&a));
    assert(!NavCan_Decode(0x301,0,0,8,p,0));
    puts("PASS CAN byte order, channel mapping and malformed frames");
}
static void test_freshness(void)
{
    Simulation s; unsigned i; init_sim(&s); sensors(&s);
    for(i=0;i<20;i++) { s.input.now_ms+=10; Nav_Tick(&s.nav,&s.input,&s.frame); }
    assert(s.nav.state==(NAV_FILTER_FRAMES==1?NAV_READY:NAV_BOOT) && s.nav.filter[NAV_FRONT].count==1);
    for(i=1;i<NAV_FILTER_FRAMES;i++) { sensors(&s); Nav_Tick(&s.nav,&s.input,&s.frame); }
    assert(s.nav.state==NAV_READY); /* Equal distances in real new frames count. */
    s.input.now_ms=s.frame.received_ms+300;
    Nav_Tick(&s.nav,&s.input,&s.frame); assert(s.nav.state==NAV_BOOT);
    Nav_Init(&s.nav,&nav_calibration,0); s.input.start=1; sensors(&s);
    for(i=0;i<10;i++) { sensors(&s); Nav_Tick(&s.nav,&s.input,&s.frame); }
    assert(!s.nav.output.drive_enabled && s.nav.reason==NAV_NEED_CALIBRATION);
    puts("PASS new-frame counting, exact TTL and uncalibrated start gate");
}
static void test_button(void)
{
    NavButton b; NavButton_Init(&b);
    assert(!NavButton_Update(&b,1,0,1,0));
    assert(!NavButton_Update(&b,1,0,1,100));
    assert(!NavButton_Update(&b,0,0,1,110));
    assert(!NavButton_Update(&b,0,0,1,150)); /* Held at boot cannot start. */
    assert(!NavButton_Update(&b,1,0,1,200));
    assert(!NavButton_Update(&b,1,0,1,240));
    assert(!NavButton_Update(&b,0,0,1,250));
    assert(NavButton_Update(&b,0,0,1,290)==NAV_KEY_START);
    assert(!NavButton_Update(&b,1,1,1,300));
    assert(NavButton_Update(&b,1,1,1,340)==NAV_KEY_STOP);
    assert(!NavButton_Update(&b,0,0,1,350)); assert(!NavButton_Update(&b,0,0,1,390));
    puts("PASS startup-held button, release-to-start and latched manual stop");
}
static void test_complete(int conflicting_returns)
{
    Simulation s; start_sim(&s);
    s.conflicting_returns=conflicting_returns;
    while(s.input.now_ms<300000 && s.nav.state!=NAV_DONE) {
        step(&s);
        if(s.nav.state==NAV_ALIGN && nav_plan[s.nav.target-1U].motion==NAV_MOVE_CROSS)
            assert(fabsf(s.nav.output.vy)<=NAV_SPEED_CROSS_APPROACH);
    }
    if(s.nav.state!=NAV_DONE) {
        printf("FULL ROUTE STALLED P%u %s %s x%.1f y%.1f\n",s.nav.target,Nav_StateName(s.nav.state),Nav_ReasonName(s.nav.reason),s.x,s.y);
        fflush(stdout);
    }
    assert(s.nav.state==NAV_DONE && s.nav.confirmed==17);
    assert(s.visited==0x1FFFFU && fabsf(s.y+500)<NAV_POINT_TOL_MM+2);
    assert(fabsf(s.x-100)<NAV_POINT_TOL_MM+2 && !s.nav.output.drive_enabled);
    for(unsigned i=0;i<100;i++) step(&s);
    assert(s.nav.state==NAV_DONE && !s.nav.output.drive_enabled);
    printf("PASS complete P01-P17 with %s (%lu ms simulated)\n",
           conflicting_returns?"conflicting side and longitudinal echoes":"nominal references and the short-ridge gap",
           (unsigned long)s.input.now_ms);
}
static void test_recovery_pause_stop(void)
{
    Simulation s; unsigned i; float saved;
    start_sim(&s); until(&s,6,800,1600); saved=s.nav.start_front_mm;
    s.bad_front=1;
    for(i=0;i<20;i++) step(&s);
    assert(s.nav.state!=NAV_WAIT_LIDAR && s.nav.target==6); /* Back wall remains usable. */
    s.bad_back=1;
    for(i=0;i<12;i++) step(&s);
    assert(s.nav.state==NAV_WAIT_LIDAR && s.nav.target==6 && s.nav.output.vx==0 && s.nav.output.vy==0);
    s.bad_front=s.bad_back=0;
    for(i=0;i<100 && s.nav.state==NAV_WAIT_LIDAR;i++) step(&s);
    assert(s.nav.state!=NAV_WAIT_LIDAR && s.nav.target==6);
    s.drop=1;
    for(i=0;i<45;i++) step(&s);
    assert(s.nav.state==NAV_WAIT_LIDAR && s.nav.target==6);
    s.drop=0;
    for(i=0;i<100 && s.nav.state==NAV_WAIT_LIDAR;i++) step(&s);
    assert(s.nav.state!=NAV_WAIT_LIDAR);
    s.input.pause=1; step(&s);
    for(i=0;i<5;i++) { s.input.pause=1; step(&s); } /* Repeated pause requests are idempotent. */
    assert(s.nav.state==NAV_PAUSED && s.nav.output.work_stopped);
    s.input.resume=1; step(&s); assert(s.nav.state==NAV_WAIT_LIDAR);
    for(i=0;i<100 && s.nav.state==NAV_WAIT_LIDAR;i++) step(&s);
    assert(s.nav.target==6 && s.nav.start_front_mm==saved);
    s.input.stop=1; step(&s);
    for(i=0;i<60;i++) step(&s);
    assert(s.nav.state==NAV_STOPPED && !s.nav.output.drive_enabled);
    puts("PASS single-channel fallback, multi-channel/CAN loss, automatic recovery, work pause and manual-stop latch");
    start_sim(&s); until(&s,2,600,1200); s.input.enabled=0; step(&s);
    s.input.enabled=1; for(i=0;i<50;i++) step(&s);
    assert(s.nav.state==NAV_STOPPED && s.nav.reason==NAV_DISABLED && !s.nav.output.drive_enabled);
    puts("PASS hardware disable cannot auto-restart on data recovery");
}
static void test_no_repeated_arrival(void)
{
    Simulation s; unsigned i;
    start_sim(&s); s.nav.target=2; s.nav.confirmed=1; s.x=2700; s.y=0;
    memset(s.nav.filter,0,sizeof(s.nav.filter));
    for(i=0;i<NAV_FILTER_FRAMES;i++) { s.input.now_ms+=10; sensors(&s); Nav_Tick(&s.nav,&s.input,&s.frame); }
    if(NAV_FILTER_FRAMES==1) {
        assert(s.nav.target==3 && s.nav.confirmed==2);
        for(i=0;i<20;i++) { s.input.now_ms+=10; Nav_Tick(&s.nav,&s.input,&s.frame); }
        assert(s.nav.target==3 && s.nav.confirmed==2);
        puts("PASS single-frame mode does not repeatedly advance on the same snapshot");
        return;
    }
    assert(s.nav.target==2 && s.nav.arrival_count==1);
    for(i=0;i<20;i++) { s.input.now_ms+=10; Nav_Tick(&s.nav,&s.input,&s.frame); }
    assert(s.nav.target==2 && s.nav.arrival_count==1);
    for(i=1;i<NAV_FILTER_FRAMES;i++) { s.input.now_ms+=10; sensors(&s); Nav_Tick(&s.nav,&s.input,&s.frame); }
    assert(s.nav.target==3 && s.nav.confirmed==2);
    puts("PASS a repeated snapshot cannot confirm a waypoint multiple times");
}
static void test_pi_ramp(void)
{
    WheelPI pi={0,0}; unsigned i;
    for(i=0;i<1000;i++) WheelPI_Update(&pi,0,1,300,300,500);
    assert(pi.pwm==500);
    WheelPI_Reset(&pi); assert(pi.pwm==0 && pi.last_error==0);
    WheelPI_Update(&pi,0,0.1f,300,300,16700);
    WheelPI_Update(&pi,0,0,300,300,16700);
    assert(pi.pwm>0); /* A zero target does not magically clear accumulated I. */
    assert(fabsf(WheelRamp(0.2f,0.08f,0.005f)-0.195f)<0.00001f);
    assert(fabsf(WheelRamp(0.2f,-0.08f,0.005f)-0.195f)<0.00001f);
    assert(NavCalibration_Valid(&nav_calibration));
    puts("PASS PI clamp/reset and symmetric acceleration/deceleration/reversal ramp");
}

static void test_wheel_drive(void)
{
    WheelDrive wheel={0}, other={0};
    const float start_pwm=2000.0f;
    float full, fading, moving, tracking, turning, pwm=0;
    unsigned i;

    pwm=WheelDrive_Update(&wheel,0,0.008f,0,0,start_pwm,1);
    full=WheelDrive_Update(&wheel,0,0.08f,0,0,start_pwm,1);
    assert(pwm>0 && pwm<full && full>start_pwm);
    fading=WheelDrive_Update(&wheel,0.03f,0.08f,0,0,start_pwm,1);
    moving=WheelDrive_Update(&wheel,0.04f,0.08f,0,0,start_pwm,1);
    assert(full>fading && fading>moving && moving>0);
    assert(WheelDrive_Update(&other,0,0.08f,0,0,start_pwm*0.5f,1)==full-start_pwm*0.5f);

    for(i=0;i<19;i++) assert(WheelDrive_Update(&wheel,0.005f,0.08f,0,0,start_pwm,1)==moving);
    assert(WheelDrive_Update(&wheel,0.006f,0.08f,0,0,start_pwm,1)==moving);
    for(i=0;i<19;i++) assert(WheelDrive_Update(&wheel,0,0.08f,0,0,start_pwm,1)==moving);
    assert(WheelDrive_Update(&wheel,0,0.08f,0,0,start_pwm,1)==full);

    WheelDrive_Update(&wheel,0.04f,0.08f,0,0,start_pwm,1);
    assert(WheelDrive_Update(&wheel,0,-0.08f,0,0,start_pwm,1)==-full);
    WheelDrive_Update(&wheel,-0.04f,-0.08f,0,0,start_pwm,1);
    assert(WheelDrive_Update(&wheel,0,0,0,0,start_pwm,1)==0);
    assert(WheelDrive_Update(&wheel,0,-0.08f,0,0,start_pwm,1)==-full);

    WheelDrive_Reset(&wheel);
    tracking=WheelDrive_Update(&wheel,0.08f,0.08f,0,0,start_pwm,1);
    pwm=WheelDrive_Update(&wheel,0.10f,0.08f,0,0,start_pwm,1);
    assert(tracking>pwm && pwm>0);
    assert(WheelDrive_Update(&wheel,0.12f,0.08f,0,0,start_pwm,1)==0);
    assert(WheelDrive_Update(&wheel,0.12f,0.08f,NAV_WHEEL_KP,NAV_WHEEL_KI,start_pwm,1)<0);

    WheelDrive_Reset(&wheel);
    WheelDrive_Reset(&other);
    turning=WheelDrive_Update(&wheel,0,0.02f,0,0,start_pwm,0);
    assert(turning>0 && turning<WheelDrive_Update(&other,0,0.02f,0,0,start_pwm,1));
    assert(WheelDrive_Update(&wheel,0.02f,0.02f,0,0,start_pwm,0)<turning);

    WheelDrive_Reset(&wheel);
    for(i=0;i<1000;i++) {
        pwm=WheelDrive_Update(&wheel,0,0.08f,NAV_WHEEL_KP,NAV_WHEEL_KI,start_pwm,1);
        assert(pwm<=NAV_PWM_LIMIT);
    }
    assert(pwm==NAV_PWM_LIMIT && fabsf(wheel.pi.pwm-(NAV_PWM_LIMIT-full))<0.01f);
    WheelDrive_Reset(&wheel);
    assert(WheelDrive_Update(&wheel,0,0,NAV_WHEEL_KP,NAV_WHEEL_KI,start_pwm,1)==0);
    puts("PASS independent wheel startup, taper, reverse, stall rearm, braking and PI headroom");
}

static void test_heading_damping(void)
{
    Simulation s;
    unsigned i;
    assert(fabsf(Nav_HeadingError(3.13f,-3.13f)+0.0231853f)<0.0001f);
    assert(Nav_HeadingCorrection(0.01f,0.02f)==0);
    assert(Nav_HeadingCorrection(0.10f,0.50f)<0);
    assert(Nav_HeadingCorrection(-0.1f,1.0f)==-NAV_HEADING_MAX_RAD_S);

    start_sim(&s); s.nav.target=2; s.nav.confirmed=1; s.x=2700; s.y=0;
    memset(s.nav.filter,0,sizeof(s.nav.filter));
    s.input.wheel_max_mps=0;
    s.input.yaw_rate_rad_s=0.08f;
    for(i=0;i<NAV_FILTER_FRAMES+1U;i++) {
        s.input.now_ms+=10; sensors(&s); Nav_Tick(&s.nav,&s.input,&s.frame);
        assert(s.nav.output.vx==0 && s.nav.output.vy==0 && s.nav.output.wz<0);
        assert(s.nav.target==2 && s.nav.confirmed==1);
    }
    s.input.yaw_rate_rad_s=0;
    for(i=0;i<NAV_FILTER_FRAMES;i++) {
        s.input.now_ms+=10; sensors(&s); Nav_Tick(&s.nav,&s.input,&s.frame);
    }
    assert(s.nav.target==3 && s.nav.confirmed==2);
    puts("PASS gyro damping persists at the waypoint until rotation settles");
}

static void tick_cross_nudge(Simulation *s)
{
    s->input.now_ms+=10;
    s->frame.received_ms=s->input.now_ms;
    s->frame.sequence++;
    Nav_Tick(&s->nav,&s->input,&s->frame);
    assert(s->nav.output.drive_enabled && !s->nav.output.quick_stop && s->nav.reason==NAV_OK);
}

static void test_cross_nudge(void)
{
    Simulation s;
    unsigned elapsed;
    init_sim(&s);
    s.nav.state=NAV_RUN; s.nav.target=2; s.nav.confirmed=1;
    s.frame.mm[NAV_FRONT]=173; s.frame.mm[NAV_BACK]=4463;
    s.frame.mm[NAV_LEFT]=310; s.frame.mm[NAV_RIGHT]=2490;
    s.input.loaded_lateral=-1;
    s.input.wheel_max_mps=2*NAV_STOP_WHEEL_MPS;
    for(elapsed=0;elapsed<=NAV_CROSS_STALL_MS;elapsed+=10) {
        tick_cross_nudge(&s);
        assert(s.nav.output.vx==0 && s.nav.output.vy<0);
    }
    s.input.wheel_max_mps=0;
    for(elapsed=0;elapsed<NAV_CROSS_STALL_MS;elapsed+=10) {
        tick_cross_nudge(&s);
        assert(s.nav.output.vx==0 && s.nav.output.vy<0);
    }
    tick_cross_nudge(&s);
    assert(s.nav.output.reset_pi && s.nav.output.vx==NAV_CROSS_NUDGE_MPS && s.nav.output.vy==0);

    s.input.loaded_lateral=0;
    s.input.forward_step_mm=NAV_CROSS_NUDGE_MM*0.5f;
    tick_cross_nudge(&s);
    assert(!s.nav.output.reset_pi && s.nav.output.vx==NAV_CROSS_NUDGE_MPS && s.nav.output.vy==0);
    tick_cross_nudge(&s);
    assert(s.nav.output.reset_pi && s.nav.output.vx==0 && s.nav.output.vy<0);
    s.input.forward_step_mm=0; s.input.loaded_lateral=-1;
    for(elapsed=0;elapsed<=NAV_CROSS_STALL_MS;elapsed+=10) {
        tick_cross_nudge(&s);
        assert(s.nav.output.vx==0 && s.nav.output.vy<0);
    }
    assert(s.nav.target==2 && s.nav.confirmed==1);

    init_sim(&s);
    s.nav.state=NAV_RUN; s.nav.target=4; s.nav.confirmed=3;
    s.frame.mm[NAV_FRONT]=45; s.frame.mm[NAV_BACK]=45;
    s.frame.mm[NAV_LEFT]=2550; s.frame.mm[NAV_RIGHT]=250;
    s.input.loaded_lateral=1;
    for(elapsed=0;elapsed<=NAV_CROSS_STALL_MS;elapsed+=10) {
        tick_cross_nudge(&s);
        assert(s.nav.output.vx==0 && s.nav.output.vy>0);
    }
    s.frame.mm[NAV_BACK]=100;
    tick_cross_nudge(&s);
    assert(s.nav.output.reset_pi && s.nav.output.vx==-NAV_CROSS_NUDGE_MPS && s.nav.output.vy==0);
    s.input.loaded_lateral=0;
    for(elapsed=10;elapsed<NAV_CROSS_NUDGE_MAX_MS;elapsed+=10) {
        tick_cross_nudge(&s);
        assert(s.nav.output.vx==-NAV_CROSS_NUDGE_MPS && s.nav.output.vy==0);
    }
    tick_cross_nudge(&s);
    assert(s.nav.output.reset_pi && s.nav.output.vx==0 && s.nav.output.vy>0);
    assert(s.nav.target==4 && s.nav.confirmed==3);
    puts("PASS cross stall nudge in both directions, distance/timeout exit, one attempt and no extra stop");
}

int main(void)
{
    test_can(); test_freshness(); test_button(); test_pi_ramp(); test_wheel_drive(); test_heading_damping();
    test_cross_nudge();
    test_no_repeated_arrival(); test_complete(0); test_complete(1); test_recovery_pause_stop();
    puts("ALL TESTS PASSED (host simulation; not a physical acceptance test)");
    return 0;
}
