#include "nav_core.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void check(unsigned point,float x,float y,unsigned mask)
{
    NavCore n; NavInput in; NavLidar f;
    memset(&in,0,sizeof(in)); memset(&f,0,sizeof(f));
    Nav_Init(&n,&nav_calibration,1); n.state=NAV_RUN; n.target=(uint8_t)point; n.confirmed=(uint8_t)(point-1);
    in.enabled=1; in.imu_ready=1; in.now_ms=100;
    f.sequence=1; f.received_ms=100;
    f.mm[NAV_LEFT]=(uint16_t)x; f.mm[NAV_RIGHT]=(uint16_t)(2800-x);
    f.mm[NAV_FRONT]=(uint16_t)(4300-y); f.mm[NAV_BACK]=(uint16_t)(100+y);
    if(x==100) f.mm[NAV_BACK]=777;
    if(point==8) { f.mm[NAV_BACK]=100; if(x>=550) f.mm[NAV_FRONT]=100; }
    Nav_Tick(&n,&in,&f);
    assert(n.reference_mask==mask);
    if(point==3) { assert(n.reference_target_mm[NAV_FRONT]==3600); assert(n.reference_target_mm[NAV_BACK]==800); }
    if(point==5) assert(!(n.reference_mask&(1U<<NAV_BACK)));
    if(point==8) assert(n.reference_target_mm[NAV_BACK]==100);
}
int main(void)
{
    check(3,2700,350,15); check(5,100,1050,13);
    check(8,400,2100,14); check(8,800,2100,15);
    puts("PASS reference diagnostics reflect real side/near/short-ridge choices and ignore outside rear");
    return 0;
}
