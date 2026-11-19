#include "nav_vofa.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static float get_float(const uint8_t *p)
{
    uint32_t u=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
    float f; memcpy(&f,&u,4); return f;
}
int main(int argc,char **argv)
{
    NavStatus s;
    NavVofaQueue q;
    uint8_t packet[NAV_VOFA_FRAME_BYTES], copy[NAV_VOFA_FRAME_BYTES], byte;
    unsigned i;
    memset(&s,0,sizeof(s)); memset(&q,0,sizeof(q));
    s.raw_mm[0]=100; s.raw_mm[1]=120; s.raw_mm[2]=300; s.raw_mm[3]=2500;
    for(i=0;i<4;i++) { s.filtered_mm[i]=(float)s.raw_mm[i]; s.reference_target_mm[i]=(float)s.raw_mm[i];
        s.wheel_target[i]=0.2f+0.01f*(float)i; s.wheel_actual[i]=0.18f+0.01f*(float)i;
        s.wheel_pwm[i]=(i&1U)?-100.0f*(float)(i+1U):100.0f*(float)(i+1U); }
    s.reference_target_mm[1]=0;
    s.valid_mask=15; s.reference_mask=13; s.lidar_age_ms=20;
    s.state=NAV_READY; s.reason=NAV_OK; s.target=1; s.confirmed=0;
    s.main_error_mm=12.5f; s.cross_error_mm=-6.25f; s.heading_error_rad=0.1f;
    s.command[0]=0.2f; s.command[1]=-0.03f; s.command[2]=0.1f;
    s.battery_voltage=12.2f; s.motor_allowed=1; s.hardware_enable=1;
    s.imu_ready=1; s.can_fresh=1; s.start_front_mm=4800; s.lidar_sequence=5; s.now_ms=1234;
    memset(packet,0xCC,sizeof(packet));
    assert(NavVofa_Build(&s,1,packet,sizeof(packet)-1)==0 && packet[0]==0xCC);
    assert(NavVofa_Build(&s,1,packet,sizeof(packet))==196);
    assert(get_float(packet)==100 && get_float(packet+4)==120);
    assert(get_float(packet+13*4)==13 && get_float(packet+46*4)==1 && get_float(packet+47*4)==1234);
    assert(packet[192]==0 && packet[193]==0 && packet[194]==0x80 && packet[195]==0x7F);
    memcpy(copy,packet,sizeof(copy));
    assert(NavVofaQueue_Due(&q,100));
    assert(NavVofaQueue_Submit(&q,packet,sizeof(packet),100) && q.sequence==1);
    assert(!NavVofaQueue_Submit(&q,packet,sizeof(packet),400)); /* Cannot overwrite busy TX. */
    for(i=0;i<196;i++) { assert(NavVofaQueue_Take(&q,&byte)); assert(byte==copy[i]); }
    assert(!NavVofaQueue_Take(&q,&byte) && !q.busy);
    assert(!NavVofaQueue_Due(&q,399) && NavVofaQueue_Due(&q,400));
    assert(NavVofaQueue_Submit(&q,packet,sizeof(packet),400) && q.sequence==2);
    q.busy=0; q.last_ms=0xFFFFFFFFU-150U;
    assert(!NavVofaQueue_Due(&q,148) && NavVofaQueue_Due(&q,149));
    q.sequence=NAV_VOFA_SEQUENCE_MAX; assert(NavVofaQueue_NextSequence(&q)==1);
    if(argc>1) {
        FILE *file=fopen(argv[1],"wb"); assert(file);
        assert(fwrite(copy,1,sizeof(copy),file)==sizeof(copy)); assert(fclose(file)==0);
    }
    s.lidar_age_ms=0xFFFFFFFFU; NavVofa_Build(&s,1,packet,sizeof(packet));
    assert(get_float(packet+14*4)==-1);
    for(i=NAV_EXIT_RANGE;i<=NAV_EXIT_TIMEOUT;i++) {
        s.reason=(uint8_t)i;
        assert(NavVofa_Build(&s,1,packet,sizeof(packet))==196);
        assert(get_float(packet+16*4)==NAV_BAD_SCENE);
    }
    puts("PASS firmware 48-channel LE layout, tail, bounded buffer, TX busy/period, time wrap and exact sequence");
    return 0;
}
