#include "nav_vofa.h"
#include <string.h>
typedef char require_float32[(sizeof(float)==4)?1:-1];
static void put_float(uint8_t *p,float value)
{
    uint32_t bits;
    memcpy(&bits,&value,4);
    p[0]=(uint8_t)bits; p[1]=(uint8_t)(bits>>8);
    p[2]=(uint8_t)(bits>>16); p[3]=(uint8_t)(bits>>24);
}
size_t NavVofa_Build(const NavStatus *s,uint32_t sequence,uint8_t *out,size_t capacity)
{
    float v[NAV_VOFA_CHANNELS];
    unsigned i;
    if(!s || !out || capacity<NAV_VOFA_FRAME_BYTES) return 0;
    for(i=0;i<4;i++) {
        v[i]=(float)s->raw_mm[i]; v[4+i]=s->filtered_mm[i];
        v[8+i]=s->reference_target_mm[i];
        v[25+i]=s->wheel_target[i]; v[29+i]=s->wheel_actual[i]; v[33+i]=s->wheel_pwm[i];
    }
    v[12]=(float)s->valid_mask; v[13]=(float)s->reference_mask;
    v[14]=s->lidar_age_ms==0xFFFFFFFFU?-1.0f:(float)s->lidar_age_ms;
    v[15]=(float)s->state;
    /* NAV48 接收端识别原有原因码。详细退场原因由 OLED 和串口 CSV 显示。 */
    v[16]=(float)(s->reason>=NAV_EXIT_RANGE?NAV_BAD_SCENE:s->reason);
    v[17]=(float)s->target; v[18]=(float)s->confirmed;
    v[19]=s->main_error_mm; v[20]=s->cross_error_mm;
    v[21]=s->heading_error_rad*57.295779513f;
    v[22]=s->command[0]; v[23]=s->command[1]; v[24]=s->command[2];
    v[37]=s->battery_voltage; v[38]=(float)s->motor_allowed;
    v[39]=(float)s->hardware_enable; v[40]=(float)s->software_stop;
    v[41]=(float)s->imu_ready; v[42]=(float)s->can_fresh; v[43]=(float)s->work_stopped;
    v[44]=s->start_front_mm; v[45]=(float)s->lidar_sequence;
    v[46]=(float)sequence; v[47]=(float)s->now_ms;
    for(i=0;i<NAV_VOFA_CHANNELS;i++) put_float(out+4*i,v[i]);
    out[192]=0; out[193]=0; out[194]=0x80; out[195]=0x7F;
    return NAV_VOFA_FRAME_BYTES;
}
uint32_t NavVofaQueue_NextSequence(const NavVofaQueue *q)
{
    return q->sequence>=NAV_VOFA_SEQUENCE_MAX?1U:q->sequence+1U;
}
int NavVofaQueue_Due(const NavVofaQueue *q,uint32_t now)
{
    return !q->busy && (!q->started || (uint32_t)(now-q->last_ms)>=NAV_BLE_PERIOD_MS);
}
int NavVofaQueue_Submit(NavVofaQueue *q,const uint8_t *packet,size_t bytes,uint32_t now)
{
    if(!packet || bytes!=NAV_VOFA_FRAME_BYTES || !NavVofaQueue_Due(q,now)) return 0;
    memcpy(q->bytes,packet,bytes);
    q->position=0; q->length=(uint16_t)bytes;
    q->last_ms=now; q->started=1; q->sequence=NavVofaQueue_NextSequence(q);
    q->busy=1;
    return 1;
}
int NavVofaQueue_Take(NavVofaQueue *q,uint8_t *out)
{
    if(!q->busy || q->position>=q->length) { q->busy=0; return 0; }
    *out=q->bytes[q->position++];
    if(q->position==q->length) q->busy=0;
    return 1;
}
