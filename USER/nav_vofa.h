#ifndef MANGO_NAV_VOFA_H
#define MANGO_NAV_VOFA_H
#include <stddef.h>
#include "nav_stm32.h"
#define NAV_VOFA_CHANNELS 48U
#define NAV_VOFA_FRAME_BYTES (NAV_VOFA_CHANNELS*4U+4U)
#define NAV_VOFA_SEQUENCE_MAX 16777215U
size_t NavVofa_Build(const NavStatus *s,uint32_t sequence,uint8_t *out,size_t capacity);
/* 单生产者、IRQ 消费；硬件适配层保护提交时的短拷贝。 */
typedef struct {
    uint8_t bytes[NAV_VOFA_FRAME_BYTES];
    volatile uint16_t position, length;
    volatile uint8_t busy;
    uint8_t started;
    uint32_t last_ms, sequence;
} NavVofaQueue;
int NavVofaQueue_Due(const NavVofaQueue *q,uint32_t now_ms);
int NavVofaQueue_Submit(NavVofaQueue *q,const uint8_t *packet,size_t bytes,uint32_t now_ms);
int NavVofaQueue_Take(NavVofaQueue *q,uint8_t *out);
uint32_t NavVofaQueue_NextSequence(const NavVofaQueue *q);
void NavBle_Poll(void);
void NavBle_TxIRQ(void);
#endif
