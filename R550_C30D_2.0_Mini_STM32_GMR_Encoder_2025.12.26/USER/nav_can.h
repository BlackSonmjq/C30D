#ifndef MANGO_NAV_CAN_H
#define MANGO_NAV_CAN_H
#include <stdint.h>
#include "nav_plan.h"
typedef struct {
    uint16_t mm[NAV_DIRECTIONS]; /* 内部 F B L R；线上 L F R B */
    uint32_t sequence, received_ms;
} NavLidar;
int NavCan_Decode(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t dlc,
                  const uint8_t *data, NavLidar *out);
#endif
