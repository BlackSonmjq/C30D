#include "nav_can.h"
static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0]<<8)|p[1]); }
int NavCan_Decode(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t dlc,
                  const uint8_t *p, NavLidar *out)
{
    if (!p || !out || id!=0x301U || ide || rtr || dlc!=8) return 0;
    out->mm[NAV_LEFT]=be16(p); out->mm[NAV_FRONT]=be16(p+2);
    out->mm[NAV_RIGHT]=be16(p+4); out->mm[NAV_BACK]=be16(p+6);
    return 1; /* 通道有效性由导航逐路判断，不改造线上 8 字节。 */
}
