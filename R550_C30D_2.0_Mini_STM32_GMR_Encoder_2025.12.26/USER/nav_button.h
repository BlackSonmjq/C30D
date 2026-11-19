#ifndef MANGO_NAV_BUTTON_H
#define MANGO_NAV_BUTTON_H
#include <stdint.h>
enum { NAV_KEY_NONE, NAV_KEY_START, NAV_KEY_STOP };
typedef struct {
    uint32_t changed_ms;
    uint8_t raw, stable, initialized, saw_release, armed;
} NavButton;
void NavButton_Init(NavButton *b);
unsigned NavButton_Update(NavButton *b,int pressed,int running,int can_start,uint32_t now);
#endif
