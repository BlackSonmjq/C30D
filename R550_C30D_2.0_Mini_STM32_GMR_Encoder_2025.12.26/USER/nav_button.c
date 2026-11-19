#include "nav_button.h"
#include "nav_config.h"
#include <string.h>
void NavButton_Init(NavButton *b) { memset(b,0,sizeof(*b)); }
unsigned NavButton_Update(NavButton *b,int pressed,int running,int can_start,uint32_t now)
{
    uint8_t value=(uint8_t)(pressed!=0);
    if(!b->initialized) {
        b->initialized=1; b->raw=value; b->stable=value; b->changed_ms=now;
        b->saw_release=(uint8_t)!value; return NAV_KEY_NONE;
    }
    if(value!=b->raw) { b->raw=value; b->changed_ms=now; }
    if((uint32_t)(now-b->changed_ms)<NAV_KEY_DEBOUNCE_MS || value==b->stable) return NAV_KEY_NONE;
    b->stable=value;
    if(!value) {
        b->saw_release=1;
        if(b->armed) { b->armed=0; return !running&&can_start?NAV_KEY_START:NAV_KEY_NONE; }
    } else if(b->saw_release) {
        if(running) { b->armed=0; return NAV_KEY_STOP; }
        b->armed=(uint8_t)(can_start!=0);
    }
    return NAV_KEY_NONE;
}
