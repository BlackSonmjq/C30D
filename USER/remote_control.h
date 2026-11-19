#ifndef MANGO_REMOTE_CONTROL_H
#define MANGO_REMOTE_CONTROL_H

#include "nav_button.h"
#include "nav_core.h"

#define REMOTE_TIMEOUT_MS 200U

typedef struct {
    uint32_t received_ms;
    uint8_t ly, rx, mode_pressed, nav_start_pressed, valid, link_reset;
} RemoteSample;

typedef struct {
    NavButton mode_button, nav_start_button;
    uint8_t manual, armed;
} RemoteControl;

typedef enum { REMOTE_NONE, REMOTE_MODE_CHANGED, REMOTE_NAV_START } RemoteEvent;

void RemoteControl_Init(RemoteControl *remote);
/* 返回本周期的遥控事件。out 仅包含手动模式的命令。 */
RemoteEvent RemoteControl_Step(RemoteControl *remote, const RemoteSample *sample,
                               const NavInput *in, int can_switch, int can_start_nav, NavOutput *out);

#endif
