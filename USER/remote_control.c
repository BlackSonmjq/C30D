#include "remote_control.h"
#include <string.h>

#define REMOTE_STICK_CENTER 127
#define REMOTE_STICK_DEADZONE 20
#define REMOTE_AXIS_SPEED_MPS 0.50f

void RemoteControl_Init(RemoteControl *remote)
{
    memset(remote, 0, sizeof(*remote));
}

static int stick_delta(int delta)
{
    return delta > -REMOTE_STICK_DEADZONE && delta < REMOTE_STICK_DEADZONE ? 0 : delta;
}

RemoteEvent RemoteControl_Step(RemoteControl *remote, const RemoteSample *sample,
                               const NavInput *in, int can_switch, int can_start_nav, NavOutput *out)
{
    int forward, left;
    int fresh = sample->valid && (uint32_t)(in->now_ms - sample->received_ms) < REMOTE_TIMEOUT_MS;

    memset(out, 0, sizeof(*out));
    if (!fresh || sample->link_reset) {
        NavButton_Init(&remote->mode_button);
        NavButton_Init(&remote->nav_start_button);
        remote->armed = 0;
    }
    if (!fresh) return REMOTE_NONE;

    if (NavButton_Update(&remote->mode_button, sample->mode_pressed, 0,
                         can_switch, in->now_ms) == NAV_KEY_START) {
        remote->manual = (uint8_t)!remote->manual;
        remote->armed = 0;
        NavButton_Init(&remote->nav_start_button);
        return REMOTE_MODE_CHANGED;
    }

    if (!can_start_nav || remote->manual || sample->mode_pressed || remote->mode_button.stable) {
        NavButton_Init(&remote->nav_start_button);
    } else if (NavButton_Update(&remote->nav_start_button, sample->nav_start_pressed,
                                0, 1, in->now_ms) == NAV_KEY_START) {
        return REMOTE_NAV_START;
    }

    if (!remote->manual || !in->enabled) {
        remote->armed = 0;
        return REMOTE_NONE;
    }
    forward = stick_delta((int)sample->ly - REMOTE_STICK_CENTER);
    left = stick_delta(REMOTE_STICK_CENTER - (int)sample->rx);
    if (!remote->armed) {
        remote->armed = (uint8_t)(!forward && !left && in->wheel_max_mps <= NAV_STOP_WHEEL_MPS);
        return REMOTE_NONE;
    }
    out->vx = forward * (REMOTE_AXIS_SPEED_MPS / 128.0f);
    out->vy = left * (REMOTE_AXIS_SPEED_MPS / 128.0f);
    out->drive_enabled = (uint8_t)(forward != 0 || left != 0);
    return REMOTE_NONE;
}
