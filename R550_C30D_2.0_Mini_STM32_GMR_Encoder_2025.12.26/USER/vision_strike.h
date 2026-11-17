#ifndef VISION_STRIKE_H
#define VISION_STRIKE_H
#include "camera_protocol.h"

/* REGION 1..4 maps directly to existing K1..K4 complete picking jobs. */
#ifndef VISION_STRIKE_ENABLE
#define VISION_STRIKE_ENABLE 0
#endif
#ifndef VISION_STRIKE_CONFIRM_FRAMES
#define VISION_STRIKE_CONFIRM_FRAMES 3U
#endif
#ifndef VISION_STRIKE_CLEAR_FRAMES
#define VISION_STRIKE_CLEAR_FRAMES 3U
#endif
#ifndef VISION_STRIKE_MAX_GAP_MS
#define VISION_STRIKE_MAX_GAP_MS 300UL
#endif

typedef struct {
    uint32_t last_frames, last_seen_ms;
    uint8_t candidate, confirmed, clear_frames, latched;
} VisionStrike_State;

/* Count newly observed valid frames, never repeated snapshots. Blocked work
 * is discarded. After acceptance, require target absence while idle to rearm;
 * changing regions alone must not repeatedly strike the same fruit. */
static inline uint8_t VisionStrike_Update(VisionStrike_State *s, const Camera_Data *data,
                                         uint32_t now_ms, uint8_t blocked)
{
    uint8_t target;
    if (s->last_frames && (uint32_t)(now_ms - s->last_seen_ms) >= VISION_STRIKE_MAX_GAP_MS) {
        s->candidate = s->confirmed = s->clear_frames = 0U;
    }
    if (blocked) s->candidate = s->confirmed = s->clear_frames = 0U;
    if (!data->frames || data->frames == s->last_frames) return 0U;
    s->last_frames = data->frames;
    s->last_seen_ms = now_ms;
    if (blocked) return 0U;
    target = data->pole == 1U && data->green == 1U && data->region >= 1U && data->region <= 4U;
    if (s->latched) {
        if (target) s->clear_frames = 0U;
        else if (++s->clear_frames >= VISION_STRIKE_CLEAR_FRAMES) {
            s->latched = s->clear_frames = 0U;
        }
        return 0U;
    }
    if (!target) {
        s->candidate = s->confirmed = 0U;
        return 0U;
    }
    if (s->candidate != data->region) {
        s->candidate = data->region;
        s->confirmed = 1U;
    } else if (s->confirmed < VISION_STRIKE_CONFIRM_FRAMES) {
        ++s->confirmed;
    }
    return s->confirmed >= VISION_STRIKE_CONFIRM_FRAMES ? s->candidate : 0U;
}

static inline void VisionStrike_Accept(VisionStrike_State *s)
{
    s->latched = 1U;
    s->candidate = s->confirmed = s->clear_frames = 0U;
}

/* Task context only. Camera RX ISR never starts mechanisms or calls RTOS. */
void VisionStrike_Process(void);
#endif
