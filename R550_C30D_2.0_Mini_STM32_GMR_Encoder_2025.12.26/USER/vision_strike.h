#ifndef VISION_STRIKE_H
#define VISION_STRIKE_H
#include "camera_protocol.h"

/* Full workflow is owned by the 100 Hz chassis task, after alignment only.
 * Keep the independent VOFA-task dispatcher below disabled. */
#ifndef VISION_PICK_ENABLE
#define VISION_PICK_ENABLE 1
#endif
typedef enum {
    PICK_IDLE, PICK_WAIT, PICK_HOLD, PICK_REPORT, PICK_COOLDOWN, PICK_WORK
} VisionPickPhase;
/* Chassis publishes once; DATA_task clears after writing the completion line. */
extern volatile uint8_t vision_hit_report;
#define VISION_PICK_COOLDOWN_MS 2000UL
#define VISION_PICK_FRUIT_GAP_MS 2000UL
#define VISION_PICK_GREEN_GAP_MS 2000UL
#define VISION_PICK_GREEN_WINDOW_MS 2000UL
#define VISION_PICK_REARM_CLEAR_MS 300UL
#define VISION_PICK_STOP_HOLD_MS 200UL
#define VISION_PICK_MAX_REPEATS 0U
#define VISION_PICK_BOOT_MAX_ATTEMPTS 4U
#define VISION_ALIGN_BOOT_MAX_TREES 3U
#define VISION_PICK_STOP_MPS 0.005f
#define VISION_PICK_REAR_MIN_MM 40U
#define VISION_PICK_REAR_MAX_MM 80U

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

#define VISION_STRIKE_WINDOW_FRAMES 5U
static inline uint8_t VisionStrike_RegionEnabled(uint8_t region)
{
    return region>=1U && region<=4U && region!=3U;
}

typedef struct {
    uint32_t last_frames, last_seen_ms;
    uint32_t observed_ms[VISION_STRIKE_WINDOW_FRAMES];
    uint8_t region[VISION_STRIKE_WINDOW_FRAMES];
    uint8_t count, next, candidate, confirmed, clear_frames, latched;
} VisionStrike_State;

static inline void VisionStrike_ResetEvidence(VisionStrike_State *s)
{
    s->candidate = s->confirmed = s->clear_frames = s->count = s->next = 0U;
}

typedef struct {
    uint32_t sequence;
    uint16_t sample[3];
    uint8_t count, next;
} VisionPickRange_State;

/* A CAN snapshot is one sample, regardless of how often the chassis reads it. */
static inline uint16_t VisionPickRange_Update(VisionPickRange_State *s, uint16_t mm,
                                             uint32_t sequence, uint8_t fresh)
{
    uint16_t a, b, c, swap;
    if (!fresh || !sequence || !mm) {
        s->sequence = 0U;
        s->count = s->next = 0U;
        return 0U;
    }
    if (sequence != s->sequence) {
        s->sequence = sequence;
        s->sample[s->next] = mm;
        s->next = (uint8_t)((s->next + 1U) % 3U);
        if (s->count < 3U) ++s->count;
    }
    if (s->count < 3U) return mm;
    a = s->sample[0]; b = s->sample[1]; c = s->sample[2];
    if (a > b) { swap = a; a = b; b = swap; }
    if (b > c) { swap = b; b = c; c = swap; }
    return a > b ? a : b;
}

/* Count newly observed valid frames, never repeated snapshots. Blocked work
 * is discarded. After acceptance, require target absence while idle to rearm;
 * changing regions alone must not repeatedly strike the same fruit. */
static inline uint8_t VisionStrike_Update(VisionStrike_State *s, const Camera_Data *data,
                                         uint32_t now_ms, uint8_t blocked)
{
    uint8_t target, votes[4] = {0U}, best = 0U, winner = 0U;
    unsigned i;
    if (s->last_frames && (uint32_t)(now_ms - s->last_seen_ms) >= VISION_STRIKE_MAX_GAP_MS) {
        VisionStrike_ResetEvidence(s);
    }
    if (blocked) VisionStrike_ResetEvidence(s);
    if (!data->frames || data->frames == s->last_frames) return 0U;
    s->last_frames = data->frames;
    s->last_seen_ms = now_ms;
    if (blocked) return 0U;
    target = data->pole == 1U && data->green == 1U && VisionStrike_RegionEnabled(data->region);
    if (s->latched) {
        if (target || (data->green==1U && data->region==3U)) s->clear_frames = 0U;
        else if (++s->clear_frames >= VISION_STRIKE_CLEAR_FRAMES) {
            s->latched = s->clear_frames = 0U;
        }
        return 0U;
    }
    if (!target) {
        VisionStrike_ResetEvidence(s);
        return 0U;
    }
    s->region[s->next] = data->region;
    s->observed_ms[s->next] = now_ms;
    s->next = (uint8_t)((s->next + 1U) % VISION_STRIKE_WINDOW_FRAMES);
    if (s->count < VISION_STRIKE_WINDOW_FRAMES) ++s->count;
    for (i = 0U; i < s->count; ++i) {
        if ((uint32_t)(now_ms - s->observed_ms[i]) < VISION_STRIKE_MAX_GAP_MS)
            ++votes[s->region[i] - 1U];
    }
    for (i = 0U; i < 4U; ++i) {
        if (votes[i] > best) { best = votes[i]; winner = (uint8_t)(i + 1U); }
    }
    /* Preserve the old candidate on a tie; three of five is always unique. */
    if (!s->candidate || votes[s->candidate - 1U] < best) s->candidate = winner;
    s->confirmed = best;
    return best >= VISION_STRIKE_CONFIRM_FRAMES && data->region == s->candidate ? s->candidate : 0U;
}

static inline void VisionStrike_Accept(VisionStrike_State *s)
{
    s->latched = 1U;
    VisionStrike_ResetEvidence(s);
}

/* Task context only. Camera RX ISR never starts mechanisms or calls RTOS. */
void VisionStrike_Process(void);
#endif
