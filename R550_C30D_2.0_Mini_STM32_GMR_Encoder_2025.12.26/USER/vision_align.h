#ifndef VISION_ALIGN_H
#define VISION_ALIGN_H
#include "camera_protocol.h"

#ifndef VISION_ALIGN_ENABLE
#define VISION_ALIGN_ENABLE 1
#endif
/* Body velocity per positive DX: X forward, Y left. Set for camera mounting. */
#ifndef VISION_ALIGN_DX_TO_X
#define VISION_ALIGN_DX_TO_X 0.0f
#endif
#ifndef VISION_ALIGN_DX_TO_Y
#define VISION_ALIGN_DX_TO_Y 1.0f
#endif
#define VISION_ALIGN_CONFIRM_FRAMES 3U
#define VISION_ALIGN_RECOVER_FRAMES 6U
#define VISION_ALIGN_ZERO_FRAMES 3U
#define VISION_ALIGN_DX_TOLERANCE 2
#define VISION_ALIGN_FRESH_MS 300UL
#define VISION_ALIGN_TIMEOUT_MS 10000UL
#define VISION_ALIGN_RESTART_DX 5
#define VISION_ALIGN_RESTART_FRAMES 3U
#define VISION_ALIGN_RETRY_PAUSE_MS 1000UL
#define VISION_ALIGN_DONE_HOLD_MS 1000UL
#define VISION_ALIGN_REAR_MM 50U
#define VISION_ALIGN_REAR_TOLERANCE_MM 3
#define VISION_ALIGN_REAR_KP 0.003f
#define VISION_ALIGN_REAR_MIN_MPS 0.02f
#define VISION_ALIGN_REAR_MAX_MPS 0.12f
#define VISION_ALIGN_SLOW_DX 8
#define VISION_ALIGN_REVERSE_FRAMES 2U
#define VISION_ALIGN_KP 0.010f
#define VISION_ALIGN_MIN_MPS 0.12f
#define VISION_ALIGN_MAX_MPS 0.24f

typedef enum {
    VISION_ALIGN_IDLE, VISION_ALIGN_ACQUIRE, VISION_ALIGN_TRACK,
    VISION_ALIGN_DONE, VISION_ALIGN_LOST, VISION_ALIGN_TIMEOUT, VISION_ALIGN_WAIT
} VisionAlign_Phase;
typedef struct {
    uint32_t last_frames, last_seen_ms, started_ms, stopped_ms;
    uint8_t phase, confirmed, zero_frames, restart_frames, manual_confirmed, approaching, reverse_frames;
    int8_t restart_sign, manual_sign, lateral_sign;
} VisionAlign_State;
typedef struct { float x_mps, y_mps; uint8_t owned; } VisionAlign_Output;
typedef struct {
    uint8_t phase, locked, ly, rx, stick_active, ready;
    uint8_t pick_phase, hit_region;
    uint32_t hit_count;
    uint16_t blocked;
    uint32_t control_flags;
    float vx_mps, vy_mps, wz_radps, rc_velocity_mmps;
    float wheel_target[4], wheel_feedback[4], wheel_pwm[4];
    float yaw_target_deg, yaw_deg, yaw_error_deg, gyro_z_dps, heading_i_radps;
    uint8_t heading_locked, imu_fresh;
} VisionAlign_Diagnostics;

static inline uint8_t VisionAlign_Centered(int dx)
{
    return dx >= -VISION_ALIGN_DX_TOLERANCE && dx <= VISION_ALIGN_DX_TOLERANCE;
}

static inline void VisionAlign_Cancel(VisionAlign_State *s)
{
    s->phase = VISION_ALIGN_IDLE;
    s->confirmed = s->zero_frames = 0U;
    s->restart_frames = 0U;
    s->restart_sign = 0;
    s->manual_confirmed = 0U;
    s->manual_sign = 0;
    s->approaching = 0U;
    s->reverse_frames = 0U;
    s->lateral_sign = 0;
}

static inline void VisionAlign_Begin(VisionAlign_State *s, uint32_t now_ms)
{
    s->started_ms = now_ms;
    s->phase = VISION_ALIGN_ACQUIRE;
    s->confirmed = s->zero_frames = s->restart_frames = 0U;
    s->restart_sign = 0;
    s->manual_confirmed = 0U;
    s->manual_sign = 0;
    s->approaching = 0U;
    s->reverse_frames = 0U;
    s->lateral_sign = 0;
}

/* One chassis owner calls this at 100 Hz; only new valid camera snapshots
 * qualify/acquire/complete. Rear range must also be centered to finish. */
static inline VisionAlign_Output VisionAlign_Update(VisionAlign_State *s,
    const Camera_Data *data, uint32_t now_ms, uint8_t manual_active, uint8_t ready,
    uint16_t rear_mm, uint8_t range_valid)
{
    VisionAlign_Output out = {0.0f, 0.0f, 0U};
    uint8_t fresh = data->frames != s->last_frames;
    uint8_t expired = (uint32_t)(now_ms - s->last_seen_ms) >= VISION_ALIGN_FRESH_MS;
    uint8_t starting = 0U;
    uint8_t target = data->pole == 1U && data->green == 1U &&
        data->region >= 1U && data->region <= 4U;
    float speed;
    int range_error = (int)rear_mm - (int)VISION_ALIGN_REAR_MM;
    uint8_t range_centered = range_valid &&
        range_error >= -VISION_ALIGN_REAR_TOLERANCE_MM &&
        range_error <= VISION_ALIGN_REAR_TOLERANCE_MM;
    int error = data->offset;
    int magnitude = error < 0 ? -error : error;
    if (fresh) { s->last_frames = data->frames; s->last_seen_ms = now_ms; }
    if (!ready) {
        VisionAlign_Cancel(s);
        return out;
    }
    /* A new G=0 snapshot releases visual ownership, including the DONE hold. */
    if (fresh && data->green == 0U) {
        VisionAlign_Cancel(s);
        s->phase = VISION_ALIGN_WAIT;
        return out;
    }
    if (!range_valid) {
        if (s->phase != VISION_ALIGN_IDLE && s->phase != VISION_ALIGN_WAIT) {
            if (s->phase != VISION_ALIGN_DONE) s->phase = VISION_ALIGN_LOST;
            s->confirmed = s->zero_frames = s->restart_frames = 0U;
            out.owned = 1U;
        }
        return out;
    }
    /* Completion owns a full zero for one second, including held sticks.
     * Keep consuming snapshots so hold-period data cannot count for reentry. */
    if (s->phase == VISION_ALIGN_DONE &&
        (uint32_t)(now_ms - s->stopped_ms) < VISION_ALIGN_DONE_HOLD_MS) {
        out.owned = 1U;
        return out;
    }
    /* A flickering trunk cannot repeatedly restart the motors. Keep ownership
     * and zero output until six consecutive new valid snapshots recover it. */
    if (s->phase == VISION_ALIGN_LOST) {
        out.owned = 1U;
        if (expired || (fresh && !target)) s->confirmed = 0U;
        if (fresh && target && s->confirmed < VISION_ALIGN_RECOVER_FRAMES) ++s->confirmed;
        if (s->confirmed < VISION_ALIGN_RECOVER_FRAMES) return out;
        VisionAlign_Begin(s, now_ms);
        s->confirmed = VISION_ALIGN_CONFIRM_FRAMES;
        starting = 1U;
    }
    /* ACQUIRE/TRACK are fully automatic; keep reading but ignore the sticks. */
    if (manual_active && s->phase != VISION_ALIGN_ACQUIRE && s->phase != VISION_ALIGN_TRACK) {
        /* Keep manual motion during confirmation; three NEW trunk/green-fruit snapshots
         * take over even with a held stick. Cached frames cannot qualify.
         * After DONE, near-centre data must not repeatedly steal control. */
        if (expired || (fresh && !target) ||
            (s->phase == VISION_ALIGN_DONE && (magnitude < VISION_ALIGN_RESTART_DX && range_centered)))
            s->manual_confirmed = 0U;
        if (fresh && target &&
            (s->phase != VISION_ALIGN_DONE || (magnitude >= VISION_ALIGN_RESTART_DX || !range_centered))) {
            int restart_error = VisionAlign_Centered(error) ? range_error : error;
            int8_t sign = restart_error > 0 ? 1 : restart_error < 0 ? -1 : 0;
            if (s->phase == VISION_ALIGN_DONE && sign != s->manual_sign)
                s->manual_confirmed = 0U;
            s->manual_sign = sign;
            if (s->manual_confirmed < VISION_ALIGN_CONFIRM_FRAMES) ++s->manual_confirmed;
        }
        if (s->manual_confirmed < VISION_ALIGN_CONFIRM_FRAMES) {
            if (s->phase != VISION_ALIGN_DONE) s->phase = VISION_ALIGN_WAIT;
            return out;
        }
        VisionAlign_Begin(s, now_ms);
        s->confirmed = VISION_ALIGN_CONFIRM_FRAMES;
        starting = 1U;
    }
    /* A ready chassis starts on fresh trunk/green-fruit targets without a hand controller. */
    if (s->phase == VISION_ALIGN_IDLE) s->phase = VISION_ALIGN_WAIT;
    if (s->phase == VISION_ALIGN_WAIT && fresh && target) {
        VisionAlign_Begin(s, now_ms);
        starting = 1U;
    }
    if (s->phase == VISION_ALIGN_WAIT) return out;
    out.owned = 1U;
    if (s->phase == VISION_ALIGN_DONE) {
        out.owned = 0U; /* Hold expired: release to manual/idle while monitoring. */
        if (expired || (fresh && !target)) {
            out.owned = 1U;
            s->phase = VISION_ALIGN_LOST;
            s->restart_frames = s->confirmed = s->zero_frames = 0U;
            return out;
        }
        if (!fresh) return out;
        if (magnitude < VISION_ALIGN_RESTART_DX && range_centered) {
            s->restart_frames = 0U;
            s->restart_sign = 0;
            return out;
        }
        /* Stable same-side error rearms; cached data, near-zero jitter or
         * alternating signs cannot immediately restart a completed cycle. */
        if (s->restart_sign != ((VisionAlign_Centered(error) ? range_error : error) > 0 ? 1 : -1)) {
            s->restart_sign = (VisionAlign_Centered(error) ? range_error : error) > 0 ? 1 : -1;
            s->restart_frames = 1U;
        } else if (s->restart_frames < VISION_ALIGN_RESTART_FRAMES) ++s->restart_frames;
        if (s->restart_frames < VISION_ALIGN_RESTART_FRAMES) return out;
        VisionAlign_Begin(s, now_ms);
        s->confirmed = VISION_ALIGN_CONFIRM_FRAMES - 1U;
        starting = 1U;
        out.owned = 1U;
    }
    if (s->phase == VISION_ALIGN_TIMEOUT) {
        if ((uint32_t)(now_ms - s->stopped_ms) < VISION_ALIGN_RETRY_PAUSE_MS) return out;
        if (!fresh || !target) return out;
        VisionAlign_Begin(s, now_ms);
        starting = 1U;
    }
    if ((uint32_t)(now_ms - s->started_ms) >= VISION_ALIGN_TIMEOUT_MS) {
        s->phase = VISION_ALIGN_TIMEOUT;
        s->stopped_ms = now_ms;
        return out;
    }
    if ((!starting && expired) || (uint32_t)(now_ms - s->last_seen_ms) >= VISION_ALIGN_FRESH_MS) {
        s->phase = VISION_ALIGN_LOST;
        s->confirmed = s->zero_frames = 0U;
        return out;
    }
    if (fresh) {
        if (!target) {
            s->phase = VISION_ALIGN_LOST;
            s->confirmed = s->zero_frames = 0U;
            return out;
        }
        if (s->confirmed < VISION_ALIGN_CONFIRM_FRAMES) ++s->confirmed;
        if (!VisionAlign_Centered(error) &&
            (!s->approaching || magnitude >= VISION_ALIGN_RESTART_DX)) {
            s->approaching = 0U;
            s->zero_frames = 0U;
        } else if (!s->approaching) {
            if (s->zero_frames < VISION_ALIGN_ZERO_FRAMES) ++s->zero_frames;
            if (s->confirmed >= VISION_ALIGN_CONFIRM_FRAMES &&
                s->zero_frames >= VISION_ALIGN_ZERO_FRAMES) {
                s->approaching = 1U;
                s->zero_frames = range_centered ? VISION_ALIGN_ZERO_FRAMES : 0U;
            }
        } else if (VisionAlign_Centered(error) && range_centered) {
            if (s->zero_frames < VISION_ALIGN_ZERO_FRAMES) ++s->zero_frames;
        } else s->zero_frames = 0U;
        if (s->confirmed >= VISION_ALIGN_CONFIRM_FRAMES) s->phase = VISION_ALIGN_TRACK;
        if (s->confirmed >= VISION_ALIGN_CONFIRM_FRAMES &&
            s->approaching && s->zero_frames >= VISION_ALIGN_ZERO_FRAMES) {
            s->phase = VISION_ALIGN_DONE;
            s->stopped_ms = now_ms;
            s->manual_confirmed = s->restart_frames = 0U;
            return out;
        }
    }
    if (s->phase != VISION_ALIGN_TRACK) return out;
    if (s->approaching && !range_centered) {
        speed = (float)(range_error < 0 ? -range_error : range_error) * VISION_ALIGN_REAR_KP;
        if (speed < VISION_ALIGN_REAR_MIN_MPS) speed = VISION_ALIGN_REAR_MIN_MPS;
        if (speed > VISION_ALIGN_REAR_MAX_MPS) speed = VISION_ALIGN_REAR_MAX_MPS;
        out.x_mps = range_error > 0 ? -speed : speed;
    }
    if (VisionAlign_Centered(error) || s->approaching) {
        s->reverse_frames = 0U;
        return out;
    }
    if (magnitude < VISION_ALIGN_SLOW_DX && s->lateral_sign &&
        s->lateral_sign != (error > 0 ? 1 : -1)) {
        if (fresh && s->reverse_frames < VISION_ALIGN_REVERSE_FRAMES) ++s->reverse_frames;
        if (s->reverse_frames < VISION_ALIGN_REVERSE_FRAMES) return out;
    }
    s->lateral_sign = error > 0 ? 1 : -1;
    s->reverse_frames = 0U;
    speed = (float)(error < 0 ? -error : error) * VISION_ALIGN_KP;
    if (speed < VISION_ALIGN_MIN_MPS) speed = VISION_ALIGN_MIN_MPS;
    if (speed > VISION_ALIGN_MAX_MPS) speed = VISION_ALIGN_MAX_MPS;
    if (magnitude < VISION_ALIGN_SLOW_DX)
        speed *= (float)magnitude / VISION_ALIGN_SLOW_DX;
    if (error < 0) speed = -speed;
    out.x_mps += speed * VISION_ALIGN_DX_TO_X;
    out.y_mps = speed * VISION_ALIGN_DX_TO_Y;
    return out;
}
/* Atomic phase snapshot for the existing OLED camera page. */
uint8_t VisionAlign_GetPhase(void);
void VisionAlign_GetDiagnostics(VisionAlign_Diagnostics *data);
#endif
