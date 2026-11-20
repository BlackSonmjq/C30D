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
#define VISION_ALIGN_CENTER_WINDOW 5U
#define VISION_ALIGN_CENTER_MASK 0x1FU /* Last five real trunk observations. */
#define VISION_ALIGN_DX_TOLERANCE 2
#define VISION_ALIGN_FRESH_MS 300UL
#define VISION_ALIGN_TIMEOUT_MS 10000UL
#define VISION_ALIGN_RESTART_DX 5
#define VISION_ALIGN_RESTART_FRAMES 3U
#define VISION_ALIGN_PARK_RESTART_MS 200UL
#define VISION_ALIGN_RETRY_PAUSE_MS 1000UL
#define VISION_ALIGN_DONE_HOLD_MS 1000UL
#define VISION_ALIGN_REAR_MM 45U
#define VISION_ALIGN_REAR_TOLERANCE_MM 3
#define VISION_ALIGN_REAR_KP 0.004f
#define VISION_ALIGN_REAR_MIN_MPS 0.04f
#define VISION_ALIGN_REAR_MAX_MPS 0.12f
#define VISION_ALIGN_SLOW_DX 8
#define VISION_ALIGN_REVERSE_FRAMES 3U
#define VISION_ALIGN_NEAR_MAX_MPS 0.06f
#define VISION_ALIGN_KP 0.010f
#define VISION_ALIGN_MIN_MPS 0.12f
#define VISION_ALIGN_MAX_MPS 0.24f
#define VISION_ALIGN_DX_HOLD_GAIN 1.5f

typedef enum {
    VISION_ALIGN_IDLE, VISION_ALIGN_ACQUIRE, VISION_ALIGN_TRACK,
    VISION_ALIGN_DONE, VISION_ALIGN_LOST, VISION_ALIGN_TIMEOUT, VISION_ALIGN_WAIT
} VisionAlign_Phase;
typedef enum {
    VISION_STOP_NONE, VISION_STOP_BASE, VISION_STOP_FRONT_LIMIT, VISION_STOP_REAR_LIMIT,
    VISION_STOP_LIDAR, VISION_STOP_GREEN_WAIT, VISION_STOP_POLE_WAIT, VISION_STOP_CONFIRM,
    VISION_STOP_REALIGN_WAIT, VISION_STOP_POSITION, VISION_STOP_WHEELS, VISION_STOP_HOLD,
    VISION_STOP_WORK, VISION_STOP_REPORT, VISION_STOP_COOLDOWN, VISION_STOP_BRAKE, VISION_STOP_RELEASE
} VisionStopReason;
typedef enum {
    VISION_RELEASE_NONE, VISION_RELEASE_GREEN_LOST, VISION_RELEASE_GREEN_RATIO,
    VISION_RELEASE_CAMERA_TIMEOUT, VISION_RELEASE_REGION_TIMEOUT, VISION_RELEASE_ALIGN_TIMEOUT,
    VISION_RELEASE_POST_HIT_EMPTY, VISION_RELEASE_POST_HIT_REGION, VISION_RELEASE_QUOTA,
    VISION_RELEASE_MODE, VISION_RELEASE_BASE, VISION_RELEASE_COOLDOWN, VISION_RELEASE_TARGET_INVALID
} VisionReleaseReason;
typedef struct {
    uint32_t last_frames, last_seen_ms, started_ms, stopped_ms, dx_seen_ms, restart_since_ms;
    uint32_t center_tick[VISION_ALIGN_CENTER_WINDOW];
    float dx_hold_speed;
    uint8_t phase, confirmed, zero_frames, restart_frames, manual_confirmed, approaching, reverse_frames;
    int8_t restart_sign, manual_sign, lateral_sign;
    uint8_t center_history;
} VisionAlign_State;
typedef struct { float x_mps, y_mps; uint8_t owned; } VisionAlign_Output;
typedef struct {
    uint8_t phase, locked, ly, rx, stick_active, ready;
    uint8_t pick_phase, hit_region, locked_region;
    uint32_t hit_count, lock_count;
    uint16_t blocked;
    uint32_t control_flags;
    float vx_mps, vy_mps, wz_radps, rc_velocity_mmps;
    float wheel_target[4], wheel_feedback[4], wheel_pwm[4];
    float yaw_target_deg, yaw_deg, yaw_error_deg, gyro_z_dps, heading_i_radps;
    uint8_t heading_locked, imu_fresh;
    uint8_t stop_reason, release_reason;
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
    s->dx_hold_speed = 0.0f;
    s->center_history = 0U;
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
    s->dx_hold_speed = 0.0f;
    s->center_history = 0U;
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
    uint8_t target = data->pole == 1U && data->green == 1U;
    float speed;
    int range_error = (int)rear_mm - (int)VISION_ALIGN_REAR_MM;
    uint8_t range_centered = range_valid && rear_mm >= 45U &&
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
            s->center_history = s->approaching = 0U;
            out.owned = 1U;
        }
        return out;
    }
    if (fresh && target) {
        if (s->phase == VISION_ALIGN_TRACK &&
            (uint32_t)(now_ms - s->dx_seen_ms) >= VISION_ALIGN_FRESH_MS) {
            s->phase = VISION_ALIGN_LOST;
            s->confirmed = s->zero_frames = s->center_history = s->approaching = 0U;
        }
        s->dx_seen_ms = now_ms;
        s->dx_hold_speed = 0.0f;
    }
    if (s->phase == VISION_ALIGN_TRACK && !target) {
        out.owned = 1U;
        s->zero_frames = 0U;
        /* Short trunk gaps pause approach without erasing recent centre evidence.
         * Only lateral tracking may use remembered DX; never advance blind. */
        if (data->green == 1U &&
            (uint32_t)(now_ms - s->dx_seen_ms) < VISION_ALIGN_FRESH_MS &&
            (uint32_t)(now_ms - s->started_ms) < VISION_ALIGN_TIMEOUT_MS) {
            if (!s->approaching && s->dx_hold_speed != 0.0f) {
                speed = s->dx_hold_speed * VISION_ALIGN_DX_HOLD_GAIN;
                if (speed > VISION_ALIGN_MAX_MPS) speed = VISION_ALIGN_MAX_MPS;
                if (speed < -VISION_ALIGN_MAX_MPS) speed = -VISION_ALIGN_MAX_MPS;
                out.x_mps = speed * VISION_ALIGN_DX_TO_X;
                out.y_mps = speed * VISION_ALIGN_DX_TO_Y;
            }
            return out;
        }
        s->dx_hold_speed = 0.0f;
        s->phase = VISION_ALIGN_LOST;
        s->confirmed = 0U;
        s->center_history = s->approaching = 0U;
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
        {
            uint32_t attempt_started_ms = s->started_ms;
            VisionAlign_Begin(s, now_ms);
            s->started_ms = attempt_started_ms;
        }
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
        s->center_history = s->approaching = 0U;
        return out;
    }
    if (fresh) {
        if (!target) {
            s->phase = VISION_ALIGN_LOST;
            s->confirmed = s->zero_frames = 0U;
            return out;
        }
        if (s->confirmed < VISION_ALIGN_CONFIRM_FRAMES) ++s->confirmed;
        if (magnitude >= VISION_ALIGN_RESTART_DX) {
            s->approaching = 0U;
            s->zero_frames = 0U;
            s->center_history = 0U;
        } else if (!s->approaching) {
            uint8_t i, centered = 0U;
            s->center_history = (uint8_t)(((s->center_history << 1U) |
                VisionAlign_Centered(error)) & VISION_ALIGN_CENTER_MASK);
            for (i = VISION_ALIGN_CENTER_WINDOW - 1U; i > 0U; --i)
                if (s->center_history & (1U << i)) s->center_tick[i] = s->center_tick[i-1U];
            s->center_tick[0] = now_ms;
            for (i = 0U; i < VISION_ALIGN_CENTER_WINDOW; ++i) {
                if (!(s->center_history & (1U << i))) continue;
                if ((uint32_t)(now_ms - s->center_tick[i]) >= VISION_ALIGN_FRESH_MS)
                    s->center_history &= (uint8_t)~(1U << i);
                else ++centered;
            }
            if (VisionAlign_Centered(error) && s->confirmed >= VISION_ALIGN_CONFIRM_FRAMES &&
                centered >= VISION_ALIGN_ZERO_FRAMES) {
                s->approaching = 1U;
                s->zero_frames = range_centered ? VISION_ALIGN_ZERO_FRAMES : 0U;
            } else s->zero_frames = 0U;
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
    if (magnitude < VISION_ALIGN_SLOW_DX && speed > VISION_ALIGN_NEAR_MAX_MPS)
        speed = VISION_ALIGN_NEAR_MAX_MPS;
    if (error < 0) speed = -speed;
    s->dx_hold_speed = speed;
    out.x_mps += speed * VISION_ALIGN_DX_TO_X;
    out.y_mps = speed * VISION_ALIGN_DX_TO_Y;
    return out;
}
/* Atomic phase snapshot for the existing OLED camera page. */
uint8_t VisionAlign_GetPhase(void);
uint8_t VisionAlign_ControlOwned(void);
void VisionAlign_GetDiagnostics(VisionAlign_Diagnostics *data);
#endif
