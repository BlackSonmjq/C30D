#include "route_nav.h"
#include "route_config.h"
#include <math.h>
#include <string.h>

static float limit_trim(float error)
{
    float speed;
    if (fabsf(error) <= ROUTE_TRIM_DEADBAND_MM) return 0.0f;
    speed = error * ROUTE_TRIM_KP;
    if (speed > ROUTE_TRIM_MAX_MPS) return ROUTE_TRIM_MAX_MPS;
    if (speed < -ROUTE_TRIM_MAX_MPS) return -ROUTE_TRIM_MAX_MPS;
    return speed;
}

float RouteNav_WallTrim(uint16_t mm, uint16_t target_mm)
{
    float error = (float)mm - target_mm;
    return mm != 0U && target_mm != 0U && fabsf(error) <= ROUTE_TRIM_VALID_MM ?
           limit_trim(error) : 0.0f;
}

float RouteNav_LaneVx(uint16_t front, uint16_t behind)
{
    float ef = (float)front - ROUTE_PHYSICAL_FRONT_CENTER_MM;
    float eb = (float)behind - ROUTE_PHYSICAL_REAR_CENTER_MM;
    uint8_t fv = front != 0U && ROUTE_PHYSICAL_FRONT_CENTER_MM != 0U && fabsf(ef) <= ROUTE_TRIM_VALID_MM;
    uint8_t bv = behind != 0U && ROUTE_PHYSICAL_REAR_CENTER_MM != 0U && fabsf(eb) <= ROUTE_TRIM_VALID_MM;
    if (fv && bv) return limit_trim((ef - eb) * 0.5f);
    if (fv) return limit_trim(ef);
    if (bv) return limit_trim(-eb);
    return 0.0f;
}

/* 保留原距离函数供调试调用；实际导航还须捕获可信参考并用新帧确认。 */
RouteMarkResult RouteNav_CheckMark(const RouteMark *mark, uint16_t mm, uint8_t increases)
{
    float remaining;
    if (mm == 0U || mark->target_mm == 0U) return ROUTE_MARK_HOLD;
    remaining = increases ? (float)mark->target_mm - mm : (float)mm - mark->target_mm;
    if (remaining < -(float)mark->turn_band_mm) return ROUTE_MARK_HOLD;
    if (remaining <= mark->turn_band_mm) return ROUTE_MARK_SWITCH;
    return remaining <= mark->slow_band_mm ? ROUTE_MARK_SLOW : ROUTE_MARK_CRUISE;
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t swap;
    if (a > b) { swap = a; a = b; b = swap; }
    if (b > c) { swap = b; b = c; c = swap; }
    return a > b ? a : b;
}

static uint8_t raw_valid(uint16_t mm)
{
    return mm >= ROUTE_LIDAR_MIN_MM && mm <= ROUTE_LIDAR_MAX_MM && mm != 65535U;
}

static void reset_aux_recheck(RouteAuxTracker *a)
{
    a->recheck_frames = 0U;
    a->recheck_at = 0U;
}

static void invalidate_aux_recheck(RouteController *c)
{
    RouteAuxTracker *a = &c->auxiliary;
    reset_aux_recheck(a);
    a->revalidated = a->near_verified = 0U;
    if (a->near_arrival) {
        a->interrupted = 1U;
        a->settle_at = 0U;
        c->status.aux_good_frames = c->status.confirmed = 0U;
        c->status.aux_confidence = ROUTE_CONF_CANDIDATE;
        c->status.aux_reject_reason = ROUTE_REJECT_WARMUP;
    }
}

/* 序号来自接收端实际发布的新 CAN 帧。重复读同一帧不会推进任何确认计数。 */
static uint8_t update_filters(RouteController *c, const RouteInputs *in)
{
    uint8_t direction, updated = 0U;
    if (in->lidar.sequence == 0U || in->lidar.sequence == c->sequence) return 0U;
    c->sequence = in->lidar.sequence;
    if (!in->lidar.fresh) return 0U;
    for (direction = 0U; direction < ROUTE_LIDAR_COUNT; ++direction) {
        RouteLaserFilter *f = &c->laser[direction];
        uint16_t mm = in->lidar.mm[direction];
        uint32_t elapsed = in->now_ms - f->sample_at;
        float allowance = ROUTE_WALL_STEP_NOISE_MM +
                          (ROUTE_CRUISE_MPS + ROUTE_TRIM_MAX_MPS) * (float)elapsed;
        if (!raw_valid(mm)) { f->good_frames = 0U; continue; }
        if ((uint32_t)(in->now_ms - f->valid_at) >= ROUTE_LIDAR_TTL_MS) {
            f->count = 0U;
            f->next = 0U;
            f->good_frames = f->sample_seen = 0U;
        }
        f->samples[f->next] = mm;
        f->next = (uint8_t)((f->next + 1U) % 3U);
        if (f->count < 3U) ++f->count;
        f->valid_at = in->now_ms;
        if (f->count == 3U) {
            f->median = median3(f->samples[0], f->samples[1], f->samples[2]);
            updated |= (uint8_t)(1U << direction);
            /* 纠偏通道也要连续可信。判垄端仍用独立近/远事件，不在此吞掉真实边缘。 */
            if (fabsf((float)mm - f->median) > ROUTE_WALL_INNOVATION_MM) {
                f->good_frames = 0U;
                continue;
            }
            if (f->sample_seen && fabsf((float)f->median - f->previous_mm) > allowance)
                f->good_frames = 0U;
            else if (f->good_frames < ROUTE_CAPTURE_FRAMES) ++f->good_frames;
            f->previous_mm = f->median;
            f->sample_at = in->now_ms;
            f->sample_seen = 1U;
        }
    }
    return updated;
}

static uint16_t reading(const RouteController *c, const RouteInputs *in, RouteLidarDirection sensor)
{
    const RouteLaserFilter *f;
    if (sensor >= ROUTE_LIDAR_COUNT || !in->lidar.fresh) return 0U;
    f = &c->laser[sensor];
    return f->count == 3U && (uint32_t)(in->now_ms - f->valid_at) < ROUTE_LIDAR_TTL_MS ?
           f->median : 0U;
}

static uint16_t control_reading(RouteController *c, const RouteInputs *in,
                                RouteLidarDirection sensor, uint16_t target)
{
    uint16_t mm = reading(c, in, sensor);
    RouteLaserFilter *f;
    if (sensor >= ROUTE_LIDAR_COUNT) return 0U;
    f = &c->laser[sensor];
    if (target != f->trim_target_mm) {
        f->trim_target_mm = target;
        f->trim_frames = 0U;
    }
    if (mm == 0U || target == 0U || !raw_valid(in->lidar.mm[sensor]) ||
        c->laser[sensor].good_frames < ROUTE_CAPTURE_FRAMES ||
        fabsf((float)in->lidar.mm[sensor] - mm) > ROUTE_WALL_INNOVATION_MM ||
        fabsf((float)in->lidar.mm[sensor] - target) > ROUTE_TRIM_VALID_MM ||
        fabsf((float)mm - target) > ROUTE_TRIM_VALID_MM) {
        f->trim_frames = 0U;
        f->trim_sequence = in->lidar.sequence;
        return 0U;
    }
    if (f->trim_sequence != in->lidar.sequence) {
        f->trim_sequence = in->lidar.sequence;
        if (f->trim_frames < ROUTE_CAPTURE_FRAMES) ++f->trim_frames;
    }
    return f->trim_frames >= ROUTE_CAPTURE_FRAMES ? mm : 0U;
}

void RouteNav_Init(RouteController *c)
{
    memset(c, 0, sizeof(*c));
    c->status.phase = ROUTE_WAIT_START;
    c->status.motion = ROUTE_STOP;
    c->status.next_motion = RoutePlan_Get(1U)->motion;
    c->status.sensor = ROUTE_LIDAR_COUNT;
    c->status.laser_remaining_mm = c->status.gate_mm = -1.0f;
}

/* 状态机和诊断使用同一个门控结果，避免显示值与实际判定不一致。 */
static float laser_gate(const RouteSegment *s)
{
    float gate = s->length_mm * ROUTE_LASER_GATE_RATIO;
    if (s->reference == ROUTE_REF_ODOM) return -1.0f;
    if (gate > s->length_mm - ROUTE_CAPTURE_RESERVE_MM)
        gate = s->length_mm - ROUTE_CAPTURE_RESERVE_MM;
    if (gate < 0.0f) gate = 0.0f;
    /* 这里只打开捕获窗口。是否已进入近距范围，在窗口内用真实回波判断。
     * 不再拿“允许的里程误差”推迟窗口，否则调大容差反而来不及减速。 */
    return gate;
}

static void begin_segment(RouteController *c, const RouteInputs *in)
{
    uint8_t direction;
    const RouteSegment *s = RoutePlan_Get(c->status.target_point);
    const RouteSegment *next = RoutePlan_Get((uint8_t)(c->status.target_point + 1U));
    c->start_x = in->odom.x_mm;
    c->start_y = in->odom.y_mm;
    c->segment_at = in->now_ms;
    c->progress_at = in->now_ms;
    c->progress_checkpoint = 0.0f;
    c->edge_found = c->edge_armed = c->near_count = c->far_count = c->overrun_count = 0U;
    c->edge_near_at = c->edge_far_at = c->edge_near_since = c->edge_frame_at = 0U;
    c->edge_near_seen = c->edge_candidate_valid = 0U;
    memset(&c->auxiliary, 0, sizeof(c->auxiliary));
    c->auxiliary.progress_at = in->now_ms;
    c->recovering = 0U;
    c->wall_at = c->recover_at = 0U;
    c->wall_sample_remaining = c->wall_sample_start = c->wall_remaining_mm = 0.0f;
    c->wall_trend_best = 0.0f;
    c->wall_best_error = 0.0f;
    c->wall_sample_at = c->search_at = 0U;
    c->wall_progress_at = in->now_ms;
    c->wall_sample_seen = c->wall_sample_count = c->wall_approach_seen = 0U;
    c->wall_progress_seen = 0U;
    c->wall_window_open = c->searching = 0U;
    c->restore_count = c->recover_exit_count = 0U;
    c->status.confirmed = 0U;
    c->status.edge_state = c->status.edge_confirmed = 0U;
    c->status.edge_last_near_mm = c->status.edge_first_far_mm = 0.0f;
    c->status.edge_anchor_mm = c->status.edge_uncertainty_mm = 0.0f;
    c->status.edge_speed_cap_mps = ROUTE_CRUISE_MPS;
    c->status.aux_sensor = ROUTE_LIDAR_COUNT;
    c->status.aux_measured_mm = c->status.aux_target_mm = 0U;
    c->status.aux_confidence = ROUTE_CONF_WAIT;
    c->status.aux_reject_reason = ROUTE_REJECT_WARMUP;
    c->status.aux_good_frames = c->status.aux_bad_frames = 0U;
    c->status.aux_remaining_mm = 0.0f;
    c->status.control_source = ROUTE_CONTROL_ODOM;
    c->status.progress_mm = 0.0f;
    c->status.length_mm = s->length_mm;
    c->status.odom_remaining_mm = s->length_mm;
    c->status.laser_remaining_mm = -1.0f;
    c->status.mismatch_mm = 0.0f;
    c->status.gate_mm = laser_gate(s);
    c->status.stop_flags = 0U;
    c->status.wall_locked = c->status.capture_confirmed = 0U;
    c->status.odom_untrusted = 0U;
    c->status.confidence = ROUTE_CONF_WAIT;
    c->status.reject_reason = ROUTE_REJECT_WARMUP;
    c->status.good_frames = c->status.bad_frames = c->status.speed_limited = 0U;
    c->status.odom_warn_count = c->status.recovering = c->status.recovery_resets = 0U;
    c->status.wall_innovation_mm = 0.0f;
    c->status.search_elapsed_ms = c->status.reference_idle_ms = 0U;
    c->status.estimated_end_mm = s->length_mm;
    c->status.control_remaining_mm = s->length_mm;
    c->status.motion = s->motion;
    c->status.next_motion = next ? next->motion : ROUTE_STOP;
    c->status.reference = s->reference;
    c->status.sensor = s->trigger;
    c->status.target_mm = s->mark.target_mm;
    c->status.measured_mm = 0U;
    c->status.lane = s->lane;
    c->status.phase = ROUTE_ODOM_RUN;
    c->status.reason = ROUTE_ERROR_NONE;
    for (direction = 0U; direction < ROUTE_LIDAR_COUNT; ++direction)
        c->laser[direction].trim_frames = 0U;
    if (s->reference == ROUTE_REF_WALL) {
        /* 新段的判点通道可能刚从侧墙/垄面切换过来。重新收集本段新帧，
         * 不能把上一段的近距中值当成本段提前到点或里程失准的证据。 */
        memset(&c->laser[s->trigger], 0, sizeof(c->laser[s->trigger]));
    }
    if (s->reference == ROUTE_REF_EDGE) {
        const RouteAuxReference *a = RoutePlan_AuxReference(c->status.target_point);
        if (a) {
            c->status.aux_sensor = (uint8_t)a->sensor;
            c->status.aux_target_mm = a->mark.target_mm;
            memset(&c->laser[a->sensor], 0, sizeof(c->laser[a->sensor]));
        }
    }
}

void RouteNav_Start(RouteController *c, const RouteInputs *in)
{
    c->started = 1U;
    c->run_at = in->now_ms;
    c->status.target_point = 1U;
    begin_segment(c, in);
}

void RouteNav_Fail(RouteController *c, RouteError error)
{
    c->status.phase = ROUTE_FAULT;
    c->status.reason = error;
    c->status.motion = ROUTE_STOP;
}

static float segment_progress(const RouteController *c, const RouteInputs *in, RouteMotion motion)
{
    if (motion == ROUTE_BACK) return c->start_x - in->odom.x_mm;
    if (motion == ROUTE_FORWARD) return in->odom.x_mm - c->start_x;
    if (motion == ROUTE_LEFT) return in->odom.y_mm - c->start_y;
    return c->start_y - in->odom.y_mm;
}

static void complete_point(RouteController *c, const RouteInputs *in, RouteOutput *out)
{
    RouteInputs next = *in;
    const RouteSegment *s = RoutePlan_Get(c->status.target_point);
    float x, y;
    c->status.at_point = c->status.target_point;

    /* 墙或垄端只约束运动轴。另一轴和航向保留估计，禁止把整个位姿一并归零。 */
    if (s->reference != ROUTE_REF_ODOM) {
        RoutePlan_Target(c->status.target_point, &x, &y);
        out->correct_axis = (s->motion == ROUTE_LEFT || s->motion == ROUTE_RIGHT) ? 1U : 0U;
        out->correct_position_mm = out->correct_axis == 0U ? x : y;
        if (out->correct_axis == 0U) next.odom.x_mm = x;
        else next.odom.y_mm = y;
    }
    if (c->status.at_point == 14U && ROUTE_REQUIRE_WORK && !ROUTE_DEBUG_MODE &&
        c->status.work_completed == 0U) {
        RouteNav_Fail(c, ROUTE_ERROR_NO_WORK);
        return;
    }
    if (c->status.at_point == ROUTE_POINT_COUNT) {
        c->status.phase = ROUTE_DONE;
        c->status.reason = ROUTE_ERROR_NONE;
        c->status.motion = ROUTE_STOP;
        return;
    }
    ++c->status.target_point;
    begin_segment(c, &next);
    /* 本次到点只产生一次切段，输出保持为零；下一周期先计算新段的里程。 */
}

static void wait_for_sensor(RouteController *c, const RouteInputs *in, RouteError reason)
{
    if (c->status.phase != ROUTE_WAIT_LIDAR) {
        c->wait_at = in->now_ms;
        c->status.phase = ROUTE_WAIT_LIDAR;
    }
    c->status.confirmed = 0U;
    c->status.reason = reason;
    c->progress_at = in->now_ms;
    if ((uint32_t)(in->now_ms - c->wait_at) >= ROUTE_SENSOR_WAIT_MS) RouteNav_Fail(c, reason);
}

static void observe_edge(RouteController *c, const RouteInputs *in,
                         const RouteSegment *s, uint16_t mm, uint8_t new_frame)
{
    float progress = c->status.progress_mm;
    uint16_t raw = in->lidar.mm[s->trigger];
    uint16_t far = s->mark.target_mm + ROUTE_EDGE_NEAR_BAND_MM + ROUTE_EDGE_FAR_GAP_MM;
    uint8_t window = progress >= s->length_mm * ROUTE_EDGE_MIN_PROGRESS_RATIO ||
                     (c->status.aux_confidence == ROUTE_CONF_TRUSTED &&
                      c->auxiliary.remaining <= s->edge_to_center_mm + ROUTE_EDGE_AUX_REQUIRED_MM);
    if (c->edge_found || !new_frame) return;
    if (c->edge_frame_at && (uint32_t)(in->now_ms - c->edge_frame_at) >= ROUTE_LIDAR_TTL_MS) {
        c->near_count = c->far_count = c->edge_armed = c->edge_near_seen = c->edge_candidate_valid = 0U;
    }
    c->edge_frame_at = in->now_ms;
    if (!in->lidar.fresh || !raw_valid(raw)) {
        c->near_count = c->far_count = c->edge_candidate_valid = 0U;
        return;
    }
    if (c->edge_near_seen && (uint32_t)(in->now_ms - c->edge_near_at) >= ROUTE_LIDAR_TTL_MS &&
        !c->edge_candidate_valid) c->edge_armed = c->edge_near_seen = 0U;

    /* 保存原始近/远帧的位置。中值和后续帧负责确认，不把确认完成时的位置当作边沿。 */
    if (fabsf((float)raw - s->mark.target_mm) <= ROUTE_EDGE_NEAR_BAND_MM) {
        if (c->near_count == 0U) c->edge_near_since = in->now_ms;
        c->far_count = 0U;
        c->edge_candidate_valid = 0U;
        if (c->near_count < ROUTE_EDGE_NEAR_FRAMES) ++c->near_count;
        c->edge_near_at = in->now_ms;
        c->edge_near_seen = 1U;
        c->status.edge_last_near_mm = progress;
        if (mm != 0U && fabsf((float)mm - s->mark.target_mm) <= ROUTE_EDGE_NEAR_BAND_MM &&
            c->near_count >= ROUTE_EDGE_NEAR_FRAMES &&
            (uint32_t)(in->now_ms - c->edge_near_since) >= ROUTE_EDGE_NEAR_MIN_MS)
            c->edge_armed = 1U;
    } else {
        c->near_count = 0U;
        if (c->edge_armed && c->edge_near_seen && raw >= far && window &&
            (c->edge_candidate_valid || (uint32_t)(in->now_ms - c->edge_near_at) < ROUTE_LIDAR_TTL_MS)) {
            if (!c->edge_candidate_valid) {
                c->edge_candidate_valid = 1U;
                c->edge_far_at = in->now_ms;
                c->status.edge_first_far_mm = progress;
                c->edge_candidate = (c->status.edge_last_near_mm + progress) * 0.5f;
                c->status.edge_anchor_mm = c->edge_candidate;
                c->status.edge_uncertainty_mm = fabsf(progress - c->status.edge_last_near_mm) * 0.5f;
            }
            if (c->far_count < ROUTE_EDGE_FAR_FRAMES) ++c->far_count;
            if (c->far_count >= ROUTE_EDGE_FAR_FRAMES && mm >= far) {
                c->edge_goal = c->edge_candidate + s->edge_to_center_mm;
                if (!c->auxiliary.locked) c->status.estimated_end_mm = c->edge_goal;
                /* 此处只是几何估计；不能让打滑里程否决独立的可信围栏参考。 */
                if (fabsf(c->edge_goal - s->length_mm) > ROUTE_EDGE_ODOM_ERROR_MM)
                    c->status.odom_untrusted = c->status.speed_limited = 1U;
                c->edge_found = 1U;
            }
        } else c->far_count = c->edge_candidate_valid = 0U;
    }
}

static uint8_t handle_work(RouteController *c, const RouteInputs *in, RouteOutput *out)
{
    if ((in->work_request != 0U || in->tool_busy) && c->status.phase != ROUTE_WORK_STOP &&
        c->status.phase != ROUTE_WAIT_TOOL) {
        c->resume_phase = c->status.phase;
        c->work_id = in->work_request;
        c->work_dispatched = in->tool_busy;
        c->work_at = in->now_ms;
        c->settle_at = in->now_ms;
        c->status.confirmed = 0U;
        invalidate_aux_recheck(c);
        c->status.phase = in->tool_busy ? ROUTE_WAIT_TOOL : ROUTE_WORK_STOP;
    }
    if (c->status.phase != ROUTE_WORK_STOP && c->status.phase != ROUTE_WAIT_TOOL) return 0U;
    if (in->tool_fault) {
        RouteNav_Fail(c, ROUTE_ERROR_TOOL);
        out->work_finished = 1U;
        return 1U;
    }
    if ((uint32_t)(in->now_ms - c->work_at) >= ROUTE_TOOL_TIMEOUT_MS) {
        RouteNav_Fail(c, ROUTE_ERROR_TOOL_TIME);
        return 1U;
    }
    c->progress_at = in->now_ms;
    if (c->status.phase == ROUTE_WORK_STOP) {
        if (!in->chassis_stopped) c->settle_at = in->now_ms;
        if (in->chassis_stopped && (uint32_t)(in->now_ms - c->settle_at) >= ROUTE_TOOL_SETTLE_MS) {
            out->start_work = c->work_id;
            c->work_dispatched = 1U;
            c->status.phase = ROUTE_WAIT_TOOL;
        }
    } else if (c->work_dispatched && !in->tool_busy) {
        out->work_finished = 1U;
        if (in->tool_result != 0U) RouteNav_Fail(c, ROUTE_ERROR_TOOL);
        else {
            /* 机构动作期间本来就要求停车，不能把这段时间算成回调超时。 */
            if (c->recovering) c->recover_at += (uint32_t)(in->now_ms - c->work_at);
            if (c->status.work_completed < 255U) ++c->status.work_completed;
            c->status.phase = c->resume_phase;
            c->wait_at = in->now_ms;
            c->wall_progress_at = in->now_ms;
            c->auxiliary.progress_at = in->now_ms;
            c->auxiliary.settle_at = 0U;
            if (c->searching) c->search_at = in->now_ms;
            c->status.reason = ROUTE_ERROR_NONE;
            c->progress_at = in->now_ms;
        }
    }
    return 1U;
}

/* 远处逐渐从巡航降至慢速，近处再按误差比例降低；区间内由调用者停车。 */
static float approach_speed(float remaining, float slow_band, float band)
{
    float speed;
    if (remaining > slow_band)
        speed = ROUTE_SLOW_MPS + (ROUTE_CRUISE_MPS - ROUTE_SLOW_MPS) *
                (remaining - slow_band) / slow_band;
    else {
        speed = (remaining - band) * ROUTE_APPROACH_KP;
        if (speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
    }
    if (speed < ROUTE_APPROACH_MIN_MPS) speed = ROUTE_APPROACH_MIN_MPS;
    if (speed > ROUTE_CRUISE_MPS) speed = ROUTE_CRUISE_MPS;
    return speed;
}

static RouteMotion opposite_motion(RouteMotion motion)
{
    if (motion == ROUTE_BACK) return ROUTE_FORWARD;
    if (motion == ROUTE_FORWARD) return ROUTE_BACK;
    if (motion == ROUTE_LEFT) return ROUTE_RIGHT;
    if (motion == ROUTE_RIGHT) return ROUTE_LEFT;
    return ROUTE_STOP;
}

/* speed 为相对本段计划方向的有符号速度；负值只在已确认的小幅回调中使用。 */
static void drive_segment(RouteController *c, const RouteInputs *in,
                          const RouteSegment *s, float speed, RouteOutput *out)
{
    float trim;
    if (s->reference == ROUTE_REF_EDGE && speed >= 0.0f) {
        if (c->status.speed_limited && speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
        if (speed < c->status.edge_speed_cap_mps) c->status.edge_speed_cap_mps = speed;
        if (speed > c->status.edge_speed_cap_mps) speed = c->status.edge_speed_cap_mps;
    }
    c->status.motion = speed < 0.0f ? opposite_motion(s->motion) : s->motion;
    if (s->motion == ROUTE_BACK || s->motion == ROUTE_FORWARD) {
        out->vx_mps = s->motion == ROUTE_BACK ? -speed : speed;
        trim = RouteNav_WallTrim(control_reading(c, in, s->side_wall, s->side_target_mm), s->side_target_mm);
        out->vy_mps = s->side_wall == ROUTE_LIDAR_LEFT ? trim : -trim;
    } else {
        out->vy_mps = s->motion == ROUTE_LEFT ? speed : -speed;
        out->vx_mps = RouteNav_LaneVx(control_reading(c, in, ROUTE_LIDAR_FRONT, ROUTE_PHYSICAL_FRONT_CENTER_MM),
                                     control_reading(c, in, ROUTE_LIDAR_BEHIND, ROUTE_PHYSICAL_REAR_CENTER_MM));
    }
}

static uint8_t recovery_time_ok(RouteController *c, const RouteInputs *in)
{
    if (c->recovering &&
        (uint32_t)(in->now_ms - c->recover_at) >= ROUTE_RECOVER_TIMEOUT_MS) {
        c->status.stop_flags |= ROUTE_STOP_RECOVER_LIMIT;
        RouteNav_Fail(c, ROUTE_ERROR_RECOVER_TIME);
        return 0U;
    }
    return 1U;
}

static uint8_t start_recovery(RouteController *c, const RouteInputs *in, float remaining)
{
    if (-remaining > ROUTE_RECOVER_MAX_MM) {
        c->status.stop_flags |= ROUTE_STOP_RECOVER_LIMIT;
        RouteNav_Fail(c, ROUTE_ERROR_OVERRUN);
        return 0U;
    }
    if (!c->recovering) {
        c->recovering = 1U;
        c->status.recovering = 1U;
        c->recover_at = in->now_ms;
    }
    c->status.phase = ROUTE_RECOVER;
    return recovery_time_ok(c, in);
}

/* P01/P17，以及捕获垄端之后的补偿行程，依靠有效编码器收敛到终点。 */
static void run_odom_goal(RouteController *c, const RouteInputs *in,
                          const RouteSegment *s, float goal, RouteOutput *out)
{
    float remaining = goal - c->status.progress_mm;
    float speed;
    c->status.estimated_end_mm = goal;
    c->status.control_remaining_mm = remaining;
    c->status.control_source = ROUTE_CONTROL_ODOM;
    c->status.reason = ROUTE_ERROR_NONE;
    if (!recovery_time_ok(c, in)) return;
    if (fabsf(remaining) <= ROUTE_ODOM_TOL_MM) {
        complete_point(c, in, out);
        return;
    }
    if (remaining < -ROUTE_ODOM_TOL_MM) {
        if (!start_recovery(c, in, remaining)) return;
        speed = -approach_speed(-remaining, s->mark.slow_band_mm, ROUTE_ODOM_TOL_MM);
        if (speed < -ROUTE_RECOVER_MPS) speed = -ROUTE_RECOVER_MPS;
    } else {
        c->status.phase = s->reference == ROUTE_REF_ODOM ? ROUTE_ODOM_RUN : ROUTE_LASER_NEAR;
        speed = approach_speed(remaining, s->reference == ROUTE_REF_ODOM ?
                               ROUTE_SLOW_DISTANCE_MM : s->mark.slow_band_mm, ROUTE_ODOM_TOL_MM);
        if (s->reference == ROUTE_REF_EDGE && speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
    }
    drive_segment(c, in, s, speed, out);
}

static float wall_remaining(const RouteSegment *s, uint16_t mm)
{
    return s->distance_increases ? (float)s->mark.target_mm - mm :
                                  (float)mm - s->mark.target_mm;
}

/* 验证激光自身的连续性，不拿可能打滑的轮式位移否决激光。
 * 远回波也积累接近证据，但只有连续近距新帧才能接管。
 * 大跳变重新捕获，清除原来的接近证据，不能由跳变直接确认到点/反向。 */
static uint8_t observe_wall(RouteController *c, const RouteInputs *in,
                            const RouteSegment *s, uint16_t mm, uint8_t updated)
{
    uint16_t raw = in->lidar.mm[s->trigger];
    float remaining, allowance, error;
    uint32_t elapsed;
    uint8_t previous_count;
    c->status.reject_reason = 0U;
    if (mm == 0U || !raw_valid(raw) || !in->lidar.fresh) {
        c->status.capture_confirmed = c->wall_sample_count = 0U;
        c->status.reject_reason = ROUTE_REJECT_NO_DATA;
        return ROUTE_STOP_NO_LIDAR;
    }
    if (fabsf((float)raw - mm) > ROUTE_WALL_INNOVATION_MM) {
        c->status.capture_confirmed = c->wall_sample_count = 0U;
        c->status.reject_reason = ROUTE_REJECT_SPREAD;
        return ROUTE_STOP_REFERENCE_INVALID;
    }
    if (updated) {
        remaining = wall_remaining(s, mm);
        elapsed = in->now_ms - c->wall_sample_at;
        previous_count = c->wall_sample_count;
        allowance = ROUTE_WALL_STEP_NOISE_MM + ROUTE_CRUISE_MPS * (float)elapsed;
        if (c->wall_sample_seen &&
            fabsf(remaining - c->wall_sample_remaining) > allowance) {
            c->wall_sample_remaining = c->wall_sample_start = remaining;
            c->wall_trend_best = remaining;
            c->wall_sample_at = in->now_ms;
            c->wall_sample_count = 1U;
            c->wall_approach_seen = c->status.capture_confirmed = 0U;
            c->status.reject_reason = ROUTE_REJECT_STEP;
            return ROUTE_STOP_REFERENCE_INVALID;
        }
        if (elapsed >= ROUTE_LIDAR_TTL_MS) {
            c->wall_sample_count = c->status.capture_confirmed = 0U;
            c->wall_approach_seen = 0U;
            previous_count = 0U;
        }
        if (!c->wall_sample_seen || previous_count == 0U)
            c->wall_sample_start = c->wall_trend_best = remaining;
        if (!c->recovering && remaining - c->wall_trend_best > ROUTE_CONF_REVERSE_TREND_MM) {
            c->wall_sample_remaining = c->wall_sample_start = remaining;
            c->wall_trend_best = remaining;
            c->wall_sample_at = in->now_ms;
            c->wall_sample_count = 1U;
            c->wall_approach_seen = c->status.capture_confirmed = 0U;
            c->status.reject_reason = ROUTE_REJECT_DIRECTION;
            return ROUTE_STOP_REFERENCE_INVALID;
        }
        c->wall_sample_seen = 1U;
        c->wall_sample_remaining = remaining;
        if (remaining < c->wall_trend_best) c->wall_trend_best = remaining;
        c->wall_sample_at = in->now_ms;
        if (c->wall_sample_count < ROUTE_CAPTURE_FRAMES) ++c->wall_sample_count;
        if (c->wall_sample_start - remaining >= ROUTE_WALL_CAPTURE_PROGRESS_MM)
            c->wall_approach_seen = 1U;
        if (mm <= RoutePlan_ReferenceMax(s) && raw <= RoutePlan_ReferenceMax(s)) {
            if (c->status.capture_confirmed < ROUTE_CAPTURE_FRAMES) ++c->status.capture_confirmed;
        } else c->status.capture_confirmed = 0U;

        /* 重捕获可重建距离基线，但只有实际接近才能延长进展计时。
         * 否则周期性噪声后的重新验证也会掩盖轮子空转。 */
        if (c->wall_sample_count >= ROUTE_CAPTURE_FRAMES) {
            error = fabsf(remaining);
            if (previous_count < ROUTE_CAPTURE_FRAMES) {
                if (!c->wall_progress_seen ||
                    fabsf(c->wall_sample_start) - error >= ROUTE_WALL_PROGRESS_STEP_MM)
                    c->wall_progress_at = in->now_ms;
                c->wall_best_error = error;
                c->wall_progress_seen = 1U;
            } else if (c->wall_best_error - error >= ROUTE_WALL_PROGRESS_STEP_MM) {
                c->wall_best_error = error;
                c->wall_progress_at = in->now_ms;
            }
        }
    }
    if (mm > RoutePlan_ReferenceMax(s) || raw > RoutePlan_ReferenceMax(s)) {
        c->status.reject_reason = ROUTE_REJECT_FAR;
        return ROUTE_STOP_FAR_REFERENCE;
    }
    if (wall_remaining(s, mm) <= ROUTE_CONF_NEAR_MM && !c->wall_approach_seen) {
        c->status.reject_reason = ROUTE_REJECT_NO_APPROACH;
        return ROUTE_STOP_REFERENCE_INVALID;
    }
    return 0U;
}

static void mark_odom_untrusted(RouteController *c)
{
    if (!c->status.odom_untrusted) {
        c->status.speed_limited = 1U;
        c->restore_count = 0U;
    }
    c->status.odom_untrusted = 1U;
}

/* 确认计数只随新发布的 CAN 帧改变；20 ms 重读不积累好/坏帧。 */
static void wall_quality(RouteController *c, const RouteInputs *in,
                          uint8_t flags, uint8_t updated, uint8_t new_frame)
{
    RouteStatus *st = &c->status;
    uint8_t warming = st->reject_reason == ROUTE_REJECT_NO_DATA && in->lidar.fresh &&
                      raw_valid(in->lidar.mm[st->sensor]) && c->laser[st->sensor].count < 3U;
    if (warming) {
        st->confidence = ROUTE_CONF_WAIT;
        st->reject_reason = ROUTE_REJECT_WARMUP;
        if (new_frame) st->good_frames = st->bad_frames = c->restore_count = st->odom_warn_count = 0U;
        if (new_frame) c->recover_exit_count = 0U;
    } else if (flags == ROUTE_STOP_FAR_REFERENCE) {
        st->confidence = ROUTE_CONF_CANDIDATE;
        if (updated) {
            st->bad_frames = c->restore_count = st->odom_warn_count = 0U;
            c->recover_exit_count = 0U;
            if (st->good_frames < ROUTE_CONF_RESTORE_FRAMES) ++st->good_frames;
        }
    } else if (flags != 0U) {
        st->confidence = ROUTE_CONF_SUSPECT;
        if (new_frame) {
            st->good_frames = c->restore_count = st->odom_warn_count = 0U;
            c->recover_exit_count = 0U;
            if (st->bad_frames < ROUTE_CONF_BAD_FRAMES) ++st->bad_frames;
            if (st->bad_frames >= ROUTE_CONF_BAD_FRAMES) st->speed_limited = 1U;
        }
    } else {
        if (updated) {
            st->bad_frames = 0U;
            if (st->good_frames < ROUTE_CONF_RESTORE_FRAMES) ++st->good_frames;
        }
        st->confidence = st->capture_confirmed >= ROUTE_CAPTURE_FRAMES ?
                         ROUTE_CONF_TRUSTED : ROUTE_CONF_CANDIDATE;
        if (st->confidence != ROUTE_CONF_TRUSTED) st->reject_reason = ROUTE_REJECT_WARMUP;
    }
}

static void accepted_reference(RouteController *c, float remaining,
                           float nominal, float innovation)
{
    uint8_t mismatch = fabsf(remaining - nominal) > ROUTE_LASER_ODOM_ERROR_MM ||
                      (c->status.wall_locked && fabsf(innovation) > ROUTE_WALL_INNOVATION_MM);
    if (mismatch) {
        if (c->status.odom_warn_count < ROUTE_CONF_ODOM_FRAMES) ++c->status.odom_warn_count;
        if (c->status.odom_warn_count >= ROUTE_CONF_ODOM_FRAMES) mark_odom_untrusted(c);
    } else c->status.odom_warn_count = 0U;
    if (c->restore_count < ROUTE_CONF_RESTORE_FRAMES) ++c->restore_count;
    if (c->restore_count >= ROUTE_CONF_RESTORE_FRAMES && remaining > ROUTE_CONF_RESTORE_MM)
        c->status.speed_limited = 0U;
    if (c->recovering && remaining > ROUTE_CONF_RECOVER_EXIT_MM) {
        if (c->recover_exit_count < ROUTE_CONF_RESTORE_FRAMES) ++c->recover_exit_count;
    } else c->recover_exit_count = 0U;
    if (c->recover_exit_count >= ROUTE_CONF_RESTORE_FRAMES) {
        /* 连续远距新参考证明旧回调终点错误；近点的真实回调不走此分支。 */
        c->recovering = c->status.recovering = 0U;
        c->recover_at = 0U;
        c->wall_trend_best = remaining;
        c->auxiliary.trend_best = remaining;
        c->recover_exit_count = c->overrun_count = c->status.confirmed = 0U;
        if (c->status.recovery_resets < 255U) ++c->status.recovery_resets;
    }
}

/* 轮式里程只负责粗进度和短时预测。可信激光负责墙参考终点。
 * 里程告警与临时限速分开；参考失效、无进展、搜索/段/全程超时仍停车。 */
static void run_wall(RouteController *c, const RouteInputs *in, const RouteSegment *s,
                     uint16_t mm, uint8_t sensor_updated, uint8_t new_frame, RouteOutput *out)
{
    uint16_t raw = in->lidar.mm[s->trigger];
    uint8_t accepted = 0U;
    uint8_t flags = observe_wall(c, in, s, mm, sensor_updated);
    float remaining, raw_remaining = 0.0f, speed;
    float band = (float)s->mark.turn_band_mm;
    float nominal = s->length_mm - c->status.progress_mm;
    float predicted = c->status.estimated_end_mm - c->status.progress_mm;

    wall_quality(c, in, flags, sensor_updated, new_frame);
    c->status.reference_idle_ms = in->now_ms - c->wall_progress_at;
    if (mm != 0U) {
        remaining = wall_remaining(s, mm);
        c->status.wall_innovation_mm = remaining - predicted;
        /* 里程少计时允许由连续接近的近距参考提前打开窗口。
         * 初始就读到目标/越程值时先停车验证，不能靠单个近物回波继续驶入。 */
        if ((flags == 0U && c->status.capture_confirmed >= ROUTE_CAPTURE_FRAMES && c->wall_approach_seen) ||
            (raw_valid(raw) && fabsf((float)raw - mm) <= ROUTE_WALL_INNOVATION_MM &&
             remaining <= ROUTE_CONF_NEAR_MM)) c->wall_window_open = 1U;
    }
    if (c->status.progress_mm >= c->status.gate_mm) c->wall_window_open = 1U;

    /* 还未接管、粗进度也未到捕获区时，稳定远距可撤销假近物打开的窗口。
     * 该远距仅恢复粗导航速度，不更新 G、不确认到点；无进展计时继续生效。 */
    if (!c->status.wall_locked && flags == ROUTE_STOP_FAR_REFERENCE &&
        c->status.progress_mm < c->status.gate_mm &&
        c->status.good_frames >= ROUTE_CONF_RESTORE_FRAMES &&
        c->wall_sample_count >= ROUTE_CAPTURE_FRAMES &&
        wall_remaining(s, mm) > ROUTE_CONF_RESTORE_MM) {
        c->wall_window_open = c->status.speed_limited = c->searching = 0U;
        c->status.search_elapsed_ms = 0U;
    }

    if (!c->wall_window_open) {
        c->status.phase = ROUTE_ODOM_RUN;
        c->status.reason = ROUTE_ERROR_NONE;
        c->status.control_remaining_mm = nominal;
        if (c->status.reference_idle_ms >= ROUTE_WALL_NO_PROGRESS_MS) {
            RouteNav_Fail(c, ROUTE_ERROR_STALL);
            return;
        }
        speed = approach_speed(nominal, s->mark.slow_band_mm, band);
        if (c->status.speed_limited && speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
        drive_segment(c, in, s, speed, out);
        return;
    }

    if (sensor_updated && flags == 0U && c->status.capture_confirmed >= ROUTE_CAPTURE_FRAMES) {
        remaining = wall_remaining(s, mm);
        if (remaining > ROUTE_CONF_NEAR_MM || c->wall_approach_seen) {
            accepted_reference(c, remaining, nominal, c->status.wall_innovation_mm);
            c->status.wall_locked = 1U;
            c->status.estimated_end_mm = c->status.progress_mm + remaining;
            c->wall_remaining_mm = remaining;
            c->wall_at = in->now_ms;
            c->searching = 0U;
            accepted = 1U;
            raw_remaining = wall_remaining(s, raw);
        }
    }

    /* 墙参考先处理本周期的新可信证据，再检查旧回调时限。 */
    if (!recovery_time_ok(c, in)) return;

    if (!accepted && (!c->status.wall_locked || flags != 0U ||
                      c->status.capture_confirmed < ROUTE_CAPTURE_FRAMES)) {
        if (!c->searching) { c->searching = 1U; c->search_at = in->now_ms; }
    }
    c->status.search_elapsed_ms = c->searching ? in->now_ms - c->search_at : 0U;
    if (c->status.search_elapsed_ms >= ROUTE_WALL_SEARCH_TIMEOUT_MS) {
        c->status.stop_flags = flags != 0U ? flags : ROUTE_STOP_REFERENCE_INVALID;
        RouteNav_Fail(c, ROUTE_ERROR_SEARCH_TIME);
        return;
    }
    if (!c->status.wall_locked) {
        c->status.control_remaining_mm = nominal;
        if (flags & (ROUTE_STOP_NO_LIDAR | ROUTE_STOP_REFERENCE_INVALID)) {
            c->status.stop_flags = flags;
            wait_for_sensor(c, in, flags & ROUTE_STOP_NO_LIDAR ? ROUTE_ERROR_LIDAR : ROUTE_ERROR_REFERENCE);
            return;
        }
        if (mm != 0U && wall_remaining(s, mm) <= ROUTE_CONF_NEAR_MM && !c->wall_approach_seen) {
            c->status.capture_confirmed = 0U;
            c->status.stop_flags = ROUTE_STOP_REFERENCE_INVALID;
            wait_for_sensor(c, in, ROUTE_ERROR_REFERENCE);
            return;
        }
        c->status.phase = ROUTE_CAPTURE;
        c->status.reason = ROUTE_ERROR_NONE;
        if (c->status.reference_idle_ms >= ROUTE_WALL_NO_PROGRESS_MS) {
            RouteNav_Fail(c, ROUTE_ERROR_STALL);
            return;
        }
        /* 远回波/尚未凑齐近距帧时仅低速搜索；计划剩余为负也不能靠它反向或切段。 */
        drive_segment(c, in, s, ROUTE_SLOW_MPS, out);
        return;
    }

    /* 里程失准后，两帧之间保持最后可信激光剩余，不让打滑预测制造虚假越程。 */
    c->status.control_source = ROUTE_CONTROL_WALL;
    remaining = c->status.odom_untrusted ? c->wall_remaining_mm :
                c->status.estimated_end_mm - c->status.progress_mm;
    c->status.control_remaining_mm = remaining;
    if (new_frame && !accepted) c->status.confirmed = c->overrun_count = 0U;
    if ((uint32_t)(in->now_ms - c->wall_at) > ROUTE_WALL_PREDICT_MS) {
        c->status.stop_flags = flags != 0U ? flags : ROUTE_STOP_REFERENCE_INVALID;
        wait_for_sensor(c, in, flags & ROUTE_STOP_NO_LIDAR ? ROUTE_ERROR_LIDAR : ROUTE_ERROR_REFERENCE);
        return;
    }
    c->status.reason = ROUTE_ERROR_NONE;
    if (c->status.reference_idle_ms >= ROUTE_WALL_NO_PROGRESS_MS) {
        RouteNav_Fail(c, ROUTE_ERROR_STALL);
        return;
    }
    if (fabsf(remaining) <= band) {
        c->overrun_count = 0U;
        c->status.phase = ROUTE_CONFIRM;
        if (accepted && fabsf(raw_remaining) <= band &&
            (c->status.target_point != 1U || in->chassis_stopped)) {
            if (c->status.confirmed < ROUTE_CONFIRM_FRAMES) ++c->status.confirmed;
        } else if (new_frame) c->status.confirmed = 0U;
        if (c->status.confirmed >= ROUTE_CONFIRM_FRAMES) complete_point(c, in, out);
        return;
    }
    c->status.confirmed = 0U;
    if (remaining < -band) {
        /* 预测越程只会停车。反向和大越程报错均须连续的新激光帧证实。 */
        if (accepted && raw_remaining < -band) {
            if (c->overrun_count < ROUTE_CONFIRM_FRAMES) ++c->overrun_count;
        } else if (new_frame) c->overrun_count = 0U;
        if (!c->recovering && c->overrun_count < ROUTE_CONFIRM_FRAMES) {
            c->status.phase = ROUTE_CONFIRM;
            return;
        }
        if (accepted && -remaining > ROUTE_RECOVER_MAX_MM) {
            c->status.stop_flags = ROUTE_STOP_RECOVER_LIMIT;
            RouteNav_Fail(c, ROUTE_ERROR_OVERRUN);
            return;
        }
        if (!c->recovering) {
            if (!start_recovery(c, in, remaining)) return;
        }
        c->status.phase = ROUTE_RECOVER;
        speed = -approach_speed(-remaining, s->mark.slow_band_mm, band);
        if (speed < -ROUTE_RECOVER_MPS) speed = -ROUTE_RECOVER_MPS;
    } else {
        c->overrun_count = 0U;
        c->status.phase = ROUTE_LASER_NEAR;
        speed = approach_speed(remaining, s->mark.slow_band_mm, band);
    }
    /* 导航 20 ms、CAN 常为 100 ms：正常两帧间保持连续速度，不能每个旧帧周期都降速。
     * 遇到被拒绝的测距或更新已经延迟，再降为慢速预测，完整预测预算耗尽则在上方停车。 */
    if ((c->status.speed_limited || !in->lidar.fresh ||
         (uint32_t)(in->now_ms - c->wall_at) > ROUTE_WALL_PREDICT_MS / 2U) &&
        speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
    drive_segment(c, in, s, speed, out);
}

static void reject_aux(RouteController *c, uint8_t reason, uint8_t new_frame)
{
    c->status.aux_reject_reason = reason;
    c->status.aux_confidence = reason == ROUTE_REJECT_WARMUP ? ROUTE_CONF_WAIT :
                               reason == ROUTE_REJECT_FAR ? ROUTE_CONF_CANDIDATE : ROUTE_CONF_SUSPECT;
    c->status.aux_good_frames = 0U;
    c->restore_count = c->recover_exit_count = 0U;
    if (reason == ROUTE_REJECT_WARMUP || reason == ROUTE_REJECT_FAR) c->status.aux_bad_frames = 0U;
    else if (new_frame) {
        if (c->status.aux_bad_frames < ROUTE_CONF_BAD_FRAMES) ++c->status.aux_bad_frames;
        if (c->status.aux_bad_frames >= ROUTE_CONF_BAD_FRAMES) c->status.speed_limited = 1U;
    }
}

/* 首次捕获仍须接近证据。停车重验必须有历史资格、完整垄端和新帧稳定性。
 * 到点附近的小负剩余可复核到位；此函数本身不启动反向回调。 */
static uint8_t recheck_aux_at_rest(RouteController *c, const RouteInputs *in,
                                  const RouteAuxReference *ref, uint16_t mm, uint16_t raw,
                                  float remaining, float raw_remaining)
{
    RouteAuxTracker *a = &c->auxiliary;
    float lower = a->near_arrival ? ROUTE_EDGE_ARRIVAL_VERIFY_MM : (float)ref->mark.turn_band_mm;
    uint16_t low = raw < mm ? raw : mm;
    uint16_t high = raw > mm ? raw : mm;
    if ((!a->locked && !a->approach_qualified) || !a->interrupted || !c->edge_found || !in->chassis_stopped ||
        c->status.phase != ROUTE_WAIT_LIDAR || c->recovering ||
        (uint32_t)(in->now_ms - c->wait_at) >= ROUTE_SENSOR_WAIT_MS ||
        remaining < -lower || raw_remaining < -lower ||
        remaining > ROUTE_CONF_NEAR_MM || raw_remaining > ROUTE_CONF_NEAR_MM ||
        (float)(high - low) > ROUTE_EDGE_AUX_RECHECK_SPREAD_MM) {
        reset_aux_recheck(a);
        return 0U;
    }
    if (a->recheck_frames) {
        if (a->recheck_min_mm < low) low = a->recheck_min_mm;
        if (a->recheck_max_mm > high) high = a->recheck_max_mm;
        if ((float)(high - low) > ROUTE_EDGE_AUX_RECHECK_SPREAD_MM) {
            reset_aux_recheck(a);
            low = raw < mm ? raw : mm;
            high = raw > mm ? raw : mm;
        }
    }
    if (!a->recheck_frames) a->recheck_at = in->now_ms;
    a->recheck_min_mm = low;
    a->recheck_max_mm = high;
    if (a->recheck_frames < ROUTE_CAPTURE_FRAMES) ++a->recheck_frames;
    if (a->recheck_frames < ROUTE_CAPTURE_FRAMES ||
        (uint32_t)(in->now_ms - a->recheck_at) < ROUTE_EDGE_AUX_RECHECK_MS) return 0U;

    a->revalidated = 1U;
    c->status.aux_good_frames = ROUTE_CAPTURE_FRAMES;
    c->status.speed_limited = 1U;
    if (c->status.edge_speed_cap_mps > ROUTE_EDGE_AUX_RECHECK_MPS)
        c->status.edge_speed_cap_mps = ROUTE_EDGE_AUX_RECHECK_MPS;
    c->status.confirmed = c->overrun_count = 0U;
    return 1U;
}

static void pending_aux_recheck(RouteController *c, uint8_t new_frame)
{
    reject_aux(c, c->auxiliary.recheck_frames ? ROUTE_REJECT_WARMUP : ROUTE_REJECT_NO_APPROACH, new_frame);
    if (c->auxiliary.recheck_frames) {
        c->status.aux_good_frames = c->auxiliary.recheck_frames;
        c->status.aux_confidence = ROUTE_CONF_CANDIDATE;
    }
}

/* 独立围栏参考只与自己的量程、时间、趋势、中心窗口比较；不拿打滑编码器否决它。
 * 返回1仅表示本周期接受了一帧新参考，旧值/重复帧绝不推进到点确认。 */
static uint8_t observe_aux(RouteController *c, const RouteInputs *in,
                           const RouteSegment *s, const RouteAuxReference *ref,
                           uint8_t updated, uint8_t new_frame)
{
    RouteAuxTracker *a = &c->auxiliary;
    uint16_t raw = in->lidar.mm[ref->sensor];
    uint16_t mm = reading(c, in, ref->sensor);
    float remaining, raw_remaining, allowance, error;
    uint32_t elapsed = in->now_ms - a->sample_at;
    uint8_t near;
    c->status.aux_measured_mm = mm;
    if (a->near_verified && (uint32_t)(in->now_ms - a->trusted_at) > ROUTE_EDGE_AUX_PREDICT_MS)
        invalidate_aux_recheck(c);
    if (!in->chassis_stopped && a->near_arrival) invalidate_aux_recheck(c);
    else if (!in->chassis_stopped || c->status.phase != ROUTE_WAIT_LIDAR) reset_aux_recheck(a);
    if (!in->lidar.fresh || !raw_valid(raw) || mm == 0U) {
        uint8_t warming = in->lidar.fresh && raw_valid(raw) && c->laser[ref->sensor].count < 3U;
        if (!warming && (a->locked || a->approach_qualified)) a->interrupted = 1U;
        a->history_frames = 0U;
        invalidate_aux_recheck(c);
        reject_aux(c, warming ? ROUTE_REJECT_WARMUP : ROUTE_REJECT_NO_DATA, new_frame);
        return 0U;
    }
    remaining = ref->distance_increases ? (float)ref->mark.target_mm - mm :
                                         (float)mm - ref->mark.target_mm;
    raw_remaining = ref->distance_increases ? (float)ref->mark.target_mm - raw :
                                             (float)raw - ref->mark.target_mm;
    c->status.aux_remaining_mm = remaining;
    if (fabsf((float)raw - mm) > ROUTE_WALL_INNOVATION_MM) {
        a->history_frames = 0U;
        invalidate_aux_recheck(c);
        reject_aux(c, ROUTE_REJECT_SPREAD, new_frame);
        return 0U;
    }
    if (fabsf(remaining) > ROUTE_EDGE_AUX_WINDOW_MM || fabsf(raw_remaining) > ROUTE_EDGE_AUX_WINDOW_MM) {
        a->history_frames = 0U;
        invalidate_aux_recheck(c);
        reject_aux(c, ROUTE_REJECT_FAR, new_frame);
        return 0U;
    }
    if (!updated) return 0U;
    allowance = ROUTE_WALL_STEP_NOISE_MM + ROUTE_CRUISE_MPS * (float)elapsed;
    if (a->seen && fabsf(remaining - a->sample_remaining) > allowance) {
        a->sample_remaining = a->epoch_start = a->trend_best = remaining;
        a->sample_at = in->now_ms;
        a->approach_seen = 0U;
        a->history_frames = 0U;
        invalidate_aux_recheck(c);
        reject_aux(c, ROUTE_REJECT_STEP, new_frame);
        return 0U;
    }
    if (!a->seen || elapsed >= ROUTE_LIDAR_TTL_MS) {
        a->epoch_start = a->trend_best = remaining;
        a->approach_seen = c->status.aux_good_frames = 0U;
        a->history_frames = 0U;
        if (a->locked || a->approach_qualified) a->interrupted = 1U;
        invalidate_aux_recheck(c);
    }
    if (!c->recovering && remaining - a->trend_best > ROUTE_CONF_REVERSE_TREND_MM) {
        a->sample_remaining = a->epoch_start = a->trend_best = remaining;
        a->sample_at = in->now_ms;
        a->approach_seen = 0U;
        a->history_frames = 0U;
        invalidate_aux_recheck(c);
        reject_aux(c, ROUTE_REJECT_DIRECTION, new_frame);
        return 0U;
    }
    a->sample_remaining = remaining;
    a->sample_at = in->now_ms;
    a->seen = 1U;
    if (remaining < a->trend_best) a->trend_best = remaining;
    if (a->epoch_start - remaining >= ROUTE_WALL_CAPTURE_PROGRESS_MM) a->approach_seen = 1U;
    if (a->history_frames < ROUTE_EDGE_AUX_HISTORY_FRAMES) ++a->history_frames;
    /* 合格历史跨中断保留到本段结束，当前计数仍会被异常清零；不能把历史当作当前可信帧。 */
    if (a->history_frames >= ROUTE_EDGE_AUX_HISTORY_FRAMES && a->approach_seen)
        a->approach_qualified = 1U;
    c->status.aux_bad_frames = 0U;
    near = fabsf(remaining) <= ROUTE_EDGE_ARRIVAL_VERIFY_MM &&
           fabsf(raw_remaining) <= ROUTE_EDGE_ARRIVAL_VERIFY_MM;
    if (!a->near_arrival && near && c->edge_found && !c->recovering &&
        ((c->status.phase == ROUTE_WAIT_LIDAR && (a->locked || a->approach_qualified)) ||
         (a->approach_qualified && (c->status.phase == ROUTE_ODOM_RUN ||
          c->status.phase == ROUTE_CAPTURE || c->status.phase == ROUTE_LASER_NEAR)))) {
        uint8_t proactive = c->status.phase != ROUTE_WAIT_LIDAR;
        a->near_arrival = a->interrupted = 1U;
        reset_aux_recheck(a);
        if (proactive) {
            /* 先提交零速。触发帧不计入停车后的稳定证据，后续须有新的CAN帧。 */
            c->status.phase = ROUTE_WAIT_LIDAR;
            c->wait_at = in->now_ms;
            c->status.confirmed = c->overrun_count = 0U;
            a->settle_at = 0U;
            reject_aux(c, ROUTE_REJECT_WARMUP, new_frame);
            return 0U;
        }
    }
    if (a->near_arrival && a->near_verified && !near) invalidate_aux_recheck(c);
    if (a->near_arrival && !a->near_verified && near) {
        if (!recheck_aux_at_rest(c, in, ref, mm, raw, remaining, raw_remaining)) {
            pending_aux_recheck(c, new_frame);
            return 0U;
        }
        a->near_verified = 1U;
    }
    if (remaining <= ROUTE_CONF_NEAR_MM && !a->approach_seen && !a->revalidated &&
        !(a->locked && fabsf(remaining - a->remaining) <= ROUTE_WALL_INNOVATION_MM)) {
        if (!recheck_aux_at_rest(c, in, ref, mm, raw, remaining, raw_remaining)) {
            pending_aux_recheck(c, new_frame);
            return 0U;
        }
    }
    if (c->status.aux_good_frames < ROUTE_CAPTURE_FRAMES) ++c->status.aux_good_frames;
    if (c->status.aux_good_frames < ROUTE_CAPTURE_FRAMES) {
        c->status.aux_confidence = ROUTE_CONF_CANDIDATE;
        c->status.aux_reject_reason = ROUTE_REJECT_WARMUP;
        return 0U;
    }
    c->status.aux_confidence = ROUTE_CONF_TRUSTED;
    c->status.aux_reject_reason = 0U;
    accepted_reference(c, remaining, s->length_mm - c->status.progress_mm,
                       remaining - (c->status.estimated_end_mm - c->status.progress_mm));
    if (c->restore_count >= ROUTE_CONF_RESTORE_FRAMES && remaining > ROUTE_EDGE_AUX_REQUIRED_MM)
        c->status.speed_limited = 0U; /* 已减下来的换沟速度上限仍保持，不因解除告警反弹。 */
    a->locked = 1U;
    if (a->near_arrival && !near) a->near_arrival = a->near_verified = 0U;
    a->interrupted = 0U;
    reset_aux_recheck(a);
    a->remaining = remaining;
    a->trusted_at = in->now_ms;
    c->status.estimated_end_mm = c->status.progress_mm + remaining;
    error = fabsf(remaining);
    if (!a->progress_seen || a->best_error - error >= ROUTE_WALL_PROGRESS_STEP_MM) {
        a->progress_at = in->now_ms;
        a->best_error = error;
        a->progress_seen = 1U;
    }
    return 1U;
}

static void wait_edge_reference(RouteController *c, const RouteInputs *in)
{
    uint8_t missing = (c->status.aux_reject_reason & ROUTE_REJECT_NO_DATA) != 0U;
    if (c->auxiliary.near_verified) invalidate_aux_recheck(c);
    c->status.stop_flags = missing ? ROUTE_STOP_NO_LIDAR : ROUTE_STOP_REFERENCE_INVALID;
    c->auxiliary.settle_at = 0U;
    wait_for_sensor(c, in, missing ? ROUTE_ERROR_LIDAR : ROUTE_ERROR_REFERENCE);
}

/* 主边沿决定已经通过垄端，独立围栏距离决定最终停点；编码器只负责粗进度。
 * 末端不能靠不断延后的几何估计、重复帧或旧参考自行切换到下一沟。 */
static void run_edge(RouteController *c, const RouteInputs *in, const RouteSegment *s,
                      const RouteAuxReference *ref, uint16_t mm, uint8_t accepted,
                      uint8_t new_frame, RouteOutput *out)
{
    RouteAuxTracker *a = &c->auxiliary;
    float remaining, raw_remaining, speed;
    float band = a->near_verified ? ROUTE_EDGE_ARRIVAL_VERIFY_MM : (float)ref->mark.turn_band_mm;
    float nominal = s->length_mm - c->status.progress_mm;
    if (!recovery_time_ok(c, in)) return;
    c->status.reference_idle_ms = in->now_ms - a->progress_at;
    if (a->near_arrival && !a->near_verified) {
        wait_edge_reference(c, in);
        return;
    }
    if (a->locked) {
        c->status.control_source = ROUTE_CONTROL_EDGE_AUX;
        remaining = a->remaining; /* 两帧间保持可信剩余，轮式打滑不能制造虚假到点。 */
        c->status.control_remaining_mm = remaining;
        c->status.laser_remaining_mm = remaining;
        c->status.mismatch_mm = remaining - nominal;
        if (new_frame && !accepted) c->status.confirmed = c->overrun_count = 0U;
        if ((uint32_t)(in->now_ms - a->trusted_at) > ROUTE_EDGE_AUX_PREDICT_MS) {
            wait_edge_reference(c, in);
            return;
        }
        if (c->status.reference_idle_ms >= ROUTE_WALL_NO_PROGRESS_MS) {
            RouteNav_Fail(c, ROUTE_ERROR_STALL);
            return;
        }
        if (!c->edge_found && (mm == 0U || remaining <= band ||
            c->status.progress_mm > s->length_mm + ROUTE_EDGE_SEARCH_EXTRA_MM)) {
            c->status.stop_flags = mm == 0U ? ROUTE_STOP_NO_LIDAR : ROUTE_STOP_REFERENCE_INVALID;
            a->settle_at = 0U;
            wait_for_sensor(c, in, mm == 0U ? ROUTE_ERROR_LIDAR : ROUTE_ERROR_REFERENCE);
            return;
        }
        c->status.reason = ROUTE_ERROR_NONE;
        raw_remaining = ref->distance_increases ? (float)ref->mark.target_mm - in->lidar.mm[ref->sensor] :
                                                   (float)in->lidar.mm[ref->sensor] - ref->mark.target_mm;
        if (fabsf(remaining) <= band) {
            c->status.phase = ROUTE_CONFIRM;
            c->overrun_count = 0U;
            if (accepted && fabsf(raw_remaining) <= band && c->edge_found) {
                if (c->status.confirmed < ROUTE_CONFIRM_FRAMES) ++c->status.confirmed;
            } else if (new_frame) c->status.confirmed = 0U;
            if (!in->chassis_stopped || !c->status.confirmed) a->settle_at = 0U;
            else if (!a->settle_at) a->settle_at = in->now_ms;
            if (accepted && c->status.confirmed >= ROUTE_CONFIRM_FRAMES && in->chassis_stopped &&
                a->settle_at && (uint32_t)(in->now_ms - a->settle_at) >= ROUTE_EDGE_SETTLE_MS)
                complete_point(c, in, out);
            return;
        }
        a->settle_at = 0U;
        c->status.confirmed = 0U;
        if (remaining < -band) {
            if (accepted && raw_remaining < -band) {
                if (c->overrun_count < ROUTE_CONFIRM_FRAMES) ++c->overrun_count;
            } else if (new_frame) c->overrun_count = 0U;
            if (!c->recovering && c->overrun_count < ROUTE_CONFIRM_FRAMES) {
                c->status.phase = ROUTE_CONFIRM;
                return;
            }
            if (!start_recovery(c, in, remaining)) return;
            speed = -approach_speed(-remaining, ref->mark.slow_band_mm, band);
            if (speed < -ROUTE_RECOVER_MPS) speed = -ROUTE_RECOVER_MPS;
        } else {
            c->overrun_count = 0U;
            c->status.phase = c->edge_found ? ROUTE_LASER_NEAR : ROUTE_CAPTURE;
            speed = approach_speed(remaining, ref->mark.slow_band_mm, band);
            if (speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
        }
        drive_segment(c, in, s, speed, out);
        return;
    }
    c->status.control_source = ROUTE_CONTROL_ODOM;
    remaining = c->edge_found ? c->edge_goal - c->status.progress_mm : nominal;
    c->status.control_remaining_mm = remaining;
    if (c->status.aux_measured_mm != 0U && raw_valid(in->lidar.mm[ref->sensor]) &&
        fabsf((float)in->lidar.mm[ref->sensor] - c->status.aux_measured_mm) <= ROUTE_WALL_INNOVATION_MM &&
        fabsf(c->status.aux_remaining_mm) <= ROUTE_EDGE_AUX_WINDOW_MM &&
        c->status.aux_remaining_mm <= ROUTE_EDGE_AUX_REQUIRED_MM) {
        wait_edge_reference(c, in);
        return;
    }
    if (remaining <= ROUTE_EDGE_AUX_REQUIRED_MM || nominal <= ROUTE_EDGE_AUX_REQUIRED_MM ||
        c->status.progress_mm > s->length_mm + ROUTE_EDGE_SEARCH_EXTRA_MM) {
        wait_edge_reference(c, in);
        return;
    }
    if (c->status.progress_mm >= c->status.gate_mm && mm == 0U) {
        c->status.stop_flags = ROUTE_STOP_NO_LIDAR;
        wait_for_sensor(c, in, ROUTE_ERROR_LIDAR);
        return;
    }
    c->status.reason = ROUTE_ERROR_NONE;
    c->status.phase = c->status.progress_mm >= c->status.gate_mm ? ROUTE_CAPTURE : ROUTE_ODOM_RUN;
    speed = approach_speed(remaining, s->mark.slow_band_mm, ROUTE_ODOM_TOL_MM);
    if (c->status.phase == ROUTE_CAPTURE && speed > ROUTE_SLOW_MPS) speed = ROUTE_SLOW_MPS;
    drive_segment(c, in, s, speed, out);
}

void RouteNav_Step(RouteController *c, const RouteInputs *in, RouteOutput *out)
{
    const RouteSegment *s;
    const RouteAuxReference *aux_ref = NULL;
    uint8_t updated, new_frame, sensor_updated, aux_accepted = 0U;
    uint16_t mm;
    float remaining, max_progress;
    memset(out, 0, sizeof(*out));
    out->correct_axis = 2U;
    new_frame = in->lidar.sequence != 0U && in->lidar.sequence != c->sequence;
    updated = update_filters(c, in);
    c->status.yaw_rad = in->odom.yaw_rad;
    if (!c->started || c->status.phase == ROUTE_DONE || c->status.phase == ROUTE_FAULT) return;
    c->status.stop_flags = 0U;
    c->status.motion = ROUTE_STOP; /* 只有实际写入运动命令时才显示运动方向。 */
    c->status.elapsed_ms = in->now_ms - c->run_at;
    if (c->status.elapsed_ms >= ROUTE_RUN_TIMEOUT_MS) { RouteNav_Fail(c, ROUTE_ERROR_RUN_TIME); return; }

    /* 底盘失能或里程无效时等待，恢复条件满足后回到先前导航阶段。 */
    if (!in->ready || !in->odom.valid ||
        (uint32_t)(in->now_ms - in->odom.tick_ms) > ROUTE_ODOM_MAX_GAP_MS) {
        invalidate_aux_recheck(c);
        if (c->status.phase != ROUTE_WAIT_READY) {
            c->paused_phase = c->status.phase;
            c->ready_at = in->now_ms;
            c->status.phase = ROUTE_WAIT_READY;
        }
        c->status.reason = in->ready ? ROUTE_ERROR_ODOM : ROUTE_ERROR_READY;
        if (!in->ready) c->status.stop_flags |= ROUTE_STOP_BASE_UNREADY;
        if (!in->odom.valid || (uint32_t)(in->now_ms - in->odom.tick_ms) > ROUTE_ODOM_MAX_GAP_MS)
            c->status.stop_flags |= ROUTE_STOP_ODOM_INVALID;
        c->status.confirmed = 0U;
        if ((uint32_t)(in->now_ms - c->ready_at) >= ROUTE_READY_WAIT_MS)
            RouteNav_Fail(c, c->status.reason);
        return;
    }
    if (c->status.phase == ROUTE_WAIT_READY) {
        c->status.phase = c->paused_phase;
        c->status.reason = ROUTE_ERROR_NONE;
        c->progress_at = in->now_ms;
        c->wall_progress_at = in->now_ms;
        c->auxiliary.progress_at = in->now_ms;
        c->auxiliary.settle_at = 0U;
        if (c->searching) c->search_at = in->now_ms;
    }
    if (handle_work(c, in, out)) return;
    if ((uint32_t)(in->now_ms - c->segment_at) >= ROUTE_SEGMENT_TIMEOUT_MS) {
        RouteNav_Fail(c, ROUTE_ERROR_SEGMENT_TIME); return;
    }
    s = RoutePlan_Get(c->status.target_point);
    if (s->reference == ROUTE_REF_ODOM && !recovery_time_ok(c, in)) return;
    c->status.progress_mm = segment_progress(c, in, s->motion);
    mm = reading(c, in, s->trigger);
    sensor_updated = s->trigger < ROUTE_LIDAR_COUNT &&
                     (updated & (uint8_t)(1U << s->trigger)) != 0U;
    c->status.measured_mm = mm;
    remaining = s->length_mm - c->status.progress_mm;
    c->status.odom_remaining_mm = remaining;
    c->status.gate_mm = laser_gate(s);
    c->status.laser_remaining_mm = -1.0f;
    c->status.mismatch_mm = 0.0f;
    c->status.control_remaining_mm = c->status.estimated_end_mm - c->status.progress_mm;
    if (s->reference == ROUTE_REF_WALL && mm != 0U) {
        c->status.laser_remaining_mm = wall_remaining(s, mm);
        c->status.mismatch_mm = c->status.laser_remaining_mm - remaining;
    } else if (s->reference == ROUTE_REF_EDGE && c->edge_found) {
        c->status.laser_remaining_mm = c->edge_goal - c->status.progress_mm;
        c->status.mismatch_mm = c->edge_goal - s->length_mm;
    }
    if (s->reference == ROUTE_REF_EDGE) {
        aux_ref = RoutePlan_AuxReference(c->status.target_point);
        if (!aux_ref) { RouteNav_Fail(c, ROUTE_ERROR_CALIB); return; }
        aux_accepted = observe_aux(c, in, s, aux_ref,
            (updated & (uint8_t)(1U << aux_ref->sensor)) != 0U, new_frame);
    }

    /* 墙参考的轮式行程只告警。地形打滑时，不能让它否决已验证的激光。
     * 纯里程/垄端段仍保留硬界限；墙参考由搜索、激光进展和段/全程时限约束。 */
    max_progress = s->length_mm + ROUTE_MAX_OVERRUN_MM;
    if (s->reference == ROUTE_REF_WALL) max_progress = s->length_mm + ROUTE_WALL_MAX_EXTRA_MM;
    else if (s->reference == ROUTE_REF_EDGE) max_progress += ROUTE_EDGE_ODOM_ERROR_MM;
    if (c->status.progress_mm > max_progress || c->status.progress_mm < -ROUTE_MAX_OVERRUN_MM) {
        if (s->reference == ROUTE_REF_WALL) mark_odom_untrusted(c);
        else {
            c->status.stop_flags = ROUTE_STOP_TRAVEL_LIMIT;
            RouteNav_Fail(c, ROUTE_ERROR_OVERRUN);
            return;
        }
    }
    /* 反向回调也要能刷新进展计时，不能沿用只认正向里程的旧判断。 */
    if (fabsf(c->status.progress_mm - c->progress_checkpoint) >= ROUTE_PROGRESS_STEP_MM) {
        c->progress_checkpoint = c->status.progress_mm;
        c->progress_at = in->now_ms;
    }
    if (s->reference != ROUTE_REF_WALL && !(s->reference == ROUTE_REF_EDGE && c->auxiliary.locked) &&
        (uint32_t)(in->now_ms - c->progress_at) >= ROUTE_NO_PROGRESS_MS &&
        c->status.phase != ROUTE_CONFIRM && c->status.phase != ROUTE_WAIT_LIDAR) {
        RouteNav_Fail(c, ROUTE_ERROR_STALL); return;
    }
    if (new_frame && !sensor_updated && s->reference != ROUTE_REF_EDGE) {
        c->status.confirmed = 0U;
        if (!c->status.wall_locked) c->status.capture_confirmed = 0U;
        c->far_count = c->near_count = c->overrun_count = 0U;
    }

    if (s->reference == ROUTE_REF_ODOM) {
        run_odom_goal(c, in, s, s->length_mm, out);
    } else if (s->reference == ROUTE_REF_WALL) {
        run_wall(c, in, s, mm, sensor_updated, new_frame, out);
    } else {
        observe_edge(c, in, s, mm, new_frame);
        c->status.edge_state = c->edge_found ? 2U : c->edge_armed ? 1U : 0U;
        c->status.edge_confirmed = c->edge_found ? ROUTE_EDGE_FAR_FRAMES :
                                   c->edge_armed ? c->far_count : c->near_count;
        if (c->status.phase == ROUTE_FAULT) return;
        run_edge(c, in, s, aux_ref, mm, aux_accepted, new_frame, out);
    }
}

const char *RouteNav_PhaseName(RoutePhase phase)
{
    static const char *const names[] = {
        "WAIT", "ODOM", "APPROACH", "CONFIRM", "WAIT_LIDAR", "WAIT_READY",
        "WORK_STOP", "WAIT_TOOL", "DONE", "FAULT", "CAPTURE", "RECOVER"
    };
    return (unsigned int)phase <= ROUTE_RECOVER ? names[phase] : "FAULT";
}
const char *RouteNav_ErrorName(RouteError error)
{
    static const char *const names[] = {
        "NONE", "CALIB", "READY", "ODOM", "LIDAR", "MISMATCH", "OVERRUN",
        "NO_PROGRESS", "SEG_TIMEOUT", "RUN_TIMEOUT", "TOOL_FAIL", "TOOL_TIMEOUT", "NO_WORK",
        "REC_TIMEOUT", "REFERENCE", "SEARCH_TIME"
    };
    return (unsigned int)error <= ROUTE_ERROR_SEARCH_TIME ? names[error] : "UNKNOWN";
}
