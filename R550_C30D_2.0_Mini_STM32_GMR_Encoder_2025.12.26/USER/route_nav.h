#ifndef ROUTE_NAV_H
#define ROUTE_NAV_H
#include "route_plan.h"
#include "chassis_odometry.h"

typedef enum { ROUTE_MARK_HOLD, ROUTE_MARK_CRUISE, ROUTE_MARK_SLOW, ROUTE_MARK_SWITCH } RouteMarkResult;
typedef enum {
    ROUTE_WAIT_START, ROUTE_ODOM_RUN, ROUTE_LASER_NEAR, ROUTE_CONFIRM,
    ROUTE_WAIT_LIDAR, ROUTE_WAIT_READY, ROUTE_WORK_STOP, ROUTE_WAIT_TOOL,
    ROUTE_DONE, ROUTE_FAULT,
    ROUTE_CAPTURE, ROUTE_RECOVER /* 追加编号，原有 OLED/VOFA 状态编号保持不变。 */
} RoutePhase;
typedef enum {
    ROUTE_ERROR_NONE, ROUTE_ERROR_CALIB, ROUTE_ERROR_READY, ROUTE_ERROR_ODOM,
    ROUTE_ERROR_LIDAR, ROUTE_ERROR_MISMATCH, ROUTE_ERROR_OVERRUN,
    ROUTE_ERROR_STALL, ROUTE_ERROR_SEGMENT_TIME, ROUTE_ERROR_RUN_TIME,
    ROUTE_ERROR_TOOL, ROUTE_ERROR_TOOL_TIME, ROUTE_ERROR_NO_WORK,
    ROUTE_ERROR_RECOVER_TIME, ROUTE_ERROR_REFERENCE, ROUTE_ERROR_SEARCH_TIME
} RouteError;

/* 可组合的停车条件；故障页 D:xx 和 VOFA 通道 22 用十六进制/十进制显示。 */
typedef enum {
    ROUTE_STOP_NO_LIDAR = 1U,       /* 当前判点通道无有效测距。 */
    ROUTE_STOP_FAR_REFERENCE = 2U,  /* 墙回波超过近距上限。 */
    ROUTE_STOP_POSITION_MISMATCH = 4U, /* 激光与里程位置不一致。 */
    ROUTE_STOP_ODOM_INVALID = 8U,   /* 里程无效或样本过期。 */
    ROUTE_STOP_BASE_UNREADY = 16U,  /* 底盘使能/电压/车型条件不满足。 */
    ROUTE_STOP_TRAVEL_LIMIT = 32U,  /* 触及独立的单段最大行程。 */
    ROUTE_STOP_RECOVER_LIMIT = 64U, /* 回调距离或回调时间超限。 */
    ROUTE_STOP_REFERENCE_INVALID = 128U /* 激光自身不连续或未确认接近过程。 */
} RouteStopFlag;

typedef enum {
    ROUTE_CONF_WAIT, ROUTE_CONF_CANDIDATE, ROUTE_CONF_TRUSTED, ROUTE_CONF_SUSPECT
} RouteConfidence;

/* 独立于停车码：当前参考为什么尚不能用于控制，可按位组合。 */
typedef enum {
    ROUTE_REJECT_NO_DATA = 1U, ROUTE_REJECT_SPREAD = 2U,
    ROUTE_REJECT_STEP = 4U, ROUTE_REJECT_DIRECTION = 8U,
    ROUTE_REJECT_FAR = 16U, ROUTE_REJECT_NO_APPROACH = 32U,
    ROUTE_REJECT_WARMUP = 64U
} RouteRejectReason;

typedef enum {
    ROUTE_CONTROL_NONE, ROUTE_CONTROL_ODOM, ROUTE_CONTROL_WALL, ROUTE_CONTROL_EDGE_AUX
} RouteControlSource;

typedef struct {
    uint8_t at_point, target_point, lane;
    RoutePhase phase;
    RouteError reason;
    RouteMotion motion, next_motion;
    RouteReference reference;
    RouteLidarDirection sensor;
    uint16_t measured_mm, target_mm;
    uint8_t confirmed, work_completed;
    uint8_t edge_state, edge_confirmed; /* 0 等近回波，1 等离开垄端，2 已捕获垄端。 */
    uint8_t stop_flags;
    float progress_mm, length_mm, yaw_rad;
    float odom_remaining_mm, laser_remaining_mm, mismatch_mm, gate_mm;
    float estimated_end_mm;       /* 本段坐标中的估计终点；接管前为计划长度。 */
    float control_remaining_mm;   /* 实际用于速度控制的剩余距离，负值表示越过目标。 */
    uint8_t wall_locked, capture_confirmed; /* 墙参考已接管、捕获连续新帧数。 */
    uint8_t odom_untrusted;       /* 本段里程告警；速度能在参考恢复可信后单独恢复。 */
    float wall_innovation_mm;    /* 新激光剩余 - 上次参考按里程预测的剩余。 */
    uint32_t search_elapsed_ms, reference_idle_ms;
    uint8_t confidence, reject_reason, good_frames, bad_frames;
    uint8_t speed_limited, odom_warn_count, recovering, recovery_resets;
    float edge_last_near_mm, edge_first_far_mm, edge_anchor_mm, edge_uncertainty_mm;
    uint8_t aux_sensor, aux_confidence, aux_reject_reason, aux_good_frames, aux_bad_frames;
    uint16_t aux_measured_mm, aux_target_mm;
    float aux_remaining_mm, edge_speed_cap_mps;
    uint8_t control_source;
    uint32_t elapsed_ms;
} RouteStatus;

typedef struct {
    uint16_t samples[3], median;
    uint8_t count, next;
    uint32_t valid_at;
    uint16_t previous_mm;
    uint32_t sample_at;
    uint8_t sample_seen, good_frames;
    uint16_t trim_target_mm;
    uint32_t trim_sequence;
    uint8_t trim_frames;
} RouteLaserFilter;

typedef struct {
    float sample_remaining, epoch_start, trend_best, remaining, best_error;
    uint32_t sample_at, trusted_at, progress_at, settle_at;
    uint8_t seen, locked, approach_seen, progress_seen;
    uint32_t recheck_at;
    uint16_t recheck_min_mm, recheck_max_mm;
    uint8_t interrupted, recheck_frames, revalidated;
    uint8_t approach_qualified, history_frames;
    uint8_t near_arrival, near_verified;
} RouteAuxTracker;

typedef struct {
    RouteStatus status;
    RouteLaserFilter laser[ROUTE_LIDAR_COUNT];
    RouteAuxTracker auxiliary;
    uint32_t sequence, run_at, segment_at, progress_at, wait_at, ready_at, work_at, settle_at;
    float start_x, start_y, progress_checkpoint, edge_candidate, edge_goal;
    uint32_t edge_near_at, edge_far_at, edge_near_since, edge_frame_at;
    uint8_t edge_near_seen, edge_candidate_valid;
    float wall_sample_remaining, wall_sample_start, wall_trend_best, wall_remaining_mm, wall_best_error;
    uint32_t wall_sample_at, wall_progress_at, search_at;
    uint8_t wall_sample_seen, wall_sample_count, wall_approach_seen, wall_progress_seen;
    uint8_t wall_window_open, searching;
    uint8_t restore_count, recover_exit_count;
    uint32_t wall_at, recover_at;  /* 最后可信墙测距时间、首次开始回调的时间。 */
    uint8_t started, near_count, far_count, edge_armed, edge_found, overrun_count;
    uint8_t work_id, work_dispatched;
    uint8_t recovering;            /* 一次回调的总时限跨 APPROACH/CONFIRM 保持，到下一段清零。 */
    RoutePhase resume_phase, paused_phase;
} RouteController;

typedef struct {
    uint32_t now_ms;
    RouteLidarFrame lidar;
    ChassisOdomFrame odom;
    uint8_t ready, tool_busy, tool_result, tool_fault, chassis_stopped, work_request;
} RouteInputs;

typedef struct {
    float vx_mps, vy_mps; /* 起跑坐标系中的期望速度，任务层再转换到当前车体坐标。 */
    uint8_t start_work, work_finished;
    uint8_t correct_axis; /* 0/1 只校正对应轴，2 表示没有地标校正。 */
    float correct_position_mm;
} RouteOutput;

void RouteNav_Init(RouteController *controller);
void RouteNav_Start(RouteController *controller, const RouteInputs *inputs);
void RouteNav_Step(RouteController *controller, const RouteInputs *inputs, RouteOutput *output);
void RouteNav_Fail(RouteController *controller, RouteError error);
const char *RouteNav_PhaseName(RoutePhase phase);
const char *RouteNav_ErrorName(RouteError error);
RouteMarkResult RouteNav_CheckMark(const RouteMark *mark, uint16_t mm, uint8_t increases);
float RouteNav_LaneVx(uint16_t front_mm, uint16_t behind_mm);
float RouteNav_WallTrim(uint16_t mm, uint16_t target_mm);
#endif
