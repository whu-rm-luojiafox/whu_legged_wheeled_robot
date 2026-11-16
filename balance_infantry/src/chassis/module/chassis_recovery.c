#include "chassis_types.h"
#include "chassis_recovery.h"
#include "chassis_control.h"
#include <math.h>
#include <stdbool.h>

#define FALL_PITCH_RAD 0.80f
#define FALL_ROLL_RAD 0.70f

/* 缩腿等待条件位图；进入 REC_WAIT 后保留，供波形定位超时原因。 */
enum {
    RETRACT_WAIT_TRAJECTORY = 1u << 0,
    RETRACT_WAIT_LENGTH_L = 1u << 1,
    RETRACT_WAIT_LENGTH_R = 1u << 2
};

typedef struct {
    bool ready, stalled;
    float direction;
    float hold_angle;
    float progress_angle, no_progress_s;
    float last_angle;
    float rate_integral_nm;
} SwingLegContext;

typedef struct {
    chassis_recovery_stage_e state;
    bool last_key;
    float elapsed_s, settled_s;
    float control_dt_s;
    float swing_direction; /* 双腿统一方向：从车体左侧看顺时针。 */
    SwingLegContext swing_L, swing_R;
    float retract_time_s;
    float start_length_L, start_length_R, target_length;
    float start_angle_L, start_angle_R, target_angle;
    float force_bias_L, force_bias_R;
    float torque_bias_L, torque_bias_R;
    float length_error_L, length_error_R; /* 目标 - 实测，m */
    float angle_error_L, angle_error_R;   /* 目标 - 实测，rad */
    uint8_t retract_blockers;
    float hold_force_L, hold_force_R;   /* LQR 接管前最后的径向 PID 力，N */
    float hold_torque_L, hold_torque_R; /* LQR 接管前最后的虚拟腿角 PD 力矩，N*m */
} RecoveryContext;

RecoveryContext recovery;

static float smoothstep(float progress)
{
    progress = fp32_constrain(progress, 0.0f, 1.0f);
    return progress * progress * (3.0f - 2.0f * progress);
}

static float body_leg_angle(const chassis_leg_posture_t *leg, bool right_leg)
{
    /* 只把腿摆到机身后方；不随 pitch 追对地角，右侧镜像只统一一次。 */
    return rad_format(right_leg ? -leg->leg_angle_local : leg->leg_angle_local);
}

static float body_leg_rate(const chassis_leg_posture_t *leg, bool right_leg)
{
    return right_leg ? -leg->leg_gyro_local : leg->leg_gyro_local;
}

static bool leg_in_swing_window(const chassis_leg_posture_t *leg, bool right_leg)
{
    const float angle = body_leg_angle(leg, right_leg);
    return angle >= STARTUP_LEG_ANGLE_MIN_RAD && angle <= STARTUP_LEG_ANGLE_MAX_RAD;
}

static void update_swing_leg(SwingLegContext *swing, const chassis_leg_posture_t *leg, bool right_leg, float dt_s)
{
    const float angle = body_leg_angle(leg, right_leg);
    const float travel = swing->direction * rad_format(angle - swing->last_angle);
    const float to_entry = swing->direction * rad_format(STARTUP_LEG_ANGLE_MAX_RAD - swing->last_angle);
    const bool crossed_entry = travel > 0.0f && to_entry >= 0.0f && to_entry <= travel;
    swing->last_angle = angle;
    if (!swing->ready && (leg_in_swing_window(leg, right_leg) || crossed_entry))
    {
        swing->ready = true;
        /* 到后方即停，保持实际到位角，不回追标称值或重新绕圈。 */
        swing->hold_angle = angle;
        swing->rate_integral_nm = 0.0f;
        swing->stalled = false;
        swing->no_progress_s = 0.0f;
    }
    if (swing->ready) return;

    /* 不对机械卡点加力硬顶；未到窗口且局部腿角长期无进展就中止本次动作。 */
    if (fabsf(rad_format(leg->leg_angle_local - swing->progress_angle)) >= STARTUP_SWING_STALL_PROGRESS_RAD)
    {
        swing->progress_angle = leg->leg_angle_local;
        swing->no_progress_s = 0.0f;
    }
    else
        swing->no_progress_s += dt_s;
    swing->stalled = swing->no_progress_s >= STARTUP_SWING_STALL_TIME_S;
}

static float swing_leg_rate(const SwingLegContext *swing)
{
    return swing->ready ? 0.0f : swing->direction * LEG_SWING_MOTOR_SPEED_RAD_S;
}

static void enter_stage(chassis_recovery_stage_e state)
{
    recovery.state = state;
    recovery.elapsed_s = recovery.settled_s = 0.0f;
}

static float startup_length(const chassis_move_t *c)
{
    return fp32_constrain(c->chassis_data_->high_set, 0.17f, 0.34f);
}

static bool both_legs_short(const chassis_move_t *c)
{
    const float length = startup_length(c);
    return c->chassis_posture_info.chassis_posture_L.leg_length <= length &&
           c->chassis_posture_info.chassis_posture_R.leg_length <= length;
}

static void begin_balance(chassis_move_t *c)
{
    chassis_posture_info_t *p = &c->chassis_posture_info;
    enter_stage(REC_IDLE);
    recovery.retract_blockers = 0u;
    recovery.target_length = startup_length(c);
    c->mode.chassis_balancing_mode = BALANCING_READY;
    c->mode.sport_mode = NORMAL_MOVING_MODE;
    p->chassis_posture_L.leg_length_set = p->chassis_posture_R.leg_length_set = recovery.target_length;
    p->chassis_posture_L.leg_angle_set = p->chassis_posture_R.leg_angle_set = 0.0f;
    p->foot_speed_set = p->yaw_gyro_set = 0.0f;
    p->foot_distance_set = p->target_distance_set = p->foot_distance_K;
    p->position_lock_flag = p->position_lock_state = 0u;
    p->yaw_angle_sett = p->yaw_angle_total;
    PID_clear(&c->leg_L_length_pid);
    PID_clear(&c->leg_R_length_pid);
    PID_clear(&c->chassis_roll_pid);
}

static SwingLegContext begin_swing_leg(const chassis_leg_posture_t *leg, bool right_leg)
{
    return (SwingLegContext){
        .direction = STARTUP_SWING_DIRECTION,
        .progress_angle = leg->leg_angle_local,
        .last_angle = body_leg_angle(leg, right_leg)
    };
}

static void begin_swing(chassis_move_t *c)
{
    enter_stage(REC_SWING);
    recovery.retract_blockers = 0u;
    recovery.length_error_L = recovery.length_error_R = 0.0f;
    recovery.angle_error_L = recovery.angle_error_R = 0.0f;
    recovery.force_bias_L = recovery.force_bias_R = 0.0f;
    recovery.torque_bias_L = recovery.torque_bias_R = 0.0f;
    /* 劈叉时两腿也统一从左侧看顺时针，不按各自角差选择最短路径。
     * 左侧电机负速度，右侧驱动端镜像为正速度，由 ControlMap 统一转换。 */
    recovery.swing_L = begin_swing_leg(&c->chassis_posture_info.chassis_posture_L, false);
    recovery.swing_R = begin_swing_leg(&c->chassis_posture_info.chassis_posture_R, true);
    recovery.swing_direction = recovery.swing_L.direction;
    update_swing_leg(&recovery.swing_L, &c->chassis_posture_info.chassis_posture_L, false, 0.0f);
    update_swing_leg(&recovery.swing_R, &c->chassis_posture_info.chassis_posture_R, true, 0.0f);
    c->mode.chassis_balancing_mode = FOOT_LAUNCHING;
    PID_clear(&c->leg_L_length_pid);
    PID_clear(&c->leg_R_length_pid);
    PID_clear(&c->chassis_roll_pid);
}

static void begin_startup(chassis_move_t *c)
{
    if (both_legs_short(c)) begin_balance(c);
    else begin_swing(c);
}

static void update_retract_status(const chassis_posture_info_t *p)
{
    recovery.length_error_L = recovery.target_length - p->chassis_posture_L.leg_length;
    recovery.length_error_R = recovery.target_length - p->chassis_posture_R.leg_length;
    recovery.angle_error_L = rad_format(recovery.start_angle_L - body_leg_angle(&p->chassis_posture_L, false));
    recovery.angle_error_R = rad_format(recovery.start_angle_R - body_leg_angle(&p->chassis_posture_R, true));
    uint8_t blockers = 0u;
    if (recovery.elapsed_s < recovery.retract_time_s) blockers |= RETRACT_WAIT_TRAJECTORY;
    if (!(p->chassis_posture_L.leg_length <= recovery.target_length + STARTUP_LENGTH_TOLERANCE_M)) blockers |= RETRACT_WAIT_LENGTH_L;
    if (!(p->chassis_posture_R.leg_length <= recovery.target_length + STARTUP_LENGTH_TOLERANCE_M)) blockers |= RETRACT_WAIT_LENGTH_R;
    /* 腿角误差仅供观察，不作为缩腿完成或进入 LQR 的门槛。 */
    recovery.retract_blockers = blockers;
}

static void begin_retract(chassis_move_t *c)
{
    const chassis_leg_posture_t *left = &c->chassis_posture_info.chassis_posture_L;
    const chassis_leg_posture_t *right = &c->chassis_posture_info.chassis_posture_R;
    enter_stage(REC_RETRACT);
    c->mode.chassis_balancing_mode = JOINT_LAUNCHING;
    recovery.start_length_L = left->leg_length;
    recovery.start_length_R = right->leg_length;
    recovery.start_angle_L = body_leg_angle(left, false);
    recovery.start_angle_R = body_leg_angle(right, true);
    recovery.target_length = startup_length(c);
    recovery.target_angle = STARTUP_SWING_TARGET_ANGLE_RAD;
    const float distance = fmaxf(0.0f, fmaxf(left->leg_length - recovery.target_length,
                                            right->leg_length - recovery.target_length));
    /* 三次曲线最大斜率为 1.5，确保整段缩腿参考速度不超过设定值。 */
    recovery.retract_time_s = fmaxf(STARTUP_RETRACT_MIN_TIME_S,
                                    1.5f * distance / STARTUP_RETRACT_SPEED_M_S);
    update_retract_status(&c->chassis_posture_info);
    /* 氮气弹簧提供被动支撑，不把长腿姿态的力矩反馈灌进收腿 PID。 */
    PID_clear(&c->leg_L_length_pid);
    PID_clear(&c->leg_R_length_pid);
    recovery.force_bias_L = recovery.force_bias_R = 0.0f;
    /* 速度积分不能冻结成收腿阶段的持续推力；改为保持各腿实际后摆角。 */
    recovery.torque_bias_L = recovery.torque_bias_R = 0.0f;
}

static void begin_blend(const chassis_move_t *c)
{
    enter_stage(REC_BLEND);
    recovery.hold_force_L = c->torque_info.joint_stand_torque_L;
    recovery.hold_force_R = c->torque_info.joint_stand_torque_R;
    recovery.hold_torque_L = c->torque_info.joint_balancing_torque_L;
    recovery.hold_torque_R = c->torque_info.joint_balancing_torque_R;
    /* 保留腿长 PID 的积分和误差历史，避免接管时丢掉弹簧负载补偿。 */
}

static void set_retract_references(chassis_move_t *c)
{
    chassis_leg_posture_t *left = &c->chassis_posture_info.chassis_posture_L;
    chassis_leg_posture_t *right = &c->chassis_posture_info.chassis_posture_R;
    const float progress = recovery.state == REC_BLEND ? 1.0f :
        smoothstep(recovery.elapsed_s / recovery.retract_time_s);
    const float balance_angle = chassisControlLegAngleReference();
    const float alpha = recovery.state == REC_BLEND ? chassisRecoveryBalanceScale() : 0.0f;
    const float target_left = fminf(recovery.start_length_L, recovery.target_length);
    const float target_right = fminf(recovery.start_length_R, recovery.target_length);
    /* 收腿阶段不伸长已短的腿；只有 LQR 开始交接后才恢复正常站立高度。 */
    left->leg_length_set = recovery.start_length_L + progress * (target_left - recovery.start_length_L) +
                           alpha * (recovery.target_length - target_left);
    right->leg_length_set = recovery.start_length_R + progress * (target_right - recovery.start_length_R) +
                            alpha * (recovery.target_length - target_right);
    /* 缩腿保持相对机身后摆角；LQR 使用对地角，交接时才转换并回配平角。
     * 加 pitch 仅用于生成同坐标的 LQR 参考，不驱动缩腿 PD 去补偿机身姿态。 */
    left->leg_angle_set = (1.0f - alpha) *
        (recovery.start_angle_L + c->chassis_posture_info.pitch_angle - balance_angle);
    right->leg_angle_set = (1.0f - alpha) *
        (recovery.start_angle_R + c->chassis_posture_info.pitch_angle - balance_angle);
}

void chassisRecoveryUpdate(chassis_move_t *c, float dt_s)
{
    chassis_posture_info_t *p = &c->chassis_posture_info;
    const bool key = c->chassis_data_->recovery_flag != 0u;
    const bool pressed = key && !recovery.last_key;
    recovery.last_key = key;

    if (c->mode.chassis_mode != ENABLE_CHASSIS)
    {
        recovery = (RecoveryContext){.last_key = key};
        return;
    }

    recovery.control_dt_s = isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.05f ? dt_s : 0.0f;
    recovery.elapsed_s += dt_s;
    switch (recovery.state)
    {
    case REC_IDLE:
        if (c->mode.chassis_balancing_mode == FOOT_LAUNCHING)
            begin_startup(c);
        else if (c->mode.sport_mode != JUMPING_MODE &&
                 (fabsf(p->pitch_angle) > FALL_PITCH_RAD || fabsf(p->roll_angle) > FALL_ROLL_RAD))
        {
            if (pressed) begin_startup(c);
            else enter_stage(REC_WAIT);
        }
        break;

    case REC_WAIT:
        if (pressed) begin_startup(c);
        break;

    case REC_SWING:
        if (both_legs_short(c))
        {
            begin_balance(c);
            break;
        }
        update_swing_leg(&recovery.swing_L, &p->chassis_posture_L, false, dt_s);
        update_swing_leg(&recovery.swing_R, &p->chassis_posture_R, true, dt_s);
        /* 各腿到位分别锁存，不要求两腿同一拍卡在窄窗口内。 */
        if (recovery.swing_L.ready && recovery.swing_R.ready) begin_retract(c);
        else if (recovery.swing_L.stalled || recovery.swing_R.stalled) enter_stage(REC_WAIT);
        else if (recovery.elapsed_s >= STARTUP_PHASE_TIMEOUT_S) enter_stage(REC_WAIT);
        break;

    case REC_RETRACT:
        update_retract_status(p);
        if (recovery.retract_blockers == 0u)
            recovery.settled_s += dt_s;
        else
            recovery.settled_s = 0.0f;
        if (recovery.settled_s >= STARTUP_SETTLE_TIME_S) begin_blend(c);
        else if (recovery.elapsed_s >= recovery.retract_time_s + STARTUP_PHASE_TIMEOUT_S) enter_stage(REC_WAIT);
        break;

    case REC_BLEND:
        if (fabsf(p->pitch_angle) > FALL_PITCH_RAD || fabsf(p->roll_angle) > FALL_ROLL_RAD)
            enter_stage(REC_WAIT);
        else if (recovery.elapsed_s >= STARTUP_LQR_BLEND_TIME_S)
        {
            enter_stage(REC_IDLE);
            c->mode.chassis_balancing_mode = BALANCING_READY;
            c->mode.sport_mode = NORMAL_MOVING_MODE;
        }
        break;
    }

    if (recovery.state == REC_IDLE) return;
    c->mode.sport_mode = NONE;
    c->mode.jumping_stage = FINISHED;
    if (recovery.state == REC_WAIT) c->mode.chassis_balancing_mode = NO_FORCE;
    p->foot_speed_set = p->yaw_gyro_set = 0.0f;
    p->foot_distance_set = p->target_distance_set = p->foot_distance_K;
    p->position_lock_flag = p->position_lock_state = 0u;
    p->yaw_angle_sett = p->yaw_angle_total;
    if (recovery.state == REC_RETRACT || recovery.state == REC_BLEND) set_retract_references(c);
}

static void clear_commands(torque_info_t *t)
{
    t->foot_balancing_torque_L = t->foot_balancing_torque_R = 0.0f;
    t->foot_moving_torque_L = t->foot_moving_torque_R = 0.0f;
    t->joint_balancing_torque_L = t->joint_balancing_torque_R = 0.0f;
    t->joint_moving_torque_L = t->joint_moving_torque_R = 0.0f;
    t->joint_roll_torque_L = t->joint_roll_torque_R = 0.0f;
    t->joint_stand_torque_L = t->joint_stand_torque_R = 0.0f;
}

static float leg_angle_torque(const chassis_leg_posture_t *leg, bool right_leg, float target_angle)
{
    return fp32_constrain(STARTUP_ANGLE_KP * rad_format(target_angle - body_leg_angle(leg, right_leg)) -
                                STARTUP_ANGLE_KD * body_leg_rate(leg, right_leg),
                          -STARTUP_LEG_TORQUE_LIMIT_NM, STARTUP_LEG_TORQUE_LIMIT_NM);
}

static float swing_angle_torque(SwingLegContext *swing, const chassis_leg_posture_t *leg,
                                bool right_leg, float desired_rate)
{
    if (swing->ready)
    {
        swing->rate_integral_nm = 0.0f;
        return leg_angle_torque(leg, right_leg, swing->hold_angle);
    }
    const float error = desired_rate - body_leg_rate(leg, right_leg);
    if (!isfinite(error)) return 0.0f;
    const float limit = STARTUP_SWING_TORQUE_LIMIT_NM;
    const float p = STARTUP_SWING_RATE_KP * error;
    const float delta_i = STARTUP_SWING_RATE_KI * error * recovery.control_dt_s;
    const float candidate = fp32_constrain(swing->rate_integral_nm + delta_i, -limit, limit);
    if (fabsf(p + candidate) <= limit || (p + candidate) * delta_i <= 0.0f)
        swing->rate_integral_nm = candidate;
    return fp32_constrain(p + swing->rate_integral_nm, -limit, limit);
}

static float retract_force(pid_type_def *pid, float length, float target)
{
    PID_calc(pid, length, target);
    /* 仅收腿：超调或短腿不输出正向撑长力，也不积累正向积分。 */
    pid->Iout = fminf(pid->Iout, 0.0f);
    pid->out = fp32_constrain(pid->Pout + pid->Iout + pid->Dout, -pid->max_out, 0.0f);
    return pid->out;
}

void chassisRecoveryApply(chassis_move_t *c)
{
    torque_info_t *t = &c->torque_info;
    const chassis_leg_posture_t *left = &c->chassis_posture_info.chassis_posture_L;
    const chassis_leg_posture_t *right = &c->chassis_posture_info.chassis_posture_R;
    c->swing_velocity_active = 0u;
    c->swing_rate_L = c->swing_rate_R = 0.0f;
    switch (recovery.state)
    {
    case REC_IDLE:
        break;
    case REC_WAIT:
        clear_commands(t);
        break;
    case REC_SWING:
        clear_commands(t);
        c->swing_rate_L = swing_leg_rate(&recovery.swing_L);
        c->swing_rate_R = swing_leg_rate(&recovery.swing_R);
        /* 仅角向力矩：径向力始终为零，允许弹簧改变腿长。
         * 两关节独立速度环会间接锁住开合角，因此此处必须用 MIT + VMC。 */
        t->joint_balancing_torque_L = swing_angle_torque(&recovery.swing_L, left, false, c->swing_rate_L);
        t->joint_balancing_torque_R = swing_angle_torque(&recovery.swing_R, right, true, c->swing_rate_R);
        break;
    case REC_RETRACT:
        clear_commands(t);
        t->joint_stand_torque_L = retract_force(&c->leg_L_length_pid, left->leg_length, left->leg_length_set);
        t->joint_stand_torque_R = retract_force(&c->leg_R_length_pid, right->leg_length, right->leg_length_set);
        t->joint_balancing_torque_L = leg_angle_torque(left, false, recovery.start_angle_L);
        t->joint_balancing_torque_R = leg_angle_torque(right, true, recovery.start_angle_R);
        break;
    case REC_BLEND:
    {
        const float alpha = chassisRecoveryBalanceScale();
        t->joint_stand_torque_L = (1.0f - alpha) * recovery.hold_force_L + alpha * t->joint_stand_torque_L;
        t->joint_stand_torque_R = (1.0f - alpha) * recovery.hold_force_R + alpha * t->joint_stand_torque_R;
        t->joint_balancing_torque_L = (1.0f - alpha) * recovery.hold_torque_L + alpha * t->joint_balancing_torque_L;
        t->joint_balancing_torque_R = (1.0f - alpha) * recovery.hold_torque_R + alpha * t->joint_balancing_torque_R;
        t->joint_moving_torque_L *= alpha;
        t->joint_moving_torque_R *= alpha;
        t->joint_roll_torque_L *= alpha;
        t->joint_roll_torque_R *= alpha;
        t->foot_balancing_torque_L *= alpha;
        t->foot_balancing_torque_R *= alpha;
        t->foot_moving_torque_L *= alpha;
        t->foot_moving_torque_R *= alpha;
        break;
    }
    }
}

uint8_t chassisRecoveryIsActive(void)
{
    return recovery.state != REC_IDLE;
}

float chassisRecoveryBalanceScale(void)
{
    if (recovery.state == REC_IDLE) return 1.0f;
    if (recovery.state == REC_BLEND) return smoothstep(recovery.elapsed_s / STARTUP_LQR_BLEND_TIME_S);
    return 0.0f;
}

float chassisRecoveryJointTorqueLimit(void)
{
    if (recovery.state == REC_WAIT) return 0.0f;
    return STARTUP_JOINT_TORQUE_LIMIT_NM + chassisRecoveryBalanceScale() *
        (JOINT_TORQUE_LIMIT - STARTUP_JOINT_TORQUE_LIMIT_NM);
}
