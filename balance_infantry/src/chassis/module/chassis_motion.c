#include "chassis_types.h"
#include "chassis_motion.h"
#include "chassis_follow.h"
#include "chassis_mode.h"
#include "chassis_observer.h"
#include "chassis_climb.h"
#include "chassis_recovery.h"
#include "chassis_power.h"
#include "arm_math.h"
#include <math.h>

chassis_follow_t chassis_follow __attribute__((section(".bss.chassis_follow")));
chassis_position_t chassis_position __attribute__((section(".bss.chassis_position")));
volatile fp32 tune_follow_kp __attribute__((section(".data.tune_follow_kp"))) = FOLLOW_KP_RAD_S_PER_DEG;
volatile fp32 tune_follow_rate_limit __attribute__((section(".data.tune_follow_rate_limit"))) =
    FOLLOW_MAX_RATE_RAD_S;
volatile fp32 tune_follow_accel __attribute__((section(".data.tune_follow_accel"))) =
    FOLLOW_RATE_ACCEL_RAD_S2;
volatile fp32 tune_follow_filter_s __attribute__((section(".data.tune_follow_filter_s"))) =
    FOLLOW_ERROR_FILTER_S;
volatile fp32 tune_gimbal_forward_deg __attribute__((section(".data.tune_gimbal_forward_deg"))) =
    CHASSIS_GIMBAL_FORWARD_DEG;
fp32 normal_move_scale __attribute__((section(".data.normal_move_scale"))) = 1.0f;

static float normalized_speed, move_angle;
static void chassis_update_stop_position(chassis_move_t *chassis, float dt_s);
static void chassis_update_yaw_follow(chassis_move_t *chassis, float move_deg, float dt_s);

float chassis_position_speed_request(float command_m_s)
{
    if (!isfinite(command_m_s) || fabsf(command_m_s) <= POSITION_COMMAND_DEADBAND_M_S) return 0.0f;
    return command_m_s;
}

static void release(chassis_position_t *state, float distance_m)
{
    state->locked = 0u;
    state->stopped_time_s = 0.0f;
    state->reference_m = distance_m;
}

void chassis_position_step(chassis_position_t *state, const chassis_position_input_t *input)
{
    if (!input->allowed || !isfinite(input->distance_m) || !isfinite(input->dt_s) || input->dt_s <= 0.0f ||
        input->dt_s > 0.05f || !isfinite(input->command_speed_m_s) || !isfinite(input->measured_speed_m_s) ||
        chassis_position_speed_request(input->command_speed_m_s) != 0.0f)
    {
        release(state, isfinite(input->distance_m) ? input->distance_m : 0.0f);
        return;
    }
    if (state->locked) return; /* 捕获后固定目标，不能每拍重新跟随实际位置。 */

    /* 松杆先让零速反馈制动，位置参考随车走，避免立刻倒回松杆点。 */
    state->reference_m = input->distance_m;
    if (fabsf(input->measured_speed_m_s) <= POSITION_SETTLED_SPEED_M_S)
        state->stopped_time_s += input->dt_s;
    else
        state->stopped_time_s = 0.0f;

    /* 必须连续低速后才捕获停车点，不能因超时在滑行中强制锁定。 */
    if (state->stopped_time_s >= POSITION_SETTLE_TIME_S)
        state->locked = 1u;
}

void chassisMotionUpdate(chassis_move_t *target_value_set, float dt_s)
{
    const chassis_data_t *cmd = target_value_set->chassis_data_;
    float speed = fminf(sqrtf(cmd->vx_set * cmd->vx_set + cmd->vy_set * cmd->vy_set), 2.8f);
    normalized_speed = cmd->vx_set >= 0.0f ? speed : -speed;
    move_angle = atan2f(cmd->vy_set, cmd->vx_set) * 180.0f / PI;
    if (move_angle > 90.0f)
        move_angle -= 180.0f;
    else if (move_angle < -90.0f)
        move_angle += 180.0f;

    //底盘正方向速度控制
    if (target_value_set->mode.sport_mode != NONE &&
        target_value_set->flag_info.suspend_flag_L == ON_GROUND &&
        target_value_set->flag_info.suspend_flag_R == ON_GROUND)
    {
        if (fabs(target_value_set->chassis_data_->wz_set) < CHASSIS_RC_WZ_DEADLINE)
        {
            target_value_set->chassis_posture_info.foot_speed_set = chassis_position_speed_request(
                fp32_constrain(normalized_speed * normal_move_scale, -2.2f, 2.2f));
            chassis_power_limit(target_value_set);
        }
        else
        {
            /* A manual spin must not reuse the previous translation command. */
            target_value_set->chassis_posture_info.foot_speed_set = 0.0f;
        }
    }
    else
    {
        target_value_set->chassis_posture_info.foot_speed_set = 0;
    }

    /* Preserve real operator input during climb preparation/extension. The
     * climb module limits speed; stop-position hold releases on movement and
     * captures again after release, just as in ordinary control. */

    //位置环控制器
    chassis_update_stop_position(target_value_set, dt_s);

    chassis_update_yaw_follow(target_value_set, move_angle, dt_s);

    target_value_set->chassis_posture_info.roll_angle_set = 0.0f;

    target_value_set->chassis_posture_info.ideal_high =
        fp32_constrain(target_value_set->chassis_data_->high_set, 0.17f, 0.34f);
    float height = target_value_set->chassis_posture_info.ideal_high;
    float offset = 0.0f;
    if (target_value_set->flag_info.suspend_flag_L != OFF_GROUND ||
        target_value_set->flag_info.suspend_flag_R != OFF_GROUND)
    {
        float roll = target_value_set->chassis_posture_info.roll_angle;
        target_value_set->chassis_posture_info.foot_roll_angle = roll;
        offset = 0.24f * arm_sin_f32(roll) / arm_cos_f32(roll);
    }
    target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = height - offset;
    target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = height + offset;
}

static void chassis_update_stop_position(chassis_move_t *chassis, float dt_s)
{
    chassis_position_input_t input = {0};
    /* Position hold remains available during preparation, extension and return.
     * Genuine movement releases it through chassis_position_step(). Feedforward
     * and cross-leg feedback retain their existing mode selection. */
    input.allowed =
        chassisIsEnabledOnGround(chassis) && chassis->mode.chassis_balancing_mode == BALANCING_READY &&
        chassis->mode.sport_mode == NORMAL_MOVING_MODE && chassisClimbAllowsPositionHold() &&
        !chassisRecoveryIsActive() && !chassis->chassis_data_->recovery_flag &&
        chassis_odometry.sample_valid && fabsf(chassis->chassis_data_->wz_set) < CHASSIS_RC_WZ_DEADLINE;
    input.dt_s = dt_s;
    input.command_speed_m_s = chassis->chassis_posture_info.foot_speed_set;
    input.measured_speed_m_s = chassis_odometry.speed_m_s;
    input.distance_m = chassis_odometry.distance_m;
    chassis_position_step(&chassis_position, &input);
    chassis->chassis_posture_info.position_lock_flag = chassis_position.locked;
    chassis->chassis_posture_info.position_lock_state = chassis_position.locked;
    chassis->chassis_posture_info.target_distance_set = chassis_position.reference_m;
    chassis->chassis_posture_info.foot_distance_set = chassis_position.reference_m;
}

static void chassis_update_yaw_follow(chassis_move_t *chassis, float move_deg, float dt_s)
{
    chassis_follow_input_t input = {0};
    const fp32 forward_deg = tune_gimbal_forward_deg;
    if (isfinite(forward_deg) && forward_deg >= -180.0f && forward_deg <= 180.0f)
        chassis->gimbal_yaw_motor.relative_angle_init = forward_deg;
    input.dt_s = dt_s;
    input.allowed = chassisIsEnabledOnGround(chassis) && chassis->mode.sport_mode != NONE;
    input.relative_deg = chassis->gimbal_yaw_motor.relative_angle;
    input.zero_deg = chassis->gimbal_yaw_motor.relative_angle_init;
    /* 无平移指令时朝向云台，忽略死区内 atan2 得到的不稳定方向。 */
    input.move_deg = chassis->chassis_posture_info.foot_speed_set != 0.0f ? move_deg : 0.0f;
    input.yaw_rad = chassis->chassis_posture_info.yaw_angle_total;
    input.manual_rate_rad_s = chassis->chassis_data_->wz_set;
    input.kp_rad_s_per_deg = tune_follow_kp;
    input.rate_limit_rad_s = tune_follow_rate_limit;
    input.accel_rad_s2 = tune_follow_accel;
    input.filter_s = tune_follow_filter_s;
    chassis_follow_step(&chassis_follow, &input);
    chassis->chassis_posture_info.yaw_angle_sett = chassis_follow.angle_ref_rad;
    chassis->chassis_posture_info.yaw_gyro_set = chassis_follow.rate_ref_rad_s;
    chassis->chassis_posture_info.yaw_lock_flag = !chassis_follow.heading_locked;
    chassis->gimbal_yaw_motor.relative_limit = chassis_follow.error_deg;
}
