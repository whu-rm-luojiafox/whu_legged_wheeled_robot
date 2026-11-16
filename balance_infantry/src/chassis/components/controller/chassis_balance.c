#include "chassis_balance.h"
#include <math.h>

static float approach(float current, float target, float step)
{
    return current + fmaxf(-step, fminf(target - current, step));
}

void chassis_balance_reference_step(chassis_balance_reference_t *state,
                                    uint8_t enabled, float pitch_rad,
                                    float leg_ground_rad, float dt_s)
{
    if (!state) return;
    if (!state->initialized) {
        state->pitch_rad = BALANCE_PITCH_DEFAULT_RAD;
        state->leg_ground_rad = BALANCE_LEG_GROUND_DEFAULT_RAD;
        state->leg_local_rad = state->leg_ground_rad - state->pitch_rad;
        state->initialized = 1u;
    }
    state->request_valid = 0u;
    if (!isfinite(pitch_rad) || fabsf(pitch_rad) > BALANCE_PITCH_LIMIT_RAD ||
        !isfinite(leg_ground_rad) || fabsf(leg_ground_rad) > BALANCE_LEG_GROUND_LIMIT_RAD ||
        fabsf(leg_ground_rad - pitch_rad) > BALANCE_LEG_LOCAL_LIMIT_RAD)
        return;
    if (enabled && (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f))
        return;

    state->request_valid = 1u;
    if (enabled) {
        const float step = BALANCE_BIAS_SLEW_RAD_S * dt_s;
        state->pitch_rad = approach(state->pitch_rad, pitch_rad, step);
        state->leg_ground_rad = approach(state->leg_ground_rad, leg_ground_rad, step);
    } else {
        state->pitch_rad = pitch_rad;
        state->leg_ground_rad = leg_ground_rad;
    }
    state->leg_local_rad = state->leg_ground_rad - state->pitch_rad;
}

/* 将一行增益逐项展开：每个字段都能直接对应一个状态产生的力矩。 */
static void calculate_torque_row(const float gain[CHASSIS_LQR_STATE_COUNT],
                                const float error[CHASSIS_LQR_STATE_COUNT],
                                float left_leg_scale, float right_leg_scale,
                                chassis_lqr_torque_t *torque)
{
    torque->position_nm = gain[LQR_X_POSITION] * error[LQR_X_POSITION];
    torque->speed_nm = gain[LQR_X_SPEED] * error[LQR_X_SPEED];
    torque->yaw_nm = gain[LQR_X_YAW] * error[LQR_X_YAW];
    torque->yaw_rate_nm = gain[LQR_X_YAW_RATE] * error[LQR_X_YAW_RATE];

    torque->left_leg_angle_nm = left_leg_scale * gain[LQR_X_LEG_L_ANGLE] * error[LQR_X_LEG_L_ANGLE];
    torque->left_leg_rate_nm = left_leg_scale * gain[LQR_X_LEG_L_RATE] * error[LQR_X_LEG_L_RATE];
    torque->right_leg_angle_nm = right_leg_scale * gain[LQR_X_LEG_R_ANGLE] * error[LQR_X_LEG_R_ANGLE];
    torque->right_leg_rate_nm = right_leg_scale * gain[LQR_X_LEG_R_RATE] * error[LQR_X_LEG_R_RATE];

    torque->body_pitch_nm = gain[LQR_X_BODY_PITCH] * error[LQR_X_BODY_PITCH];
    torque->body_pitch_rate_nm = gain[LQR_X_BODY_PITCH_RATE] * error[LQR_X_BODY_PITCH_RATE];

    torque->motion_nm = torque->position_nm + torque->speed_nm + torque->yaw_nm + torque->yaw_rate_nm;
    torque->balance_nm = torque->left_leg_angle_nm + torque->left_leg_rate_nm +
                         torque->right_leg_angle_nm + torque->right_leg_rate_nm +
                         torque->body_pitch_nm + torque->body_pitch_rate_nm;
    torque->total_nm = torque->motion_nm + torque->balance_nm;
}

void chassis_lqr_feedback_calculate(const float gain[CHASSIS_LQR_INPUT_COUNT][CHASSIS_LQR_STATE_COUNT],
                                   const float error[CHASSIS_LQR_STATE_COUNT],
                                   float cross_leg_scale, chassis_lqr_feedback_t *output)
{
    if (!gain || !error || !output) return;
    if (!isfinite(cross_leg_scale) || cross_leg_scale < 0.0f || cross_leg_scale > 1.0f)
        cross_leg_scale = 0.0f;

    /* 左侧输出：左腿权重为 1，右腿权重为 cross_leg_scale。 */
    calculate_torque_row(gain[LQR_U_WHEEL_L], error, 1.0f, cross_leg_scale, &output->left_wheel);
    calculate_torque_row(gain[LQR_U_LEG_L], error, 1.0f, cross_leg_scale, &output->left_leg);
    /* 右侧输出：右腿权重为 1，左腿权重为 cross_leg_scale。 */
    calculate_torque_row(gain[LQR_U_WHEEL_R], error, cross_leg_scale, 1.0f, &output->right_wheel);
    calculate_torque_row(gain[LQR_U_LEG_R], error, cross_leg_scale, 1.0f, &output->right_leg);
}

static float clamp(float value, float limit)
{
    return fmaxf(-limit, fminf(value, limit));
}

void chassis_feedforward_step(chassis_feedforward_t *state, uint8_t allowed,
                              uint8_t enabled, float signed_mass_offset_kg_m,
                              float pitch_rad, float dt_s)
{
    if (!state) return;
    if (!allowed || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f ||
        !isfinite(pitch_rad) || fabsf(pitch_rad) > 0.35f ||
        !isfinite(signed_mass_offset_kg_m) ||
        fabsf(signed_mass_offset_kg_m) > COM_FF_MAX_MOMENT_KG_M) {
        state->target_per_leg_nm = state->applied_per_leg_nm = 0.0f;
        return;
    }

    /* 水平偏置重力矩 g*m*dx*cos(pitch)，左右平均分担。
     * 本模块不自动学习偏置，也不抵消已有角度偏置；必须单独对比试验。 */
    state->target_per_leg_nm = enabled ?
        clamp(0.5f * 9.81f * signed_mass_offset_kg_m * cosf(pitch_rad),
              COM_FF_MAX_LEG_TORQUE_NM) : 0.0f;
    state->applied_per_leg_nm += clamp(state->target_per_leg_nm - state->applied_per_leg_nm,
                                      COM_FF_SLEW_NM_S * dt_s);
}
