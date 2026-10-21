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

void chassis_lqr_feedback_calculate(const float gain[4][10], const float error[10],
                                   float cross_leg_scale, chassis_lqr_feedback_t *output)
{
    if (!gain || !error || !output) return;
    if (!isfinite(cross_leg_scale) || cross_leg_scale < 0.0f || cross_leg_scale > 1.0f)
        cross_leg_scale = 0.0f;

    for (int row = 0; row < CHASSIS_LQR_INPUT_COUNT; ++row) {
        const int own_leg = (row == 0 || row == 2) ? 4 : 6;
        const int other_leg = own_leg == 4 ? 6 : 4;
        output->motion[row] = gain[row][0] * error[0] + gain[row][1] * error[1] +
                              gain[row][2] * error[2] + gain[row][3] * error[3];
        output->balance[row] = gain[row][own_leg] * error[own_leg] +
                               gain[row][own_leg + 1] * error[own_leg + 1] +
                               gain[row][8] * error[8] + gain[row][9] * error[9];
        output->cross_leg[row] = gain[row][other_leg] * error[other_leg] +
                                 gain[row][other_leg + 1] * error[other_leg + 1];
        if (cross_leg_scale != 0.0f)
            output->balance[row] += cross_leg_scale * output->cross_leg[row];
    }
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
