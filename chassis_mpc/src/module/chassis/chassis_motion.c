#include "chassis_motion.h"
#include <math.h>

void chassis_odometry_step(chassis_odometry_t *state, float left_m_s,
                           float right_m_s, float dt_s)
{
    if (!state) return;
    state->sample_valid = 0u;
    state->speed_m_s = 0.0f;
    if (!isfinite(left_m_s) || !isfinite(right_m_s) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f)
        return;

    const float speed_m_s = 0.5f * left_m_s + 0.5f * right_m_s;
    const float next_distance_m = state->distance_m + speed_m_s * dt_s;
    if (!isfinite(speed_m_s) || !isfinite(next_distance_m)) return;
    state->speed_m_s = speed_m_s;
    state->distance_m = next_distance_m;
    state->sample_valid = 1u;
}

float chassis_position_speed_request(float command_m_s)
{
    if (!isfinite(command_m_s) || fabsf(command_m_s) <= POSITION_COMMAND_DEADBAND_M_S)
        return 0.0f;
    return command_m_s;
}

static void release(chassis_position_t *state, float distance_m)
{
    state->locked = 0u;
    state->stopped_time_s = 0.0f;
    state->braking_time_s = 0.0f;
    state->reference_m = distance_m;
}

void chassis_position_step(chassis_position_t *state,
                           const chassis_position_input_t *input)
{
    if (!state || !input) return;
    if (!isfinite(input->distance_m)) {
        release(state, 0.0f);
        return;
    }
    if (!input->allowed || !isfinite(input->dt_s) || input->dt_s <= 0.0f ||
        input->dt_s > 0.05f || !isfinite(input->command_speed_m_s) ||
        !isfinite(input->measured_speed_m_s) ||
        chassis_position_speed_request(input->command_speed_m_s) != 0.0f) {
        release(state, input->distance_m);
        return;
    }
    if (state->locked) return; /* 捕获后固定目标，不能每拍重新跟随实际位置。 */

    /* 松杆先让零速反馈制动，位置参考随车走，避免立刻倒回松杆点。 */
    state->reference_m = input->distance_m;
    state->braking_time_s += input->dt_s;
    if (fabsf(input->measured_speed_m_s) <= POSITION_SETTLED_SPEED_M_S)
        state->stopped_time_s += input->dt_s;
    else
        state->stopped_time_s = 0.0f;

    /* 持续漂移时也不能永远放弃位置反馈：超时后捕获当时的位置。 */
    if (state->stopped_time_s >= POSITION_SETTLE_TIME_S ||
        state->braking_time_s >= POSITION_BRAKE_TIMEOUT_S)
        state->locked = 1u;
}
