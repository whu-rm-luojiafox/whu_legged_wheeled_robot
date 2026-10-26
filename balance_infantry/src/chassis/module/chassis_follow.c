#include "chassis_follow.h"
#include <math.h>

#define DEG_TO_RAD 0.01745329252f
#define FOLLOW_MAX_STEP_S 0.05f
#define FOLLOW_MANUAL_STOP_RATE_RAD_S 0.02f

static float clamp(float value, float lower, float upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

static float wrap_deg(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg > 180.0f) angle_deg -= 360.0f;
    if (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* 失能、数据异常或调试暂停后，从当前航向重新开始，避免恢复时突然转向。 */
static void reset_to_heading(chassis_follow_t *state, float yaw_rad)
{
    state->initialized = 1u;
    state->auto_following = 0u;
    state->manual_turning = 0u;
    state->manual_stopping = 0u;
    state->heading_locked = 1u;
    state->error_deg = 0.0f;
    state->angle_ref_rad = yaw_rad;
    state->rate_ref_rad_s = 0.0f;
}

static uint8_t input_valid(const chassis_follow_input_t *input)
{
    /* 只排除不可计算的输入和错误符号，不限制试验时可选的 P、滤波等上限。
     * 真实指令约束在后面的转速、加速度和角度超前限幅中执行。 */
    return isfinite(input->dt_s) && input->dt_s > 0.0f &&
        input->dt_s <= FOLLOW_MAX_STEP_S &&
        isfinite(input->relative_deg) && isfinite(input->zero_deg) &&
        isfinite(input->move_deg) && isfinite(input->speed_m_s) &&
        isfinite(input->manual_rate_rad_s) &&
        isfinite(input->kp_rad_s_per_deg) &&
        input->kp_rad_s_per_deg >= 0.0f &&
        isfinite(input->rate_limit_rad_s) &&
        input->rate_limit_rad_s >= 0.0f &&
        isfinite(input->accel_rad_s2) &&
        input->accel_rad_s2 > 0.0f &&
        isfinite(input->filter_s) && input->filter_s >= 0.0f &&
        isfinite(input->yaw_lead_s) && input->yaw_lead_s >= 0.0f;
}

/* 手动转向优先；进入和退出使用不同阈值，避免摇杆边缘反复切换。 */
static void update_manual_mode(chassis_follow_t *state, float manual_rate_rad_s)
{
    if (state->manual_turning) {
        if (fabsf(manual_rate_rad_s) <= FOLLOW_MANUAL_EXIT_RAD_S)
            state->manual_turning = 0u;
    } else if (fabsf(manual_rate_rad_s) >= FOLLOW_MANUAL_ENTER_RAD_S) {
        state->manual_turning = 1u;
    }
}

/* 自动跟随只在平移时启动。速度和角差均有滞回，原地小角差保持航向。 */
static float automatic_rate_target(chassis_follow_t *state,
                                   const chassis_follow_input_t *input,
                                   float raw_error_deg)
{
    const uint8_t was_active = state->auto_following;
    if (state->auto_following) {
        if (fabsf(input->speed_m_s) <= FOLLOW_SPEED_EXIT_M_S ||
            fabsf(raw_error_deg) <= FOLLOW_ANGLE_EXIT_DEG)
            state->auto_following = 0u;
    } else if (fabsf(input->speed_m_s) >= FOLLOW_SPEED_ENTER_M_S &&
               fabsf(raw_error_deg) >= FOLLOW_ANGLE_ENTER_DEG) {
        state->auto_following = 1u;
    }

    if (!was_active || !state->auto_following) {
        state->error_deg = raw_error_deg;
    } else {
        /* 沿最短方向滤波，跨越 +/-180 度时不绕远路。 */
        const float alpha = input->dt_s / (input->filter_s + input->dt_s);
        state->error_deg = wrap_deg(state->error_deg +
            alpha * wrap_deg(raw_error_deg - state->error_deg));
    }

    if (!state->auto_following) return 0.0f;

    /* P 的单位为 (rad/s)/deg，负号沿用原跟随方向约定。
     * sqrt(2*a*角差) 是指令侧的制动限速，不代表已测得真实停车距离。 */
    const float braking_rate_rad_s = sqrtf(2.0f * input->accel_rad_s2 *
                                          fabsf(raw_error_deg) * DEG_TO_RAD);
    const float rate_limit_rad_s = fminf(input->rate_limit_rad_s, braking_rate_rad_s);
    return clamp(-input->kp_rad_s_per_deg * state->error_deg,
                 -rate_limit_rad_s, rate_limit_rad_s);
}

/* 手动松杆后先衰减到零；完成前不让自动跟随接管。 */
static void brake_manual_turn(chassis_follow_t *state, float dt_s, float raw_error_deg)
{
    state->manual_stopping = 1u;
    state->auto_following = 0u;
    state->error_deg = raw_error_deg;
    state->rate_ref_rad_s *= expf(-dt_s / FOLLOW_MANUAL_STOP_TIME_S);
    if (fabsf(state->rate_ref_rad_s) < FOLLOW_MANUAL_STOP_RATE_RAD_S) {
        state->rate_ref_rad_s = 0.0f;
        state->manual_stopping = 0u;
    }
}

static void update_heading_reference(chassis_follow_t *state,
                                     const chassis_follow_input_t *input)
{
    if (state->manual_turning || state->auto_following || state->rate_ref_rad_s != 0.0f) {
        state->angle_ref_rad = input->yaw_rad;
        if (!state->manual_turning && !state->manual_stopping) {
            /* 自动转向：实际 yaw + 有界超前，使 LQR 的角度反馈也参与转向。
             * 每拍从实际 yaw 计算，不累加目标角，避免积攒无法消除的角差。 */
            state->angle_ref_rad += clamp(input->yaw_lead_s * state->rate_ref_rad_s,
                                         -FOLLOW_MAX_ANGLE_LEAD_RAD,
                                         FOLLOW_MAX_ANGLE_LEAD_RAD);
        }
        state->heading_locked = 0u;
    } else if (!state->heading_locked) {
        /* 指令转速归零时只锁定一次航向，之后由 LQR 保持此角度。 */
        state->angle_ref_rad = input->yaw_rad;
        state->heading_locked = 1u;
    }
}

void chassis_follow_step(chassis_follow_t *state, const chassis_follow_input_t *input)
{
    if (!state || !input) return;

    /* 1. 检查输入；首拍仅建立基准，下一拍才处理跟随。 */
    if (!isfinite(input->yaw_rad)) {
        reset_to_heading(state, 0.0f);
        state->initialized = 0u;
        return;
    }
    if (!state->initialized || !input->allowed || !input_valid(input)) {
        reset_to_heading(state, input->yaw_rad);
        return;
    }

    /* 2. 选择控制来源：手动转向 > 手动松杆制动 > 移动自动跟随。 */
    const float raw_error_deg = wrap_deg(input->zero_deg - input->move_deg - input->relative_deg);
    const uint8_t was_manual = state->manual_turning;
    float desired_rate_rad_s = 0.0f;
    update_manual_mode(state, input->manual_rate_rad_s);

    if (state->manual_turning) {
        state->auto_following = 0u;
        state->manual_stopping = 0u;
        state->error_deg = raw_error_deg;
        desired_rate_rad_s = clamp(input->manual_rate_rad_s,
                                  -FOLLOW_MANUAL_MAX_RAD_S, FOLLOW_MANUAL_MAX_RAD_S);
    } else if (was_manual || state->manual_stopping) {
        brake_manual_turn(state, input->dt_s, raw_error_deg);
    } else {
        desired_rate_rad_s = automatic_rate_target(state, input, raw_error_deg);
    }

    /* 3. 限制自动转速变化，再生成交给 LQR 的角速度和航向参考。 */
    if (state->manual_turning) {
        state->rate_ref_rad_s = desired_rate_rad_s;
    } else if (!state->manual_stopping) {
        const float max_step_rad_s = input->accel_rad_s2 * input->dt_s;
        state->rate_ref_rad_s += clamp(desired_rate_rad_s - state->rate_ref_rad_s,
                                      -max_step_rad_s, max_step_rad_s);
    }
    update_heading_reference(state, input);
}
