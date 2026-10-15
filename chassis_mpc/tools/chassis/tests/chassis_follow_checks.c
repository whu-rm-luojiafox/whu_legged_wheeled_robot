#include "chassis_follow.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.0001f)

static chassis_follow_input_t input(void)
{
    chassis_follow_input_t in = {0};
    in.allowed = 1;
    in.dt_s = 0.004f;
    in.speed_m_s = 1.0f;
    in.relative_deg = 110.0f;
    in.zero_deg = 100.0f;
    in.kp_rad_s_per_deg = 0.2f;
    in.rate_limit_rad_s = FOLLOW_MAX_RATE_RAD_S;
    in.accel_rad_s2 = FOLLOW_RATE_ACCEL_RAD_S2;
    in.filter_s = FOLLOW_ERROR_FILTER_S;
    in.yaw_lead_s = FOLLOW_YAW_LEAD_S;
    return in;
}

static void switching(void)
{
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    chassis_follow_step(&s, &in);
    in.speed_m_s = 0.20f;
    chassis_follow_step(&s, &in);
    CHECK(!s.auto_following);
    in.speed_m_s = 0.26f;
    chassis_follow_step(&s, &in);
    CHECK(s.auto_following);
    CHECK(s.rate_ref_rad_s > 0); /* Same proportional polarity as old controller. */
    in.speed_m_s = 0.20f;
    in.relative_deg = 100.6f;
    chassis_follow_step(&s, &in);
    CHECK(s.auto_following); /* Both values lie inside their hysteresis bands. */
    in.relative_deg = 100.3f;
    chassis_follow_step(&s, &in);
    CHECK(!s.auto_following);
    in.relative_deg = 100.6f;
    chassis_follow_step(&s, &in);
    CHECK(!s.auto_following);
    in.relative_deg = 110;
    in.speed_m_s = -1;
    chassis_follow_step(&s, &in);
    CHECK(s.auto_following); /* Reverse translation must also follow. */
    in.speed_m_s = -0.14f;
    chassis_follow_step(&s, &in);
    CHECK(!s.auto_following);
}

static void wrap_and_direction(void)
{
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    in.zero_deg = -179;
    in.relative_deg = 179;
    chassis_follow_step(&s, &in);
    chassis_follow_step(&s, &in);
    NEAR(s.error_deg, 2);
    CHECK(s.rate_ref_rad_s < 0);
    s.initialized = 0;
    in.zero_deg = in.relative_deg = 100;
    in.move_deg = 30;
    chassis_follow_step(&s, &in);
    chassis_follow_step(&s, &in);
    CHECK(s.auto_following); /* Compare against travel direction, not encoder zero alone. */
    NEAR(s.error_deg, -30);
}

static void manual_stop_and_hold(void)
{
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    in.manual_rate_rad_s = -1;
    chassis_follow_step(&s, &in);
    for (int i = 0; i < 100; ++i) chassis_follow_step(&s, &in);
    CHECK(s.manual_turning && !s.auto_following && s.rate_ref_rad_s < 0);
    in.manual_rate_rad_s = -0.45f;
    chassis_follow_step(&s, &in);
    CHECK(s.manual_turning);
    in.manual_rate_rad_s = 0;
    chassis_follow_step(&s, &in);
    CHECK(s.manual_stopping && !s.auto_following);
    CHECK(s.rate_ref_rad_s < 0 && s.rate_ref_rad_s > -0.45f);
    in.speed_m_s = 0;
    in.yaw_rad = 2;
    for (int i = 0; i < 150; ++i) chassis_follow_step(&s, &in);
    CHECK(!s.manual_turning && !s.auto_following && s.heading_locked);
    NEAR(s.rate_ref_rad_s, 0);
    NEAR(s.angle_ref_rad, 2);
    in.yaw_rad = 2.1f;
    chassis_follow_step(&s, &in);
    NEAR(s.angle_ref_rad, 2); /* Heading is held after braking, not re-captured. */
    in.allowed = 0;
    chassis_follow_step(&s, &in);
    NEAR(s.angle_ref_rad, 2.1f);
    NEAR(s.rate_ref_rad_s, 0);
    CHECK(!s.auto_following && !s.manual_turning);
}

static float after_100ms(float dt, int count)
{
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    in.dt_s = dt;
    in.relative_deg = 140;
    chassis_follow_step(&s, &in);
    for (int i = 0; i < count; ++i) chassis_follow_step(&s, &in);
    return s.rate_ref_rad_s;
}

static void timing_and_bounds(void)
{
    NEAR(after_100ms(0.002f, 50), after_100ms(0.004f, 25));
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    in.relative_deg = 170;
    chassis_follow_step(&s, &in);
    for (int i = 0; i < 500; ++i) {
        const float before = s.rate_ref_rad_s;
        chassis_follow_step(&s, &in);
        CHECK(fabsf(s.rate_ref_rad_s) <= FOLLOW_MAX_RATE_RAD_S + 0.0001f);
        CHECK(fabsf(s.rate_ref_rad_s-before) <=
              FOLLOW_RATE_ACCEL_RAD_S2*in.dt_s + 0.0001f);
    }
    in.dt_s = 1; /* A long debugger pause must not create a large turn step. */
    chassis_follow_step(&s, &in);
    NEAR(s.rate_ref_rad_s, 0);
    in.dt_s = 0.004f;
    in.relative_deg = NAN;
    chassis_follow_step(&s, &in);
    CHECK(isfinite(s.rate_ref_rad_s));
    NEAR(s.rate_ref_rad_s, 0);
}

static void ideal_yaw_tracking(void)
{
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    for (int i = 0; i < 2000; ++i) {
        in.relative_deg = 110.0f - in.yaw_rad * 57.29577951f;
        chassis_follow_step(&s, &in);
        in.yaw_rad += s.rate_ref_rad_s * in.dt_s;
    }
    CHECK(fabsf(110.0f - in.yaw_rad*57.29577951f - in.zero_deg) < 1.0f);
    CHECK(!s.auto_following && s.heading_locked);
}

static void preview_and_live_limits(void)
{
    chassis_follow_t s = {0};
    chassis_follow_input_t in = input();
    chassis_follow_step(&s, &in);
    chassis_follow_step(&s, &in);
    CHECK(s.angle_ref_rad > in.yaw_rad);
    NEAR(s.angle_ref_rad-in.yaw_rad, in.yaw_lead_s*s.rate_ref_rad_s);
    in.yaw_lead_s = 0;
    chassis_follow_step(&s, &in);
    NEAR(s.angle_ref_rad, in.yaw_rad); /* Online zero restores the previous behavior. */
    in.yaw_lead_s = 0.2f;
    in.relative_deg = 170;
    in.rate_limit_rad_s = 2.0f;
    in.accel_rad_s2 = 4.0f;
    for (int i=0; i<300; ++i) {
        chassis_follow_step(&s, &in);
        CHECK(fabsf(s.angle_ref_rad-in.yaw_rad) <= FOLLOW_MAX_ANGLE_LEAD_RAD+0.0001f);
        CHECK(fabsf(s.rate_ref_rad_s) <= 2.0001f);
    }
    CHECK(s.rate_ref_rad_s > FOLLOW_MAX_RATE_RAD_S);
    in.yaw_lead_s = NAN;
    chassis_follow_step(&s, &in);
    NEAR(s.rate_ref_rad_s, 0);
    NEAR(s.angle_ref_rad, in.yaw_rad);
}

static void calibrated_forward_heading(void)
{
    chassis_follow_t state = {0};
    chassis_follow_input_t in = input();
    in.zero_deg = CHASSIS_GIMBAL_FORWARD_DEG;
    in.relative_deg = CHASSIS_GIMBAL_FORWARD_DEG;
    chassis_follow_step(&state, &in);
    chassis_follow_step(&state, &in);
    CHECK(!state.auto_following);
    NEAR(state.error_deg, 0);
    NEAR(state.rate_ref_rad_s, 0);
    in.relative_deg += 10;
    chassis_follow_step(&state, &in);
    CHECK(state.auto_following && state.rate_ref_rad_s > 0);
}

static void tuning_without_trial_caps(void)
{
    chassis_follow_t state = {0};
    chassis_follow_input_t in = input();
    /* Valid tuning values used to reset the controller at arbitrary trial caps. */
    in.kp_rad_s_per_deg = 0.6f;
    in.rate_limit_rad_s = 4.0f;
    in.accel_rad_s2 = 0.25f;
    in.filter_s = 0.3f;
    in.yaw_lead_s = 0.3f;
    chassis_follow_step(&state, &in);
    for (int i = 0; i < 1000; ++i) {
        const float before = state.rate_ref_rad_s;
        chassis_follow_step(&state, &in);
        CHECK(state.auto_following && state.rate_ref_rad_s > 0);
        CHECK(fabsf(state.rate_ref_rad_s) <= in.rate_limit_rad_s);
        CHECK(fabsf(state.rate_ref_rad_s - before) <= in.accel_rad_s2 * in.dt_s + 0.00001f);
        CHECK(fabsf(state.angle_ref_rad - in.yaw_rad) <= FOLLOW_MAX_ANGLE_LEAD_RAD + 0.00001f);
    }
    in.accel_rad_s2 = 0;
    chassis_follow_step(&state, &in);
    NEAR(state.rate_ref_rad_s, 0);
    in.accel_rad_s2 = .25f;
    in.kp_rad_s_per_deg = -1;
    chassis_follow_step(&state, &in);
    NEAR(state.rate_ref_rad_s, 0);
}

int main(void)
{
    switching();
    wrap_and_direction();
    manual_stop_and_hold();
    timing_and_bounds();
    ideal_yaw_tracking();
    preview_and_live_limits();
    calibrated_forward_heading();
    tuning_without_trial_caps();
    puts("follow checks passed: switching, wrap, manual_turning priority, hold, timing, bounds, ideal yaw tracking");
    return 0;
}
