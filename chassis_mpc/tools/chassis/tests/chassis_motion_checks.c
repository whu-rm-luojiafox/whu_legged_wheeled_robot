#include "chassis_motion.h"
#include "chassis_balance.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.0001f)

static void odometry_distance(void)
{
    chassis_odometry_t state = {0};
    /* Known 1 second journeys, including the old 1.6 m/s freeze boundary. */
    for (int i = 0; i < 250; ++i) chassis_odometry_step(&state, 2, 2, .004f);
    NEAR(state.distance_m, 2);
    CHECK(state.sample_valid);
    for (int i = 0; i < 500; ++i) chassis_odometry_step(&state, -1, -1, .002f);
    NEAR(state.distance_m, 1);
    for (int i = 0; i < 100; ++i) {
        chassis_odometry_step(&state, 1, 1, .003f);
        chassis_odometry_step(&state, 1, 1, .007f);
    }
    NEAR(state.distance_m, 2);
    chassis_odometry_step(&state, -2, 2, .004f); /* pure rotation */
    NEAR(state.distance_m, 2);
    chassis_odometry_step(&state, 2, 2, 1); /* debugger pause */
    CHECK(!state.sample_valid);
    NEAR(state.distance_m, 2);
    chassis_odometry_step(&state, NAN, 2, .004f);
    CHECK(!state.sample_valid);
    NEAR(state.distance_m, 2);
}

static void stop_and_reenable(void)
{
    chassis_position_t state = {0};
    chassis_position_input_t in = {1, .01f, 1, 1, 0};
    chassis_position_step(&state, &in);
    CHECK(!state.locked);
    in.command_speed_m_s = 0;
    /* Coast from 0 to 0.2m after releasing the stick: do not pull back to 0. */
    for (int i = 0; i < 20; ++i) {
        in.distance_m += .01f;
        chassis_position_step(&state, &in);
        CHECK(!state.locked);
        NEAR(state.reference_m, in.distance_m);
    }
    in.measured_speed_m_s = 0;
    for (int i = 0; i < 16; ++i) chassis_position_step(&state, &in);
    CHECK(state.locked);
    const float held_m = state.reference_m;
    in.distance_m += .1f;
    chassis_position_step(&state, &in);
    NEAR(state.reference_m, held_m); /* actual disturbance should be corrected */

    /* Disabled / airborne / special action: all release the old target. */
    in.allowed = 0;
    chassis_position_step(&state, &in);
    CHECK(!state.locked);
    in.distance_m = 4;
    chassis_position_step(&state, &in);
    in.allowed = 1;
    for (int i = 0; i < 16; ++i) chassis_position_step(&state, &in);
    CHECK(state.locked);
    NEAR(state.reference_m, 4);
    in.command_speed_m_s = -.01f;
    chassis_position_step(&state, &in);
    CHECK(state.locked);
    in.command_speed_m_s = -.03f;
    chassis_position_step(&state, &in);
    CHECK(!state.locked);
    NEAR(chassis_position_speed_request(-.01f), 0);
    NEAR(chassis_position_speed_request(NAN), 0);
}

static void persistent_drift_and_invalid_data(void)
{
    chassis_position_t state = {0};
    chassis_position_input_t in = {1, .01f, 0, -.2f, 0};
    for (int i = 0; i < 52; ++i) {
        in.distance_m -= .002f;
        chassis_position_step(&state, &in);
    }
    CHECK(state.locked); /* braking timeout must not allow indefinite drift */
    CHECK(state.reference_m < 0);
    in.dt_s = 1;
    chassis_position_step(&state, &in);
    CHECK(!state.locked);
    NEAR(state.reference_m, in.distance_m);
    in.dt_s = .01f;
    in.command_speed_m_s = NAN;
    chassis_position_step(&state, &in);
    CHECK(!state.locked);
}

static void feedforward_gates_and_slew(void)
{
    chassis_feedforward_t state = {0};
    for (int i = 0; i < 100; ++i)
        chassis_feedforward_step(&state, 1, 0, .02f, 0, .004f);
    NEAR(state.applied_per_leg_nm, 0); /* opt-in, even with a nonzero moment */
    chassis_feedforward_step(&state, 1, 1, .02f, 0, .004f);
    NEAR(state.applied_per_leg_nm, .002f);
    for (int i = 0; i < 100; ++i)
        chassis_feedforward_step(&state, 1, 1, .02f, 0, .004f);
    NEAR(state.applied_per_leg_nm, .0981f);
    for (int i = 0; i < 300; ++i) {
        const float before = state.applied_per_leg_nm;
        chassis_feedforward_step(&state, 1, 1, -.1f, .2f, .004f);
        CHECK(fabsf(state.applied_per_leg_nm - before) <= .00201f);
        CHECK(fabsf(state.applied_per_leg_nm) <= COM_FF_MAX_LEG_TORQUE_NM);
    }
    CHECK(state.applied_per_leg_nm < 0);
    chassis_feedforward_step(&state, 0, 1, -.1f, .2f, .004f);
    NEAR(state.applied_per_leg_nm, 0); /* disabled/airborne clears immediately */
    chassis_feedforward_step(&state, 1, 1, NAN, 0, .004f);
    NEAR(state.applied_per_leg_nm, 0);
    chassis_feedforward_step(&state, 1, 1, .02f, .36f, .004f);
    NEAR(state.applied_per_leg_nm, 0);
    chassis_feedforward_step(&state, 1, 1, .02f, 0, 1);
    NEAR(state.applied_per_leg_nm, 0);
}

static void balance_reference_coordinates(void)
{
    chassis_balance_reference_t state = {0};
    chassis_balance_reference_step(&state, 0, BALANCE_PITCH_DEFAULT_RAD,
                                    BALANCE_LEG_GROUND_DEFAULT_RAD, 0);
    NEAR(state.pitch_rad, 0);
    NEAR(state.leg_ground_rad, .1072f);
    NEAR(state.leg_local_rad, .1072f);
    /* Preserve legacy effective references when explicitly requested. */
    chassis_balance_reference_step(&state, 0, .1072f, .1172f, 0);
    NEAR(state.pitch_rad, .1072f);
    NEAR(state.leg_ground_rad, .1172f);
    CHECK(state.request_valid);

    /* Leveling the body must not move the independent ground-leg reference. */
    for (int i = 0; i < 260; ++i) {
        const float before = state.pitch_rad;
        chassis_balance_reference_step(&state, 1, .0972f, .1172f, .004f);
        CHECK(fabsf(state.pitch_rad - before) <= .0000401f);
        NEAR(state.leg_ground_rad, .1172f);
    }
    NEAR(state.pitch_rad, .0972f);
    NEAR(state.leg_local_rad, .02f);
    for (int i = 0; i < 2600; ++i)
        chassis_balance_reference_step(&state, 1, 0, .1172f, .004f);
    NEAR(state.pitch_rad, 0);
    NEAR(state.leg_ground_rad, .1172f);
    NEAR(state.leg_local_rad, .1172f);

    /* Invalid requests / debugger pauses cannot move the effective reference. */
    chassis_balance_reference_step(&state, 1, .1f, .02f, 1);
    CHECK(!state.request_valid);
    NEAR(state.pitch_rad, 0);
    chassis_balance_reference_step(&state, 0, NAN, .01f, .004f);
    CHECK(!state.request_valid);
    NEAR(state.leg_ground_rad, .1172f);
    chassis_balance_reference_step(&state, 0, .16f, .01f, .004f);
    CHECK(!state.request_valid);
    NEAR(state.pitch_rad, 0);
    chassis_balance_reference_step(&state, 0, .05f, -.02f, 0);
    CHECK(state.request_valid);
    NEAR(state.pitch_rad, .05f);
    NEAR(state.leg_ground_rad, -.02f);
    NEAR(state.leg_local_rad, -.07f);
    chassis_balance_reference_step(&state, 0, .05f, .31f, 0);
    CHECK(!state.request_valid);
    NEAR(state.leg_ground_rad, -.02f);
    chassis_balance_reference_step(&state, 0, -.15f, .15f, 0);
    CHECK(!state.request_valid);
    NEAR(state.pitch_rad, .05f);
}

static void lqr_matrix_regression(void)
{
    float gain[4][10], error[10];
    chassis_lqr_feedback_t output;
    /* Sweep independent leg/pitch and motion errors, including non-symmetric poses. */
    for (int case_id = 0; case_id < 200; ++case_id) {
        for (int col = 0; col < 10; ++col) {
            error[col] = sinf((float)(case_id * 7 + col));
            for (int row = 0; row < 4; ++row)
                gain[row][col] = 3.0f * cosf((float)(case_id + row * 10 + col));
        }
        chassis_lqr_feedback_calculate(gain, error, 0, &output);
        for (int row = 0; row < 4; ++row) {
            const int own = row % 2 == 0 ? 4 : 6;
            const float legacy = gain[row][own] * error[own] +
                gain[row][own + 1] * error[own + 1] + gain[row][8] * error[8] + gain[row][9] * error[9];
            const float moving = gain[row][0] * error[0] + gain[row][1] * error[1] +
                gain[row][2] * error[2] + gain[row][3] * error[3];
            NEAR(output.balance[row], legacy);
            NEAR(output.motion[row], moving);
        }
        chassis_lqr_feedback_calculate(gain, error, 1, &output);
        for (int row = 0; row < 4; ++row) {
            float full = 0;
            for (int col = 0; col < 10; ++col) full += gain[row][col] * error[col];
            NEAR(output.balance[row] + output.motion[row], full);
        }
        chassis_lqr_feedback_calculate(gain, error, .1f, &output);
        const float blended = output.balance[2];
        const float cross = output.cross_leg[2];
        chassis_lqr_feedback_calculate(gain, error, NAN, &output);
        NEAR(blended - output.balance[2], .1f * cross);
    }
}

int main(void)
{
    odometry_distance();
    stop_and_reenable();
    persistent_drift_and_invalid_data();
    feedforward_gates_and_slew();
    balance_reference_coordinates();
    lqr_matrix_regression();
    puts("motion checks passed: distance, stop/hold, re-enable, drift, feedforward");
    return 0;
}
