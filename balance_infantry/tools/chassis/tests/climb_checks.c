#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "chassis_types.h"
#include "chassis_motion.h"

static uint32_t test_tick;
uint32_t xTaskGetTickCount(void)
{
    return test_tick;
}
chassis_lqr_feedback_t chassis_lqr_feedback;
chassis_feedforward_t chassis_feedforward;

/* Include the production state machine so tests can seed individual stages
 * without adding any test-only API to the firmware. */
#include "climb_under_test.c"
#include "combine_under_test.c"

static chassis_move_t chassis;
static chassis_data_t command;
static motor_measure_t wheel[2];
static float normal_target_left, normal_target_right;

static void reset_chassis(void)
{
    memset(&chassis, 0, sizeof(chassis));
    memset(&command, 0, sizeof(command));
    memset(wheel, 0, sizeof(wheel));
    memset(&climbContext, 0, sizeof(climbContext));
    climbContext.lastCommand = true;
    climbContext.normalLegLength = .25f;
    chassis.chassis_data_ = &command;
    chassis.foot_motor_L.motor_measure = &wheel[0];
    chassis.foot_motor_R.motor_measure = &wheel[1];
    chassis.mode.chassis_mode = ENABLE_CHASSIS;
    chassis.mode.chassis_balancing_mode = BALANCING_READY;
    chassis.mode.sport_mode = NORMAL_MOVING_MODE;
    chassis.chassis_posture_info.ideal_high = .25f;
    normal_target_left = normal_target_right = .25f;
    chassis.chassis_posture_info.chassis_posture_L.leg_length = .19f;
    chassis.chassis_posture_info.chassis_posture_R.leg_length = .19f;
    test_tick = 1;
}

static void step(uint32_t tick)
{
    test_tick = tick;
    /* Ordinary Target_Value_Set runs before the climb module every cycle. */
    chassis.chassis_posture_info.chassis_posture_L.leg_length_set = normal_target_left;
    chassis.chassis_posture_info.chassis_posture_R.leg_length_set = normal_target_right;
    chassisClimbProcess(&chassis);
}

static void test_pitch_hit(void)
{
    const float speeds[] = {0.f, -.5f, .5f, .5f, .5f};
    const float pitches[] = {.25f, .25f, -.25f, CLIMB_HIT_PITCH_RAD - .01f, CLIMB_HIT_PITCH_RAD};
    for (unsigned i = 0; i < sizeof(speeds) / sizeof(speeds[0]); ++i)
    {
        reset_chassis();
        climbEnterState(&climbContext, CLIMB_APPROACH, 100);
        chassis.chassis_posture_info.foot_speed_set = speeds[i];
        chassis.chassis_posture_info.pitch_angle = pitches[i];
        chassis.chassis_posture_info.chassis_posture_L.leg_angle = .8f;
        chassis.chassis_posture_info.chassis_posture_R.leg_angle = -.8f;
        step(101);
        step(200);
        assert(!hitRiser && chassisClimbGetState() == CLIMB_APPROACH);
    }
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_APPROACH, 100);
    chassis.chassis_posture_info.foot_speed_set = .5f;
    chassis.chassis_posture_info.pitch_angle = CLIMB_HIT_PITCH_RAD + .01f;
    step(101);
    step(101 + CLIMB_HIT_CONFIRM_MS - 1);
    assert(hitRiser && chassisClimbGetState() == CLIMB_APPROACH);
    step(101 + CLIMB_HIT_CONFIRM_MS);
    assert(chassisClimbGetState() == CLIMB_PULL_UP && climbContext.riserDetected);
    assert(chassis.chassis_posture_info.foot_speed_set == 0.f);

    reset_chassis();
    climbEnterState(&climbContext, CLIMB_APPROACH, 100);
    chassis.chassis_posture_info.foot_speed_set = .5f;
    chassis.chassis_posture_info.pitch_angle = CLIMB_HIT_PITCH_RAD + .01f;
    step(101);
    chassis.chassis_posture_info.pitch_angle = .1f;
    step(103);
    chassis.chassis_posture_info.pitch_angle = CLIMB_HIT_PITCH_RAD + .01f;
    step(110);
    step(110 + CLIMB_HIT_CONFIRM_MS - 1);
    assert(chassisClimbGetState() == CLIMB_APPROACH); /* Angle drop restarts confirmation. */
    chassis.chassis_posture_info.foot_speed_set = 0;
    step(110 + CLIMB_HIT_CONFIRM_MS);
    chassis.chassis_posture_info.foot_speed_set = .5f;
    step(120 + CLIMB_HIT_CONFIRM_MS);
    step(120 + 2 * CLIMB_HIT_CONFIRM_MS - 1);
    assert(chassisClimbGetState() == CLIMB_APPROACH); /* Releasing forward also resets it. */
    step(120 + 2 * CLIMB_HIT_CONFIRM_MS);
    assert(chassisClimbGetState() == CLIMB_PULL_UP);

    reset_chassis();
    climbEnterState(&climbContext, CLIMB_APPROACH, 100);
    chassis.chassis_posture_info.foot_speed_set = .5f;
    chassis.chassis_posture_info.pitch_angle = CLIMB_HIT_PITCH_RAD + .01f;
    step(101);
    step(101 + CLIMB_HIT_CONFIRM_MS - 1);
    assert(chassisClimbGetState() == CLIMB_APPROACH);
    command.climb_flag = 1; /* Cancel wins over the pending contact event. */
    step(101 + CLIMB_HIT_CONFIRM_MS);
    assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
}

static void test_entry_and_extension(void)
{
    reset_chassis();
    command.climb_flag = 1;
    step(1);
    assert(chassisClimbGetState() == CLIMB_IDLE); /* Must see release first. */
    command.climb_flag = 0;
    step(2);
    command.climb_flag = 1;
    step(3);
    assert(chassisClimbGetState() == CLIMB_READY);
    command.climb_flag = 0; /* F remains an event, not hold-to-run. */
    chassis.flag_info.suspend_flag_L = chassis.flag_info.suspend_flag_R = OFF_GROUND;
    chassis.chassis_posture_info.pitch_angle = .25f;
    chassis.chassis_posture_info.roll_angle = .20f;
    chassis.foot_motor_L.speed = chassis.foot_motor_R.speed = .4f;
    step(4);
    step(24);
    assert(chassisClimbGetState() == CLIMB_EXTEND);
    assert(chassisClimbAllowsPositionHold());
    step(474);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set -
                 .5f * (.19f + CLIMB_HIGH_LENGTH_M)) < 1e-5f);
    chassis.chassis_posture_info.chassis_posture_L.leg_length = CLIMB_HIGH_LENGTH_M - .015f;
    chassis.chassis_posture_info.chassis_posture_R.leg_length = CLIMB_HIGH_LENGTH_M + .015f;
    step(923);
    assert(chassisClimbGetState() == CLIMB_EXTEND); /* Full 0.9 s required. */
    step(924);
    step(1024);
    assert(chassisClimbGetState() == CLIMB_APPROACH);
    assert(!chassisClimbAllowsPositionHold());
}

static void test_abort_and_timeouts(void)
{
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_EXTEND, 100);
    climbContext.motionStartLength = .19f;
    step(1600);
    assert(chassisClimbGetState() == CLIMB_EXTEND); /* Short legs cannot finish. */
    step(10099);
    assert(chassisClimbGetState() == CLIMB_EXTEND);
    step(10100);
    assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_READY, 100);
    chassis.chassis_posture_info.pitch_angle = .60f;
    step(101);
    assert(chassisClimbGetState() == CLIMB_ABORT);
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_EXTEND, 100);
    chassis.chassis_posture_info.chassis_posture_L.leg_length = NAN;
    step(101);
    assert(chassisClimbGetState() == CLIMB_ABORT);
}

static void seed_torque(void)
{
    chassis.torque_info.foot_balancing_torque_L = 1.f;
    chassis.torque_info.foot_balancing_torque_R = 2.f;
    chassis.torque_info.foot_moving_torque_L = 3.f;
    chassis.torque_info.foot_moving_torque_R = 4.f;
    chassis.torque_info.joint_balancing_torque_L = 5.f;
    chassis.torque_info.joint_balancing_torque_R = 6.f;
    chassis.torque_info.joint_moving_torque_L = 7.f;
    chassis.torque_info.joint_moving_torque_R = 8.f;
    chassis.torque_info.joint_roll_torque_L = 9.f;
    chassis.torque_info.joint_roll_torque_R = -9.f;
    chassis.torque_info.joint_stand_torque_L = 30.f;
    chassis.torque_info.joint_stand_torque_R = 35.f;
    chassis.leg_L_length_pid.out = -26.2f;
    chassis.leg_R_length_pid.out = -27.7f;
    chassis_feedforward.target_per_leg_nm = .3f;
    chassis_feedforward.applied_per_leg_nm = .3f;
    for (int row = 0; row < 4; ++row)
    {
        chassis_lqr_feedback.balance[row] = 1.f;
        chassis_lqr_feedback.motion[row] = 2.f;
    }
}

static void test_final_mapping_policy(void)
{
    const climb_state_e states[] = {CLIMB_IDLE, CLIMB_READY, CLIMB_EXTEND, CLIMB_APPROACH,
                                    CLIMB_PULL_UP, CLIMB_WHEEL_CONFIRM, CLIMB_STAND_UP,
                                    CLIMB_FINISHED, CLIMB_ABORT, CLIMB_RETURN_NORMAL};
    for (unsigned i = 0; i < sizeof(states) / sizeof(states[0]); ++i)
    {
        const climb_state_e state = states[i];
        reset_chassis();
        climbContext.state = (climb_state_e)state;
        /* Real FK/Jacobian and VMC; these are nonsingular bent-leg poses. */
        chassis_leg_forward_kinematics(&chassis.chassis_posture_info.chassis_posture_L, -2.70f, 0, 3.08f, 0);
        chassis_leg_forward_kinematics(&chassis.chassis_posture_info.chassis_posture_R, -3.12f, 0, 2.74f, 0);
        seed_torque();
        chassisControlMap(&chassis);
        if (state == CLIMB_PULL_UP || state == CLIMB_WHEEL_CONFIRM)
        {
            assert(chassis.foot_motor_L.torque_out == 0.f && chassis.foot_motor_R.torque_out == 0.f);
            assert(chassis.torque_info.joint_horizontal_torque_L == 0.f);
            assert(chassis.torque_info.joint_horizontal_torque_R == 0.f);
            assert(chassis.torque_info.joint_vertical_torque_L == -26.2f);
            assert(chassis.torque_info.joint_vertical_torque_R == -27.7f);
            assert(chassis.torque_info.joint_horizontal_torque_temp1_L == 0.f);
            assert(chassis.torque_info.joint_horizontal_torque_temp2_R == 0.f);
            assert(fabsf(chassis.joint_motor_1.torque_out) > .01f); /* Retraction survives. */
            assert(chassis_feedforward.applied_per_leg_nm == 0.f);
            assert(chassis_lqr_feedback.balance[0] == 0.f);
            assert(chassis_lqr_feedback.motion[2] == 0.f);
            seed_torque(); /* A later override/remap must also be suppressed. */
            chassisControlMap(&chassis);
            assert(chassis.torque_info.joint_horizontal_torque_L == 0.f);
            assert(chassis.torque_info.joint_horizontal_torque_R == 0.f);
            assert(chassis.torque_info.joint_vertical_torque_L == -26.2f);
            assert(chassis.torque_info.joint_vertical_torque_R == -27.7f);
            chassis.leg_L_length_pid.out = chassis.leg_R_length_pid.out = 0.f;
            chassis.torque_info.joint_stand_torque_L = 35.f;
            chassis.torque_info.joint_stand_torque_R = 21.f;
            chassisControlMap(&chassis);
            assert(chassis.torque_info.joint_vertical_torque_L == 0.f);
            assert(chassis.torque_info.joint_vertical_torque_R == 0.f);
            assert(chassis.joint_motor_1.torque_out == 0.f);
            assert(chassis.joint_motor_2.torque_out == 0.f);
            assert(chassis.joint_motor_3.torque_out == 0.f);
            assert(chassis.joint_motor_4.torque_out == 0.f);
        }
        else
        {
            assert(chassis.foot_motor_L.torque_out == 4.f);
            assert(chassis.foot_motor_R.torque_out == 6.f);
            assert(chassis.torque_info.joint_horizontal_torque_L == 12.f + FORWARD_TORQUE);
            assert(chassis.torque_info.joint_horizontal_torque_R == 14.f + FORWARD_TORQUE);
            assert(chassis.torque_info.joint_stand_torque_L == 30.f);
            assert(chassis.torque_info.joint_stand_torque_R == 35.f);
        }
    }
}

static void test_pull_handoff(void)
{
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_APPROACH, 100);
    chassis.chassis_posture_info.foot_speed_set = .5f;
    /* This tilted, moving posture failed the removed top confirmation. */
    chassis.chassis_posture_info.pitch_angle = CLIMB_HIT_PITCH_RAD + .01f;
    chassis.chassis_posture_info.roll_angle = .12f;
    chassis.chassis_posture_info.pitch_gyro = 1.2f;
    chassis.chassis_posture_info.chassis_posture_L.leg_length = .40f;
    chassis.chassis_posture_info.chassis_posture_R.leg_length = .42f;
    step(101);
    step(101 + CLIMB_HIT_CONFIRM_MS);
    assert(chassisClimbGetState() == CLIMB_PULL_UP);
    assert(climbContext.riserDetected);
    assert(chassis.chassis_posture_info.foot_speed_set == 0.f);
    assert(fabsf(climbContext.motionStartLength - .41f) < 1e-6f);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set - .41f) < 1e-6f);
    seed_torque();
    assert(chassisClimbLengthOnly()); /* Same handoff cycle. */
    step(101 + CLIMB_HIT_CONFIRM_MS + CLIMB_PULL_TIME_MS / 2);
    assert(chassisClimbGetState() == CLIMB_PULL_UP);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set -
                 .5f * (.41f + CLIMB_PULL_LENGTH_M)) < 1e-6f);
    /* Both legs reach the short target while the body is still rotating.
     * Do not wait for gyro settling; finish the pull trajectory and stand. */
    chassis.chassis_posture_info.chassis_posture_L.leg_length = .16f;
    chassis.chassis_posture_info.chassis_posture_R.leg_length = .17f;
    chassis.flag_info.suspend_flag_L = chassis.flag_info.suspend_flag_R = OFF_GROUND;
    const uint32_t pullEnd = 101 + CLIMB_HIT_CONFIRM_MS + CLIMB_PULL_TIME_MS;
    step(pullEnd - CLIMB_PULL_STABLE_MS);
    step(pullEnd - 1);
    assert(chassisClimbGetState() == CLIMB_PULL_UP);
    step(pullEnd);
    assert(chassisClimbGetState() == CLIMB_STAND_UP);
    assert(fabsf(climbContext.motionStartLength - CLIMB_PULL_LENGTH_M) < 1e-6f);
    float previousTarget = chassis.chassis_posture_info.chassis_posture_L.leg_length_set;
    step(pullEnd + 4);
    float nextTarget = chassis.chassis_posture_info.chassis_posture_L.leg_length_set;
    assert(nextTarget > previousTarget && nextTarget - previousTarget < .001f);
    seed_torque();
    assert(!chassisClimbLengthOnly());
    assert(chassis.torque_info.joint_balancing_torque_L == 5.f);
}

static void test_manual_control_and_cancel(void)
{
    const climb_state_e stages[] = {CLIMB_READY, CLIMB_EXTEND, CLIMB_APPROACH};
    for (unsigned i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i)
    {
        reset_chassis();
        climbEnterState(&climbContext, stages[i], 100);
        climbContext.motionStartLength = .19f;
        chassis.chassis_posture_info.foot_speed_set = -10.f;
        step(101);
        float limit = CLIMB_APPROACH_SPEED_MAX;
        assert(fabsf(chassis.chassis_posture_info.foot_speed_set + limit) < 1e-6f);
        chassis.chassis_posture_info.foot_speed_set = 10.f;
        step(102);
        assert(fabsf(chassis.chassis_posture_info.foot_speed_set - limit) < 1e-6f);
        chassis.chassis_posture_info.foot_speed_set = 0.f;
        step(103);
        assert(chassis.chassis_posture_info.foot_speed_set == 0.f);
    }

    reset_chassis();
    chassis.chassis_posture_info.ideal_high = .19f;
    normal_target_left = normal_target_right = .19f;
    step(1); /* Release F. */
    command.climb_flag = 1;
    step(2);
    assert(fabsf(climbContext.normalLegLength - .19f) < 1e-6f); /* No 22 cm floor. */
    command.climb_flag = 0;
    step(3);
    step(23);
    step(1523);
    float last_target = chassis.chassis_posture_info.chassis_posture_L.leg_length_set;
    command.climb_flag = 1;
    step(1527);
    assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
    assert(chassis.chassis_posture_info.chassis_posture_L.leg_length_set == last_target);
    step(3027);
    assert(chassisClimbGetState() == CLIMB_IDLE);
    step(3028); /* A held cancel key cannot start another climb. */
    assert(chassisClimbGetState() == CLIMB_IDLE);

    /* Position hold is still present at rest, and genuine movement releases it. */
    climbContext.state = CLIMB_EXTEND;
    chassis_position_t position = {.locked = 1, .reference_m = 1.f};
    chassis_position_input_t input = {.allowed = chassisClimbAllowsPositionHold(),
                                      .dt_s = .004f,
                                      .command_speed_m_s = -.35f,
                                      .measured_speed_m_s = 0.f,
                                      .distance_m = .5f};
    chassis_position_step(&position, &input);
    assert(!position.locked);
    input.command_speed_m_s = 0.f;
    for (unsigned i = 0; i < 40; ++i)
        chassis_position_step(&position, &input);
    assert(position.locked);
}

static void test_soft_return_for_every_timeout(void)
{
    const climb_state_e stages[] = {CLIMB_READY, CLIMB_EXTEND, CLIMB_PULL_UP, CLIMB_STAND_UP};
    const uint32_t timeouts[] = {8000, 10000, 5000, 6000};
    for (unsigned i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i)
    {
        reset_chassis();
        climbEnterState(&climbContext, stages[i], 100);
        climbContext.motionStartLength = .19f;
        if (stages[i] == CLIMB_READY) chassis.mode.chassis_balancing_mode = NO_FORCE;
        step(99 + timeouts[i]);
        assert(chassisClimbGetState() == stages[i]);
        float previous = chassis.chassis_posture_info.chassis_posture_L.leg_length_set;
        step(100 + timeouts[i]);
        assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
        assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set - previous) < 1e-4f);
    }

    /* A high-leg timeout return must not jump to IDLE after 300 ms.
     * Also blend the two roll-adjusted ordinary targets, so final handoff
     * cannot introduce a left/right target step. */
    reset_chassis();
    normal_target_left = .18f;
    normal_target_right = .20f;
    climbEnterState(&climbContext, CLIMB_EXTEND, 100);
    climbContext.motionStartLength = .19f;
    step(10100);
    assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
    float previous_l = CLIMB_HIGH_LENGTH_M, previous_r = CLIMB_HIGH_LENGTH_M;
    /* Cubic easing peaks at 1.5 * travel / duration; account for the configured height. */
    const float max_step = 1.5f * fmaxf(CLIMB_HIGH_LENGTH_M - normal_target_left,
                                       CLIMB_HIGH_LENGTH_M - normal_target_right) *
                               4.f / CLIMB_RETURN_TIME_MS + 1e-6f;
    for (uint32_t dt = 4; dt <= 1500; dt += 4)
    {
        chassis.chassis_posture_info.foot_speed_set = -.1f;
        step(10100 + dt);
        float left = chassis.chassis_posture_info.chassis_posture_L.leg_length_set;
        float right = chassis.chassis_posture_info.chassis_posture_R.leg_length_set;
        assert(left <= previous_l && right <= previous_r);
        assert(fabsf(left - previous_l) <= max_step && fabsf(right - previous_r) <= max_step);
        assert(chassis.chassis_posture_info.foot_speed_set == -.1f);
        if (dt < 1500) assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
        previous_l = left;
        previous_r = right;
    }
    assert(chassisClimbGetState() == CLIMB_IDLE);
    assert(fabsf(previous_l - normal_target_left) < 1e-6f);
    assert(fabsf(previous_r - normal_target_right) < 1e-6f);
    step(11604);
    assert(chassis.chassis_posture_info.chassis_posture_L.leg_length_set == normal_target_left);
    assert(chassis.chassis_posture_info.chassis_posture_R.leg_length_set == normal_target_right);

    reset_chassis();
    climbEnterAbort(&climbContext, &chassis, 100);
    chassis.mode.chassis_mode = DISABLE_CHASSIS;
    step(101);
    step(501);
    assert(chassisClimbGetState() == CLIMB_ABORT);
    chassis.mode.chassis_mode = ENABLE_CHASSIS;
    step(502);
    step(802);
    assert(chassisClimbGetState() == CLIMB_RETURN_NORMAL);
}

static void test_pull_and_stand_timing(void)
{
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_PULL_UP, 100);
    climbContext.motionStartLength = CLIMB_HIGH_LENGTH_M;
    chassis.chassis_posture_info.chassis_posture_L.leg_length = CLIMB_HIGH_LENGTH_M;
    chassis.chassis_posture_info.chassis_posture_R.leg_length = CLIMB_HIGH_LENGTH_M;
    step(100);
    assert(chassis.chassis_posture_info.chassis_posture_L.leg_length_set == CLIMB_HIGH_LENGTH_M);
    step(100 + CLIMB_PULL_TIME_MS / 2);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set -
                 .5f * (CLIMB_HIGH_LENGTH_M + CLIMB_PULL_LENGTH_M)) < 1e-5f);
    step(550);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set - CLIMB_PULL_LENGTH_M) <
           1e-5f);
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_STAND_UP, 100);
    climbContext.motionStartLength = CLIMB_PULL_LENGTH_M;
    step(100);
    assert(chassis.chassis_posture_info.chassis_posture_L.leg_length_set == CLIMB_PULL_LENGTH_M);
    step(100 + CLIMB_STAND_TIME_MS / 2);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set -
                 .5f * (CLIMB_PULL_LENGTH_M + .25f)) < 1e-5f);
    step(600);
    assert(fabsf(chassis.chassis_posture_info.chassis_posture_L.leg_length_set - .25f) < 1e-5f);

    /* At the low ordinary height, the old 6 cm tolerance accepted a 14 cm leg
     * immediately and ended STAND_UP before its 500 ms trajectory finished. */
    reset_chassis();
    climbContext.normalLegLength = .19f;
    climbEnterState(&climbContext, CLIMB_STAND_UP, 100);
    climbContext.motionStartLength = CLIMB_PULL_LENGTH_M;
    step(101);
    step(301);
    assert(chassisClimbGetState() == CLIMB_STAND_UP);
    assert(chassis.chassis_posture_info.chassis_posture_L.leg_length_set < .19f);
    step(600);
    assert(chassisClimbGetState() == CLIMB_STAND_UP);
    step(600 + CLIMB_STAND_STABLE_MS);
    assert(chassisClimbGetState() == CLIMB_FINISHED);

    /* A short leg on only one side must not trigger the direct stand handoff. */
    reset_chassis();
    climbEnterState(&climbContext, CLIMB_PULL_UP, 100);
    climbContext.motionStartLength = CLIMB_HIGH_LENGTH_M;
    chassis.chassis_posture_info.chassis_posture_L.leg_length = .14f;
    chassis.chassis_posture_info.chassis_posture_R.leg_length = .25f;
    step(400);
    step(500);
    assert(chassisClimbGetState() == CLIMB_PULL_UP);
    chassis.chassis_posture_info.chassis_posture_R.leg_length = .16f;
    step(504);
    step(504 + CLIMB_PULL_STABLE_MS);
    assert(chassisClimbGetState() == CLIMB_STAND_UP);
}

int main(void)
{
    test_pitch_hit();
    test_entry_and_extension();
    test_abort_and_timeouts();
    test_final_mapping_policy();
    test_pull_handoff();
    test_manual_control_and_cancel();
    test_soft_return_for_every_timeout();
    test_pull_and_stand_timing();
    puts("PASS: forward positive-pitch hit detection, configured confirmation/reset, direct pull handoff, climb entry/extension, safety/long timeouts, final VMC "
         "policy, pull/contact handoff, manual control/F cancel, continuous soft return");
    return 0;
}
