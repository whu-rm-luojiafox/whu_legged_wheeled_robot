#include "chassis_types.h"
#include "chassis_observer.h"
#include "chassis_mode.h"
#include "chassis_motion.h"
#include "chassis_follow.h"
#include "chassis_control.h"
#include "chassis_climb.h"
#include "chassis_recovery.h"
#include "chassis_balance.h"
#include "kalman_service.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
extern uint32_t host_tick;
extern unsigned allocation_count, host_enable_count;
extern motor_measure_t host_wheels[2];
extern float host_accel[3];
extern chassis_feedforward_t chassis_feedforward;
extern chassis_follow_t chassis_follow;

static void near(float actual, float expected, float tolerance)
{
    if (fabsf(actual - expected) > tolerance)
    {
        printf("actual=%g expected=%g\n", actual, expected);
        fflush(stdout);
        assert(0);
    }
}

static void test_filter(void)
{
    assert(ChassisVelocityKF_Init(0));
    unsigned allocated = allocation_count;
    for (int i = 0; i < 1000; ++i)
        assert(ChassisVelocityKF_Update(0, 0, .004f, 1));
    near(ChassisVelocityKF_Get()->speed_m_s, 0, 1e-7f);
    for (int i = 0; i < 20000; ++i)
        assert(ChassisVelocityKF_Update(.2f, 0, .004f, 1));
    near(ChassisVelocityKF_Get()->speed_m_s, 0, .002f);
    near(ChassisVelocityKF_Get()->accel_bias_m_s2, .2f, .015f);
    float held = ChassisVelocityKF_Get()->speed_m_s;
    assert(!ChassisVelocityKF_Update(10, 0, .004f, 0));
    assert(!ChassisVelocityKF_Get()->valid);
    near(ChassisVelocityKF_Get()->speed_m_s, held, 1e-7f);
    assert(!ChassisVelocityKF_Update(NAN, 0, .004f, 1));
    assert(!ChassisVelocityKF_Update(0, 0, 0, 1));
    assert(!ChassisVelocityKF_Update(0, 0, .06f, 1));
    for (int i = 0; i < 20; ++i)
        assert(ChassisVelocityKF_Init(0));
    assert(allocation_count == allocated);
}

static void test_mode_targets(chassis_move_t *c, chassis_data_t *command)
{
    chassisModeInit(c);
    command->chassis_mode = 1;
    c->chassis_posture_info.chassis_posture_L.leg_length = .14f;
    c->chassis_posture_info.chassis_posture_R.leg_length = .14f;
    unsigned enabled = host_enable_count;
    chassisModeUpdate(c);
    assert(c->mode.chassis_balancing_mode == FOOT_LAUNCHING);
    chassisModeUpdate(c);
    assert(c->mode.chassis_balancing_mode == JOINT_LAUNCHING);
    chassisModeUpdate(c);
    assert(c->mode.chassis_balancing_mode == BALANCING_READY);
    chassisModeUpdate(c);
    assert(c->mode.sport_mode == NORMAL_MOVING_MODE);
    assert(host_enable_count == enabled + 4);
    command->jump_flag = 1;
    chassisModeUpdate(c);
    assert(c->mode.jumping_stage == READY_TO_JUMP);
    command->jump_flag = 0;
    chassisModeUpdate(c);
    assert(c->mode.jumping_stage == PREPARING_STAND_JUMPING);
    chassisModeSetTargets(c);
    near(c->chassis_posture_info.chassis_posture_L.leg_length_set, .13f, 1e-6f);
    near(c->chassis_posture_info.chassis_posture_R.leg_angle_set, .174532f, 1e-6f);
    chassisModeUpdate(c);
    assert(c->mode.jumping_stage == EXTENDING_LEGS);
    chassisModeSetTargets(c);
    near(c->chassis_posture_info.chassis_posture_L.leg_length_set, .34f, 1e-6f);
    c->mode.jumping_stage = CONSTACTING_LEGS_2;
    chassisModeSetTargets(c);
    near(c->chassis_posture_info.chassis_posture_R.leg_length_set, .10f, 1e-6f);
    c->mode.jumping_stage = PREPARING_LANDING;
    chassisModeSetTargets(c);
    near(c->chassis_posture_info.chassis_posture_L.leg_length_set, .15f, 1e-6f);
    command->chassis_mode = 0;
    chassisModeUpdate(c);
    assert(c->mode.chassis_balancing_mode == NO_FORCE);
}

static void test_observer(chassis_move_t *c)
{
    chassisObserverInit(c);
    c->torque_info.joint_vertical_torque_L = c->torque_info.joint_vertical_torque_R = 35;
    host_wheels[0].speed_rpm = -1000;
    host_wheels[1].speed_rpm = 1000;
    host_tick = 1000;
    host_wheels[0].last_update_time = host_wheels[1].last_update_time = host_tick;
    chassisObserverUpdate(c, .004f);
    assert(chassis_odometry.sample_valid && ChassisVelocityKF_Get()->valid);
    float speed = c->chassis_posture_info.foot_speed_KF;
    assert(speed > 0);
    near(chassis_odometry.distance_m, speed * .004f, 1e-6f);
    float distance = chassis_odometry.distance_m;
    host_accel[0] = NAN;
    chassisObserverUpdate(c, .004f);
    assert(chassis_odometry.sample_valid && !ChassisVelocityKF_Get()->valid);
    near(c->chassis_posture_info.foot_speed_KF, c->chassis_posture_info.foot_speed, 1e-6f);
    near(chassis_odometry.distance_m, distance + c->chassis_posture_info.foot_speed * .004f, 1e-6f);
    host_accel[0] = 0;
    host_tick += 100;
    distance = chassis_odometry.distance_m;
    chassisObserverUpdate(c, .004f);
    assert(!chassis_odometry.sample_valid);
    near(chassis_odometry.distance_m, distance, 1e-6f);
    host_wheels[0].last_update_time = host_wheels[1].last_update_time = host_tick;
    chassisObserverUpdate(c, .004f);
    if (!chassis_odometry.sample_valid)
    {
        printf("observer flags=%d,%d forces=%g,%g accel=%g,%g,%g kf=%d\n", c->flag_info.suspend_flag_L,
               c->flag_info.suspend_flag_R, c->torque_info.supportive_force_L,
               c->torque_info.supportive_force_R, c->chassis_posture_info.x_accel,
               c->chassis_posture_info.y_accel, c->chassis_posture_info.z_accel,
               ChassisVelocityKF_Get()->valid);
        fflush(stdout);
    }
    assert(chassis_odometry.sample_valid);
    near(chassis_odometry.distance_m, distance + c->chassis_posture_info.foot_speed_KF * .004f, 1e-6f);
    distance = chassis_odometry.distance_m;
    chassisObserverUpdate(c, .06f);
    assert(!chassis_odometry.sample_valid);
    near(chassis_odometry.distance_m, distance, 1e-6f);
}

static void test_motion_control(chassis_move_t *c, chassis_data_t *command)
{
    c->mode.chassis_mode = ENABLE_CHASSIS;
    c->mode.chassis_balancing_mode = BALANCING_READY;
    c->mode.sport_mode = NORMAL_MOVING_MODE;
    c->mode.jumping_stage = FINISHED;
    c->flag_info.suspend_flag_L = c->flag_info.suspend_flag_R = ON_GROUND;
    chassis_odometry.sample_valid = 1;
    chassis_odometry.speed_m_s = 0;
    chassis_odometry.distance_m = 0;
    command->vx_set = .5f;
    command->vy_set = 0;
    command->wz_set = 0;
    command->high_set = .25f;
    c->chassis_posture_info.roll_angle = .1f;
    chassisMotionUpdate(c, .004f);
    chassisModeSetTargets(c);
    near(c->chassis_posture_info.foot_speed_set, .5f, 1e-6f);
    near(c->chassis_posture_info.chassis_posture_L.leg_length_set, .25f - .24f * tanf(.1f), 1e-6f);
    near(c->chassis_posture_info.chassis_posture_R.leg_length_set, .25f + .24f * tanf(.1f), 1e-6f);
    c->chassis_posture_info.roll_angle = 0;
    for (int mask = 0; mask < 4; ++mask)
    {
        chassisControlInit(c);
        c->flag_info.suspend_flag_L = mask & 1 ? OFF_GROUND : ON_GROUND;
        c->flag_info.suspend_flag_R = mask & 2 ? OFF_GROUND : ON_GROUND;
        c->chassis_posture_info.chassis_posture_L.leg_length = .25f;
        c->chassis_posture_info.chassis_posture_R.leg_length = .25f;
        c->chassis_posture_info.chassis_posture_L.leg_length_set = .25f;
        c->chassis_posture_info.chassis_posture_R.leg_length_set = .25f;
        chassisControlCalculate(c, .004f);
        near(c->torque_info.joint_stand_torque_L, mask & 1 ? 21 : 35, 1e-5f);
        near(c->torque_info.joint_stand_torque_R, mask & 2 ? 21 : 35, 1e-5f);
        chassisControlMap(c);
        assert(isfinite(c->joint_motor_1.torque_out));
        assert(fabsf(c->joint_motor_1.torque_out) <= JOINT_TORQUE_LIMIT);
    }
    chassis_feedforward.applied_per_leg_nm = .2f;
    c->mode.chassis_balancing_mode = FOOT_LAUNCHING;
    chassisControlCalculate(c, .004f);
    near(chassis_feedforward.applied_per_leg_nm, 0, 1e-7f);
}

static void test_stop_position(void)
{
    chassis_position_t state = {0};
    chassis_position_input_t input = {.allowed = 1, .dt_s = .004f, .measured_speed_m_s = .3f};
    /* Coast longer than the former timeout: position feedback must remain off. */
    for (unsigned i = 0; i < 250; ++i)
    {
        input.distance_m += input.measured_speed_m_s * input.dt_s;
        chassis_position_step(&state, &input);
        assert(!state.locked);
        near(state.reference_m, input.distance_m, 1e-7f);
    }
    input.measured_speed_m_s = .03f;
    for (unsigned i = 0; i < 25; ++i)
        chassis_position_step(&state, &input);
    assert(!state.locked);
    input.measured_speed_m_s = -.2f;
    chassis_position_step(&state, &input);
    input.measured_speed_m_s = -.03f;
    for (unsigned i = 0; i < 25; ++i)
        chassis_position_step(&state, &input);
    assert(!state.locked); /* A speed excursion restarts the settling interval. */
    for (unsigned i = 0; i < 15; ++i)
        chassis_position_step(&state, &input);
    assert(state.locked);
    float captured = state.reference_m;
    input.distance_m += .1f;
    chassis_position_step(&state, &input);
    near(state.reference_m, captured, 1e-7f);
    input.command_speed_m_s = -.5f;
    chassis_position_step(&state, &input);
    assert(!state.locked);
    near(state.reference_m, input.distance_m, 1e-7f);
    input.command_speed_m_s = 0;
    input.measured_speed_m_s = NAN;
    chassis_position_step(&state, &input);
    assert(!state.locked && state.stopped_time_s == 0);
}

static void test_yaw_follow(void)
{
    chassis_follow_t state = {0};
    chassis_follow_input_t input = {.allowed = 1,
                                    .dt_s = .004f,
                                    .relative_deg = 20,
                                    .yaw_rad = 7,
                                    .kp_rad_s_per_deg = FOLLOW_KP_RAD_S_PER_DEG,
                                    .rate_limit_rad_s = FOLLOW_MAX_RATE_RAD_S,
                                    .accel_rad_s2 = FOLLOW_RATE_ACCEL_RAD_S2,
                                    .filter_s = FOLLOW_ERROR_FILTER_S};
    chassis_follow_step(&state, &input);
    assert(!state.auto_following && state.heading_locked);
    for (unsigned i = 0; i < 100; ++i)
    {
        float previous_rate = state.rate_ref_rad_s;
        input.yaw_rad += .001f;
        chassis_follow_step(&state, &input);
        assert(state.auto_following && !state.heading_locked);
        assert(state.rate_ref_rad_s > 0 && state.rate_ref_rad_s <= FOLLOW_MAX_RATE_RAD_S);
        assert(fabsf(state.rate_ref_rad_s - previous_rate) <= input.accel_rad_s2 * input.dt_s + 1e-6f);
        near(state.angle_ref_rad, input.yaw_rad, 1e-7f); /* No competing angle lead. */
    }
    input.manual_rate_rad_s = -8;
    chassis_follow_step(&state, &input);
    assert(state.manual_turning && !state.auto_following);
    near(state.rate_ref_rad_s, -8, 1e-6f);
    input.manual_rate_rad_s = 0;
    chassis_follow_step(&state, &input);
    assert(state.manual_stopping && !state.auto_following);
    assert(state.rate_ref_rad_s > -8 && state.rate_ref_rad_s < 0);
    for (unsigned i = 0; i < 200; ++i)
        chassis_follow_step(&state, &input);
    assert(state.auto_following && !state.manual_stopping && state.rate_ref_rad_s > 0);

    input.relative_deg = .3f;
    for (unsigned i = 0; i < 120; ++i)
        chassis_follow_step(&state, &input);
    assert(!state.auto_following && state.heading_locked && state.rate_ref_rad_s == 0);
    float captured = state.angle_ref_rad;
    input.yaw_rad += .01f;
    input.relative_deg = .7f;
    chassis_follow_step(&state, &input);
    assert(!state.auto_following);
    near(state.angle_ref_rad, captured, 1e-7f);
    input.relative_deg = 1.2f;
    chassis_follow_step(&state, &input);
    assert(state.auto_following && state.rate_ref_rad_s > 0);
    input.allowed = 0;
    chassis_follow_step(&state, &input);
    assert(!state.auto_following && state.heading_locked && state.rate_ref_rad_s == 0);

    /* Across the encoder wrap, take the two-degree route rather than 358 degrees. */
    input.allowed = 1;
    input.zero_deg = -179;
    input.relative_deg = 179;
    chassis_follow_step(&state, &input);
    assert(state.auto_following && state.rate_ref_rad_s < 0);
    near(state.error_deg, 2, 1e-5f);
    input.dt_s = .06f;
    chassis_follow_step(&state, &input);
    assert(!state.auto_following && state.heading_locked && state.rate_ref_rad_s == 0);
}

static void test_follow_motion_adapter(void)
{
    chassis_data_t command = {.high_set = .25f};
    chassis_move_t chassis = {0};
    chassis.chassis_data_ = &command;
    chassis.mode.chassis_mode = ENABLE_CHASSIS;
    chassis.mode.chassis_balancing_mode = BALANCING_READY;
    chassis.mode.sport_mode = NORMAL_MOVING_MODE;
    chassis.flag_info.suspend_flag_L = chassis.flag_info.suspend_flag_R = ON_GROUND;
    chassis.chassis_posture_info.yaw_angle_total = 7;
    chassis.gimbal_yaw_motor.relative_angle = tune_gimbal_forward_deg + 20;
    chassis_odometry.sample_valid = 1;
    chassis_odometry.speed_m_s = 0;
    memset(&chassis_follow, 0, sizeof(chassis_follow));
    for (unsigned i = 0; i < 100; ++i)
        chassisMotionUpdate(&chassis, .004f);
    assert(chassis_follow.auto_following && chassis.chassis_posture_info.yaw_gyro_set > 0);
    near(chassis.chassis_posture_info.yaw_angle_sett, 7, 1e-7f);
    near(chassis.chassis_posture_info.foot_speed_set, 0, 1e-7f);

    /* A tiny lateral stick deflection must not change the stationary heading target. */
    command.vx_set = -.001f;
    command.vy_set = .001f;
    for (unsigned i = 0; i < 100; ++i)
        chassisMotionUpdate(&chassis, .004f);
    near(chassis_follow.error_deg, -20, .001f);
    near(chassis.chassis_posture_info.foot_speed_set, 0, 1e-7f);

    /* A/D retain their +/-90-degree movement offsets; release restores gimbal alignment. */
    chassis.gimbal_yaw_motor.relative_angle = tune_gimbal_forward_deg;
    command.vx_set = 0;
    for (int direction = 1; direction >= -1; direction -= 2)
    {
        command.vy_set = direction * 2.2f;
        for (unsigned i = 0; i < 220; ++i)
            chassisMotionUpdate(&chassis, .004f);
        near(chassis_follow.error_deg, direction * -90, .001f);
        assert(chassis.chassis_posture_info.yaw_gyro_set * direction > 0);
        near(chassis.chassis_posture_info.foot_speed_set, 2.2f, 1e-6f);
        command.vy_set = 0;
        for (unsigned i = 0; i < 220; ++i)
            chassisMotionUpdate(&chassis, .004f);
        near(chassis_follow.error_deg, 0, 1e-5f);
        assert(chassis_follow.heading_locked && !chassis_follow.auto_following);
        near(chassis.chassis_posture_info.yaw_gyro_set, 0, 1e-7f);
    }
    command.wz_set = 8;
    chassisMotionUpdate(&chassis, .004f);
    assert(chassis_follow.manual_turning && !chassis_follow.auto_following);
    near(chassis.chassis_posture_info.yaw_gyro_set, 8, 1e-6f);
    chassis.mode.chassis_mode = DISABLE_CHASSIS;
    chassisMotionUpdate(&chassis, .004f);
    assert(chassis_follow.heading_locked && !chassis_follow.manual_turning);
    near(chassis.chassis_posture_info.yaw_gyro_set, 0, 1e-7f);
}

static void test_follow_feedforward(void)
{
    chassis_follow_t state = {0};
    chassis_follow_input_t input = {.allowed = 1, .dt_s = .004f,
                                    .kp_rad_s_per_deg = FOLLOW_KP_RAD_S_PER_DEG,
                                    .rate_limit_rad_s = FOLLOW_MAX_RATE_RAD_S,
                                    .accel_rad_s2 = FOLLOW_RATE_ACCEL_RAD_S2,
                                    .filter_s = FOLLOW_ERROR_FILTER_S};
    chassis_follow_step(&state, &input);
    chassis_follow_step(&state, &input);
    /* A world-fixed gimbal must not feed the chassis' own rotation back as
     * target motion. Move offsets and zero calibration are excluded as well. */
    for (unsigned i = 0; i < 100; ++i)
    {
        input.yaw_rad += .002f;
        input.relative_deg -= .002f * 57.2957795f;
        input.move_deg = i & 1 ? 90 : -90;
        input.zero_deg = i & 1 ? 10 : 0;
        chassis_follow_step(&state, &input);
        near(state.target_rate_rad_s, 0, 1e-5f);
    }
    /* Cross +/-180 smoothly; do not interpret an encoder wrap as a full turn. */
    memset(&state, 0, sizeof(state));
    input.yaw_rad = input.move_deg = input.zero_deg = 0;
    input.relative_deg = 179.9f;
    chassis_follow_step(&state, &input);
    chassis_follow_step(&state, &input);
    input.relative_deg = -179.9f;
    chassis_follow_step(&state, &input);
    assert(state.target_rate_rad_s > 0 && state.target_rate_rad_s < .1f);
    input.allowed = 0;
    chassis_follow_step(&state, &input);
    assert(!state.target_rate_valid && state.target_rate_rad_s == 0);

    /* Closed-loop regression with a simple first-order yaw-rate plant.
     * It exercises moving-target tracking, abrupt stop and both directions;
     * the 40 ms plant is a test fixture, not an identified robot model. */
    for (int direction = -1; direction <= 1; direction += 2)
    {
        memset(&state, 0, sizeof(state));
        input.allowed = 1;
        float yaw = 0, rate = 0, target = 0, lag = 0, overshoot = 0;
        for (unsigned i = 0; i < 1250; ++i)
        {
            if (i >= 125 && i < 375) target += direction * .8f * input.dt_s;
            input.yaw_rad = yaw;
            input.relative_deg = (target - yaw) * 57.2957795f;
            float previous_rate = state.rate_ref_rad_s;
            chassis_follow_step(&state, &input);
            assert(fabsf(state.rate_ref_rad_s) <= input.rate_limit_rad_s);
            assert(fabsf(state.rate_ref_rad_s - previous_rate) <=
                   FOLLOW_BRAKE_ACCEL_RATIO * input.accel_rad_s2 * input.dt_s + 1e-6f);
            rate += (state.rate_ref_rad_s - rate) * input.dt_s / (.04f + input.dt_s);
            yaw += rate * input.dt_s;
            if (i >= 250 && i < 375) lag += fabsf(target - yaw) * 57.2957795f / 125;
            if (i >= 375) overshoot = fmaxf(overshoot, direction * (yaw - target) * 57.2957795f);
        }
        assert(lag < 4.5f); /* Before PD tuning: about 4.98 deg in this fixture. */
        assert(overshoot < .5f);
        assert(fabsf(target - yaw) * 57.2957795f < FOLLOW_ANGLE_EXIT_DEG);
    }
}

static void test_follow_damping(void)
{
    /* A world-fixed gimbal and an externally rotating chassis must request
     * braking, even with P disabled. Exercise the physical sign both ways. */
    for (int direction = -1; direction <= 1; direction += 2)
    {
        chassis_follow_t state = {0};
        chassis_follow_input_t input = {.allowed = 1, .dt_s = .004f,
                                        .relative_deg = direction * 20.0f,
                                        .rate_limit_rad_s = FOLLOW_MAX_RATE_RAD_S,
                                        .accel_rad_s2 = FOLLOW_RATE_ACCEL_RAD_S2,
                                        .filter_s = FOLLOW_ERROR_FILTER_S};
        chassis_follow_step(&state, &input);
        chassis_follow_step(&state, &input);
        /* Direction-offset and calibration steps must not kick the D path. */
        input.move_deg = 10;
        input.zero_deg = 5;
        chassis_follow_step(&state, &input);
        near(state.rate_ref_rad_s, 0, 1e-6f);
        input.move_deg = input.zero_deg = 0;
        for (unsigned i = 0; i < 100; ++i)
        {
            input.yaw_rad += direction * .002f;
            input.relative_deg -= direction * .002f * 57.2957795f;
            chassis_follow_step(&state, &input);
        }
        near(state.target_rate_rad_s, 0, 1e-5f);
        assert(direction * state.rate_ref_rad_s < -.05f);
        input.manual_rate_rad_s = direction;
        chassis_follow_step(&state, &input);
        near(state.rate_ref_rad_s, direction, 1e-6f);
        near(state.relative_rate_rad_s, 0, 1e-6f);
        input.allowed = 0;
        chassis_follow_step(&state, &input);
        assert(state.heading_locked && state.relative_rate_rad_s == 0);
    }
}

static void test_recovery(chassis_move_t *c, chassis_data_t *command)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        c->mode.chassis_mode = ENABLE_CHASSIS;
        c->mode.sport_mode = NORMAL_MOVING_MODE;
        c->chassis_posture_info.pitch_angle = 0;
        c->chassis_posture_info.pitch_gyro = 0;
        c->chassis_posture_info.roll_angle = 0;
        c->chassis_posture_info.chassis_posture_L.leg_length = .35f;
        c->chassis_posture_info.chassis_posture_R.leg_length = .35f;
        c->chassis_posture_info.chassis_posture_L.leg_angle = 0;
        c->chassis_posture_info.chassis_posture_R.leg_angle = 0;
        c->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian = 0;
        c->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian = 0;
        command->recovery_flag = 0;
        host_tick += 5000;
        assert(!chassisRecoveryProcess(c));
        c->chassis_posture_info.pitch_angle = direction * 2.0f;
        assert(chassisRecoveryProcess(c) && chassisRecoveryIsActive());
        command->recovery_flag = 1;
        assert(chassisRecoveryProcess(c));
        host_tick += 1;
        chassisRecoveryProcess(c);
        host_tick += 200;
        chassisRecoveryProcess(c);
        near(c->torque_info.joint_balancing_torque_L, direction * 5.0f, 1e-6f);
        near(c->torque_info.joint_balancing_torque_R, direction * 5.0f, 1e-6f);
        c->chassis_posture_info.pitch_angle = 0;
        host_tick += 1;
        chassisRecoveryProcess(c);
        host_tick += 1500;
        assert(chassisRecoveryProcess(c) && !chassisRecoveryIsActive());
        assert(c->mode.chassis_balancing_mode == FOOT_LAUNCHING);
    }
}

int main(void)
{
    test_filter();
    chassis_move_t chassis = {0};
    chassis_data_t command = {0};
    chassis.chassis_data_ = &command;
    test_mode_targets(&chassis, &command);
    test_observer(&chassis);
    test_motion_control(&chassis, &command);
    test_stop_position();
    test_yaw_follow();
    test_follow_motion_adapter();
    test_follow_feedforward();
    test_follow_damping();
    test_recovery(&chassis, &command);
    puts("PASS: KF zero/bias/dropout/reset, startup/jump targets, observer fusion/fallback/integration, "
         "motion/contact PID paths, settled position capture, stationary/rate-only yaw follow, "
         "A/D release alignment, target-rate feedforward/stop regression, manual priority and forward/backward recovery");
    return 0;
}
