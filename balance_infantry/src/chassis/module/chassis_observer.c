#include "chassis_types.h"
#include "chassis_observer.h"
#include "chassis_contact.h"
#include "kalman_service.h"
#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

chassis_odometry_t chassis_odometry __attribute__((section(".bss.chassis_odometry")));
static float yaw_delta;
static float alpha_da = 0.7f;

static void read_feedback(chassis_move_t *fdb, float dt_s)
{

    /*------------------------------- Update HT Motor info ------------------------------ */
    /* 1~4 号髋关节电机统一约定：从各自电机轴侧观察，逆时针位置、
     * 速度和输出力矩为正。下式保留驱动反馈的原始正方向。
     */
    fdb->joint_motor_1.position = fdb->joint_motor_1.motor_measure->pos;
    fdb->joint_motor_2.position = fdb->joint_motor_2.motor_measure->pos;
    fdb->joint_motor_3.position = fdb->joint_motor_3.motor_measure->pos;
    fdb->joint_motor_4.position = fdb->joint_motor_4.motor_measure->pos;

    fdb->joint_motor_1.velocity = fdb->joint_motor_1.motor_measure->vel;
    fdb->joint_motor_2.velocity = fdb->joint_motor_2.motor_measure->vel;
    fdb->joint_motor_3.velocity = fdb->joint_motor_3.motor_measure->vel;
    fdb->joint_motor_4.velocity = fdb->joint_motor_4.motor_measure->vel;

    //更新力矩反馈
    fdb->joint_motor_1.torque_get = fdb->joint_motor_1.motor_measure->torque;
    fdb->joint_motor_2.torque_get = fdb->joint_motor_2.motor_measure->torque;
    fdb->joint_motor_3.torque_get = fdb->joint_motor_3.motor_measure->torque;
    fdb->joint_motor_4.torque_get = fdb->joint_motor_4.motor_measure->torque;
    fdb->foot_motor_L.speed = -fdb->foot_motor_L.motor_measure->speed_rpm * 2 * PI / 60 / (268.0f / 17.0f) *
                              WHEEL_RADIUS; // dps -> m/s
    fdb->foot_motor_R.speed =
        fdb->foot_motor_R.motor_measure->speed_rpm * 2 * PI / 60 / (268.0f / 17.0f) * WHEEL_RADIUS;
    fdb->chassis_posture_info.foot_speed = (fdb->foot_motor_L.speed + fdb->foot_motor_R.speed) / 2.0f;

    //足端角度解算
    chassis_leg_forward_kinematics(&fdb->chassis_posture_info.chassis_posture_L, fdb->joint_motor_1.position,
                                   fdb->joint_motor_1.velocity, fdb->joint_motor_2.position,
                                   fdb->joint_motor_2.velocity);
    chassis_leg_forward_kinematics(&fdb->chassis_posture_info.chassis_posture_R, fdb->joint_motor_4.position,
                                   fdb->joint_motor_4.velocity, fdb->joint_motor_3.position,
                                   fdb->joint_motor_3.velocity);

    //陀螺仪数据更新
    fdb->chassis_posture_info.yaw_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_YAW_ADDRESS_OFFSET);
    fdb->chassis_posture_info.pitch_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_PITCH_ADDRESS_OFFSET);
    fdb->chassis_posture_info.roll_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_ROLL_ADDRESS_OFFSET);

    fdb->chassis_posture_info.x_accel = *(fdb->chassis_INS_accel + INS_ACCEL_X_ADDRESS_OFFSET);
    fdb->chassis_posture_info.y_accel = *(fdb->chassis_INS_accel + INS_ACCEL_Y_ADDRESS_OFFSET);
    fdb->chassis_posture_info.z_accel = *(fdb->chassis_INS_accel + INS_ACCEL_Z_ADDRESS_OFFSET);

    fdb->chassis_posture_info.yaw_angle = rad_format(*(fdb->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    yaw_delta = fdb->chassis_posture_info.yaw_angle - fdb->chassis_posture_info.yaw_angle_last;
    if (yaw_delta < -PI)
    {
        fdb->chassis_posture_info.yaw_round_cnt++;
    }
    else if (yaw_delta > PI)
    {
        fdb->chassis_posture_info.yaw_round_cnt--;
    }
    fdb->chassis_posture_info.yaw_angle_total =
        fdb->chassis_posture_info.yaw_round_cnt * PI2 + fdb->chassis_posture_info.yaw_angle;
    fdb->chassis_posture_info.yaw_angle_last = fdb->chassis_posture_info.yaw_angle;
    fdb->chassis_posture_info.pitch_angle = rad_format(*(fdb->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    fdb->chassis_posture_info.roll_angle = *(fdb->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

    /* Publish only ground-frame feedback. Writing the local value first lets
     * asynchronous debugger sampling observe an intermediate coordinate frame.
     * The right leg's local angle/rate has the opposite model sign. */
    fdb->chassis_posture_info.chassis_posture_L.leg_angle =
        fdb->chassis_posture_info.chassis_posture_L.leg_angle_local + fdb->chassis_posture_info.pitch_angle;
    fdb->chassis_posture_info.chassis_posture_R.leg_angle =
        -fdb->chassis_posture_info.chassis_posture_R.leg_angle_local + fdb->chassis_posture_info.pitch_angle;
    fdb->chassis_posture_info.chassis_posture_L.leg_gyro =
        fdb->chassis_posture_info.chassis_posture_L.leg_gyro_local + fdb->chassis_posture_info.pitch_gyro;
    fdb->chassis_posture_info.chassis_posture_R.leg_gyro =
        -fdb->chassis_posture_info.chassis_posture_R.leg_gyro_local + fdb->chassis_posture_info.pitch_gyro;

    // 计算加速度
    /* Motor transmission delays are part of the feedback interval. Do not
     * differentiate with the 2 ms loop-tail delay (normally about 4 ms total).
     * Rebase after startup or an invalid/stalled interval instead of creating
     * an acceleration impulse in the ground-contact force estimate. */
    if (fdb->flag_info.init_flag || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f)
    {
        /* Establish a derivative baseline before off-ground detection starts. */
        fdb->chassis_posture_info.chassis_posture_L.leg_ddlength = 0.0f;
        fdb->chassis_posture_info.chassis_posture_R.leg_ddlength = 0.0f;
        fdb->chassis_posture_info.chassis_posture_L.last_leg_dlength_jacobian =
            fdb->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian;
        fdb->chassis_posture_info.chassis_posture_R.last_leg_dlength_jacobian =
            fdb->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian;
    }
    else
    {
        fp32 temp_a_L = (fdb->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian -
                         fdb->chassis_posture_info.chassis_posture_L.last_leg_dlength_jacobian) /
                        dt_s;
        fdb->chassis_posture_info.chassis_posture_L.leg_ddlength =
            alpha_da * temp_a_L +
            (1 - alpha_da) * fdb->chassis_posture_info.chassis_posture_L.last_leg_ddlength;
        fp32 temp_a_R = (fdb->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian -
                         fdb->chassis_posture_info.chassis_posture_R.last_leg_dlength_jacobian) /
                        dt_s;
        fdb->chassis_posture_info.chassis_posture_R.leg_ddlength =
            alpha_da * temp_a_R +
            (1 - alpha_da) * fdb->chassis_posture_info.chassis_posture_R.last_leg_ddlength;
    }

    fdb->chassis_posture_info.chassis_posture_L.last_leg_angle =
        fdb->chassis_posture_info.chassis_posture_L.leg_angle;
    fdb->chassis_posture_info.chassis_posture_R.last_leg_angle =
        fdb->chassis_posture_info.chassis_posture_R.leg_angle;
    fdb->chassis_posture_info.chassis_posture_L.last_leg_length =
        fdb->chassis_posture_info.chassis_posture_L.leg_length;
    fdb->chassis_posture_info.chassis_posture_R.last_leg_length =
        fdb->chassis_posture_info.chassis_posture_R.leg_length;
    fdb->chassis_posture_info.chassis_posture_L.last_leg_dlength_jacobian =
        fdb->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian;
    fdb->chassis_posture_info.chassis_posture_R.last_leg_dlength_jacobian =
        fdb->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian;
    fdb->chassis_posture_info.chassis_posture_L.last_leg_ddlength =
        fdb->chassis_posture_info.chassis_posture_L.leg_ddlength;
    fdb->chassis_posture_info.chassis_posture_R.last_leg_ddlength =
        fdb->chassis_posture_info.chassis_posture_R.leg_ddlength;
    //云台相对角度更新
    fdb->gimbal_yaw_motor.relative_angle = theta_format(fdb->gimbal_yaw_motor.gimbal_motor_measure->angle);
}

/* ZYX 姿态，车体系 X前/Y左/Z上；水平投影中重力分量为零。 */
static float forward_acceleration(float ax, float ay, float az, float pitch, float roll)
{
    return cosf(pitch) * ax + sinf(pitch) * (sinf(roll) * ay + cosf(roll) * az);
}
float wheel_velocity;
static void update_velocity(chassis_move_t *fdb, float dt_s)
{
    /* 本周期 IMU、腿部和接地检测已更新，统一在这里发布速度和里程。 */
    const uint32_t now = (uint32_t)xTaskGetTickCount();
    const uint32_t timeout = pdMS_TO_TICKS(20);
    const uint32_t tick_l = fdb->foot_motor_L.motor_measure->last_update_time;
    const uint32_t tick_r = fdb->foot_motor_R.motor_measure->last_update_time;
    const uint8_t wheel_valid = tick_l != 0u && tick_r != 0u && (uint32_t)(now - tick_l) <= timeout &&
                                (uint32_t)(now - tick_r) <= timeout && isfinite(fdb->foot_motor_L.speed) &&
                                isfinite(fdb->foot_motor_R.speed) &&
                                fdb->flag_info.suspend_flag_L == ON_GROUND &&
                                fdb->flag_info.suspend_flag_R == ON_GROUND;
    wheel_velocity = 0.5f * fdb->foot_motor_L.speed + 0.5f * fdb->foot_motor_R.speed;
    const chassis_posture_info_t *posture = &fdb->chassis_posture_info;
    const float acceleration = forward_acceleration(posture->x_accel, posture->y_accel, posture->z_accel,
                                                    posture->pitch_angle, posture->roll_angle);
    const uint8_t fused_valid = ChassisVelocityKF_Update(acceleration, wheel_velocity, dt_s, wheel_valid);
    const chassis_velocity_kf_result_t *result = ChassisVelocityKF_Get();

    /* IMU/滤波异常但落地轮速仍可信时，回退原轮速。
     * 两者都无效则暂停积分和位置锁，不能继续使用过期的 KF 数值。 */
    const float speed = fused_valid ? result->speed_m_s : (wheel_valid ? wheel_velocity : NAN);
    chassis_odometry.sample_valid = isfinite(speed) && isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.05f;
    chassis_odometry.speed_m_s = chassis_odometry.sample_valid ? speed : 0.0f;
    if (chassis_odometry.sample_valid) chassis_odometry.distance_m += speed * dt_s;
    fdb->chassis_posture_info.foot_speed_KF = chassis_odometry.speed_m_s;
    fdb->chassis_posture_info.foot_distance_K = chassis_odometry.distance_m;
}

void chassisObserverInit(chassis_move_t *c)
{
    c->joint_motor_1.motor_measure = get_DM_motor_measure_point(0);
    c->joint_motor_2.motor_measure = get_DM_motor_measure_point(1);
    c->joint_motor_3.motor_measure = get_DM_motor_measure_point(2);
    c->joint_motor_4.motor_measure = get_DM_motor_measure_point(3);
    c->foot_motor_L.motor_measure = get_chassis_motor_measure_point(0);
    c->foot_motor_R.motor_measure = get_chassis_motor_measure_point(1);
    c->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();

    c->chassis_INS_angle = get_INS_angle_point();
    c->chassis_INS_gyro = get_gyro_data_point();
    c->chassis_INS_accel = get_accel_data_point();

    ChassisVelocityKF_Init(0.0f);
    c->flag_info.init_flag = 1;
    read_feedback(c, 0.0f);
    c->flag_info.init_flag = 0;
    c->gimbal_yaw_motor.relative_angle_init = CHASSIS_GIMBAL_FORWARD_DEG;
}

void chassisObserverUpdate(chassis_move_t *c, float dt_s)
{
    read_feedback(c, dt_s);
    Chassis_Status_Detect(c);
    update_velocity(c, dt_s);
}
