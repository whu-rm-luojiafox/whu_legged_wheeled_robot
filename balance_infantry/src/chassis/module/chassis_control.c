#include "chassis_types.h"
#include "chassis_control.h"
#include "chassis_balance.h"
#include "chassis_leg_math.h"
#include "chassis_mode.h"
#include "chassis_observer.h"
#include "chassis_climb.h"
#include "LQR.h"
#include <math.h>

#define FORWARD_TORQUE (-2.5f)
chassis_feedforward_t chassis_feedforward __attribute__((section(".bss.chassis_feedforward")));
chassis_balance_reference_t chassis_balance_reference
    __attribute__((section(".bss.chassis_balance_reference")));
chassis_lqr_feedback_t chassis_lqr_feedback __attribute__((section(".bss.chassis_lqr_feedback")));
volatile fp32 tune_leg_ki __attribute__((section(".data.tune_leg_ki"))) = LEG_SET_PID_KI;
volatile fp32 tune_leg_i_limit __attribute__((section(".data.tune_leg_i_limit"))) = LEG_SET_PID_IOUT;
volatile fp32 tune_roll_ki __attribute__((section(".data.tune_roll_ki"))) = ROLL_PID_KI;
volatile fp32 tune_roll_i_limit __attribute__((section(".data.tune_roll_i_limit"))) = ROLL_PID_IOUT;
volatile uint8_t tune_com_ff_enable __attribute__((section(".data.tune_com_ff_enable"))) = 0u;
volatile fp32 tune_com_ff_moment __attribute__((section(".data.tune_com_ff_moment"))) = 0.0f;
volatile fp32 tune_balance_pitch_rad __attribute__((section(".data.tune_balance_pitch_rad"))) =
    BALANCE_PITCH_DEFAULT_RAD;
volatile fp32 tune_balance_leg_ground_rad __attribute__((section(".data.tune_balance_leg_ground_rad"))) =
    BALANCE_LEG_GROUND_DEFAULT_RAD;
volatile fp32 tune_lqr_cross_leg_scale __attribute__((section(".data.tune_lqr_cross_leg_scale"))) = 0.0f;
fp32 jump_stand_PD_L[2] = {30000000.0f, 10.0f};
fp32 jump_stand_PD_R[2] = {30000000.0f, 10.0f};
fp32 jump_canstacting_PD_L[2] = {30000000.0f, 10.0f};
fp32 jump_canstacting_PD_R[2] = {30000000.0f, 10.0f};

static void chassis_apply_pid_tuning(chassis_move_t *chassis);
static void chassis_calculate_lqr_feedback(const chassis_move_t *chassis, const float gain[4][10]);
static void chassis_apply_com_feedforward(chassis_move_t *chassis, float dt_s);

void chassisControlInit(chassis_move_t *c)
{
    const fp32 leg_pid[3] = {LEG_SET_PID_KP, LEG_SET_PID_KI, LEG_SET_PID_KD};
    const fp32 roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
    PID_init(&c->leg_L_length_pid, PID_POSITION, leg_pid, LEG_SET_PID_OUT, LEG_SET_PID_IOUT);
    PID_init(&c->leg_R_length_pid, PID_POSITION, leg_pid, LEG_SET_PID_OUT, LEG_SET_PID_IOUT);
    PID_init(&c->chassis_roll_pid, PID_POSITION, roll_pid, ROLL_PID_OUT, ROLL_PID_IOUT);
}

static void apply_integral_tuning(pid_type_def *pid, fp32 ki, fp32 limit)
{
    if (!isfinite(ki) || !isfinite(limit) || ki < 0.0f || limit < 0.0f) return;
    /* 积分限幅不需要超过整个 PID 的输出限幅。 */
    limit = fminf(limit, pid->max_out);
    pid->Ki = ki;
    pid->max_iout = limit;
    if (pid->Iout > limit)
        pid->Iout = limit;
    else if (pid->Iout < -limit)
        pid->Iout = -limit;
}

static void chassis_apply_pid_tuning(chassis_move_t *chassis)
{
    const float leg_ki = tune_leg_ki;
    const float leg_limit = tune_leg_i_limit;
    apply_integral_tuning(&chassis->leg_L_length_pid, leg_ki, leg_limit);
    apply_integral_tuning(&chassis->leg_R_length_pid, leg_ki, leg_limit);
    apply_integral_tuning(&chassis->chassis_roll_pid, tune_roll_ki, tune_roll_i_limit);
}

static void chassis_calculate_lqr_feedback(const chassis_move_t *chassis, const float gain[4][10])
{
    const chassis_posture_info_t *posture = &chassis->chassis_posture_info;
    const chassis_leg_posture_t *left = &posture->chassis_posture_L;
    const chassis_leg_posture_t *right = &posture->chassis_posture_R;
    const float state_error[10] = {
        /* 平移位置、平移速度、连续 yaw、yaw 角速度。 */
        chassis_odometry.sample_valid ? posture->foot_distance_set - posture->foot_distance_K : 0.0f,
        chassis_odometry.sample_valid ? posture->foot_speed_set - posture->foot_speed_KF : 0.0f,
        posture->yaw_angle_sett - posture->yaw_angle_total, posture->yaw_gyro_set - posture->yaw_gyro,
        /* 左右对地腿角及角速度：独立于机身 pitch 参考。 */
        left->leg_angle_set + chassis_balance_reference.leg_ground_rad - left->leg_angle, -left->leg_gyro,
        right->leg_angle_set + chassis_balance_reference.leg_ground_rad - right->leg_angle, -right->leg_gyro,
        /* 机身 pitch 及角速度：参考偏置不改写传感器反馈。 */
        posture->pitch_angle_set + chassis_balance_reference.pitch_rad - posture->pitch_angle,
        posture->pitch_gyro_set - posture->pitch_gyro};
    chassis_lqr_feedback_calculate(gain, state_error,
                                   chassisIsBalancingOnGround(chassis) ? tune_lqr_cross_leg_scale : 0.0f,
                                   &chassis_lqr_feedback);
}

static void chassis_apply_com_feedforward(chassis_move_t *chassis, float dt_s)
{
    chassis_feedforward_step(&chassis_feedforward, chassisIsBalancingOnGround(chassis),
                             tune_com_ff_enable == 1u, tune_com_ff_moment,
                             chassis->chassis_posture_info.pitch_angle, dt_s);
    /* 模型坐标下左右同号，右腿镜像仍只在 VMC 内进行一次。
     * 不向轮端加恒定驱动力，不绕过原有关节输出限幅。 */
    chassis->torque_info.joint_balancing_torque_L += chassis_feedforward.applied_per_leg_nm;
    chassis->torque_info.joint_balancing_torque_R += chassis_feedforward.applied_per_leg_nm;
}

static void apply_airborne_torque(chassis_move_t *bl_ctrl)
{
    // 处理 joint_balancing_torque（平衡力矩）
    if (bl_ctrl->flag_info.suspend_flag_R == 1)
    {
        bl_ctrl->torque_info.joint_balancing_torque_R =
            (+LQR[3][6] * (bl_ctrl->chassis_posture_info.chassis_posture_R.leg_angle_set -
                           bl_ctrl->chassis_posture_info.chassis_posture_R.leg_angle) +
             LQR[3][7] * (0.0f - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_gyro));
        bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
    }

    if (bl_ctrl->flag_info.suspend_flag_L == 1)
    {
        bl_ctrl->torque_info.joint_balancing_torque_L =
            (+LQR[2][4] * (bl_ctrl->chassis_posture_info.chassis_posture_L.leg_angle_set -
                           bl_ctrl->chassis_posture_info.chassis_posture_L.leg_angle) +
             LQR[2][5] * (0.0f - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_gyro));
        bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
    }

    // 处理 foot 相关力矩
    if (bl_ctrl->flag_info.suspend_flag_R == 1)
    {
        bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
        bl_ctrl->torque_info.foot_moving_torque_R = 0;
    }

    if (bl_ctrl->flag_info.suspend_flag_L == 1)
    {
        bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
        bl_ctrl->torque_info.foot_moving_torque_L = 0;
    }
}

void chassisControlCalculate(chassis_move_t *bl_ctrl, float dt_s)
{
    chassis_leg_posture_t *left = &bl_ctrl->chassis_posture_info.chassis_posture_L;
    chassis_leg_posture_t *right = &bl_ctrl->chassis_posture_info.chassis_posture_R;
    chassis_apply_pid_tuning(bl_ctrl);
    chassis_balance_reference_step(&chassis_balance_reference, bl_ctrl->mode.chassis_mode == ENABLE_CHASSIS,
                                   tune_balance_pitch_rad, tune_balance_leg_ground_rad, dt_s);

    //LQR拟合矩阵数据更新
    chassis_lqr_gain_calculate(left->leg_length, right->leg_length, LQR);
    if (bl_ctrl->mode.chassis_balancing_mode == FOOT_LAUNCHING)
    {
        chassis_leg_posture_t *L = left;
        chassis_leg_posture_t *R = right;
        torque_info_t *t = &bl_ctrl->torque_info;

        if (L->leg_length > FOOT_LAUNCH_APPROPRIATE_LEG_LENGTH)
        {
            if (L->leg_angle < -PI / 10)
                t->joint_balancing_torque_L = -4;
            else if (L->leg_angle > PI / 10)
                t->joint_balancing_torque_L = 0;
            t->joint_stand_torque_L = fp32_constrain(
                200.0f * (L->leg_length_set - L->leg_length) - 40.0f * L->leg_dlength_jacobian, -8.0f, 8.0f);
        }
        else
        {
            t->joint_balancing_torque_L = 0;
            t->joint_stand_torque_L = 0;
        }
        if (R->leg_length > FOOT_LAUNCH_APPROPRIATE_LEG_LENGTH)
        {
            if (R->leg_angle < -PI / 10)
                t->joint_balancing_torque_R = -4;
            else if (R->leg_angle > PI / 10)
                t->joint_balancing_torque_R = 0;
            t->joint_stand_torque_R = fp32_constrain(
                200.0f * (R->leg_length_set - R->leg_length) - 40.0f * R->leg_dlength_jacobian, -8.0f, 8.0f);
        }
        else
        {
            t->joint_balancing_torque_R = 0;
            t->joint_stand_torque_R = 0;
        } /* Clear previous-cycle outputs before skipping the LQR branches. */
        t->joint_moving_torque_L = t->joint_moving_torque_R = 0.0f;
        t->joint_roll_torque_L = t->joint_roll_torque_R = 0.0f;
        t->foot_balancing_torque_L = t->foot_balancing_torque_R = 0.0f;
        t->foot_moving_torque_L = t->foot_moving_torque_R = 0.0f;
        chassis_apply_com_feedforward(bl_ctrl, dt_s);
        return;
    }
    //不同情况下roll轴控制
    if (bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1)
    {
        bl_ctrl->torque_info.joint_roll_torque_R = 0.0f;
        bl_ctrl->torque_info.joint_roll_torque_L = 0.0f;
    }
    else
    {
        PID_calc(&bl_ctrl->chassis_roll_pid, bl_ctrl->chassis_posture_info.roll_angle,
                 bl_ctrl->chassis_posture_info.roll_angle_set);
        bl_ctrl->torque_info.joint_roll_torque_R = -bl_ctrl->chassis_roll_pid.out;
        bl_ctrl->torque_info.joint_roll_torque_L = bl_ctrl->chassis_roll_pid.out;
    }

    //不同情况下垂直方向力控制
    if (bl_ctrl->mode.jumping_stage == EXTENDING_LEGS)
    {
        bl_ctrl->torque_info.joint_stand_torque_L =
            +jump_stand_PD_L[0] * (left->leg_length_set - left->leg_length) +
            jump_stand_PD_L[1] * (0 - left->leg_dlength_jacobian);

        bl_ctrl->torque_info.joint_stand_torque_R =
            +jump_stand_PD_R[0] * (right->leg_length_set - right->leg_length) +
            jump_stand_PD_R[1] * (0 - right->leg_dlength_jacobian);
    }
    else if (bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2)
    {
        bl_ctrl->torque_info.joint_stand_torque_L =
            +jump_canstacting_PD_L[0] * (left->leg_length_set - left->leg_length) +
            jump_canstacting_PD_L[1] * (0 - left->leg_dlength_jacobian);

        bl_ctrl->torque_info.joint_stand_torque_R =
            +jump_canstacting_PD_R[0] * (right->leg_length_set - right->leg_length) +
            jump_canstacting_PD_R[1] * (0 - right->leg_dlength_jacobian);
    }
    else
    {
        PID_calc(&bl_ctrl->leg_L_length_pid, left->leg_length, left->leg_length_set);
        PID_calc(&bl_ctrl->leg_R_length_pid, right->leg_length, right->leg_length_set);
        bl_ctrl->torque_info.joint_stand_torque_L =
            bl_ctrl->leg_L_length_pid.out +
            (bl_ctrl->flag_info.suspend_flag_L == OFF_GROUND ? 0.6f * FEED_f : FEED_f);
        bl_ctrl->torque_info.joint_stand_torque_R =
            bl_ctrl->leg_R_length_pid.out +
            (bl_ctrl->flag_info.suspend_flag_R == OFF_GROUND ? 0.6f * FEED_f : FEED_f);
    }
    //添加被动的检测到离地的相应操作
    /* 统一模型误差顺序，避免各输出复制公式时遗漏状态。 */
    chassis_calculate_lqr_feedback(bl_ctrl, LQR);
    if (bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2)
    {
        //此时处于空中，想要达到的效果是腿的度跟地面是保持垂直，
        //不在控制机身角度因为机身保持平衡的力矩其实与腿部保持竖直力矩相冲突，在没有地面支持力的情况下没有意义
        bl_ctrl->torque_info.joint_balancing_torque_L =
            (+LQR[2][4] * (left->leg_angle_set - left->leg_angle) + LQR[2][5] * (0.0f - left->leg_gyro));
        bl_ctrl->torque_info.joint_balancing_torque_R =
            (+LQR[3][6] * (right->leg_angle_set - right->leg_angle) + LQR[3][7] * (0.0f - right->leg_gyro));

        bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
        bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
    }

    else
    {
        bl_ctrl->torque_info.joint_balancing_torque_L = chassis_lqr_feedback.balance[2];
        bl_ctrl->torque_info.joint_balancing_torque_R = chassis_lqr_feedback.balance[3];
        bl_ctrl->torque_info.joint_moving_torque_L = chassis_lqr_feedback.motion[2];
        bl_ctrl->torque_info.joint_moving_torque_R = chassis_lqr_feedback.motion[3];
    }

    //轮毂控制
    /* 右轮电机侧镜像保留在任务层，模块输出始终为模型坐标。 */
    bl_ctrl->torque_info.foot_balancing_torque_L = chassis_lqr_feedback.balance[0];
    bl_ctrl->torque_info.foot_moving_torque_L = chassis_lqr_feedback.motion[0];
    bl_ctrl->torque_info.foot_balancing_torque_R = -chassis_lqr_feedback.balance[1];
    bl_ctrl->torque_info.foot_moving_torque_R = -chassis_lqr_feedback.motion[1];

    if (bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1)
    {
        // 离地状态统一处理
        apply_airborne_torque(bl_ctrl);
    }
    bl_ctrl->torque_info.foot_moving_torque_L =
        fp32_constrain(bl_ctrl->torque_info.foot_moving_torque_L, -MAX_ACCL, MAX_ACCL);
    bl_ctrl->torque_info.foot_moving_torque_R =
        fp32_constrain(bl_ctrl->torque_info.foot_moving_torque_R, -MAX_ACCL, MAX_ACCL);
    bl_ctrl->torque_info.joint_moving_torque_L =
        fp32_constrain(bl_ctrl->torque_info.joint_moving_torque_L, -MAX_ACCL_JOINT, MAX_ACCL_JOINT);
    bl_ctrl->torque_info.joint_moving_torque_R =
        fp32_constrain(bl_ctrl->torque_info.joint_moving_torque_R, -MAX_ACCL_JOINT, MAX_ACCL_JOINT);
    chassis_apply_com_feedforward(bl_ctrl, dt_s);
}

void chassisControlMap(chassis_move_t *bl_ctrl)
{
    /* Apply after LQR/contact/feedforward (and again on recovery remapping),
     * so entering PULL_UP clears torque in this same control cycle. */
    const uint8_t climb_length_only = chassisClimbLengthOnly();
    if (climb_length_only)
    {
        torque_info_t *torque = &bl_ctrl->torque_info;
        torque->joint_stand_torque_L = bl_ctrl->leg_L_length_pid.out;
        torque->joint_stand_torque_R = bl_ctrl->leg_R_length_pid.out;
        torque->foot_balancing_torque_L = torque->foot_balancing_torque_R = 0.0f;
        torque->foot_moving_torque_L = torque->foot_moving_torque_R = 0.0f;
        torque->joint_balancing_torque_L = torque->joint_balancing_torque_R = 0.0f;
        torque->joint_moving_torque_L = torque->joint_moving_torque_R = 0.0f;
        torque->joint_roll_torque_L = torque->joint_roll_torque_R = 0.0f;

        chassis_lqr_feedback = (chassis_lqr_feedback_t){0};
        chassis_feedforward.target_per_leg_nm = 0.0f;
        chassis_feedforward.applied_per_leg_nm = 0.0f;
    }

    /* N=d(phi1,phi2)/d(length,Q0) 是逆运动学雅可比。
     * phi1=-q_first, phi2=q_second，故电机力矩为
     * diag(-1,+1)*N^(-T)*[F,T_local]，不能遗漏电机坐标变换。
     * 左腿电机顺序为 (1,2)，theta_L=Q0；右腿为 (4,3)，theta_R=-Q0。
     * 因此 T_local_L=T_model_L，T_local_R=-T_model_R。
     */
    const chassis_leg_jacobian_t left_jacobian =
        chassis_leg_inverse_jacobian(&bl_ctrl->chassis_posture_info.chassis_posture_L);
    const chassis_leg_jacobian_t right_jacobian =
        chassis_leg_inverse_jacobian(&bl_ctrl->chassis_posture_info.chassis_posture_R);
    bl_ctrl->mapping_info.invJ1_L = left_jacobian.n11;
    bl_ctrl->mapping_info.invJ2_L = left_jacobian.n21;
    bl_ctrl->mapping_info.invJ3_L = left_jacobian.n12;
    bl_ctrl->mapping_info.invJ4_L = left_jacobian.n22;
    bl_ctrl->mapping_info.invJ1_R = right_jacobian.n11;
    bl_ctrl->mapping_info.invJ2_R = right_jacobian.n21;
    bl_ctrl->mapping_info.invJ3_R = right_jacobian.n12;
    bl_ctrl->mapping_info.invJ4_R = right_jacobian.n22;

    bl_ctrl->torque_info.foot_horizontal_torque_L =
        bl_ctrl->torque_info.foot_balancing_torque_L + bl_ctrl->torque_info.foot_moving_torque_L;
    bl_ctrl->torque_info.foot_horizontal_torque_R =
        bl_ctrl->torque_info.foot_balancing_torque_R +
        bl_ctrl->torque_info.foot_moving_torque_R; // 足端轮子水平力矩

    bl_ctrl->foot_motor_L.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_L;
    bl_ctrl->foot_motor_R.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_R;
    bl_ctrl->foot_motor_L.torque_out =
        fp32_constrain(bl_ctrl->foot_motor_L.torque_out, -MAX_FOOT_OUTPUT, MAX_FOOT_OUTPUT);
    bl_ctrl->foot_motor_R.torque_out =
        fp32_constrain(bl_ctrl->foot_motor_R.torque_out, -MAX_FOOT_OUTPUT, MAX_FOOT_OUTPUT);
    /* Pull-up/contact confirmation permits radial length control only.
     * Gate the fixed feedforward too, so it cannot reintroduce hip torque. */
    const fp32 forward_torque = climb_length_only ? 0.0f : FORWARD_TORQUE;
    bl_ctrl->torque_info.joint_horizontal_torque_L = bl_ctrl->torque_info.joint_balancing_torque_L +
                                                     bl_ctrl->torque_info.joint_moving_torque_L +
                                                     forward_torque;
    bl_ctrl->torque_info.joint_horizontal_torque_R = bl_ctrl->torque_info.joint_balancing_torque_R +
                                                     bl_ctrl->torque_info.joint_moving_torque_R +
                                                     forward_torque;

    bl_ctrl->torque_info.joint_vertical_torque_L =
        bl_ctrl->torque_info.joint_stand_torque_L + bl_ctrl->torque_info.joint_roll_torque_L;
    bl_ctrl->torque_info.joint_vertical_torque_R =
        bl_ctrl->torque_info.joint_stand_torque_R + bl_ctrl->torque_info.joint_roll_torque_R;

    const chassis_leg_motor_torque_t left_torque =
        chassis_leg_map_wrench(left_jacobian, bl_ctrl->torque_info.joint_vertical_torque_L,
                               bl_ctrl->torque_info.joint_horizontal_torque_L, 0);
    const chassis_leg_motor_torque_t right_torque =
        chassis_leg_map_wrench(right_jacobian, bl_ctrl->torque_info.joint_vertical_torque_R,
                               bl_ctrl->torque_info.joint_horizontal_torque_R, 1);
    bl_ctrl->torque_info.joint_vertical_torque_temp1_L = left_torque.support_first_nm;
    bl_ctrl->torque_info.joint_vertical_torque_temp2_L = left_torque.support_second_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp1_L = left_torque.balance_first_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp2_L = left_torque.balance_second_nm;
    bl_ctrl->torque_info.joint_vertical_torque_temp1_R = right_torque.support_first_nm;
    bl_ctrl->torque_info.joint_vertical_torque_temp2_R = right_torque.support_second_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp1_R = right_torque.balance_first_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp2_R = right_torque.balance_second_nm;

    /* Diagnostic components are motor-side N*m before the output clamp.
     * temp1/temp2 follow FK input order: L=(1,2), R=(4,3). */
    bl_ctrl->joint_motor_1.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp1_L +
                                        bl_ctrl->torque_info.joint_horizontal_torque_temp1_L;
    bl_ctrl->joint_motor_2.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp2_L +
                                        bl_ctrl->torque_info.joint_horizontal_torque_temp2_L;
    bl_ctrl->joint_motor_4.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp1_R +
                                        bl_ctrl->torque_info.joint_horizontal_torque_temp1_R;
    bl_ctrl->joint_motor_3.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp2_R +
                                        bl_ctrl->torque_info.joint_horizontal_torque_temp2_R;

    /* Match the HT protocol's physical torque range before the CAN layer. */
    bl_ctrl->joint_motor_1.torque_out =
        fp32_constrain(bl_ctrl->joint_motor_1.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
    bl_ctrl->joint_motor_2.torque_out =
        fp32_constrain(bl_ctrl->joint_motor_2.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
    bl_ctrl->joint_motor_3.torque_out =
        fp32_constrain(bl_ctrl->joint_motor_3.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
    bl_ctrl->joint_motor_4.torque_out =
        fp32_constrain(bl_ctrl->joint_motor_4.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
}
