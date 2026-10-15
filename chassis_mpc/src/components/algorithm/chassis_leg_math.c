#include "chassis_leg_math.h"
#include <math.h>

#define LEG_FIRST_ARM_M 0.21f
#define LEG_FIRST_LINK_M 0.25f
#define LEG_SECOND_LINK_M 0.25f
#define LEG_SECOND_ARM_M 0.21f
#define LEG_HIP_SPACING_M 0.0f

/* 五连杆机身局部几何。第一/第二关节：左腿为电机 1/2，右腿为 4/3。
 * 输入是标定后的电机角度 rad、角速度 rad/s；第一关节在此转换符号。
 * 对地腿角和 IMU pitch 的合成留在任务反馈阶段。 */

void chassis_leg_forward_kinematics(chassis_leg_posture_t *leg,
                                    float first_angle, float first_speed,
                                    float second_angle, float second_speed)
{
	float length_speed = 0, length = 0, leg_angle = 0, leg_speed = 0;
	float b_x, d_x, b_y, d_y, bd_distance_sq;
	float circle_a, circle_b, circle_c, link_angle, other_link_angle, link_speed;
	float discriminant;
	float b_vx, d_vx, b_vy, d_vy, foot_vx, foot_vy;
	float first_cos, second_cos, first_sin, second_sin;
	float foot_x, foot_y;
	/* Keep the calibrated position convention: phi1=-q_first, phi2=q_second.
	 * Apply the same derivative transform to motor velocities. */
	first_angle = -first_angle;
	first_speed = -first_speed;

	first_cos = cos(first_angle);
	first_sin = sin(first_angle);
	second_cos = cos(second_angle);
	second_sin = sin(second_angle);
	b_x = -LEG_HIP_SPACING_M / 2.0f + LEG_FIRST_ARM_M * first_cos;
	d_x = LEG_HIP_SPACING_M / 2.0f - LEG_SECOND_ARM_M * second_cos;
	b_y = LEG_FIRST_ARM_M * first_sin;
	d_y = LEG_SECOND_ARM_M * second_sin;

	bd_distance_sq = (d_x - b_x) * (d_x - b_x) + (d_y - b_y) * (d_y - b_y);
	circle_a = 2.0f * LEG_FIRST_LINK_M * (d_x - b_x);
	circle_b = 2.0f * LEG_FIRST_LINK_M * (d_y - b_y);
	circle_c = LEG_FIRST_LINK_M * LEG_FIRST_LINK_M + bd_distance_sq - LEG_SECOND_LINK_M * LEG_SECOND_LINK_M;
	discriminant = circle_a * circle_a + circle_b * circle_b - circle_c * circle_c;
	if (discriminant < 0.0f) discriminant = 0.0f;
	link_angle = 2.0f * atan2((circle_b + sqrt(discriminant)) , (circle_a + circle_c));

	foot_x = b_x + cos(link_angle) * LEG_FIRST_LINK_M;
	foot_y = b_y + sin(link_angle) * LEG_FIRST_LINK_M;

	length = sqrt(foot_x * foot_x + foot_y * foot_y);
	leg_angle = atan2(foot_x , foot_y);

	b_vx = -first_speed * LEG_FIRST_ARM_M * first_sin;
	b_vy = first_speed * LEG_FIRST_ARM_M * first_cos;
	/* D=(-LEG_SECOND_ARM_M*cos(phi2), LEG_SECOND_ARM_M*sin(phi2)), phi2_dot=second_speed. */
	d_vx = second_speed * LEG_SECOND_ARM_M * second_sin;
	d_vy = second_speed * LEG_SECOND_ARM_M * second_cos;
	other_link_angle = atan2((foot_y - d_y) , (foot_x - d_x));
	float velocity_denominator = LEG_FIRST_LINK_M * sin(other_link_angle - link_angle);
	if (fabs(velocity_denominator) > 1.0e-6f && length > 1.0e-6f)
	{
		link_speed = ((d_vx - b_vx) * cos(other_link_angle) + (d_vy - b_vy) * sin(other_link_angle)) / velocity_denominator;
		foot_vx = b_vx - link_speed * LEG_FIRST_LINK_M * sin(link_angle);
		foot_vy = b_vy + link_speed * LEG_FIRST_LINK_M * cos(link_angle);
		leg_speed = (cos(leg_angle) * foot_vx - sin(leg_angle) * foot_vy) / length;
		length_speed = (foot_x * foot_vx + foot_y * foot_vy) / length;
	}
	else
	{
		/* A singular/invalid five-bar pose must not inject NaN/Inf into LQR. */
		leg_speed = 0.0f;
		length_speed = 0.0f;
	}

	/* 保存与 B、D、phi 同一时刻的未滤波机身局部几何量。
	 * leg_angle/leg_gyro 由 chassis_feedback_update 叠加 IMU 后一次写入。
	 */
	leg->leg_length_raw = length;
	leg->leg_angle_local = leg_angle;
	leg->leg_gyro_local = leg_speed;

	leg->last_leg_length = leg->leg_length;
	leg->leg_length = 0.9f*length+0.1f*leg->last_leg_length;
	leg->leg_dlength = length_speed;
	leg->leg_dlength_jacobian = length_speed;
	leg->leg_x1 = -b_x;
	leg->leg_x2 = d_x;
	leg->leg_y1 = b_y;
	leg->leg_y2 = d_y;
	leg->leg_phi1 = first_angle;
	leg->leg_phi2 = second_angle;
}

chassis_leg_jacobian_t chassis_leg_inverse_jacobian(const chassis_leg_posture_t *leg)
{
    chassis_leg_jacobian_t result = {0};
    const float length = leg->leg_length_raw;
    const float angle = leg->leg_angle_local;
    const float foot_x = length * sinf(angle);
    const float foot_y = length * cosf(angle);
    const float first_dx = foot_x + leg->leg_x1;
    const float first_dy = foot_y - leg->leg_y1;
    const float second_dx = foot_x - leg->leg_x2;
    const float second_dy = foot_y - leg->leg_y2;
    const float first_denominator = -first_dx * sinf(leg->leg_phi1) +
                                    first_dy * cosf(leg->leg_phi1);
    const float second_denominator = second_dx * sinf(leg->leg_phi2) +
                                     second_dy * cosf(leg->leg_phi2);

    /* 一次算完整 N，避免过去按 1/2/3/4 查元素时反复计算三角函数。 */
    if (fabsf(first_denominator) >= 1.0e-6f) {
        result.n11 = (first_dx * sinf(angle) + first_dy * cosf(angle)) /
                     (LEG_FIRST_ARM_M * first_denominator);
        result.n12 = length * (first_dx * cosf(angle) - first_dy * sinf(angle)) /
                     (LEG_FIRST_ARM_M * first_denominator);
    }
    if (fabsf(second_denominator) >= 1.0e-6f) {
        result.n21 = (second_dx * sinf(angle) + second_dy * cosf(angle)) /
                     (LEG_SECOND_ARM_M * second_denominator);
        result.n22 = length * (second_dx * cosf(angle) - second_dy * sinf(angle)) /
                     (LEG_SECOND_ARM_M * second_denominator);
    }
    return result;
}

chassis_leg_motor_torque_t chassis_leg_map_wrench(chassis_leg_jacobian_t n,
                                                 float support_force_n,
                                                 float model_torque_nm, int right_leg)
{
    chassis_leg_motor_torque_t result = {0};
    const float determinant = n.n11 * n.n22 - n.n12 * n.n21;
    if (fabsf(determinant) > 1.0e-6f) {
        /* tau_motor = diag(-1,+1) * N^(-T) * [F,T_local]。
         * theta_R=-Q0_R，右腿只在这里翻转虚拟腿力矩方向。 */
        result.support_first_nm = -n.n22 * support_force_n / determinant;
        result.support_second_nm = -n.n12 * support_force_n / determinant;
        if (right_leg) {
            result.balance_first_nm = -n.n21 * model_torque_nm / determinant;
            result.balance_second_nm = -n.n11 * model_torque_nm / determinant;
        } else {
            result.balance_first_nm = n.n21 * model_torque_nm / determinant;
            result.balance_second_nm = n.n11 * model_torque_nm / determinant;
        }
    }
    return result;
}
