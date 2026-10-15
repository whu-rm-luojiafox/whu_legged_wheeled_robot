#ifndef CHASSIS_LEG_MATH_H
#define CHASSIS_LEG_MATH_H

/* 机身局部五连杆状态。保留现有字段布局供控制、触地检测和调试使用。
 * 几何长度用 m，角度用 rad，导数用每秒；leg_angle_local 不包含 IMU pitch。 */
typedef struct
{
    float leg_length_set;
    float leg_length, last_leg_length;
    float leg_length_raw;     // 当前五连杆几何长度，雅可比使用（未滤波）
    float leg_dlength, last_leg_dlength, leg_dlength_jacobian, last_leg_dlength_jacobian;
    float leg_ddlength, last_leg_ddlength;
    float leg_angle, last_leg_angle, leg_angle_set, leg_angle_kf;
    float leg_angle_local;    // 相对机身向下轴的局部角 Q0，五连杆雅可比使用
    float leg_gyro, last_leg_gyro;
    float leg_gyro_local;     // Q0 对时间的导数；右腿统一腿角速度为其相反数
    float leg_gyro_jacobian, last_leg_gyro_jacobian;
    float leg_accel;
    float leg_x1, leg_x2, leg_y1, leg_y2;
    float leg_phi1, leg_phi2;
} chassis_leg_posture_t;

typedef struct {
    /* N = d(phi_first, phi_second) / d(length, angle_local)。 */
    float n11, n12, n21, n22;
} chassis_leg_jacobian_t;

typedef struct {
    /* 电机侧支撑/平衡力矩分量；第一/第二电机左腿为 1/2，右腿为 4/3。 */
    float support_first_nm, support_second_nm;
    float balance_first_nm, balance_second_nm;
} chassis_leg_motor_torque_t;

void chassis_leg_forward_kinematics(chassis_leg_posture_t *leg,
                                    float first_angle, float first_speed,
                                    float second_angle, float second_speed);
chassis_leg_jacobian_t chassis_leg_inverse_jacobian(const chassis_leg_posture_t *leg);
chassis_leg_motor_torque_t chassis_leg_map_wrench(chassis_leg_jacobian_t inverse_jacobian,
                                                 float support_force_n,
                                                 float model_torque_nm, int right_leg);

#endif
