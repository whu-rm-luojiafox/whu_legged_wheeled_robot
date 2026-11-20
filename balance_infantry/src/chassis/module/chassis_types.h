#ifndef CHASSIS_TYPES_H
#define CHASSIS_TYPES_H

/* 底盘共享状态；任务、观测、目标和控制模块使用同一份数据定义。 */
#include "struct_typedef.h"
#include "chassis_fwd.h"
#include "can_service.h"
#include "gim_cha_protocol.h"
#include "super_power_protocol.h"
#include "pid.h"
#include "chassis_config.h"
#include "chassis_leg_math.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_motor.h"
typedef enum
{
    ENABLE_CHASSIS = 0,
    DISABLE_CHASSIS,
} chassis_mode_e;

typedef enum
{
    NO_FORCE,        /* 遥控失能，或倒地后卸力等待 */
    FOOT_LAUNCHING,  /* 双腿按对地角摆腿到位 */
    JOINT_LAUNCHING, /* 固定腿角缩腿，随后渐进接入 LQR 并回站立角 */
    BALANCING_READY, /* 正常平衡控制 */
} chassis_balancing_mode_e;

typedef enum
{
    NONE,
    NORMAL_MOVING_MODE,
    JUMPING_MODE,
    FLY_MODE,
    TK_MODE,
} sport_mode_e;

typedef enum
{
    READY_TO_JUMP,
    PREPARING_LANDING,
    PREPARING_STAND_JUMPING,
    CONSTACTING_LEGS,
    EXTENDING_LEGS,
    CONSTACTING_LEGS_2,
    FINISHED,
} jumping_stage_e;

typedef enum
{
    NOT_DEFINE,
    STANDING_JUMP,
} jumping_mode_e;

typedef enum
{
    NORMAL_MODE = 0,
    HIGH_MODE,
    EXTREMELY_HIGH_MODE,
} chassis_high_mode_e;

typedef enum
{
    MOTOR_NO_FORCE = 0,
    MOTOR_FORCE,
} chassis_motor_mode_e;

typedef enum
{
    ON_GROUND = 0,
    OFF_GROUND = 1,
} suspend_flag_e;

typedef struct
{
    chassis_mode_e chassis_mode, last_chassis_mode;
    chassis_balancing_mode_e chassis_balancing_mode, last_chassis_balancing_mode;
    sport_mode_e sport_mode, last_sport_mode;

    jumping_mode_e jumping_mode, last_jumping_mode;
    jumping_stage_e jumping_stage, last_jumping_stage;

    chassis_high_mode_e chassis_high_mode, last_chassis_high_mode;

} mode_t;

typedef struct
{
    const fp32 *chassis_INS_angle_point;
    const fp32 *chassis_INS_gyro_point;
    const fp32 *chassis_INS_accel_point;
    fp32 yaw_angle, yaw_angle_last, yaw_angle_total, pitch_angle, roll_angle, last_pitch_angle, yaw_round_cnt;
    fp32 yaw_gyro, pitch_gyro, roll_gyro, last_pitch_gyro;
    fp32 yaw_accel, pitch_accel, roll_accel;
    fp32 x_accel, y_accel, z_accel;

    fp32 yaw_angle_sett, pitch_angle_set, roll_angle_set;
    fp32 yaw_gyro_set, pitch_gyro_set, roll_gyro_set;

    fp32 ideal_high;
    chassis_leg_posture_t chassis_posture_L;
    chassis_leg_posture_t chassis_posture_R;
    fp32 foot_roll_angle;
    /* ----------debug param-------- */
    fp32 xc, yc, xb, yb;
    fp32 Q2;

    uint16_t position_lock_flag, position_lock_state;
    uint16_t yaw_lock_flag;
    fp32 foot_distance, foot_distance_K, foot_distance_set, target_distance_set;
    fp32 foot_speed, foot_speed_KF, foot_speed_set;

    //车身加速度
    fp32 chassis_accel;
    //轮子加速度计算
    fp32 foot_accel_L, foot_accel_R;

} chassis_posture_info_t;

typedef struct
{
    // -------- horizontal force --------
    fp32 foot_balancing_torque_L, foot_balancing_torque_R;
    fp32 foot_moving_torque_L, foot_moving_torque_R;
    fp32 foot_horizontal_torque_L, foot_horizontal_torque_R;

    fp32 joint_balancing_torque_L, joint_balancing_torque_R;
    fp32 joint_moving_torque_L, joint_moving_torque_R;
    fp32 joint_prevent_splits_torque_L, joint_prevent_splits_torque_R;
    fp32 joint_horizontal_torque_L, joint_horizontal_torque_R;

    fp32 joint_roll_torque_L, joint_roll_torque_R;

    // 腿角力矩映射出的电机力矩分量，单位 N*m，输出限幅前。
    // temp1/temp2 顺序与正解一致：左侧1/2号，右侧4/3号。
    fp32 joint_horizontal_torque_temp1_R, joint_horizontal_torque_temp2_R;
    fp32 joint_horizontal_torque_temp1_L, joint_horizontal_torque_temp2_L;

    fp32 yaw_torque;

    // -------- vertical force --------

    fp32 joint_stand_torque_L, joint_stand_torque_R;
    fp32 last_control_torque_L, last_control_torque_R;
    fp32 joint_vertical_torque_L, joint_vertical_torque_R;
    fp32 joint_real_vertical_torque_L, joint_real_vertical_torque_R;

    // 腿长方向力映射出的电机力矩分量，单位 N*m，输出限幅前。
    // 与同名 horizontal 分量相加后赋给对应电机；不是实测反馈力矩。
    fp32 joint_vertical_torque_temp1_R, joint_vertical_torque_temp2_R;
    fp32 joint_vertical_torque_temp1_L, joint_vertical_torque_temp2_L;

    fp32 forque_L, forque_R;
    fp32 supportive_force_L, supportive_force_R, last_supportive_force_L, last_supportive_force_R;
    fp32 balance_force_L, balance_force_R;

} torque_info_t;
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_t;

typedef struct
{
    dm_motor_measure_t *motor_measure;
    chassis_motor_mode_e motor_mode, last_motor_mode;

    bool_t offline_flag;

    fp32 position;
    fp32 init_position;
    fp32 position_offset;

    fp32 velocity;
    // fp32 velocity_kf; //滤波后的速度

    fp32 torque_out, torque_get;
    fp32 velocity_set;
    bool_t velocity_native; /* 旧速度协议标志；恢复摆腿为径向零力的 MIT 角向控制。 */
    fp32 max_torque, min_torque;
} joint_motor_t;

typedef struct
{
    motor_measure_t *motor_measure;
    chassis_motor_mode_e motor_mode, last_motor_mode;

    bool_t offline_flag;

    fp32 distance, last_position, position, turns;
    fp32 speed, speed_kf;
    fp32 torque_out, torque_get;
    fp32 last_control_torque;

} foot_motor_t;

/* -----------------------------平步标志位结构体---------------------------- */
typedef struct
{
    bool_t init_flag;
    suspend_flag_e suspend_flag_L, last_suspend_flag_L;
    suspend_flag_e suspend_flag_R, last_suspend_flag_R;
    bool_t rotation_flag;
    bool_t set_pos_after_moving;
    bool_t overpower_warning_flag;
    bool_t last_overpower_warning_flag;

    // 跳跃相关标志
    uint8_t jump_prepare_complete; // 跳跃准备完成标志
    uint32_t jump_prepare_timer;   // 跳跃准备计时器
    uint32_t jump_extend_timer;    // 伸腿计时器
    uint32_t jump_contact_timer;   // 接触计时器
} flag_info_t;

/* -----------------------------VMC结构体---------------------------- */
typedef struct
{
    fp32 J1_L, J2_L;
    fp32 J3_L, J4_L;
    fp32 J1_R, J2_R;
    fp32 J3_R, J4_R;
    fp32 invJ1_L, invJ2_L;
    fp32 invJ3_L, invJ4_L;
    fp32 invJ1_R, invJ2_R;
    fp32 invJ3_R, invJ4_R;
} mapping_info_t;

// 定义多项式系数结构体
typedef struct
{
    float c0; // 常数项
    float c1; // L0系数
    float c2; // Q0系数
    float c3; // L0^2系数
    float c4; // L0*Q0系数
    float c5; // Q0^2系数
} PolynomialCoefficients;

struct chassis_move
{

    mode_t mode;
    flag_info_t flag_info;
    chassis_posture_info_t chassis_posture_info;
    torque_info_t torque_info;
    mapping_info_t mapping_info;
    uint8_t swing_velocity_active;
    fp32 swing_rate_L, swing_rate_R; /* Fixed paired motor speeds; right side is mirrored at output. */
    const RC_ctrl_t *chassis_RC;
    const fp32 *chassis_INS_angle;
    const fp32 *chassis_INS_gyro;
    const fp32 *chassis_INS_accel;
    const chassis_data_t *chassis_data_; // 从云台接受到的底盘数据设定值

    chassis_motor_t motor_chassis[4]; // chassis motor data.底盘电机数据
    pid_type_def motor_speed_pid[4];  // motor speed PID.底盘电机速度pid
    pid_type_def leg_L_length_pid;    // 腿长设定PID
    pid_type_def leg_R_length_pid;    // 腿长设定PID
    pid_type_def chassis_roll_pid;    // roll轴控制PID

    fp32 vx;     //  底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;     // 底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;     // 底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 vx_set; // 底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set; // 底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set; // 底盘设定旋转角速度，逆时针为正 单位 rad/s
    fp32 chassis_yaw_set;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    gimbal_motor_t gimbal_yaw_motor;
    joint_motor_t joint_motor_1, joint_motor_2, joint_motor_3, joint_motor_4;
    foot_motor_t foot_motor_L, foot_motor_R;
    fp32 distance;
    super_power_t super_power;
};

#endif
