#ifndef CHASSIS_BALANCE_H
#define CHASSIS_BALANCE_H

#include <stdint.h>

#define BALANCE_PITCH_DEFAULT_RAD  0.0f// (-0.0200f)
#define BALANCE_LEG_GROUND_DEFAULT_RAD 0.1702f
#define BALANCE_PITCH_LIMIT_RAD 0.15f
#define BALANCE_LEG_GROUND_LIMIT_RAD 0.30f
#define BALANCE_LEG_LOCAL_LIMIT_RAD 0.20f
#define BALANCE_BIAS_SLEW_RAD_S 0.01f

typedef struct {
    uint8_t initialized;
    uint8_t request_valid;
    float pitch_rad;       /* 实际施加的机身姿态参考偏置 */
    float leg_local_rad;   /* 派生观察量：对地腿角偏置 - pitch 偏置 */
    float leg_ground_rad;  /* 独立施加的对地腿角参考偏置 */
} chassis_balance_reference_t;

/* 失能时接受有效标定值；使能时按实测 dt 缓慢改变，避免在线写入阶跃。
 * 非有限值、超范围请求或使能时的异常 dt 保持已有输出。 */
void chassis_balance_reference_step(chassis_balance_reference_t *state,
                                    uint8_t enabled, float pitch_rad,
                                    float leg_ground_rad, float dt_s);

/* K 的列：十个状态。误差一律为 reference - feedback。 */
enum {
    LQR_X_POSITION = 0,       /* s：轮轴中点前后位置，m */
    LQR_X_SPEED,              /* ds：前后速度，m/s */
    LQR_X_YAW,                /* yaw：连续航向角，rad */
    LQR_X_YAW_RATE,           /* dyaw：航向角速度，rad/s */
    LQR_X_LEG_L_ANGLE,        /* thetaL：左腿对地角，rad */
    LQR_X_LEG_L_RATE,         /* dthetaL：左腿对地角速度，rad/s */
    LQR_X_LEG_R_ANGLE,        /* thetaR：右腿对地角，rad */
    LQR_X_LEG_R_RATE,         /* dthetaR：右腿对地角速度，rad/s */
    LQR_X_BODY_PITCH,         /* pitch：机身俯仰角，rad */
    LQR_X_BODY_PITCH_RATE,    /* dpitch：机身俯仰角速度，rad/s */
    CHASSIS_LQR_STATE_COUNT
};

/* K 的行：四个模型力矩。第 2/3 行不是某一台关节电机的力矩。 */
enum {
    LQR_U_WHEEL_L = 0,        /* 左轮驱动力矩 */
    LQR_U_WHEEL_R,            /* 右轮驱动力矩 */
    LQR_U_LEG_L,              /* 左虚拟腿摆动力矩，VMC 后分配到电机 1/2 */
    LQR_U_LEG_R,              /* 右虚拟腿摆动力矩，VMC 后分配到电机 4/3 */
    CHASSIS_LQR_INPUT_COUNT
};

/* 一行 K 产生的力矩分解；以下每个字段的单位都是 N*m。
 * 例如 left_wheel.body_pitch_nm：机身 pitch 误差对左轮力矩的贡献。
 * 这些是 LQR 模型力矩，尚未加前馈、限幅或转换电机正方向。 */
typedef struct {
    float position_nm, speed_nm;             /* 前后位置、速度误差贡献 */
    float yaw_nm, yaw_rate_nm;               /* 航向角、角速度误差贡献 */
    float left_leg_angle_nm, left_leg_rate_nm;
    float right_leg_angle_nm, right_leg_rate_nm;
    float body_pitch_nm, body_pitch_rate_nm;
    float motion_nm;  /* position + speed + yaw + yaw_rate */
    float balance_nm; /* 左右腿角/角速度 + 机身 pitch/角速度 */
    float total_nm;   /* motion + balance；十项之和 */
} chassis_lqr_torque_t;

typedef struct {
    chassis_lqr_torque_t left_wheel;  /* K[0] */
    chassis_lqr_torque_t right_wheel; /* K[1] */
    chassis_lqr_torque_t left_leg;    /* K[2]，虚拟腿摆动力矩 */
    chassis_lqr_torque_t right_leg;   /* K[3]，虚拟腿摆动力矩 */
} chassis_lqr_feedback_t;

/* scale=1：四行都使用全部十个状态，u = K * error（正常落地默认）。
 * scale=0：去掉对侧腿的两项，保留旧行为用于对比；非法 scale 按 0 处理。
 * 起跳/离地等工况由调用方选择权重及覆盖输出。 */
void chassis_lqr_feedback_calculate(const float gain[CHASSIS_LQR_INPUT_COUNT][CHASSIS_LQR_STATE_COUNT],
                                   const float error[CHASSIS_LQR_STATE_COUNT],
                                   float cross_leg_scale, chassis_lqr_feedback_t *output);

/* 实验性水平质心偏置补偿，不包含模型已有的垂直质心项。
 * signed_mass_offset_kg_m 是带控制方向符号的 m*dx，符号需实车标定，
 * 不能仅凭“质心偏后”猜测。默认 enable=0 且 m*dx=0。
 * 输出为左右虚拟腿的模型坐标力矩，交给现有 VMC 做右腿镜像。 */
#define COM_FF_MAX_MOMENT_KG_M 0.10f
#define COM_FF_MAX_LEG_TORQUE_NM 0.50f
#define COM_FF_SLEW_NM_S 0.50f

typedef struct {
    float target_per_leg_nm;   /* 参数计算的单腿目标力矩，单位 N*m */
    float applied_per_leg_nm;  /* 经过斜率限制、实际叠加的单腿力矩 */
} chassis_feedforward_t;

void chassis_feedforward_step(chassis_feedforward_t *state, uint8_t allowed,
                              uint8_t enabled, float signed_mass_offset_kg_m,
                              float pitch_rad, float dt_s);

#endif
