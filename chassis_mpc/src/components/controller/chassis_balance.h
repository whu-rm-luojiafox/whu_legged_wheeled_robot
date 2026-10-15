#ifndef CHASSIS_BALANCE_H
#define CHASSIS_BALANCE_H

#include <stdint.h>

/* 平衡控制的三部分放在同一文件：姿态参考 -> LQR 反馈分量 -> 质心前馈。
 * 只计算模型坐标下的参考/力矩，不处理驱动、电机极性或 VMC。 */
/* 机身姿态与对地腿角分别标定。减小 pitch 时保持对地腿角参考，
 * 让机身可相对腿部转动，避免同步移动腿的支撑几何参考。
 * 机身 pitch 默认零偏移；这些参考不能从 IMU 反馈中减掉。 */
#define BALANCE_PITCH_DEFAULT_RAD 0.0f
#define BALANCE_LEG_GROUND_DEFAULT_RAD 0.1072f
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

#define CHASSIS_LQR_STATE_COUNT 10
#define CHASSIS_LQR_INPUT_COUNT 4

/* error = reference - feedback，模型坐标顺序与 K_Fit 一致：
 * [s, ds, yaw, dyaw, thetaL, dthetaL, thetaR, dthetaR, pitch, dpitch]。
 * 输出顺序：[左轮、右轮、左虚拟腿、右虚拟腿]，单位 N*m。
 * 这里不翻转电机极性，不做 VMC，也不承担离地/跳跃切换。 */
typedef struct {
    float balance[CHASSIS_LQR_INPUT_COUNT]; /* 腿角、机身姿态分量 */
    float motion[CHASSIS_LQR_INPUT_COUNT];  /* 位置、速度、航向分量 */
    float cross_leg[CHASSIS_LQR_INPUT_COUNT]; /* 对侧腿分量，未乘权重的诊断量 */
} chassis_lqr_feedback_t;

/* scale=0 复现原实现；scale=1 使用完整矩阵。
 * 输入及增益须为有效模型数据；无效 scale 回退到原实现。
 * 当前固件默认 0，完整耦合需独立验证后再启用。 */
void chassis_lqr_feedback_calculate(const float gain[4][10], const float error[10],
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
