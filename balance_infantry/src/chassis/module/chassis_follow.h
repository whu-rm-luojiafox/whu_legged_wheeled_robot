#ifndef CHASSIS_FOLLOW_H
#define CHASSIS_FOLLOW_H

#include <stdint.h>

/* 云台跟随外环，只生成参考值，不计算电机力矩，也不控制平移或腿长。
 * 调用顺序：任务填写 input -> chassis_follow_step -> 将两个 ref 交给 LQR。
 * 编码器角差用 deg；底盘连续 yaw 用 rad；转速用 rad/s。
 * 以下是编译默认值，在线 tune_follow_* 的 RAM 修改不会改写这里。 */
#define FOLLOW_ANGLE_ENTER_DEG 1.0f
#define FOLLOW_ANGLE_EXIT_DEG 0.4f
#define FOLLOW_MAX_RATE_RAD_S 1.5f
#define FOLLOW_RATE_ACCEL_RAD_S2 4.5f
#define FOLLOW_ERROR_FILTER_S 0.02f
#define FOLLOW_TARGET_RATE_FILTER_S 0.10f
#define FOLLOW_TARGET_RATE_GAIN 0.5f
#define FOLLOW_RELATIVE_RATE_D_GAIN 0.3f
#define FOLLOW_TARGET_RATE_EXIT_RAD_S 0.04f
#define FOLLOW_BRAKE_ACCEL_RATIO 2.0f
#define FOLLOW_MANUAL_ENTER_RAD_S 0.5f
#define FOLLOW_MANUAL_EXIT_RAD_S 0.4f
#define FOLLOW_MANUAL_MAX_RAD_S 10.0f
#define FOLLOW_MANUAL_STOP_TIME_S 0.035f

/* 状态跨调用保留，首次使用前清零。
 * 自动跟随退出后还可能正在减速，是否已锁定航向看 heading_locked。
 * 状态字段的新旧名称对应见 log/005-control-simplification.md。 */
typedef struct
{
    uint8_t initialized;     /* 已建立初始航向基准 */
    uint8_t auto_following;  /* 云台角差外环正在请求转向 */
    uint8_t manual_turning;  /* 手动转向优先 */
    uint8_t heading_locked;  /* 已锁定停止后的航向 */
    uint8_t manual_stopping; /* 松开手动转向后的衰减阶段 */
    uint8_t target_rate_valid; /* 已记录云台绝对转速估计的上一拍 */
    float error_deg;         /* 自动连续跟随时为滤波角差，其余阶段为原始角差 */
    float angle_ref_rad;     /* 输出：LQR 的连续 yaw 参考 */
    float rate_ref_rad_s;    /* 输出：LQR 的 yaw 角速度参考 */
    float previous_yaw_rad;
    float previous_relative_deg;
    float target_rate_rad_s; /* 底盘 yaw 增量 + 云台相对角增量，滤波后的速度前馈 */
    float relative_rate_rad_s; /* 滤波后的云台相对转速，用于跟随 D 阻尼（rad/s） */
} chassis_follow_t;

typedef struct
{
    uint8_t allowed;         /* 由任务层判断使能、运动模式和双腿触地条件 */
    float dt_s;              /* 实测调用间隔，超过 50 ms 则重置 */
    float relative_deg;      /* 云台相对底盘的编码器角度 */
    float zero_deg;          /* 云台与底盘对齐时的编码器角度 */
    float move_deg;          /* 平移方向折算到 +/-90 度；松杆后为 0，恢复朝向云台 */
    float yaw_rad;           /* 连续底盘航向，不要求归一化到 +/-pi */
    float manual_rate_rad_s; /* 操作者的转向指令 */
    float kp_rad_s_per_deg;  /* 自动外环 P，单位 (rad/s)/deg */
    float rate_limit_rad_s;  /* 自动跟随最高角速度 */
    float accel_rad_s2;      /* 自动跟随加速上限；制动按 FOLLOW_BRAKE_ACCEL_RATIO 倍执行 */
    float filter_s;          /* 连续自动跟随时的角差滤波时间常数 */
} chassis_follow_input_t;

void chassis_follow_step(chassis_follow_t *state, const chassis_follow_input_t *input);

#endif
