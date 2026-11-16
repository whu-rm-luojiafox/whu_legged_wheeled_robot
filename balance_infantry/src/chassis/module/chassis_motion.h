#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include "chassis_fwd.h"

#define POSITION_COMMAND_DEADBAND_M_S 0.02f
#define POSITION_SETTLED_SPEED_M_S 0.10f
#define POSITION_SETTLE_TIME_S 0.15f
#define POSITION_BRAKE_ERROR_LIMIT_M 0.06f

/* 只管理位置参考，位置/速度误差仍由 LQR 处理，不额外叠加 PID。 */
typedef struct
{
    uint8_t locked;       /* 捕获后固定目标，直到移动指令或控制条件将其释放 */
    uint8_t braking;      /* 松杆后保留有界位置参考，辅助零速反馈制动 */
    float stopped_time_s; /* 实际速度连续低于门限的时长，单位 s */
    float reference_m;    /* 输出给 LQR 的位置目标，单位 m */
} chassis_position_t;

typedef struct
{
    uint8_t allowed; /* 普通平衡或攀爬准备/伸腿，双腿触地、非自救且里程有效 */
    float dt_s;
    float command_speed_m_s;
    float measured_speed_m_s;
    float distance_m;
} chassis_position_input_t;

float chassis_position_speed_request(float command_m_s);
void chassis_position_step(chassis_position_t *state, const chassis_position_input_t *input);

/* 普通运动目标；不负责状态估计、跳跃或攀爬状态机。 */
void chassisMotionUpdate(chassis_move_t *chassis, float dt_s);

#endif
