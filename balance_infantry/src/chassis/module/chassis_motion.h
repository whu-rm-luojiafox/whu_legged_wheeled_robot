#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

/* 平移控制的完整状态：轮速积分得到当前位置，停车逻辑决定位置参考。
 * 两部分共用同一 dt 与距离坐标；位置/速度误差仍交由 LQR 处理。 */
/* 轮速里程，不是融合定位。输入为已统一正方向的轮缘速度 m/s。 */
typedef struct {
    uint8_t sample_valid;
    float speed_m_s;
    float distance_m;
} chassis_odometry_t;

/* dt 无效或数据非有限时保留累计距离，并标记当前样本不可用于位置锁。 */
void chassis_odometry_step(chassis_odometry_t *state, float left_m_s,
                           float right_m_s, float dt_s);

#define POSITION_COMMAND_DEADBAND_M_S 0.02f
#define POSITION_SETTLED_SPEED_M_S 0.06f
#define POSITION_SETTLE_TIME_S 0.15f
#define POSITION_BRAKE_TIMEOUT_S 0.50f

/* 只管理位置参考，位置/速度误差仍由 LQR 处理，不额外叠加 PID。 */
typedef struct {
    uint8_t locked;          /* 捕获后固定目标，直到移动指令或控制条件将其释放 */
    float stopped_time_s;   /* 实际速度连续低于门限的时长，单位 s */
    float braking_time_s;   /* 松杆后尚未锁定的时长，单位 s */
    float reference_m;      /* 输出给 LQR 的位置目标，单位 m */
} chassis_position_t;

typedef struct {
    uint8_t allowed;       /* 普通平衡或攀爬准备/伸腿，双腿触地、非自救且里程有效 */
    float dt_s;
    float command_speed_m_s;
    float measured_speed_m_s;
    float distance_m;
} chassis_position_input_t;

float chassis_position_speed_request(float command_m_s);
void chassis_position_step(chassis_position_t *state,
                           const chassis_position_input_t *input);

#endif
