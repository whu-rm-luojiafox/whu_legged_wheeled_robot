#ifndef KALMAN_SERVICE_H
#define KALMAN_SERVICE_H
#include <stdint.h>

/* 单实例，由底盘任务串行使用；全部输入采用 SI 单位，速度向前为正。 */
typedef struct
{
    float speed_m_s;
    float accel_bias_m_s2;
    uint8_t valid;
} chassis_velocity_kf_result_t;

/* Init 可重置状态，矩阵只在首次调用时分配。 */
uint8_t ChassisVelocityKF_Init(float initial_speed_m_s);
/* Select quieter Q/R only after observer confirms sustained standstill. */
void ChassisVelocityKF_SetStationary(uint8_t stationary);
/* 每拍更新；wheel_valid=0 或输入无法计算时返回 0，保留上一状态。
 * 零速是有效量测；重力投影/轮速有效性/回退由 observer 负责。
 */
uint8_t ChassisVelocityKF_Update(float accel_forward_m_s2, float wheel_velocity_m_s, float dt_s,
                                 uint8_t wheel_valid);
const chassis_velocity_kf_result_t *ChassisVelocityKF_Get(void);
#endif
