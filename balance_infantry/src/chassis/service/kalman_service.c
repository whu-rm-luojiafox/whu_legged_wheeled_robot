/**
 * (C) WHU_BAlANCE_FILTER
 * 车体速度 KF：x=[速度, 加速度等效零偏]，轮速量测 z=v。
 * 接地、反馈超时、重力投影和回退策略由 chassis_observer 负责。
 */
#include "kalman_service.h"
#include "kalman_filter.h"
#include <math.h>

/* 初始试验值：连续过程噪声强度，以及速度量测方差。 */
#define VELOCITY_KF_ACCEL_NOISE 0.04f
#define VELOCITY_KF_BIAS_NOISE 1e-5f
#define VELOCITY_KF_WHEEL_VARIANCE 0.02f

static KalmanFilter_t velocity_kf;
static chassis_velocity_kf_result_t velocity_result;
static uint8_t storage_initialized;

uint8_t ChassisVelocityKF_Init(float initial_speed_m_s)
{
    velocity_result.valid = 0u;
    if (!isfinite(initial_speed_m_s)) return 0u;

    /* 通用 KF 的 Init 会分配矩阵，只允许执行一次；后续只重置状态。 */
    if (!storage_initialized)
    {
        Kalman_Filter_Init(&velocity_kf, 2, 1, 1);
        storage_initialized = 1u;
    }

    velocity_kf.UseAutoAdjustment = 0u; /* 零速也是量测，不能被自动剔除。 */
    velocity_kf.H_data[0] = 1.0f;
    velocity_kf.H_data[1] = 0.0f;
    velocity_kf.R_data[0] = VELOCITY_KF_WHEEL_VARIANCE;
    velocity_kf.xhat_data[0] = initial_speed_m_s;
    velocity_kf.xhat_data[1] = 0.0f;
    velocity_kf.P_data[0] = 0.25f;
    velocity_kf.P_data[1] = velocity_kf.P_data[2] = 0.0f;
    velocity_kf.P_data[3] = 0.01f;
    velocity_result.speed_m_s = initial_speed_m_s;
    velocity_result.accel_bias_m_s2 = 0.0f;
    return 1u;
}

uint8_t ChassisVelocityKF_Update(float accel_forward_m_s2, float wheel_velocity_m_s, float dt_s,
                                 uint8_t wheel_valid)
{
    velocity_result.valid = 0u;
    if (!storage_initialized || !wheel_valid || !isfinite(accel_forward_m_s2) ||
        !isfinite(wheel_velocity_m_s) || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f)
        return 0u;

    /* x=[v,b], v(k+1)=v(k)+(a-b)*dt，H=[1,0]。 */
    velocity_kf.F_data[0] = 1.0f;
    velocity_kf.F_data[1] = -dt_s;
    velocity_kf.F_data[2] = 0.0f;
    velocity_kf.F_data[3] = 1.0f;
    velocity_kf.B_data[0] = dt_s;
    velocity_kf.B_data[1] = 0.0f;

    const float dt2 = dt_s * dt_s;
    velocity_kf.Q_data[0] = VELOCITY_KF_ACCEL_NOISE * dt_s + VELOCITY_KF_BIAS_NOISE * dt2 * dt_s / 3.0f;
    velocity_kf.Q_data[1] = -VELOCITY_KF_BIAS_NOISE * dt2 / 2.0f;
    velocity_kf.Q_data[2] = velocity_kf.Q_data[1];
    velocity_kf.Q_data[3] = VELOCITY_KF_BIAS_NOISE * dt_s;

    velocity_kf.ControlVector[0] = accel_forward_m_s2;
    velocity_kf.MeasuredVector[0] = wheel_velocity_m_s;
    Kalman_Filter_Update(&velocity_kf);

    if (velocity_kf.MatStatus != ARM_MATH_SUCCESS || !isfinite(velocity_kf.xhat_data[0]) ||
        !isfinite(velocity_kf.xhat_data[1]))
        return 0u;
    velocity_result.speed_m_s = velocity_kf.xhat_data[0];
    velocity_result.accel_bias_m_s2 = velocity_kf.xhat_data[1];
    velocity_result.valid = 1u;
    return 1u;
}

const chassis_velocity_kf_result_t *ChassisVelocityKF_Get(void)
{
    return &velocity_result;
}
