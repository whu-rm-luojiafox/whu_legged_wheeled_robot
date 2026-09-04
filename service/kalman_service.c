/**
  ****************************(C) WHU_BAlANCE_FILTER***************************
****************************
  * @file       kalman_service.c
  * @brief      滤波函数封装
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2025     xzicr              1. 轮腿滤波封装函数
  *
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) WHU_BAlANCE_FILTER***************************
  */

#include "kalman_service.h"
#include "can_service.h"


KalmanFilter_t WheelSpeed_KF[WHEEL_NUM];

static void WheelSpeedKF_Init_Instance(KalmanFilter_t *kf, float dt, float Kt_over_J)
{
    Kalman_Filter_Init(kf, 2, 1, 1);
    // 零时刻的状态协方差矩阵P初始化为一个对角矩阵，表示初始状态的不确定性
    float P_Init[4] = {
        5.0f, 0,        // 速度状态的不确定性
        0, 50.0f,       // 加速度状态的不确定性
    };
    // 状态转移矩阵
    float F_Init[4] = {
        1, dt,   //控制周期dt
        0, 1,
    }; 
    //控制矩阵
    float B_Init[2] = {
        0.0f,             // 控制量对速度的影响
        Kt_over_J * dt,            // 控制量对加速度的影响（具体值需要标定）
    };
    // 过程噪声协方差矩阵Q初始化为一个对角矩阵，表示过程噪声的不确定性
    float Q_Init[4] = {
        1.0f, 0,
        0, 50.0f,
    };
    //观测矩阵
    // 测量映射：第1个观测量对应状态向量中的第1个变量（速度）
    uint8_t measurement_reference[1] = {1};   

    // 观测系数：速度的观测增益为 1（直接测量）
    float measurement_degree[1] = {1.0f};  

    // 观测噪声协方差矩阵 R 的对角线元素（编码器测速方差）
    // 编码器在静止时抖动约 ±0.05m/s，方差设为 0.5 比较合适
    float mat_R_diagonal_elements[1] = {0.5f}; 

    // 防止滤波器过度自信发散，保证最小协方差下限
    float state_min_variance[2] = {0.01f, 0.5f}; 

    // 开启自动调整（启用你的 StateMinVariance 机制）
    kf->UseAutoAdjustment = 1;

    // 拷贝矩阵数据
    memcpy(kf->P_data, P_Init, sizeof(P_Init));
    memcpy(kf->F_data, F_Init, sizeof(F_Init));
    memcpy(kf->B_data, B_Init, sizeof(B_Init));   
    memcpy(kf->Q_data, Q_Init, sizeof(Q_Init));
    memcpy(kf->MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(kf->MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(kf->MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(kf->StateMinVariance, state_min_variance, sizeof(state_min_variance));
}

static void WheelSpeedKF_Update_Instance(KalmanFilter_t *kf,float torque,uint8_t index)
{
    // 设置测量值和控制输入
    kf->MeasuredVector[0] = get_wheel_velocity_point(index);
    kf->ControlVector[0] =  torque;
    // 执行卡尔曼滤波更新
    Kalman_Filter_Update(kf);
}

void WheelSpeedKF_Init(void)
{
    WheelSpeedKF_Init_Instance(&WheelSpeed_KF[WHEEL_LEFT], 0.002f, 10.0f);//参数待定
    WheelSpeedKF_Init_Instance(&WheelSpeed_KF[WHEEL_RIGHT], 0.002f, 10.0f);
}

void WheelSpeedKF_Update(float torque1,float torque2)
{
    WheelSpeedKF_Update_Instance(&WheelSpeed_KF[WHEEL_LEFT],torque1,WHEEL_LEFT);
    WheelSpeedKF_Update_Instance(&WheelSpeed_KF[WHEEL_RIGHT],torque2,WHEEL_RIGHT);
}
float *WheelSpeedKF_Get_point(uint8_t index)
{
    return WheelSpeed_KF[index].xhat_data;
}
