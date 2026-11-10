#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "main.h"
#include "cmsis_os.h"
#include "can_service.h"
#include "pid.h"
#include "shoot.h"
#include "user_lib.h"
// yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_ANGLE_PID_KP 0.35f
#define YAW_ANGLE_PID_KI 0.0f
#define YAW_ANGLE_PID_KD 0.95f
#define YAW_ANGLE_PID_MAX_OUT 30.0f
#define YAW_ANGLE_PID_MAX_IOUT 10.0f

#define YAW_GYRO_PID_KP 7201.0f
#define YAW_GYRO_PID_KI 10.0f
#define YAW_GYRO_PID_KD 0.0f
#define YAW_GYRO_PID_MAX_OUT 30000.0f
#define YAW_GYRO_PID_MAX_IOUT 9000.0f

#define GIMBAL_CONTROL_TIME 1
#define YAW_SET_NUM 0.2
#include "gimbal_motor.h"
typedef struct
{
    const chassis_data_t *yaw_ctrl_data;
    gimbal_motor_t gimbal_yaw_motor;
    shoot_control_t *shoot;

} gimbal_control_t;

extern void gimbal_task(void const *pvParameters);

#endif
