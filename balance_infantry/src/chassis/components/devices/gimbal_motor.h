#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H
#include "motor_protocol.h"
#include "pid.h"
#include "user_lib.h"

typedef enum
{
    GIMBAL_MOTOR_OFF = 0, // 电机原始值控制
    GIMBAL_MOTOR_GYRO,    // 电机陀螺仪角度控制
} gimbal_motor_mode_e;
typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_angle1_pid;
    pid_type_def gimbal_motor_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 relative_angle, relative_limit, relative_angle_init;
    fp32 relative_angle_set;
    fp32 motor_gyro, last_motor_gyro; //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 current_set;
    fp32 yaw_given_current;
    first_order_filter_type_t *yaw_angle_set;
} gimbal_motor_t;

#endif
