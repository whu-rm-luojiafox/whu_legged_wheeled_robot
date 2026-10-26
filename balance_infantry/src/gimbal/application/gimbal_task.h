/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot.h"
#include "INS_task.h"
#include "gimbal_config.h"
#include "gimbal_fwd.h"

typedef enum
{
    GIMBAL_MOTOR_OFF = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_DEBUG,  //专门调试云台，底盘电机失能
    GIMBAL_INIT,          //电机初始化
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
  motor_measure_t *gimbal_motor_measure;
  pid_type_def gimbal_motor_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  fp32 max_relative_angle; // rad
  fp32 min_relative_angle; // rad

  fp32 relative_angle;     // rad
  fp32 relative_angle_set; // rad
  fp32 absolute_angle;     // rad
  fp32 absolute_angle_set; // rad
  fp32 motor_gyro;         // rad/s
  fp32 motor_gyro_set;
  fp32 motor_speed;
  fp32 raw_cmd_current;
  fp32 current_set;
  int16_t given_current;
  float self_aim_pitch_angle;
  float last_self_aim_pitch_angle;
  float self_aim_yaw_angle;
  float last_self_aim_yaw_angle;

} gimbal_motor_t;


struct gimbal_control
{
  const RC_ctrl_t *gimbal_rc_ctrl;
  const fp32 *gimbal_INT_angle_point;
  const fp32 *gimbal_INT_gyro_point;
  const INS_t *INS;
  gimbal_motor_t gimbal_pitch_motor;
  gimbal_motor_t gimbal_yaw_motor;
  const shoot_control_t *shoot;
  bool_t press_l;
  bool_t last_press_l;
  bool_t last_press_r;
  bool_t press_r;
  uint16_t keyboard;
  uint16_t lastkeyboard;
  uint8_t aim_press,aim_last_press;

  float vx_set;
};



/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);
/**
  * @brief          返回底盘控制数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);
/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

extern void gimbal_task(void const *pvParameters);
/**
  * @brief          返回底盘数据指针供CAN_task任务使用
  * @param[out]     none
  * @retval         none
  */
extern chassis_data_t *get_chassis_data_point();
extern gimbal_control_t *get_gimbal_data();


#endif
