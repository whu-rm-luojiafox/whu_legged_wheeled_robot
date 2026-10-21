/**
  ****************************(C) WHU_BALANCE_GIMBAL****************************
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
  *  V2.0.0     Nov-16-2025     xzicr           增加控制模块
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C)WHU_BALANCE_GIMBAL****************************
  */

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"
#include "Self_aim.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "UART_task.h"
#include "gimbal_command.h"

 
#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;

#endif




/*--------------云台控制所有相关数据----------------*/
// 云台数据结构体
gimbal_control_t gimbal_control;

//标志位

// PID参数
static const fp32 Pitch_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};
static const fp32 Pitch_gyro_pid[3] = {PITCH_GYRO_PID_KP, PITCH_GYRO_PID_KI, PITCH_ANGLE_PID_KD};

/*------------底盘数据------------------*/
chassis_data_t chassis_data;
InputData *Self_aim_data;


// func
static void gimbal_init(gimbal_control_t *init);

static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);

/* Execution order is intentionally preserved, including the two historical
 * shoot_control_loop calls. Changing their count requires shooting regression. */
void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);

  // 云台初始化
  gimbal_init(&gimbal_control);

  // 腿部控制初始化
  gimbal_command_init(&chassis_data);

  // 射击初始化
  shoot_Init();

  while (1)
  {
    // 射击控制
    gimbal_control.shoot = shoot_control_loop();

    // 设置底盘控制量
    gimbal_command_update(&gimbal_control, &chassis_data);

    // 设置云台控制模式
    gimbal_set_mode(&gimbal_control);

    // 云台数据反馈
    gimbal_feedback_update(&gimbal_control);

    // 设置云台PITCH轴目标角度
    gimbal_set_control(&gimbal_control);

    // 云台控制PID计算
    gimbal_control_loop(&gimbal_control);

    gimbal_control.shoot = shoot_control_loop();

    CAN_cmd_gimbal(0, -gimbal_control.gimbal_pitch_motor.given_current, 0, 0);

    CAN_cmd_chassis(gimbal_control.shoot->shoot_left_given_current, gimbal_control.shoot->shoot_right_given_current, 0, 0);
    vTaskDelay(GIMBAL_CONTROL_TIME);
  }
}

static void gimbal_init(gimbal_control_t *init)
{
  // 初始化陀螺仪 遥控器  自瞄数据
  init->INS = get_INS();
  init->gimbal_INT_angle_point = get_INS_angle_point();
  init->gimbal_INT_gyro_point = get_gyro_data_point();
  init->gimbal_rc_ctrl = get_remote_control_point();
  Self_aim_data = get_selfaim_data();

  // 模式初始化
  init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_OFF;
  chassis_data.shoot_mode_rc = 0;
  chassis_data.chassis_mode = 0;
  init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
  init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
  chassis_data.yaw_angle_set = init->gimbal_yaw_motor.absolute_angle;

  // PID初始化
  PID_init(&init->gimbal_pitch_motor.gimbal_motor_angle_pid, PID_POSITION, Pitch_angle_pid,
           PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);
  PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_gyro_pid,
           PITCH_GYRO_PID_MAX_OUT, PITCH_GYRO_PID_MAX_IOUT);
  gimbal_feedback_update(init);
}

static void gimbal_set_mode(gimbal_control_t *set_mode)
{
  if (set_mode == NULL)
  {
    return;
  }

  if (set_mode->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_INIT)
  {
    static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    if (fabs(set_mode->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
    {
      if (init_stop_time < GIMBAL_INIT_STOP_TIME)
      {
        init_stop_time++;
      }
    }
    else
    {
      if (init_time < GIMBAL_INIT_TIME)
      {
        init_time++;
      }
    }

    // 超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
    if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
        !switch_is_down(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && !toe_is_error(DBUS_TOE))
    {
      return;
    }
    else
    {
      init_stop_time = 0;
      init_time = 0;
    }
  }
  if ((set_mode->gimbal_rc_ctrl->rc.s[0] == 0 || toe_is_error(DBUS_TOE)))
  {
    set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_OFF;
  }
  else if ((set_mode->gimbal_rc_ctrl->rc.s[0] == 1 || set_mode->gimbal_rc_ctrl->rc.s[1] == 2) )
  {
    if(gimbal_command_enabled())
    {
      set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else
    {
      set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_OFF;
    }
  }

  // 判断进入init状态机
  if (set_mode->gimbal_pitch_motor.last_gimbal_motor_mode == GIMBAL_MOTOR_OFF && set_mode->gimbal_pitch_motor.gimbal_motor_mode != GIMBAL_MOTOR_OFF)
  {
    set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_INIT;
  }
  set_mode->gimbal_pitch_motor.last_gimbal_motor_mode = set_mode->gimbal_pitch_motor.gimbal_motor_mode;
}

static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
  if (feedback_update == NULL)
  {
    return;
  }
  // 云台数据更新
  feedback_update->gimbal_pitch_motor.relative_angle = 0;//feedback_update->gimbal_pitch_motor.gimbal_motor_measure->last_ecd;
  feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
  feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
  feedback_update->gimbal_yaw_motor.relative_angle = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->last_ecd;
  feedback_update->gimbal_yaw_motor.absolute_angle = INS.YawTotalAngle;
  feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - 
  arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
  // 键鼠数据获取
  feedback_update->last_press_l = feedback_update->press_l;
  feedback_update->press_l = feedback_update->gimbal_rc_ctrl->mouse.press_l;
  feedback_update->last_press_r = feedback_update->press_r;
  feedback_update->press_r = feedback_update->gimbal_rc_ctrl->mouse.press_r;
  feedback_update->lastkeyboard = feedback_update->keyboard;
  feedback_update->keyboard = feedback_update->gimbal_rc_ctrl->key.v;
  feedback_update->aim_last_press = feedback_update->aim_press;
  feedback_update->aim_press = feedback_update->gimbal_rc_ctrl->rc.s[1];
  // 自瞄数据获取
  feedback_update->gimbal_pitch_motor.last_self_aim_pitch_angle = feedback_update->gimbal_pitch_motor.self_aim_pitch_angle;
  feedback_update->gimbal_pitch_motor.self_aim_pitch_angle = Self_aim_data->pitch / PI * 180;
  feedback_update->gimbal_yaw_motor.last_self_aim_yaw_angle = feedback_update->gimbal_yaw_motor.self_aim_yaw_angle;
  feedback_update->gimbal_yaw_motor.self_aim_yaw_angle = Self_aim_data->yaw / PI * 180;
  //模式更新

}
static void gimbal_set_control(gimbal_control_t *set_control)
{
  #if GIMBAL_RC_INPUT_ENABLED
  static int16_t pitch_channel = 0;
  rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
  #endif
  if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO && gimbal_command_auto_aim_enabled() == 1)
  {
    if(set_control->gimbal_pitch_motor.self_aim_pitch_angle==0)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set += pitch_channel * PITCH_RC_SEN + set_control->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
    }
    else
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = set_control->gimbal_pitch_motor.self_aim_pitch_angle;
    }
    if (set_control->gimbal_pitch_motor.absolute_angle_set < -26)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = -26;
    }
    else if (set_control->gimbal_pitch_motor.absolute_angle_set > 15)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = 15;
    }
    else
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = set_control->gimbal_pitch_motor.absolute_angle_set;
    }
  }
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO && gimbal_command_auto_aim_enabled() == 0)
  {
    set_control->gimbal_pitch_motor.absolute_angle_set += pitch_channel * PITCH_RC_SEN + set_control->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
    if (set_control->gimbal_pitch_motor.absolute_angle_set < -26)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = -26;
    }
    else if (set_control->gimbal_pitch_motor.absolute_angle_set > 15)  
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = 15;
    }
    else
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = set_control->gimbal_pitch_motor.absolute_angle_set;
    }
  }
  /* 初始化云台pitch轴角度 */
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_INIT)
  {
    set_control->gimbal_pitch_motor.absolute_angle_set = INIT_PITCH_SET;
  }
}
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
  if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_OFF)
  {
    control_loop->gimbal_pitch_motor.given_current = 0;
    PID_clear(&control_loop->gimbal_pitch_motor.gimbal_motor_angle_pid);
    PID_clear(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid);
  }
  else if (control_loop->gimbal_pitch_motor.absolute_angle > 90.0f || control_loop->gimbal_pitch_motor.absolute_angle < -90.0f)
  {
    control_loop->gimbal_pitch_motor.given_current = 0;
    PID_clear(&control_loop->gimbal_pitch_motor.gimbal_motor_angle_pid);
    PID_clear(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid);
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO || control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_INIT)
  {
    PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_angle_pid, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.absolute_angle_set);
    PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, control_loop->gimbal_pitch_motor.motor_gyro, gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid.out);
    control_loop->gimbal_pitch_motor.given_current = (int16_t)(control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid.out);
  }
}


chassis_data_t *get_chassis_data_point()
{
  return &chassis_data;
}
gimbal_control_t *get_gimbal_data()
{
  return &gimbal_control;
}
