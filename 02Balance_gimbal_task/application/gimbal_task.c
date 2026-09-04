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

#define RC_MODE 1
#define KEY_MODE 1
 
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

#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;

#endif

#define WINDOW_SIZE 10
float yaw_buffer[WINDOW_SIZE];


/*--------------云台控制所有相关数据----------------*/
// 云台数据结构体
gimbal_control_t gimbal_control;

//标志位
uint8_t aimflag = 0;
uint8_t key_mode_flag = 1;
// PID参数
static const fp32 Pitch_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};
static const fp32 Pitch_gyro_pid[3] = {PITCH_GYRO_PID_KP, PITCH_GYRO_PID_KI, PITCH_ANGLE_PID_KD};

/*------------底盘数据------------------*/
chassis_data_t chassis_data;
InputData *Self_aim_data;
first_order_filter_type_t chassis_self_aim_yaw;
const static fp32 chassis_self_aim_yaw_filter[1] = {CHASSIS_ACCEL_Y_NUM};

// func
static void gimbal_init(gimbal_control_t *init);
void leg_control_init(chassis_data_t *leg_contorl);
void chassis_rc_to_control_vector(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data);
void rc_control(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data);
void key_control(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data);
void yaw_set(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data);

static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);

void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);

  // 云台初始化
  gimbal_init(&gimbal_control);

  // 腿部控制初始化
  leg_control_init(&chassis_data);

  // 射击初始化
  shoot_Init();

  while (1)
  {
    // 射击控制
    gimbal_control.shoot = shoot_control_loop();

    // 设置底盘控制量
    chassis_rc_to_control_vector(&gimbal_control, &chassis_data);

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
  chassis_data.shoot_mode = 0;
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

void leg_control_init(chassis_data_t *leg_contorl)
{
  leg_contorl->high_set = 0.19f;
}
void chassis_rc_to_control_vector(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  /* --------------进入函数前提条件------------------ */
  if (gimbal_control_set == NULL)
  {
    return;
  }
  // 遥控控制
  rc_control(gimbal_control_set, chassis_data);

  // 键盘控制
  key_control(gimbal_control_set, chassis_data);

  // yaw轴设置更新
  yaw_set(gimbal_control_set, chassis_data);

  chassis_data->yaw_angle = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
  chassis_data->yaw_gyro = gimbal_control_set->gimbal_yaw_motor.motor_gyro;
  chassis_data->pitch_angle = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
}
void rc_control(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  // 模式设置
  #ifdef RC_MODE
  if (gimbal_control_set->gimbal_rc_ctrl->rc.s[0] == 0)
  {
    chassis_data->chassis_mode = CHASSIS_MODE_OFF;
  }
  else if (gimbal_control_set->gimbal_rc_ctrl->rc.s[0] == 1)
  {
    if(key_mode_flag == 1)
    {
      chassis_data->chassis_mode = CHASSIS_MODE_ON;
    }
    else
    {
      chassis_data->chassis_mode = CHASSIS_MODE_OFF;
    }
  }
  else if (gimbal_control_set->gimbal_rc_ctrl->rc.s[0] == 2)
  {
    chassis_data->chassis_mode = CHASSIS_MODE_DEBUG;
  }
  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
  
  chassis_data->vx_set = fp32_constrain(vx_set_channel,-2.0,2.0);
  chassis_data->vy_set = fp32_constrain(vy_set_channel,-2.0,2.0);
  chassis_data->wz_set = -CHASSIS_WZ_RC_SEN * gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL];
  #endif 

}
uint32_t timer;
/**
 * @brief 键盘控制输入
 * @param[in] gimbal_control_set
 * @param[in] chassis_data
 * @retval
 * @note
 */
void key_control(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  #if KEY_MODE
  timer++;
  if(timer<0xFFFFFFFF)
  {timer =0;}
  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_W)
  {
    chassis_data->vx_set = 1.82;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_S)
  {
    chassis_data->vx_set = -1.82;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_A)
  {
    chassis_data->wz_set = 10;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_D)
  {
    chassis_data->wz_set = -10;
  }

  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_W && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vx_set = 2.2;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_S && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vx_set = -2.2;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_A && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->wz_set = 13;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_D && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->wz_set = -13;
  }
    //增加键盘启停
  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_CTRL&&!(gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_CTRL))
  {
    key_mode_flag++;
    if(key_mode_flag > 1)
    {
      key_mode_flag = 0;
    }

  }
  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_G||gimbal_control_set->gimbal_rc_ctrl->rc.s[2] == 1)
  {
    chassis_data->jump_flag = 1;
  }
  else
  {
    chassis_data->jump_flag = 0;
  }
  
  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_C)
  {
    chassis_data->high_set = 0.15f;
  }

  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_X)
  {
    chassis_data->high_set += 0.0002f;
    fp32_constrain(chassis_data->high_set, 0.12, 0.34);
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_Z)
  {
    chassis_data->high_set -= 0.0002f;
    fp32_constrain(chassis_data->high_set, 0.12, 0.34);
  }
  if ((gimbal_control_set->aim_press==1&& gimbal_control_set->aim_last_press==0) || 
  (gimbal_control_set->press_r == 1&&gimbal_control_set->last_press_r == 0)||
  (gimbal_control_set->press_r == 0&&gimbal_control_set->last_press_r == 1))
  {
    aimflag = !aimflag;
    if(aimflag)
    {
      chassis_data->auto_flag = 1;
    }
    else
    {
      chassis_data->auto_flag = 0;
    }
  }
  chassis_data->shoot_mode = shoot_control.shoot_mode;
  switch(shoot_control.shoot_mode)
  {
    case 1: 
    case 3:
    case 4:
    {
      chassis_data->fric_flag = 1;
      break;
    }
    default:
    {
      chassis_data->fric_flag = 0;
      break;
    }
  }
  chassis_data->fric_speed_set = shoot_control.friction_right_speed_set;
  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_V)
  {
    chassis_data->ui_init_flag = 1;
  }
  else
  
  {
    chassis_data->ui_init_flag = 0;
  }
  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_R&&!(gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_R))
  {
    chassis_data->reset_flag =!chassis_data->reset_flag ;
  }
  if(chassis_data->chassis_mode != CHASSIS_MODE_OFF)
  {
    chassis_data->reset_flag =0 ;
  }
  #endif
}

void yaw_set(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  
  int16_t yaw_channel = 0;
  if (chassis_data->chassis_mode == CHASSIS_MODE_OFF)
  {
    chassis_data->yaw_angle_set = chassis_data->yaw_angle;
    chassis_data->high_set = 0.19f;
  }
  if(chassis_data->chassis_mode == CHASSIS_MODE_DEBUG)
  {
    chassis_data->high_set = 0.19f;
  }
  #if RC_MODE
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  #endif
  if ((chassis_data->chassis_mode != CHASSIS_MODE_OFF ) && aimflag == 0 )
  {
    chassis_data->yaw_angle_set -= yaw_channel * YAW_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
  }
else if ((chassis_data->chassis_mode != CHASSIS_MODE_OFF) && aimflag == 1)
{
    float new_yaw_angle;
    if(gimbal_control_set->gimbal_yaw_motor.self_aim_yaw_angle == 0 || Self_aim_data->mode == 0)
    {
        chassis_data->yaw_angle_set -= yaw_channel * YAW_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    }
    else
    {
        new_yaw_angle = gimbal_control_set->gimbal_yaw_motor.self_aim_yaw_angle;
        
        // ? 新增：计算角度跳变并补偿
        float angle_diff = new_yaw_angle - chassis_data->yaw_angle_set;
        
        // 规范到 [-180, 180] 范围
        while (angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } 
        while (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
        // 累加补偿到 yaw_angle_set
        new_yaw_angle = chassis_data->yaw_angle_set + angle_diff;
        chassis_data->yaw_angle_set = new_yaw_angle;
    }
}

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
    if(key_mode_flag == 1)
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
  #if RC_MODE
  static int16_t pitch_channel = 0;
  rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
  #endif
  if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO && aimflag == 1)
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
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO && aimflag == 0)
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
