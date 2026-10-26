/* RC, keyboard edges and yaw requests form one behaviour module. It produces
 * board-link references; it does not send CAN or run the pitch controller. */
#include "gimbal_command.h"
#include "gimbal_task.h"
#include "Self_aim.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include <math.h>

static uint8_t aimflag;
static uint8_t key_mode_flag = 1;
static const InputData *command_aim_data;
static void rc_control(gimbal_control_t *control, chassis_data_t *command);
static void key_control(gimbal_control_t *control, chassis_data_t *command);
static void yaw_set(gimbal_control_t *control, chassis_data_t *command);

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

void gimbal_command_init(chassis_data_t *leg_contorl)
{
  command_aim_data = get_selfaim_data();
  leg_contorl->high_set = 0.19f;
}
void gimbal_command_update(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
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
static void rc_control(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  // 模式设置
  #if GIMBAL_RC_INPUT_ENABLED
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
  
  chassis_data->vx_set = fp32_constrain(vx_set_channel,-2.0,3.0);
  chassis_data->vy_set = fp32_constrain(vy_set_channel,-2.0,3.0);
  chassis_data->wz_set = -CHASSIS_WZ_RC_SEN * gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL];
  #endif 

}

/**
 * @brief 键盘控制输入
 * @param[in] gimbal_control_set
 * @param[in] chassis_data
 * @retval
 * @note
 */
static void key_control(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  #if GIMBAL_KEY_INPUT_ENABLED

  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_W)
  {
    chassis_data->vx_set = 2.2;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_S)
  {
    chassis_data->vx_set = -2.2;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_A)
  {
    chassis_data->vy_set = 2.2;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_D)
  {
    chassis_data->vy_set = -2.2;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_Q )
  {
    chassis_data->wz_set = 8;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_E )
  {
    chassis_data->wz_set = -8;
  }
  
  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_W && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vx_set = 2.6;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_S && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vx_set = -2.6;
  }
  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_A && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vy_set = 2.6;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_D && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vy_set = -2.6;
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
  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_G)
  {
    chassis_data->jump_flag = 1;
  }
  else
  {
    chassis_data->jump_flag = 0;
  }
  
  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_Z)
  {
    chassis_data->high_set = 0.15f;
  }

  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_X)
  {
    chassis_data->high_set += 0.00015f;
    fp32_constrain(chassis_data->high_set, 0.13, 0.40);
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_C)
  {
    chassis_data->high_set -= 0.00015f;
    fp32_constrain(chassis_data->high_set, 0.13, 0.40);
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
  chassis_data->shoot_mode_rc = shoot_control.shoot_mode;
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
  /* Hold R to request recovery. The chassis checks posture and key edge. */
  chassis_data->recovery_flag =
  ((gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_R || gimbal_control_set->gimbal_rc_ctrl->rc.s[2] == 1)
  != 0u && chassis_data->chassis_mode == CHASSIS_MODE_ON);
  #else
  chassis_data->recovery_flag = 0u;
  #endif
}

static void yaw_set(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
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
  #if GIMBAL_RC_INPUT_ENABLED
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  #endif
  if ((chassis_data->chassis_mode != CHASSIS_MODE_OFF ) && aimflag == 0 )
  {
    chassis_data->yaw_angle_set -= yaw_channel * YAW_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
  }
else if ((chassis_data->chassis_mode != CHASSIS_MODE_OFF) && aimflag == 1)
{
    float new_yaw_angle;
    if(gimbal_control_set->gimbal_yaw_motor.self_aim_yaw_angle == 0 || command_aim_data->mode == 0)
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



uint8_t gimbal_command_enabled(void) { return key_mode_flag == 1u; }
uint8_t gimbal_command_auto_aim_enabled(void) { return aimflag; }
