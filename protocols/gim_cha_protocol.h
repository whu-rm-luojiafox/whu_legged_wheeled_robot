#ifndef _GIM_CHA_PROTOCOL_H
#define _GIM_CHA_PROTOCOL_H

#include "struct_typedef.h"

typedef enum
{
  CHASSIS_MODE_OFF=0,
  CHASSIS_MOVE_ON,
  CHASSIS_MODE_DEBUG,
  CHASSIS_MODE_INIT,
}RC_chassis_mode_e;

typedef enum
{
    SHOOT_STOP = 0,   
    SHOOT_READY_FRIC,  
    SHOOT_READY_BULLET,

    SHOOT_SINGLE,
    SHOOT_CONTINUE,
    SHOOT_READY,       
    SHOOT_BULLET,      
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,            
} shoot_mode_e;

typedef struct
{
	float vx_set;//底盘x轴方向设定的速度控制量；
	float vy_set;//底盘y轴方向设定的速度控制量
	float wz_set;//底盘自旋时 设定的速度控制量；
  float high_set;//高度控制量
	float yaw_angle_set;//设置yaw轴角度
	float yaw_angle;//yaw轴实时角度
	float yaw_gyro;//yaw轴角速度
  float pitch_angle;//pitch轴实时角度
	RC_chassis_mode_e chassis_mode;//底盘模式
	shoot_mode_e shoot_mode_rc;//射击模式
  uint8_t jump_flag,sit_flag,high_flag,fric_flag,auto_flag,ui_init_flag,reset_flag;
  float fric_speed_set;
}chassis_data_t;	

#endif // !