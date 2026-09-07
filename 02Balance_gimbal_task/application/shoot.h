#ifndef SHOOT_H
#define SHOOT_H
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

#define FRICTION_SPEED_PID_KP        7.0f
#define FRICTION_SPEED_PID_KI        0.0001f
#define FRICTION_SPEED_PID_KD        4.0f
#define FRICTION_SPEED_PID_MAX_OUT   30000.0f
#define FRICTION_SPEED_PID_MAX_IOUT  5000.0f

#define FRICTION_CURRENT_PID_KP        0.8f     //0.6
#define FRICTION_CURRENT_PID_KI        0.15f    //0.2
#define FRICTION_CURRENT_PID_KD        5.0f     //0.2
#define FRICTION_CURRENT_PID_MAX_OUT   30000.0f
#define FRICTION_CURRENT_PID_MAX_IOUT  10000.0f

//射击模式使用的遥控器开关通道
#define SHOOT_RC_MODE_CHANNEL       0

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_R
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_F

#define FRIC_SPEED_MAX 4000
#define FRIC_SPEED_MIN 0

//鼠标长按判断阈值
#define PRESS_LONG_TIME             400
//遥控器开关长按时间阈值
#define RC_S_LONG_TIME              800

typedef enum
{
    SHOOT_STOP = 0,   //停止射击
    SHOOT_READY_FRIC,    //摩擦轮准备就绪
    SHOOT_READY_BULLET,  //弹丸准备就绪


    SHOOT_SINGLE,
    SHOOT_CONTINUE,
    SHOOT_READY,         //射击准备就绪
    SHOOT_BULLET,        //正在射击
    SHOOT_CONTINUE_BULLET,   //继续射击
    SHOOT_DONE,            //射击完成
    SHOOT_BACK,
}shoot_mode_e;

/* 低通滤波器结构体 */
typedef struct
{
	fp32 output;
	fp32 alpha;
}low_pass_filter_t;

typedef struct
{
	shoot_mode_e shoot_mode;
	const motor_measure_t *friction_motor_measure[2];
	const RC_ctrl_t *shoot_rc;
    ramp_function_source_t fric_ramp;      //斜坡函数结构体
    /* 左轮 */
	pid_type_def friction_left_motor_speed_pid; 
    /* 右轮 */
	pid_type_def friction_right_motor_speed_pid; 
    /* 左轮 */
	pid_type_def friction_left_motor_current_pid; 
    /* 右轮 */
	pid_type_def friction_right_motor_current_pid; 
    /* 低通滤波器*/ 
    low_pass_filter_t left_speed_filter;
    low_pass_filter_t right_speed_filter;
    fp32 friction_left_last_speed;
	fp32 friction_right_last_speed;    
    fp32 friction_left_speed;
	fp32 friction_right_speed;
    fp32 friction_left_speed_set;
    fp32 friction_right_speed_set;
	int16_t shoot_left_given_current;
	int16_t shoot_right_given_current;
	bool_t press_l,last_press_l;
    bool_t press_r,last_press_r;
    uint16_t keyboard,last_keyboard;
    bool_t press_fric,last_press_fric;
    bool_t press_shoot,last_press_shoot;
    uint16_t press_l_time;
    uint16_t press_r_time;
	uint16_t rc_s_time;
}shoot_control_t;



const shoot_control_t *shoot_control_loop(void);
extern shoot_control_t shoot_control;
extern void shoot_Init(void);




#endif
