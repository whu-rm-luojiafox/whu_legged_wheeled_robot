#ifndef SHOOT_H
#define SHOOT_H
#include "gim_cha_protocol.h"
#include "motor_protocol.h"
#include "can_service.h"
#include "pid.h"
#define MOTOR_ECD_TO_ANGLE          0.00007666
#define FULL_COUNT                  9.5
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
#define TRIGGER_SPEED_PID_KP        25.0f//25
#define TRIGGER_SPEED_PID_KI        0.15f//0.15
#define TRIGGER_SPEED_PID_KD        3.0f
#define TRIGGER_BULLET_PID_MAX_OUT  16000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f
#define TRIGGER_ANGLE_PID_KP        4.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        0.50f
#define TRIGGER_ANGLE_PID_MAX_OUT  12000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 9000.0f

#define PI_TEN    1620


//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f


#define BLOCK_TIME                  700
#define REVERSE_TIME                1000
//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         2500.0f
//拨弹速度
#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      10.0f

/* -----------拨盘电机发射状态------------ */
#define SHOOT_START_SINGLE   1
#define SHOOT_START_CONTINUE   2
#define SHOOT_FINISH    3

typedef struct
{
	shoot_mode_e shoot_mode,shoot_last_mode	;
	const motor_measure_t *shoot_motor_measure;
	const chassis_data_t *shoot_control_data;
	/* 分为速度环和位置环两种模式 */
	pid_type_def trigger_position_mode_speed_pid;
	pid_type_def trigger_speed_mode_speed_pid;
	pid_type_def trigger_motor_angle_pid;
	fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;		//单位rpm
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int32_t ecd_count;
	bool_t last_press_l;
	bool_t press_l;
	uint8_t shoot_friction_mode;
	uint8_t shooter_output;
    uint16_t block_time;
    uint16_t reverse_time;
	uint16_t shoot_heat_limit;
	uint16_t shoot_heat_value;
	uint16_t shoot_cooling_rate;
	uint16_t last_shoot_heat_value;	
}shoot_control_t;
extern shoot_control_t *shoot();
extern void shoot_init();
#endif
