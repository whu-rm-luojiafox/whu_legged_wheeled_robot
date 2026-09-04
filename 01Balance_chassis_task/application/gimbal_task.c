#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "arm_math.h"
#include "user_lib.h"
#include "math.h"
#include "chassis_task.h"
#include "uart_receive.h"

gimbal_control_t gimbal_control;
extern chassis_move_t chassis_move;
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
	
    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

 fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, fp32 setgryo)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
	
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
	pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

void gimbal_Init(gimbal_control_t *gimbal_control)
{
	float yaw_angle_pid[3] ={YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD};
	float yaw_gyro_pid[3] ={YAW_GYRO_PID_KP,YAW_GYRO_PID_KI, YAW_GYRO_PID_KD};
	gimbal_control->yaw_ctrl_data=get_Uart_Chassisdata_point();
	gimbal_control->gimbal_yaw_motor.gimbal_motor_measure=get_yaw_gimbal_motor_measure_point();
	gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->gimbal_yaw_motor.absolute_angle;
	gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	gimbal_control->gimbal_yaw_motor.motor_gyro = 0;
	PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid,PID_POSITION,yaw_angle_pid,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,PID_POSITION,yaw_gyro_pid,YAW_GYRO_PID_MAX_OUT,YAW_GYRO_PID_MAX_IOUT);
}
void gimbal_set_mode(gimbal_control_t *gimbal_control)
{
	if(gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_OFF)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	}
	else if(gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MOVE_ON||gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_DEBUG)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	if(gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_INIT)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_INIT;
	}
}
void gimbal_feedback_update(gimbal_control_t *gimbal_control)
{
	gimbal_control->gimbal_yaw_motor.absolute_angle=gimbal_control->yaw_ctrl_data->yaw_angle;
	gimbal_control->gimbal_yaw_motor.motor_gyro=gimbal_control->yaw_ctrl_data->yaw_gyro;
	gimbal_control->gimbal_yaw_motor.motor_gyro=0.95*gimbal_control->gimbal_yaw_motor.motor_gyro+0.05*gimbal_control->gimbal_yaw_motor.last_motor_gyro;
	gimbal_control->gimbal_yaw_motor.last_motor_gyro=gimbal_control->gimbal_yaw_motor.motor_gyro;
	gimbal_control->gimbal_yaw_motor.relative_angle= gimbal_control->gimbal_yaw_motor.gimbal_motor_measure->angle; 
}
void gimbal_set_control(gimbal_control_t *gimbal_control)
{
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->gimbal_yaw_motor.absolute_angle;
	}
	else  if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->yaw_ctrl_data->yaw_angle_set;
	}
	else  if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_INIT)
	{
	}	
}
void gimbal_control_loop(gimbal_control_t *gimbal_control,chassis_move_t *chassis_move)
{
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
		gimbal_control->gimbal_yaw_motor.yaw_given_current=0;
		gimbal_control->gimbal_yaw_motor.motor_gyro = 0;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO )
	{

		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set);
		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid.out);//-0.08*chassis_move->chassis_posture_info.yaw_gyro);
		gimbal_control->gimbal_yaw_motor.yaw_given_current= gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_INIT)
	{
	}
}
void gimbal_task(void const *pvParameters)
{
	gimbal_Init(&gimbal_control);
	shoot_init();
	while(1)
	{
		gimbal_set_mode(&gimbal_control);
		gimbal_feedback_update(&gimbal_control);
		gimbal_set_control(&gimbal_control);
		gimbal_control_loop(&gimbal_control,&chassis_move);
		gimbal_control.shoot=shoot();
		if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
		{ 
			CAN_cmd_gimbal(0,0,0,0);
		}
		else
		{
			CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.yaw_given_current,0,gimbal_control.shoot->given_current,0);
		}
		vTaskDelay(GIMBAL_CONTROL_TIME);
	}
}
