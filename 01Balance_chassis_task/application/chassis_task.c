

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "detect_task.h"
#include "INS_task.h"
#include "vofa.h"
#include "Chassis_power_control.h"
#include "LQR.h"
#include "usart.h"
#include "user_lib.h"
#include "uart_receive.h"

#define square(x) ((x) * (x))
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define LimitMax(input, max)     \
	{                            \
		if ((input) > max)       \
		{                        \
			input = max;         \
		}                        \
		else if ((input) < -max) \
		{                        \
			input = -max;        \
		}                        \
	}
#define rc_deadband_limit(input, output, dealine)        \
	{                                                    \
		if ((input) > (dealine) || (input) < -(dealine)) \
		{                                                \
			(output) = (input);                          \
		}                                                \
		else                                             \
		{                                                \
			(output) = 0;                                \
		}                                                \
	}

#define LimitOutput(input, min, max) \
	{                                \
		if ((input) < min)           \
			input = min;             \
		else if ((input) > max)      \
			input = max;             \
	}




#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

float a1,b1,c1,d1;
float a2,b2,c2,d2;
int temp;


//物理/车体 属性
const fp32 g = 9.8f;
const fp32 m_w = 1.19f;

// 底盘运动数据
chassis_move_t chassis_move;

//速度斜坡函数
ramp_function_source_t speed_ramp_vx, speed_ramp_vy, speed_ramp_wz;


fp32 suspend_LQR[2][6] = {
	20.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0, 0, 0, 0, 0, 0};
/* ------------------------PID info------------------------ */
fp32 roll_PD[2] = {1.0f, 10.0f};	 
fp32 coordinate_PD[2] = {10.0f, 1.0f}; // 10.0f,0.5f    //15.0f,1.0f
fp32 yaw_PD_test[2] = {20.0f, 180.0f};
fp32 jump_stand_PD_L[2] = {30000000.0f, 10.0f};
fp32 jump_stand_PD_R[2] = {30000000.0f, 10.0f};

fp32 jump_canstacting_PD_L[2] = {30000000.0f, 10.0f};
fp32 jump_canstacting_PD_R[2] = {30000000.0f, 10.0f};

fp32 suspend_stand_PD[2] = {220.0f, 10.0f};

/* ------------------------平步数据------------------------ */
fp32 delta;
float alpha_dx = 1.0f;
float alpha_dv = 1.0f;
float alpha_da = 0.7f;

fp32 IDEAL_PREPARING_STAND_JUMPING_ANGLE = 0.174532f;//0.0872f;
fp32 stablize_foot_speed_threshold = 1.2f, stablize_yaw_speed_threshold = 1.5f, rotate_move_threshold = 45.0f;
fp32 rotate_speed_list[11] = {0.0, 5.0, 5.3, 5.6, 6.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0};
fp32 move_scale_list[11] = {0.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
fp32 rotate_move_scale_list[11] = {0.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8};


fp32 rollP, rollD, rollI, roll_angle_deadband = 0.004f, roll_gyro_deadband = 0.005f, leg_dlength_deadband = 0.0f;
fp32  rc_angle_temp, temp_max_spd,normalized_speed, rotate_move_offset, delta_theta, delta_theta_temp, acc_step = 0.3f;
fp32 normal_move_scale = 1.0f;
fp32 SIT_HIGH = 0.12f;
/* A non-zero zero-command bias makes the chassis drive immediately after
 * enable. Keep feed-forward disabled until it is identified on a test stand. */
#define FORWARD_SPEED 0.0f

extern gimbal_control_t gimbal_control;

/* ----------------------function definition ------------------------ */
void Joint_Motor_to_Init_Pos(void);
void Motor_Zero_CMD_Send(void);
void HT_Motor_zero_set(void);
void Forward_kinematic_solution(chassis_move_t *feedback_update, fp32 Q1, fp32 S1, fp32 Q4, fp32 S4, uint8_t ce);
void calculate_wheel_vertical_acceleration(chassis_move_t * detect );
void Supportive_Force_Cal(chassis_move_t * detect);

void chassis_init(chassis_move_t *chassis_move_init);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void Chassis_Status_Detect(chassis_move_t *detect);
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
void Target_Value_Set(chassis_move_t *target_value_set);
void Chassis_Torque_Calculation(chassis_move_t *bl_ctrl);
void handle_airborne_state(chassis_move_t *bl_ctrl);
void Chassis_Torque_Combine(chassis_move_t *bl_ctrl);
void Motor_CMD_Send(chassis_move_t *CMD_Send);
uint8_t Check_Jump_Preparation_Complete(chassis_move_t *chassis);
float get_jacobian_element(chassis_move_t *VMCJ, float L0, float Q0, uint8_t element_type);



void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);

	//初始化电机 传感器数据获取
	chassis_init(&chassis_move);

	while (1)
	{
		//更新传感器 电机数据
		chassis_feedback_update(&chassis_move);

		//状态检测
		Chassis_Status_Detect(&chassis_move);

		//模式设置
		chassis_set_mode(&chassis_move);

		//模式切换控制
		chassis_mode_change_control_transit(&chassis_move);

		//目标值设置
		Target_Value_Set(&chassis_move);
		
		//功率限制
		chassis_power_limit(&chassis_move);

		//力矩输出计算
		Chassis_Torque_Calculation(&chassis_move);

		//虚拟腿映射关节电机力矩计算
		Chassis_Torque_Combine(&chassis_move);

		//发送计算结果
		Motor_CMD_Send(&chassis_move);	
		
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}

	/*----------------------- Set HT Zero Point ---------------------- */
	
	vTaskDelay(1600);
	HT_Motor_zero_set();
	Motor_Zero_CMD_Send();
	vTaskDelay(1);
	
	/* -------------------------param get---------------------------- */
	chassis_move_init->joint_motor_1.motor_measure = get_HT_motor_measure_point(0);
	chassis_move_init->joint_motor_2.motor_measure = get_HT_motor_measure_point(1);
	chassis_move_init->joint_motor_3.motor_measure = get_HT_motor_measure_point(2);
	chassis_move_init->joint_motor_4.motor_measure = get_HT_motor_measure_point(3);
	chassis_move_init->foot_motor_L.motor_measure = get_LK_motor_measure_point(0);
	chassis_move_init->foot_motor_R.motor_measure = get_LK_motor_measure_point(1);
	chassis_move_init->gimbal_yaw_motor.gimbal_motor_measure=get_yaw_gimbal_motor_measure_point();
	
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	chassis_move_init->chassis_INS_gyro = get_gyro_data_point();
	chassis_move_init->chassis_INS_accel = get_accel_data_point();
	chassis_move_init->chassis_data_ = get_Uart_Chassisdata_point();
	
	/* ----------------------------Mode set --------------------------- */
	chassis_move_init->mode.chassis_mode = chassis_move_init->mode.last_chassis_mode = DISABLE_CHASSIS;
	chassis_move_init->mode.chassis_balancing_mode = chassis_move_init->mode.last_chassis_balancing_mode = NO_FORCE;
	chassis_move_init->mode.sport_mode = chassis_move_init->mode.last_sport_mode = NORMAL_MOVING_MODE;
	chassis_move_init->joint_motor_1.motor_mode = chassis_move_init->joint_motor_1.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->joint_motor_2.motor_mode = chassis_move_init->joint_motor_2.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->joint_motor_3.motor_mode = chassis_move_init->joint_motor_3.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->joint_motor_4.motor_mode = chassis_move_init->joint_motor_4.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->foot_motor_L.motor_mode = chassis_move_init->foot_motor_L.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->foot_motor_R.motor_mode = chassis_move_init->foot_motor_R.last_motor_mode = MOTOR_NO_FORCE;
	
	/* ----------------------------------INIT HT/LK MOTOR-------------------------------- */
	chassis_move_init->joint_motor_1.position_offset = chassis_move_init->joint_motor_1.motor_measure->ecd;
	chassis_move_init->joint_motor_2.position_offset = chassis_move_init->joint_motor_2.motor_measure->ecd;
	chassis_move_init->joint_motor_3.position_offset = chassis_move_init->joint_motor_3.motor_measure->ecd;
	chassis_move_init->joint_motor_4.position_offset = chassis_move_init->joint_motor_4.motor_measure->ecd;
	chassis_move_init->joint_motor_1.position = (chassis_move_init->joint_motor_1.motor_measure->ecd - chassis_move_init->joint_motor_1.position_offset) + LEG_OFFSET;
	chassis_move_init->joint_motor_2.position = (chassis_move_init->joint_motor_2.motor_measure->ecd - chassis_move_init->joint_motor_2.position_offset) - LEG_OFFSET;
	chassis_move_init->joint_motor_3.position = (chassis_move_init->joint_motor_3.motor_measure->ecd - chassis_move_init->joint_motor_3.position_offset) - LEG_OFFSET;
	chassis_move_init->joint_motor_4.position = (chassis_move_init->joint_motor_4.motor_measure->ecd - chassis_move_init->joint_motor_4.position_offset) + LEG_OFFSET;
	chassis_move_init->foot_motor_L.distance_offset = (chassis_move_init->foot_motor_L.position / 360.0f) * WHEEL_PERIMETER;
	chassis_move_init->foot_motor_R.distance_offset = ((360.0f - chassis_move_init->foot_motor_R.position )/ 360.0f) * WHEEL_PERIMETER;
	// 初始化yaw角度PID
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	PID_init(&chassis_move_init->chassis_yaw_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

	// 初始化腿长PID
	const static fp32 leg_length_pid[3] = {LEG_SET_PID_KP, LEG_SET_PID_KI, LEG_SET_PID_KD};
	PID_init(&chassis_move_init->leg_L_length_pid, PID_POSITION, leg_length_pid, LEG_SET_PID_OUT, LEG_SET_PID_IOUT);
	PID_init(&chassis_move_init->leg_R_length_pid, PID_POSITION, leg_length_pid, LEG_SET_PID_OUT, LEG_SET_PID_IOUT);


	chassis_move_init->flag_info.init_flag = 1;
	chassis_feedback_update(chassis_move_init);
	chassis_move_init->flag_info.init_flag = 0;

	chassis_move_init->gimbal_yaw_motor.relative_angle_init =179.0f;
	// 初始化速度斜坡函数
	ramp_init(&speed_ramp_vx, 0.003f, 2.2f, -2.2f);
	ramp_init(&speed_ramp_vy, 0.003f, 2.2f, -2.2f);
	ramp_init(&speed_ramp_wz, 0.004f,15.0f, -15.0f);
		
	/* ----------------------------------VMC J-------------------------------- */
	//N11 系数
	chassis_move_init->InverseJacobianCoefficient.N11.c0 = 0.132f;//0.1226f;	  
	chassis_move_init->InverseJacobianCoefficient.N11.c1 = -1.885f;//-1.824f;	 
	chassis_move_init->InverseJacobianCoefficient.N11.c2 = 0.0761f;//-0.08976f; 
	chassis_move_init->InverseJacobianCoefficient.N11.c3 = 3.656f;//3.55f;	 
	chassis_move_init->InverseJacobianCoefficient.N11.c4 = -0.2059f;//-0.2468f;  
	chassis_move_init->InverseJacobianCoefficient.N11.c5 = 0.03211f;//0.0434;	  
	
	// N12 系数
	chassis_move_init->InverseJacobianCoefficient.N12.c0 = 0.06866f;//0.05869f;   
	chassis_move_init->InverseJacobianCoefficient.N12.c1 = 2.388f;//2.473f;	   
	chassis_move_init->InverseJacobianCoefficient.N12.c2 = -0.6333f;//-0.6447f; 
	chassis_move_init->InverseJacobianCoefficient.N12.c3 = -4.664f;//-4.797f;	   
	chassis_move_init->InverseJacobianCoefficient.N12.c4 = 2.333f;//2.438f;	   
	chassis_move_init->InverseJacobianCoefficient.N12.c5 = 0.01124f;//-0.003233f;
	
	// N21 系数
	chassis_move_init->InverseJacobianCoefficient.N21.c0 = 0.132f;// 0.1226f;	  
	chassis_move_init->InverseJacobianCoefficient.N21.c1 = -1.855f;//-1.824f;	 
	chassis_move_init->InverseJacobianCoefficient.N21.c2 = -0.0761f;//-0.08976f; 
	chassis_move_init->InverseJacobianCoefficient.N21.c3 = 3.656f;//3.55f;	 
	chassis_move_init->InverseJacobianCoefficient.N21.c4 = 0.2095f;//-0.2468f;  
	chassis_move_init->InverseJacobianCoefficient.N21.c5 = 0.03211f;//0.0434;	  
	
	// N22 系数
	chassis_move_init->InverseJacobianCoefficient.N22.c0 = -0.06866f;//-0.05869f; 
	chassis_move_init->InverseJacobianCoefficient.N22.c1 = -2.388f;//-2.473f;	   
	chassis_move_init->InverseJacobianCoefficient.N22.c2 = -0.6333f;//-0.6447f; 
	chassis_move_init->InverseJacobianCoefficient.N22.c3 = 4.664f;//4.797f;	   
	chassis_move_init->InverseJacobianCoefficient.N22.c4 = 2.333f;//2.438f;	   
	chassis_move_init->InverseJacobianCoefficient.N22.c5 = -0.01124f;//-0.003233f; 
}
void chassis_feedback_update(chassis_move_t *fdb)
{
	if (fdb == NULL)
	{
		return;
	}
	/*------------------------------- Update mode info ----------------------------*/
	fdb->mode.last_chassis_mode = fdb->mode.chassis_mode;
	fdb->mode.last_chassis_balancing_mode = fdb->mode.chassis_balancing_mode;
	fdb->mode.last_sport_mode = fdb->mode.sport_mode;
	fdb->mode.last_jumping_mode = fdb->mode.jumping_mode;
	fdb->mode.last_jumping_stage = fdb->mode.jumping_stage;
	fdb->mode.last_chassis_high_mode = fdb->mode.chassis_high_mode;

	fdb->flag_info.last_overpower_warning_flag = fdb->flag_info.overpower_warning_flag;
	fdb->flag_info.last_stablize_high_flag = fdb->flag_info.stablize_high_flag;

	fdb->flag_info.last_suspend_flag_L = fdb->flag_info.suspend_flag_L;
	fdb->flag_info.last_suspend_flag_R = fdb->flag_info.suspend_flag_R;

	fdb->joint_motor_1.last_motor_mode = fdb->joint_motor_1.motor_mode;
	fdb->joint_motor_2.last_motor_mode = fdb->joint_motor_2.motor_mode;
	fdb->joint_motor_3.last_motor_mode = fdb->joint_motor_3.motor_mode;
	fdb->joint_motor_4.last_motor_mode = fdb->joint_motor_4.motor_mode;
	fdb->foot_motor_L.last_motor_mode = fdb->foot_motor_L.motor_mode;
	fdb->foot_motor_R.last_motor_mode = fdb->foot_motor_R.motor_mode;

	/*------------------------------- Update HT Motor info ------------------------------ */
	fdb->joint_motor_1.position = (fdb->joint_motor_1.motor_measure->ecd - fdb->joint_motor_1.position_offset) +LEG_OFFSET;
	fdb->joint_motor_2.position = (fdb->joint_motor_2.motor_measure->ecd - fdb->joint_motor_2.position_offset) -LEG_OFFSET;
	fdb->joint_motor_3.position = (fdb->joint_motor_3.motor_measure->ecd - fdb->joint_motor_3.position_offset) - LEG_OFFSET;
	fdb->joint_motor_4.position = (fdb->joint_motor_4.motor_measure->ecd - fdb->joint_motor_4.position_offset) + LEG_OFFSET;

	fdb->joint_motor_1.velocity = fdb->joint_motor_1.motor_measure->velocity_rad_s;
	fdb->joint_motor_2.velocity = fdb->joint_motor_2.motor_measure->velocity_rad_s;
	fdb->joint_motor_3.velocity = fdb->joint_motor_3.motor_measure->velocity_rad_s;
	fdb->joint_motor_4.velocity = fdb->joint_motor_4.motor_measure->velocity_rad_s;

	//更新力矩反馈
	fdb->joint_motor_1.torque_get = fdb->joint_motor_1.motor_measure->real_torque;
	fdb->joint_motor_2.torque_get = fdb->joint_motor_2.motor_measure->real_torque;
	fdb->joint_motor_3.torque_get = fdb->joint_motor_3.motor_measure->real_torque;
	fdb->joint_motor_4.torque_get = fdb->joint_motor_4.motor_measure->real_torque;
	fdb->foot_motor_L.speed = fdb->foot_motor_L.motor_measure->speed *  PI / 180.0f * WHEEL_RADIUS; // dps -> m/s
	fdb->foot_motor_R.speed = -fdb->foot_motor_R.motor_measure->speed  * PI / 180.0f * WHEEL_RADIUS;
	fdb->chassis_posture_info.foot_speed = (fdb->foot_motor_L.speed + fdb->foot_motor_R.speed) / 2.0f;

    fdb->chassis_posture_info.foot_speed_KF = ( fdb->foot_motor_L.speed + fdb->foot_motor_R.speed ) / 2.0f;
	if ( (fabs(fdb->chassis_posture_info.foot_speed_KF) < 0.6f)
	 && (fdb->flag_info.suspend_flag_L == ON_GROUND && fdb->flag_info.suspend_flag_R == ON_GROUND))
	{
		fdb->chassis_posture_info.foot_distance_K += fdb->chassis_posture_info.foot_speed_KF*CHASSIS_CONTROL_TIME;
	}


	
	//足端角度解算
	Forward_kinematic_solution(fdb, fdb->joint_motor_1.position, fdb->joint_motor_1.velocity,
							   fdb->joint_motor_2.position, fdb->joint_motor_2.velocity, 1);
	Forward_kinematic_solution(fdb, fdb->joint_motor_4.position, fdb->joint_motor_4.velocity,
							   fdb->joint_motor_3.position, fdb->joint_motor_3.velocity, 0);
	
	//陀螺仪数据更新
	fdb->chassis_posture_info.yaw_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_YAW_ADDRESS_OFFSET);	
	fdb->chassis_posture_info.pitch_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_PITCH_ADDRESS_OFFSET);
	fdb->chassis_posture_info.roll_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_ROLL_ADDRESS_OFFSET);

	fdb->chassis_posture_info.x_accel = *(fdb->chassis_INS_accel + INS_ACCEL_X_ADDRESS_OFFSET);	
	fdb->chassis_posture_info.y_accel = *(fdb->chassis_INS_accel + INS_ACCEL_Y_ADDRESS_OFFSET);	
	fdb->chassis_posture_info.z_accel = *(fdb->chassis_INS_accel + INS_ACCEL_Z_ADDRESS_OFFSET);	

	fdb->chassis_posture_info.yaw_angle = rad_format(*(fdb->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
	delta =fdb->chassis_posture_info.yaw_angle-fdb->chassis_posture_info.yaw_angle_last;
	if(delta<-PI)
	{
		 fdb->chassis_posture_info.yaw_round_cnt++;
	}
	else if(delta>PI)
	{
		fdb->chassis_posture_info.yaw_round_cnt--;
	}
	fdb->chassis_posture_info.yaw_angle_total = fdb->chassis_posture_info.yaw_round_cnt*PI2+fdb->chassis_posture_info.yaw_angle;
	fdb->chassis_posture_info.yaw_angle_last = fdb->chassis_posture_info.yaw_angle;
	fdb->chassis_posture_info.pitch_angle = rad_format(*(fdb->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET)); 
	fdb->chassis_posture_info.roll_angle = *(fdb->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

	//腿部角度、角速度更新；腿长和腿长速度已在五连杆正解中更新
	fdb->chassis_posture_info.leg_angle_L += fdb->chassis_posture_info.pitch_angle;
	fdb->chassis_posture_info.leg_angle_R += fdb->chassis_posture_info.pitch_angle;
	fdb->chassis_posture_info.leg_gyro_L += fdb->chassis_posture_info.pitch_gyro;
	fdb->chassis_posture_info.leg_gyro_R += fdb->chassis_posture_info.pitch_gyro;
	
	// 计算加速度
	if (fdb->flag_info.init_flag)
	{
		/* Establish a derivative baseline before off-ground detection starts. */
		fdb->chassis_posture_info.leg_ddlength_L = 0.0f;
		fdb->chassis_posture_info.leg_ddlength_R = 0.0f;
		fdb->chassis_posture_info.last_leg_dlength_L_jacobian = fdb->chassis_posture_info.leg_dlength_L_jacobian;
		fdb->chassis_posture_info.last_leg_dlength_R_jacobian = fdb->chassis_posture_info.leg_dlength_R_jacobian;
	}
	else
	{
		fp32 temp_a_L = (fdb->chassis_posture_info.leg_dlength_L_jacobian - fdb->chassis_posture_info.last_leg_dlength_L_jacobian) / CHASSIS_CONTROL_TIME;
		fdb->chassis_posture_info.leg_ddlength_L = alpha_da * temp_a_L + (1-alpha_da) * fdb->chassis_posture_info.last_leg_ddlength_L;
		fp32 temp_a_R = (fdb->chassis_posture_info.leg_dlength_R_jacobian - fdb->chassis_posture_info.last_leg_dlength_R_jacobian) / CHASSIS_CONTROL_TIME;
		fdb->chassis_posture_info.leg_ddlength_R = alpha_da * temp_a_R + (1-alpha_da) * fdb->chassis_posture_info.last_leg_ddlength_R;
	}
	
	fdb->chassis_posture_info.last_leg_angle_L = fdb->chassis_posture_info.leg_angle_L;
	fdb->chassis_posture_info.last_leg_angle_R = fdb->chassis_posture_info.leg_angle_R;
	fdb->chassis_posture_info.last_leg_length_L = fdb->chassis_posture_info.leg_length_L;
	fdb->chassis_posture_info.last_leg_length_R = fdb->chassis_posture_info.leg_length_R;	
	fdb->chassis_posture_info.last_leg_dlength_L_jacobian = fdb->chassis_posture_info.leg_dlength_L_jacobian;
	fdb->chassis_posture_info.last_leg_dlength_R_jacobian = fdb->chassis_posture_info.leg_dlength_R_jacobian;
	fdb->chassis_posture_info.last_leg_ddlength_L = fdb->chassis_posture_info.leg_ddlength_L;
	fdb->chassis_posture_info.last_leg_ddlength_R = fdb->chassis_posture_info.leg_ddlength_R;
	//云台相对角度更新
	fdb->gimbal_yaw_motor.relative_angle = theta_format(fdb->gimbal_yaw_motor.gimbal_motor_measure->angle);

	ramp_calc(&speed_ramp_vx, fdb->chassis_data_->vx_set);
	ramp_calc(&speed_ramp_vy, fdb->chassis_data_->vy_set);
	ramp_calc(&speed_ramp_wz, fdb->chassis_data_->wz_set);
	if(speed_ramp_vx.out>=0)
	{
		normalized_speed = fp32_constrain(sqrt(speed_ramp_vx.out*speed_ramp_vx.out+speed_ramp_vy.out*speed_ramp_vy.out),-2.2f,2.2f);
	}
	else
	{
		normalized_speed = -fp32_constrain(sqrt(speed_ramp_vx.out*speed_ramp_vx.out+speed_ramp_vy.out*speed_ramp_vy.out),-2.2f,2.2f);
	}
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}
	/* --------------------------------set chassis mode -------------------------------- */
	if (chassis_move_mode->chassis_data_->chassis_mode == CHASSIS_MODE_OFF)
		chassis_move_mode->mode.chassis_mode = DISABLE_CHASSIS;
	else if (chassis_move_mode->chassis_data_->chassis_mode == CHASSIS_MODE_DEBUG)
		chassis_move_mode->mode.chassis_mode = DEBUG_CHASSIS;
	else
		chassis_move_mode->mode.chassis_mode = ENABLE_CHASSIS;

	if (chassis_move_mode->mode.chassis_mode == ENABLE_CHASSIS)
	{
		chassis_move_mode->joint_motor_1.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_2.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_3.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_4.motor_mode = MOTOR_FORCE;
		chassis_move_mode->foot_motor_L.motor_mode = MOTOR_FORCE;
		chassis_move_mode->foot_motor_R.motor_mode = MOTOR_FORCE;
		uint32_t current_tick = xTaskGetTickCount();
		uint32_t timeout_threshold = pdMS_TO_TICKS(100);  // 100ms超时

		if (current_tick - chassis_move_mode->foot_motor_L.motor_measure->last_update_time > timeout_threshold ||
			current_tick - chassis_move_mode->foot_motor_R.motor_measure->last_update_time > timeout_threshold)
		{
			// 轮毂电机离线，让髋关节失能
			chassis_move_mode->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_L.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_R.motor_mode = MOTOR_NO_FORCE;
		}
	}
	else if (chassis_move_mode->mode.chassis_mode == DEBUG_CHASSIS)
	{
		// 检测轮毂电机超时
		chassis_move_mode->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->foot_motor_L.motor_mode = MOTOR_FORCE;
		chassis_move_mode->foot_motor_R.motor_mode = MOTOR_FORCE;
	}
	else
	{
		if (chassis_move_mode->mode.chassis_balancing_mode == JOINT_REDUCING)
		{
			chassis_move_mode->joint_motor_1.motor_mode = MOTOR_FORCE;
			chassis_move_mode->joint_motor_2.motor_mode = MOTOR_FORCE;
			chassis_move_mode->joint_motor_3.motor_mode = MOTOR_FORCE;
			chassis_move_mode->joint_motor_4.motor_mode = MOTOR_FORCE;
			chassis_move_mode->foot_motor_L.motor_mode = MOTOR_FORCE;
			chassis_move_mode->foot_motor_R.motor_mode = MOTOR_FORCE;
		}
		else
		{
			chassis_move_mode->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_L.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_R.motor_mode = MOTOR_NO_FORCE;
		}
	}

	/*-------------------------- Sport Mode Update ----------------------------------*/
	if (chassis_move_mode->mode.chassis_balancing_mode == BALANCING_READY)
	{
		/* 之后再考虑添加 */
		if (chassis_move_mode->flag_info.abnormal_flag)
			chassis_move_mode->mode.sport_mode = ABNORMAL_MOVING_MODE;
		else if (chassis_move_mode->mode.sport_mode == JUMPING_MODE && chassis_move_mode->mode.jumping_stage != FINISHED)
			chassis_move_mode->mode.sport_mode = JUMPING_MODE;
		else if (chassis_move_mode->chassis_data_->jump_flag)
			chassis_move_mode->mode.sport_mode = JUMPING_MODE;
		else
			chassis_move_mode->mode.sport_mode = NORMAL_MOVING_MODE;
	}
	else
		chassis_move_mode->mode.sport_mode = NONE;
}

uint8_t reduce_flag = 0;
fp32 reduce_high, high_offset = 0.2f;
fp32 reduce_high_speed = 0.00035f;
static void chassis_mode_change_control_transit(chassis_move_t *chassis_mode_change)
{
	if (chassis_mode_change == NULL)
	{
		return;
	}
	/* --------------------------------使能/失能 后判断是否 进入/退出 平衡模式--------------------------------  */
	if (chassis_mode_change->mode.chassis_mode == ENABLE_CHASSIS && chassis_mode_change->mode.last_chassis_mode == DISABLE_CHASSIS)
		{
			chassis_mode_change->mode.chassis_balancing_mode = FOOT_LAUNCHING;
			chassis_mode_change->chassis_posture_info.yaw_angle_sett = chassis_mode_change->chassis_posture_info.yaw_angle_total;
		}
	if (chassis_mode_change->mode.chassis_balancing_mode == FOOT_LAUNCHING &&
		fabs(chassis_mode_change->chassis_posture_info.pitch_angle) < EXIT_PITCH_ANGLE)
		chassis_mode_change->mode.chassis_balancing_mode = JOINT_LAUNCHING;
	else if (chassis_mode_change->mode.chassis_balancing_mode == JOINT_LAUNCHING)
		chassis_mode_change->mode.chassis_balancing_mode = BALANCING_READY;
	else if (chassis_mode_change->mode.chassis_balancing_mode == BALANCING_READY &&
			 chassis_mode_change->mode.chassis_mode == DISABLE_CHASSIS)
	{
		if (!reduce_flag)
		{
			reduce_high = chassis_mode_change->chassis_posture_info.ideal_high;
			reduce_flag = 1;
		}
		chassis_mode_change->mode.chassis_balancing_mode = JOINT_REDUCING;
	}
	else if (chassis_mode_change->mode.chassis_balancing_mode == JOINT_REDUCING && (fabs(chassis_mode_change->chassis_posture_info.leg_length_L - SIT_HIGH) < 0.015f ||
																					fabs(chassis_mode_change->chassis_posture_info.leg_length_R - SIT_HIGH) < 0.015f))
	{
		chassis_mode_change->mode.chassis_balancing_mode = NO_FORCE;
		reduce_flag = 0;
	}
	//跳跃模式流程
	if (chassis_mode_change->mode.sport_mode == JUMPING_MODE && chassis_mode_change->mode.last_sport_mode != JUMPING_MODE)
    {
        chassis_mode_change->mode.jumping_mode = STANDING_JUMP;
        chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
        chassis_mode_change->flag_info.jump_prepare_complete = 0; 
    }
    else if (chassis_mode_change->mode.sport_mode != JUMPING_MODE)
    {
        chassis_mode_change->mode.jumping_mode = NOT_DEFINE;
        chassis_mode_change->mode.jumping_stage = FINISHED;
    }

    else if (chassis_mode_change->mode.jumping_mode == STANDING_JUMP)
    {
	 // 站立跳跃状态机
        switch(chassis_mode_change->mode.jumping_stage)
        {
            case READY_TO_JUMP:
                // 进入准备阶段
                chassis_mode_change->mode.jumping_stage = PREPARING_STAND_JUMPING;
                chassis_mode_change->flag_info.jump_prepare_timer = xTaskGetTickCount();
                break;
                
            case PREPARING_STAND_JUMPING:
                // 检查准备条件是否满足
                if (Check_Jump_Preparation_Complete(chassis_mode_change))
                {
                    chassis_mode_change->flag_info.jump_prepare_complete = 1;
                    chassis_mode_change->mode.jumping_stage = EXTENDING_LEGS;
                    chassis_mode_change->flag_info.jump_extend_timer = xTaskGetTickCount();
                }
                else
                {
                    // 检查是否超时
                    if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_prepare_timer) > pdMS_TO_TICKS(3000))
                    {
                        // 准备超时，放弃跳跃
                        chassis_mode_change->mode.jumping_stage = FINISHED;
                        chassis_mode_change->mode.sport_mode = NORMAL_MOVING_MODE;
                    }
                }
                break;
                
            case EXTENDING_LEGS:
                // 腿伸长阶段
                if (chassis_mode_change->chassis_posture_info.leg_length_L >= 0.34f && 
                    chassis_mode_change->chassis_posture_info.leg_length_R >= 0.34f&&
				(xTaskGetTickCount() - chassis_mode_change->flag_info.jump_extend_timer) > pdMS_TO_TICKS(50))
                {
                    chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS_2;
                    chassis_mode_change->flag_info.jump_contact_timer = xTaskGetTickCount();
                }
                else if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_extend_timer) > pdMS_TO_TICKS(500))
                {
                    // 伸腿超时，认为失败
                    chassis_mode_change->mode.jumping_stage = FINISHED;
                    chassis_mode_change->mode.sport_mode = NORMAL_MOVING_MODE;
                }
                break;
                
            case CONSTACTING_LEGS_2:
                // 再次收缩腿准备落地
                if (chassis_mode_change->chassis_posture_info.leg_length_L <= 0.11f&&chassis_mode_change->chassis_posture_info.leg_length_R <= 0.11f)
                {
                    chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
                }
                else if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_contact_timer) > pdMS_TO_TICKS(300))
                {
                    // 收缩超时，强制进入落地准备
                    chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
                }
                break;
                
            case PREPARING_LANDING:
                // 等待落地检测
                if (chassis_mode_change->flag_info.suspend_flag_R == ON_GROUND &&
                    chassis_mode_change->flag_info.suspend_flag_L == ON_GROUND&&
					(xTaskGetTickCount() - chassis_mode_change->flag_info.jump_contact_timer) > pdMS_TO_TICKS(500))
                {
                    chassis_mode_change->mode.jumping_stage = FINISHED;
                }
                // 落地超时保护
                else if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_contact_timer) > pdMS_TO_TICKS(2000))
                {
                    chassis_mode_change->mode.jumping_stage = FINISHED;
                }
                break;
                
            case FINISHED:
                // 跳跃完成，重置状态
                chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
                chassis_mode_change->mode.sport_mode = NORMAL_MOVING_MODE;
                chassis_mode_change->flag_info.jump_prepare_complete = 0;
                break;
        }
    }
    else
    {
        chassis_mode_change->mode.jumping_stage = FINISHED;
    }


}

void Target_Value_Set(chassis_move_t *target_value_set)
{

	//底盘正方向速度控制
	if (target_value_set->mode.sport_mode != NONE &&
	target_value_set->flag_info.suspend_flag_L == ON_GROUND &&
	target_value_set->flag_info.suspend_flag_R == ON_GROUND)
	{
		if(fabs(target_value_set->chassis_data_->wz_set) < CHASSIS_RC_WZ_DEADLINE)
		{
			target_value_set->chassis_posture_info.foot_speed_set = fp32_constrain(normalized_speed* normal_move_scale,-2.2f,2.2f);
			chassis_power_limit(target_value_set);
		}
	}
	else 
	{
		target_value_set->chassis_posture_info.foot_speed_set=0;
	}

	//位置环控制器
	if(fabs(target_value_set->chassis_posture_info.foot_speed_set)!=0)
	{
		target_value_set->chassis_posture_info.position_lock_flag=0;
	}
	else 
	{
		target_value_set->chassis_posture_info.position_lock_flag=1;
	}
	if(target_value_set->chassis_posture_info.position_lock_flag==1)
	{
		if(target_value_set->chassis_posture_info.position_lock_state==0)
		{
			target_value_set->chassis_posture_info.position_lock_state=1;
			target_value_set->chassis_posture_info.target_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
		}
	}
	else
	{
		target_value_set->chassis_posture_info.position_lock_state=0;
	}

	// --------- Distance Set ---------
	if(target_value_set->chassis_posture_info.position_lock_flag==1)
	target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.target_distance_set;
	else if( target_value_set->mode.chassis_balancing_mode == NO_FORCE )
	target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if(target_value_set->mode.sport_mode == NORMAL_MOVING_MODE)
	target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE )
	target_value_set->chassis_posture_info.foot_distance_set =  target_value_set->chassis_posture_info.foot_distance_K;
	else if( target_value_set->flag_info.suspend_flag_R == OFF_GROUND ||
		target_value_set->flag_info.suspend_flag_L == OFF_GROUND )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	fp32 distance_error = target_value_set->chassis_posture_info.foot_distance_set - target_value_set->chassis_posture_info.foot_distance_K;
	// 误差超过30cm时，重置目标值
	if (fabs(distance_error) > 0.3f)
	{
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	}
	// 现在误差已经归零，正常计算
	distance_error = target_value_set->chassis_posture_info.foot_distance_set - target_value_set->chassis_posture_info.foot_distance_K;  // = 0
		
	// --------- yaw_gyro_set ---------
	if (target_value_set->mode.sport_mode != NONE &&
		target_value_set->flag_info.suspend_flag_L == ON_GROUND &&
		target_value_set->flag_info.suspend_flag_R == ON_GROUND)
	{
		if (target_value_set->mode.chassis_mode == ENABLE_CHASSIS && target_value_set->mode.last_chassis_mode == DISABLE_CHASSIS)
			{
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle_total;
				target_value_set->chassis_posture_info.yaw_gyro_set = 0.0f;
			}
			else if(target_value_set->mode.chassis_mode==ENABLE_CHASSIS&&target_value_set->mode.last_chassis_mode==ENABLE_CHASSIS)
			{
				if (fabs(target_value_set->chassis_posture_info.foot_speed_set) > 0.2
					&& fabs(target_value_set->gimbal_yaw_motor.relative_angle - target_value_set->gimbal_yaw_motor.relative_angle_init) > 0.6f)
				{
					float current_relative_angle = target_value_set->gimbal_yaw_motor.relative_angle;
					float target_relative_angle = target_value_set->gimbal_yaw_motor.relative_angle_init;
					float angle_diff = current_relative_angle - target_relative_angle;
					
					// 将角度差规范化到[-180, 180]区间
					if (angle_diff > 180.0f) {
						angle_diff -= 360.0f;
					} else if (angle_diff < -180.0f) {
						angle_diff += 360.0f;
					}
					
					// 使用规范化后的角度差作为PID输入
					target_value_set->gimbal_yaw_motor.relative_limit = angle_diff;
					
					target_value_set->chassis_posture_info.yaw_angle_sett -= 
						PID_calc(&target_value_set->chassis_yaw_pid, 
								angle_diff,  // 使用规范化后的角度差
								0.0f) * 0.004f;  // 目标值是0（角度差为0）
					
					target_value_set->chassis_posture_info.yaw_gyro_set = 0.0f;	
					target_value_set->chassis_posture_info.foot_speed_set = 0.60f * target_value_set->chassis_posture_info.foot_speed_set;
				}
				else
				{
					if (target_value_set->chassis_data_->wz_set < -CHASSIS_RC_WZ_DEADLINE)
					{
						target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle_total;
						target_value_set->chassis_posture_info.yaw_gyro_set = speed_ramp_wz.out;
						target_value_set->chassis_posture_info.yaw_lock_flag =1;
					}
					else if (target_value_set->chassis_data_->wz_set > CHASSIS_RC_WZ_DEADLINE)
					{
						target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle_total;
						target_value_set->chassis_posture_info.yaw_gyro_set = speed_ramp_wz.out;
						target_value_set->chassis_posture_info.yaw_lock_flag =1;
					}
					else
					{	
						if(target_value_set->chassis_posture_info.yaw_lock_flag==1)
						{
							target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle_total;
							target_value_set->chassis_posture_info.yaw_gyro_set *= 0.85f;  // 指数衰减，每次循环衰减8%
							if(fabs(target_value_set->chassis_posture_info.yaw_gyro) < 0.02f)
							{
								target_value_set->chassis_posture_info.yaw_gyro_set = 0;
								target_value_set->chassis_posture_info.yaw_lock_flag = 0;
							}
						}
					}
					chassis_power_limit(target_value_set);
				}
			}
	}
	else
	{
		target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle_total;
		target_value_set->chassis_posture_info.yaw_gyro_set = 0;
	}

	target_value_set->chassis_posture_info.roll_angle_set = 0.0f;

	// ---------------- Leg Angle Set ----------
	if (target_value_set->mode.jumping_stage == PREPARING_STAND_JUMPING)
	{
		target_value_set->chassis_posture_info.leg_angle_L_set =  IDEAL_PREPARING_STAND_JUMPING_ANGLE;
		target_value_set->chassis_posture_info.leg_angle_R_set =  IDEAL_PREPARING_STAND_JUMPING_ANGLE;
	}
	else
	{
		target_value_set->chassis_posture_info.leg_angle_L_set = 0.0f;
		target_value_set->chassis_posture_info.leg_angle_R_set = 0.0f;
	}

	// ----------------- Chassis High Mode Update -----------------
	if (target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE)
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else if (target_value_set->mode.chassis_balancing_mode == FOOT_LAUNCHING)
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if (target_value_set->mode.chassis_balancing_mode == JOINT_LAUNCHING)
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else if (target_value_set->mode.chassis_balancing_mode == JOINT_REDUCING)
		target_value_set->mode.chassis_high_mode = CHANGING_HIGH;

	else if (target_value_set->mode.jumping_stage == CONSTACTING_LEGS_2)
		target_value_set->mode.chassis_high_mode = SIT_MODE;

	else
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;

	// --------- Leg Length Set ---------
	if (target_value_set->mode.chassis_high_mode == SIT_MODE)
		target_value_set->chassis_posture_info.ideal_high = SIT_HIGH;
	else if (target_value_set->mode.chassis_high_mode == NORMAL_MODE)
		target_value_set->chassis_posture_info.ideal_high = fp32_constrain(target_value_set->chassis_data_->high_set,0.1,0.34);
	else if (target_value_set->mode.chassis_high_mode == CHANGING_HIGH)
	{
		reduce_high -= reduce_high_speed;
		if (reduce_high < SIT_HIGH)
		{
			reduce_high = SIT_HIGH;
		}
		target_value_set->chassis_posture_info.ideal_high = reduce_high;
	}
	// ============= 新增：跳跃阶段腿长设定 =============
	// 跳跃阶段优先于其他模式设定腿长
	if (target_value_set->mode.sport_mode == JUMPING_MODE)
	{
		switch(target_value_set->mode.jumping_stage)
		{
			case PREPARING_STAND_JUMPING:
				// 跳跃准备阶段：设定较低的腿长用于蓄力
				target_value_set->chassis_posture_info.leg_length_L_set = 0.13f;
				target_value_set->chassis_posture_info.leg_length_R_set = 0.13f;
				break;
				
			case EXTENDING_LEGS:
				// 起跳阶段：快速伸腿  
				target_value_set->chassis_posture_info.leg_length_L_set = 0.34f;
				target_value_set->chassis_posture_info.leg_length_R_set = 0.34f;
				break;
				
			case CONSTACTING_LEGS_2:
				// 空中收缩阶段：准备落地
				target_value_set->chassis_posture_info.leg_length_L_set = 0.10f;
				target_value_set->chassis_posture_info.leg_length_R_set = 0.10f;
				break;
				
			case PREPARING_LANDING:
				// 落地准备阶段：保持较低腿长
				target_value_set->chassis_posture_info.leg_length_L_set = 0.15f;
				target_value_set->chassis_posture_info.leg_length_R_set = 0.15f;
				break;
				
			default:
				// 其他跳跃阶段使用理想高度
				target_value_set->chassis_posture_info.leg_length_L_set = target_value_set->chassis_posture_info.ideal_high;
				target_value_set->chassis_posture_info.leg_length_R_set = target_value_set->chassis_posture_info.ideal_high;
				break;
		}
	}
	// ==========================================
	else if (target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE ||
		(target_value_set->flag_info.suspend_flag_L == 1 && target_value_set->flag_info.suspend_flag_R == 1) ||
		target_value_set->chassis_posture_info.ideal_high == SIT_HIGH ||
		target_value_set->mode.chassis_balancing_mode == JOINT_REDUCING)
	{
		target_value_set->chassis_posture_info.leg_length_L_set = target_value_set->chassis_posture_info.ideal_high;
		target_value_set->chassis_posture_info.leg_length_R_set = target_value_set->chassis_posture_info.ideal_high;
	}
	else
	{
		//目前表现良好
		target_value_set->chassis_posture_info.foot_roll_angle =
			target_value_set->chassis_posture_info.roll_angle;
		target_value_set->chassis_posture_info.leg_length_L_set =
			target_value_set->chassis_posture_info.ideal_high - 0.24f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
		target_value_set->chassis_posture_info.leg_length_R_set =
			target_value_set->chassis_posture_info.ideal_high + 0.24f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
	}
}
void Chassis_Torque_Calculation(chassis_move_t *bl_ctrl)
{
	/* Do not let stale controller state leak into a disabled/debug cycle.
	 * JOINT_REDUCING is the only disabled state that intentionally drives the
	 * hip motors to lower the chassis. */
	if ((bl_ctrl->mode.chassis_mode == DISABLE_CHASSIS &&
		 bl_ctrl->mode.chassis_balancing_mode != JOINT_REDUCING) ||
		bl_ctrl->mode.chassis_mode == DEBUG_CHASSIS)
	{
		bl_ctrl->torque_info.joint_stand_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_stand_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_roll_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_roll_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_balancing_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_balancing_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
		bl_ctrl->torque_info.foot_moving_torque_L = 0.0f;
		bl_ctrl->torque_info.foot_moving_torque_R = 0.0f;
		return;
	}
	//LQR拟合矩阵数据更新
	LQR_Data_Update(bl_ctrl);
	//不同情况下roll轴控制
	if (bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 ||
		bl_ctrl->mode.chassis_high_mode == SIT_MODE)
	{
		bl_ctrl->torque_info.joint_roll_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_roll_torque_L = 0.0f;
	}
	else
	{
		if (bl_ctrl->chassis_posture_info.roll_angle > roll_angle_deadband)
		{
			rollP =  roll_PD[0] * (bl_ctrl->chassis_posture_info.roll_angle_set - (bl_ctrl->chassis_posture_info.roll_angle));
		}
		else if (bl_ctrl->chassis_posture_info.roll_angle < -roll_angle_deadband)
		{
			rollP = roll_PD[0] * (bl_ctrl->chassis_posture_info.roll_angle_set - (bl_ctrl->chassis_posture_info.roll_angle ));
		}
		else
		{
			rollP = 0.0f;
		}

		if (bl_ctrl->chassis_posture_info.roll_gyro > roll_gyro_deadband)
		{
			rollD = roll_PD[1] * (bl_ctrl->chassis_posture_info.roll_gyro );
		}
		else if (bl_ctrl->chassis_posture_info.roll_gyro < -roll_gyro_deadband)
		{
			rollD = roll_PD[1] * (bl_ctrl->chassis_posture_info.roll_gyro );		
		}
		else
		{
			rollD = 0.0f;
		}
		bl_ctrl->torque_info.joint_roll_torque_R = rollP + rollD ;//极性问题建议自己实际尝试
		bl_ctrl->torque_info.joint_roll_torque_L = -bl_ctrl->torque_info.joint_roll_torque_R;
	}

	

	//不同情况下垂直方向力控制
	if( bl_ctrl->mode.jumping_stage == EXTENDING_LEGS )
	{
		bl_ctrl->torque_info.joint_stand_torque_L =
			+ jump_stand_PD_L[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L )
			+ jump_stand_PD_L[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_L_jacobian );

		bl_ctrl->torque_info.joint_stand_torque_R =
			+ jump_stand_PD_R[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R )
			+ jump_stand_PD_R[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_R_jacobian );
	}
	else if(bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2)
	{
		bl_ctrl->torque_info.joint_stand_torque_L =
			+ jump_canstacting_PD_L[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L )
			+ jump_canstacting_PD_L[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_L_jacobian );

		bl_ctrl->torque_info.joint_stand_torque_R =
			+ jump_canstacting_PD_R[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R )
			+ jump_canstacting_PD_R[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_R_jacobian );
	}
	else if( bl_ctrl->mode.sport_mode == ABNORMAL_MOVING_MODE )
	{
		bl_ctrl->torque_info.joint_stand_torque_L =
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L )
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L_jacobian );
		bl_ctrl->torque_info.joint_stand_torque_R =
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R )
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R_jacobian );
	}
	else if( bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 )
	{
		if( bl_ctrl->flag_info.suspend_flag_L == 1 )
		{
			bl_ctrl->torque_info.joint_stand_torque_L =
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L )
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L_jacobian );
		}
		else{
			PID_calc(&bl_ctrl->leg_L_length_pid, bl_ctrl->chassis_posture_info.leg_length_L,bl_ctrl->chassis_posture_info.leg_length_L_set);
			bl_ctrl->torque_info.joint_stand_torque_L = FEED_f+bl_ctrl->leg_L_length_pid.out;
		}

		if( bl_ctrl->flag_info.suspend_flag_R == 1 )
		{
			bl_ctrl->torque_info.joint_stand_torque_R = 
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R )
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R_jacobian );
		} else {
			PID_calc(&bl_ctrl->leg_R_length_pid, bl_ctrl->chassis_posture_info.leg_length_R,bl_ctrl->chassis_posture_info.leg_length_R_set);
			bl_ctrl->torque_info.joint_stand_torque_R = FEED_f+bl_ctrl->leg_R_length_pid.out;
		}
	}
	else if( bl_ctrl->mode.chassis_balancing_mode == JOINT_REDUCING )
	{

		bl_ctrl->torque_info.joint_stand_torque_L = 0.55f*FEED_f
			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L )
			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L_jacobian );

		bl_ctrl->torque_info.joint_stand_torque_R = 0.55f*FEED_f
			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R )
			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R_jacobian );
	}
	else
	{
		//普通情况下加重力补偿
		PID_calc(&bl_ctrl->leg_L_length_pid, bl_ctrl->chassis_posture_info.leg_length_L,bl_ctrl->chassis_posture_info.leg_length_L_set);
		bl_ctrl->torque_info.joint_stand_torque_L = FEED_f+bl_ctrl->leg_L_length_pid.out;
		PID_calc(&bl_ctrl->leg_R_length_pid, bl_ctrl->chassis_posture_info.leg_length_R,bl_ctrl->chassis_posture_info.leg_length_R_set);
		bl_ctrl->torque_info.joint_stand_torque_R = FEED_f+bl_ctrl->leg_R_length_pid.out;
	}
	//添加被动的检测到离地的相应操作
	if (bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2)
	{
		//此时处于空中，想要达到的效果是腿的度跟地面是保持垂直，
		//不在控制机身角度因为机身保持平衡的力矩其实与腿部保持竖直力矩相冲突，在没有地面支持力的情况下没有意义
		bl_ctrl->torque_info.joint_balancing_torque_L = (
			+ LQR[2][4] * (bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L)
			+ LQR[2][5] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_L) 
		);		
		bl_ctrl->torque_info.joint_balancing_torque_R = -(
			+ LQR[3][6] * (bl_ctrl->chassis_posture_info.leg_angle_R_set - bl_ctrl->chassis_posture_info.leg_angle_R) 
			+ LQR[3][7] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_R) 
		);

		bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
	}

	else
	{

		if (bl_ctrl->mode.chassis_high_mode == SIT_MODE ||
			bl_ctrl->mode.chassis_balancing_mode == JOINT_REDUCING)
		{
			bl_ctrl->torque_info.joint_balancing_torque_L = 0.0f;
			bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
			bl_ctrl->torque_info.joint_balancing_torque_R = 0.0f;
			bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
		}
		else
		{

			bl_ctrl->torque_info.joint_balancing_torque_L = (
				+ LQR[2][4] * (bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L)
				+ LQR[2][5] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_L) 
				+ LQR[2][8] * (bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle) 
				+ LQR[2][9] * (bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro)
			);
			bl_ctrl->torque_info.joint_moving_torque_L    = (
				+ LQR[2][0] * ( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K )
				+ LQR[2][1] * ( bl_ctrl->chassis_posture_info.foot_speed_set+FORWARD_SPEED    - bl_ctrl->chassis_posture_info.foot_speed_KF )
				// + LQR[2][2] * ( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle_total )
				// + LQR[2][3] * ( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
				);

			bl_ctrl->torque_info.joint_balancing_torque_R = -(
				+ LQR[3][6] * (bl_ctrl->chassis_posture_info.leg_angle_R_set - bl_ctrl->chassis_posture_info.leg_angle_R) 
				+ LQR[3][7] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_R) 
				+ LQR[3][8] * (bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle) 
				+ LQR[3][9] * (bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro)
			);
			bl_ctrl->torque_info.joint_moving_torque_R    = -(
				+ LQR[3][0] * ( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K )
				+ LQR[3][1] * ( bl_ctrl->chassis_posture_info.foot_speed_set+FORWARD_SPEED    - bl_ctrl->chassis_posture_info.foot_speed_KF )
				// + LQR[3][2] * ( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle_total )
				// + LQR[3][3] * ( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
				);
		}
	}
	//轮毂控制
	bl_ctrl->torque_info.foot_balancing_torque_L = (
		+ LQR[0][4] * (bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L) 
		+ LQR[0][5] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_L) 
		+ LQR[0][8] * (bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle) 
		+ LQR[0][9] * (bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro)
	) * TORQ_K; 
	bl_ctrl->torque_info.foot_moving_torque_L = (
		+ LQR[0][0] * (bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K )
		+ LQR[0][1] * (bl_ctrl->chassis_posture_info.foot_speed_set+FORWARD_SPEED - bl_ctrl->chassis_posture_info.foot_speed_KF)
		+ LQR[0][2]*( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle_total)
		+ LQR[0][3]*( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
	)*TORQ_K;
	bl_ctrl->torque_info.foot_balancing_torque_R = -(
		+ LQR[1][6] * (bl_ctrl->chassis_posture_info.leg_angle_R_set - bl_ctrl->chassis_posture_info.leg_angle_R) 
		+ LQR[1][7] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_R) 
		+ LQR[1][8] * (bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle) 
		+ LQR[1][9] * (bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro)
	) * TORQ_K;
	bl_ctrl->torque_info.foot_moving_torque_R = -(
		+ LQR[1][0] * (bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K )
		+ LQR[1][1] * (bl_ctrl->chassis_posture_info.foot_speed_set+FORWARD_SPEED - bl_ctrl->chassis_posture_info.foot_speed_KF)
		+ LQR[1][2]*( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle_total)
		+ LQR[1][3]*( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
	) *TORQ_K;

	// 统一的离地处理函数
	if (bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1)
	{
		// 离地状态统一处理
		handle_airborne_state(bl_ctrl);
	}
	else
	{}
	LimitMax( bl_ctrl->torque_info.foot_moving_torque_L,  MAX_ACCL );
	LimitMax( bl_ctrl->torque_info.foot_moving_torque_R,  MAX_ACCL );
	LimitMax(bl_ctrl->torque_info.joint_moving_torque_L, MAX_ACCL_JOINT);
	LimitMax(bl_ctrl->torque_info.joint_moving_torque_R, MAX_ACCL_JOINT);
}
void Chassis_Torque_Combine(chassis_move_t *bl_ctrl)
{
	/* ---------J1 J2 对应支持力分解成关节扭矩     J3 J4 对应平衡扭矩分解成关节扭矩------------------------------ */
	bl_ctrl->mapping_info.invJ1_L = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_L, bl_ctrl->chassis_posture_info.leg_angle_L, 1); // N11
	bl_ctrl->mapping_info.invJ2_L = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_L, bl_ctrl->chassis_posture_info.leg_angle_L, 3); // N21
	bl_ctrl->mapping_info.invJ3_L = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_L, bl_ctrl->chassis_posture_info.leg_angle_L, 2); // N12
	bl_ctrl->mapping_info.invJ4_L = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_L, bl_ctrl->chassis_posture_info.leg_angle_L, 4); // N22
	bl_ctrl->mapping_info.invJ1_R = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_R, bl_ctrl->chassis_posture_info.leg_angle_R, 1);
	bl_ctrl->mapping_info.invJ2_R = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_R, bl_ctrl->chassis_posture_info.leg_angle_R, 3);
	bl_ctrl->mapping_info.invJ3_R = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_R, bl_ctrl->chassis_posture_info.leg_angle_R, 2);
	bl_ctrl->mapping_info.invJ4_R = get_jacobian_element(bl_ctrl, bl_ctrl->chassis_posture_info.leg_length_R, bl_ctrl->chassis_posture_info.leg_angle_R, 4);

	bl_ctrl->torque_info.foot_horizontal_torque_L =
		bl_ctrl->torque_info.foot_balancing_torque_L + bl_ctrl->torque_info.foot_moving_torque_L;
	bl_ctrl->torque_info.foot_horizontal_torque_R =
		bl_ctrl->torque_info.foot_balancing_torque_R + bl_ctrl->torque_info.foot_moving_torque_R; // 足端轮子水平力矩

	bl_ctrl->foot_motor_L.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_L;
	bl_ctrl->foot_motor_R.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_R;
	if(bl_ctrl->chassis_data_->chassis_mode == CHASSIS_MODE_DEBUG)
	{
		/* Debug mode must be zero-force at zero stick. The previous two
		 * independent if/else blocks always ended at +/-400 even when both
		 * commands were zero. */
		bl_ctrl->foot_motor_L.torque_out = 0.0f;
		bl_ctrl->foot_motor_R.torque_out = 0.0f;
		if (speed_ramp_vx.out > 0.01f)
		{
			bl_ctrl->foot_motor_L.torque_out = 400;
			bl_ctrl->foot_motor_R.torque_out = -400;
		}
		else if (speed_ramp_vx.out < -0.01f)
		{
			bl_ctrl->foot_motor_L.torque_out = -400;
			bl_ctrl->foot_motor_R.torque_out = 400;			
		}
		else if (speed_ramp_vy.out > 0.01f)
		{
			bl_ctrl->foot_motor_L.torque_out = 400;
			bl_ctrl->foot_motor_R.torque_out = 400;
		}
		else if (speed_ramp_vy.out < -0.01f)
		{
			bl_ctrl->foot_motor_L.torque_out = -400;
			bl_ctrl->foot_motor_R.torque_out = -400;
		}
	}
	LimitMax(bl_ctrl->foot_motor_L.torque_out, MAX_FOOT_OUTPUT);
	LimitMax(bl_ctrl->foot_motor_R.torque_out, MAX_FOOT_OUTPUT);
	/* -----------------------首先尝试平衡力矩调试-------------------------	 */
	bl_ctrl->torque_info.joint_horizontal_torque_L = 
		bl_ctrl->torque_info.joint_balancing_torque_L+bl_ctrl->torque_info.joint_moving_torque_L;
	bl_ctrl->torque_info.joint_horizontal_torque_R =
		bl_ctrl->torque_info.joint_balancing_torque_R+bl_ctrl->torque_info.joint_moving_torque_R;

	bl_ctrl->torque_info.joint_vertical_torque_L = 
		bl_ctrl->torque_info.joint_stand_torque_L + bl_ctrl->torque_info.joint_roll_torque_L;
	bl_ctrl->torque_info.joint_vertical_torque_R = 
		bl_ctrl->torque_info.joint_stand_torque_R + bl_ctrl->torque_info.joint_roll_torque_R;

	/* 左视图：假设此时需要一个逆时针扭矩那么：1 2 号电机顺时针   右视图：根据上文此时计算出需要顺时针扭矩那么： 3 4 号电机 逆时针 下文又将3 4 号电机扭矩反向，所以是顺时针 */
	bl_ctrl->torque_info.joint_horizontal_torque_temp1_L =
		(bl_ctrl->torque_info.joint_horizontal_torque_L) * (-bl_ctrl->mapping_info.invJ3_L);
	bl_ctrl->torque_info.joint_horizontal_torque_temp2_L =
		(bl_ctrl->torque_info.joint_horizontal_torque_L) * (bl_ctrl->mapping_info.invJ4_L);
	bl_ctrl->torque_info.joint_horizontal_torque_temp1_R =
		(bl_ctrl->torque_info.joint_horizontal_torque_R) * (-bl_ctrl->mapping_info.invJ3_R);
	bl_ctrl->torque_info.joint_horizontal_torque_temp2_R =
		(bl_ctrl->torque_info.joint_horizontal_torque_R) * (bl_ctrl->mapping_info.invJ4_R);

	/* 以1 2 号电机举例：向下支持力1号电机逆时针，2号电机顺时针    3 4 号电机：3号电机顺时针 4号电机逆时针 */
	bl_ctrl->torque_info.joint_vertical_torque_temp1_L =
		(bl_ctrl->torque_info.joint_vertical_torque_L) * (bl_ctrl->mapping_info.invJ1_L);
	bl_ctrl->torque_info.joint_vertical_torque_temp2_L =
		(bl_ctrl->torque_info.joint_vertical_torque_L) * (-bl_ctrl->mapping_info.invJ2_L);
	bl_ctrl->torque_info.joint_vertical_torque_temp1_R =
		(bl_ctrl->torque_info.joint_vertical_torque_R) * (-bl_ctrl->mapping_info.invJ1_R);
	bl_ctrl->torque_info.joint_vertical_torque_temp2_R =
		(bl_ctrl->torque_info.joint_vertical_torque_R) * (bl_ctrl->mapping_info.invJ2_R);
	
	/****************************************/

	bl_ctrl->joint_motor_1.torque_out = +bl_ctrl->torque_info.joint_horizontal_torque_temp1_L + bl_ctrl->torque_info.joint_vertical_torque_temp1_L;
	bl_ctrl->joint_motor_2.torque_out = +bl_ctrl->torque_info.joint_horizontal_torque_temp2_L + bl_ctrl->torque_info.joint_vertical_torque_temp2_L;
	bl_ctrl->joint_motor_3.torque_out = +bl_ctrl->torque_info.joint_horizontal_torque_temp1_R + bl_ctrl->torque_info.joint_vertical_torque_temp1_R;
	bl_ctrl->joint_motor_4.torque_out = +bl_ctrl->torque_info.joint_horizontal_torque_temp2_R + bl_ctrl->torque_info.joint_vertical_torque_temp2_R;

	/* Match the HT protocol's physical torque range before the CAN layer. */
	LimitOutput(bl_ctrl->joint_motor_1.torque_out, T_MIN, T_MAX);
	LimitOutput(bl_ctrl->joint_motor_2.torque_out, T_MIN, T_MAX);
	LimitOutput(bl_ctrl->joint_motor_3.torque_out, T_MIN, T_MAX);
	LimitOutput(bl_ctrl->joint_motor_4.torque_out, T_MIN, T_MAX);
}
fp32 ground_stable_timer = 0;  // 添加这行
void Chassis_Status_Detect(chassis_move_t *detect)
{
	/*--------------------------- Off Ground Detect --------------------------*/

	if (detect->mode.chassis_balancing_mode == BALANCING_READY )
	{
		if (fabs(detect->chassis_posture_info.pitch_angle) >= DANGER_PITCH_ANGLE)
			detect->flag_info.abnormal_flag = 1;
		else if (fabs(detect->chassis_posture_info.foot_speed_KF) < MOVE_LOWER_BOUND &&
				 fabs(detect->chassis_posture_info.pitch_angle) < 0.1f &&
				 detect->flag_info.abnormal_flag)
			detect->flag_info.abnormal_flag = 0;
	}
	Supportive_Force_Cal(detect);
	
	if((detect->flag_info.last_suspend_flag_L ==OFF_GROUND&&detect->flag_info.suspend_flag_L ==ON_GROUND) ||
		(detect->flag_info.last_suspend_flag_R ==OFF_GROUND&&detect->flag_info.suspend_flag_R ==ON_GROUND))
	{
		ground_stable_timer = pdMS_TO_TICKS(450);  // 检测到落地，启动200ms计时器
	}
	if(ground_stable_timer > 0)
	{
		ground_stable_timer--;
		detect->flag_info.suspend_flag_L = ON_GROUND;
		detect->flag_info.suspend_flag_R = ON_GROUND;
	}
	else
	{
		if (detect->mode.jumping_stage == EXTENDING_LEGS)
			detect->flag_info.suspend_flag_L = detect->flag_info.suspend_flag_R = ON_GROUND;
		else
		{
			if( (detect->torque_info.supportive_force_L <= LOWER_SUPPORT_FORCE &&
				detect->chassis_posture_info.leg_length_L > 0.13f ))
				{
					detect->flag_info.suspend_flag_L = OFF_GROUND;	
				}
			else if (detect->torque_info.supportive_force_L > LOWER_SUPPORT_FORCE + 5.0f)  
			// 添加滞回区间，例如+10N的阈值差
			{
				detect->flag_info.suspend_flag_L = ON_GROUND;
			}
			if(( detect->torque_info.supportive_force_R <= LOWER_SUPPORT_FORCE &&
				detect->chassis_posture_info.leg_length_R > 0.13f ))
				{
					detect->flag_info.suspend_flag_R = OFF_GROUND;			
				}
			else if (detect->torque_info.supportive_force_R > LOWER_SUPPORT_FORCE + 5.0f)  
			// 添加滞回区间，例如+10N的阈值差
			{
				detect->flag_info.suspend_flag_R = ON_GROUND;
			}
			if(!(detect->flag_info.suspend_flag_L == OFF_GROUND) && (detect->flag_info.suspend_flag_R == OFF_GROUND))
			{
				detect->flag_info.suspend_flag_L = ON_GROUND;
				detect->flag_info.suspend_flag_R = ON_GROUND;
			}
		}
	}		

	if (detect->flag_info.abnormal_flag == 1 &&
		(detect->flag_info.last_suspend_flag_L == ON_GROUND || detect->flag_info.last_suspend_flag_R == ON_GROUND) &&
		(detect->flag_info.suspend_flag_L == OFF_GROUND || detect->flag_info.suspend_flag_R == OFF_GROUND))
		detect->flag_info.Ignore_Off_Ground = 1;
	else if (detect->flag_info.abnormal_flag != 1)
		detect->flag_info.Ignore_Off_Ground = 0;
	if (detect->flag_info.Ignore_Off_Ground)
	{
		detect->flag_info.suspend_flag_R = ON_GROUND;
		detect->flag_info.suspend_flag_L = ON_GROUND;
	}

	if (fabs(detect->chassis_posture_info.foot_speed_KF) > stablize_foot_speed_threshold &&
		fabs(detect->chassis_posture_info.yaw_gyro) > stablize_yaw_speed_threshold)
		detect->flag_info.stablize_high_flag = 1;
}
void Motor_CMD_Send(chassis_move_t *CMD_Send)
{

	//为保证轮毂电机高相应速度的要求，单独开任务负责给电机发力矩指令
	if (CMD_Send->foot_motor_R.motor_mode != MOTOR_FORCE)
		CMD_Send->foot_motor_R.torque_out = 0.0f;
	if (CMD_Send->foot_motor_L.motor_mode != MOTOR_FORCE)
		CMD_Send->foot_motor_L.torque_out = 0.0f;


	if (CMD_Send->joint_motor_1.motor_mode == MOTOR_FORCE)
		CAN_HT_CMD(0x01, CMD_Send->joint_motor_1.torque_out);
	else
		CAN_HT_CMD(0x01, 0.0);		
	if (CMD_Send->joint_motor_3.motor_mode == MOTOR_FORCE)
		CAN_HT_CMD(0x03, CMD_Send->joint_motor_3.torque_out);
	else
		CAN_HT_CMD(0x03, 0.0);
	vTaskDelay(1);
	if (CMD_Send->joint_motor_2.motor_mode == MOTOR_FORCE)
		CAN_HT_CMD(0x02, CMD_Send->joint_motor_2.torque_out);
	else
		CAN_HT_CMD(0x02, 0.0);
	if (CMD_Send->joint_motor_4.motor_mode == MOTOR_FORCE)
		CAN_HT_CMD(0x04, CMD_Send->joint_motor_4.torque_out);
	else
		CAN_HT_CMD(0x04, 0.0);
}
void Joint_Motor_to_Init_Pos()
{
	static int Init_Time = 0;
	while (Init_Time < 200)
	{
		CAN_HT_CMD(0x01, 0.8);
		vTaskDelay(2);
		CAN_HT_CMD(0x02, -0.8);
		vTaskDelay(2);
		CAN_HT_CMD(0x03, -0.8);
		vTaskDelay(2);
		CAN_HT_CMD(0x04, 0.8);
		vTaskDelay(2);
		Init_Time++;
	}
}
void HT_Motor_zero_set(void)
{
	uint8_t tx_buff[8];
	for (int i = 0; i < 7; i++)
		tx_buff[i] = 0xFF;
	tx_buff[7] = 0xfc;

	CAN_CMD_HT_Enable(0x01, tx_buff);
	vTaskDelay(50);
	CAN_CMD_HT_Enable(0x02, tx_buff);
	vTaskDelay(50);
	CAN_CMD_HT_Enable(0x03, tx_buff);
	vTaskDelay(50);
	CAN_CMD_HT_Enable(0x04, tx_buff);
	vTaskDelay(50);

	Joint_Motor_to_Init_Pos();
	// Set zero init point
	tx_buff[7] = 0xfe;

	CAN_CMD_HT_Enable(0x01, tx_buff);
	vTaskDelay(50);
	CAN_CMD_HT_Enable(0x02, tx_buff);
	vTaskDelay(50);
	CAN_CMD_HT_Enable(0x03, tx_buff);
	vTaskDelay(50);
	CAN_CMD_HT_Enable(0x04, tx_buff);

	vTaskDelay(50);
}
void Motor_Zero_CMD_Send(void)
{
	CAN_HT_CMD(0x01, 0.0);
	vTaskDelay(1);
	CAN_HT_CMD(0x02, 0.0);
	vTaskDelay(1);
	CAN_HT_CMD(0x03, 0.0);
	vTaskDelay(1);
	CAN_HT_CMD(0x04, 0.0);
	vTaskDelay(1);
}
/* -----------------计算腿部支持力----------------- */
void calculate_wheel_vertical_acceleration(chassis_move_t * detect)
{
	detect->chassis_posture_info.chassis_accel = detect->chassis_posture_info.z_accel-g*cos(detect->chassis_posture_info.pitch_angle);
	
	detect->chassis_posture_info.foot_accel_L=
	+detect->chassis_posture_info.chassis_accel
	-detect->chassis_posture_info.leg_ddlength_L*cos(detect->chassis_posture_info.leg_angle_L);
	// +2*detect->chassis_posture_info.leg_dlength_L_jacobian*detect->chassis_posture_info.leg_gyro_L*sin(detect->chassis_posture_info.leg_angle_L)
	// +detect->chassis_posture_info.leg_length_L*detect->chassis_posture_info.leg_accel_L*sin(detect->chassis_posture_info.leg_angle_L)
	// +detect->chassis_posture_info.leg_length_L*detect->chassis_posture_info.leg_gyro_L*detect->chassis_posture_info.leg_gyro_L*cos(detect->chassis_posture_info.leg_angle_L);
	
	detect->chassis_posture_info.foot_accel_R=
	+detect->chassis_posture_info.chassis_accel
	-detect->chassis_posture_info.leg_ddlength_R*cos(detect->chassis_posture_info.leg_angle_R);
	// +2*detect->chassis_posture_info.leg_dlength_R_jacobian*detect->chassis_posture_info.leg_gyro_R*sin(detect->chassis_posture_info.leg_angle_R)
	// +detect->chassis_posture_info.leg_length_R*detect->chassis_posture_info.leg_accel_R*sin(detect->chassis_posture_info.leg_angle_R)
	// +detect->chassis_posture_info.leg_length_R*detect->chassis_posture_info.leg_gyro_R*detect->chassis_posture_info.leg_gyro_R*cos(detect->chassis_posture_info.leg_angle_R);
	
}
//F_N = P + M_w*a + M_w*g 
void Supportive_Force_Cal(chassis_move_t * detect)
{
	//计算腿部支持力
	detect->torque_info.forque_L=
	detect->torque_info.joint_vertical_torque_L*cos(detect->chassis_posture_info.leg_angle_L)
	+detect->torque_info.joint_horizontal_torque_L*sin(detect->chassis_posture_info.leg_angle_L)/detect->chassis_posture_info.leg_length_L;
	detect->torque_info.forque_R=
	detect->torque_info.joint_vertical_torque_R*cos(detect->chassis_posture_info.leg_angle_R)	
	+detect->torque_info.joint_horizontal_torque_R*sin(detect->chassis_posture_info.leg_angle_R)/detect->chassis_posture_info.leg_length_R;
	fp32 temp_L = fp32_constrain(detect->torque_info.forque_L, -100.0f, 100.0f);
	fp32 temp_R = fp32_constrain(detect->torque_info.forque_R, -100.0f, 100.0f);
	//计算加速度环节
	calculate_wheel_vertical_acceleration(detect);
	//支持力计算环节
	detect->torque_info.supportive_force_L=temp_L+m_w*g+m_w*detect->chassis_posture_info.foot_accel_L;
	detect->torque_info.supportive_force_R=temp_R+m_w*g+m_w*detect->chassis_posture_info.foot_accel_R;
	detect->torque_info.supportive_force_L = 0.7f*detect->torque_info.supportive_force_L + 0.3f * detect->torque_info.last_supportive_force_L;
	detect->torque_info.supportive_force_R = 0.7f*detect->torque_info.supportive_force_R + 0.3f * detect->torque_info.last_supportive_force_R;
	detect->torque_info.last_supportive_force_L=detect->torque_info.supportive_force_L;
	detect->torque_info.last_supportive_force_R=detect->torque_info.supportive_force_R;
}
uint8_t leg_length_ready,leg_angle_ready,leg_gyro_stable,pitch_stable,both_feet_on_ground ,speed_flag,prepare_complete;
uint8_t Check_Jump_Preparation_Complete(chassis_move_t *chassis)
{
	if (chassis == NULL) return 0;
    
    // 综合判断
    prepare_complete = 1;    
    return prepare_complete;
}

void handle_airborne_state(chassis_move_t *bl_ctrl)
{
	// 2. 处理 joint_balancing_torque（平衡力矩）
    if (bl_ctrl->flag_info.suspend_flag_R == 1)
    {
		bl_ctrl->torque_info.joint_balancing_torque_R = -(
			+ LQR[3][6] * (bl_ctrl->chassis_posture_info.leg_angle_R_set - bl_ctrl->chassis_posture_info.leg_angle_R)
            + LQR[3][7] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_R) 
        );
        bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
    }
    
    if (bl_ctrl->flag_info.suspend_flag_L == 1)
    {
		bl_ctrl->torque_info.joint_balancing_torque_L = (
			+ LQR[2][4] * (bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L)
            + LQR[2][5] * (0.0f - bl_ctrl->chassis_posture_info.leg_gyro_L) 
        );
        bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
    }
    
    // 3. 处理 foot 相关力矩
    if (bl_ctrl->flag_info.suspend_flag_R == 1)
    {
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
        bl_ctrl->torque_info.foot_moving_torque_R = 0;
    }
    
    if (bl_ctrl->flag_info.suspend_flag_L == 1)
    {
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
        bl_ctrl->torque_info.foot_moving_torque_L = 0;
    }
}

void Forward_kinematic_solution(chassis_move_t *feedback_update,
								fp32 Q1, fp32 S1, fp32 Q4, fp32 S4, uint8_t ce)
{
	fp32 dL0 = 0, L0 = 0, Q0 = 0, S0 = 0;
	fp32 xb, xd, yb, yd, Lbd;
	fp32 A0, B0, C0, Q2, Q3, S2;
	fp32 temp;
	fp32 vxb, vxd, vyb, vyd, vxc, vyc;
	fp32 cos_Q1, cos_Q4, sin_Q1, sin_Q4;
	fp32 xc, yc;
	/******************************/
	Q1 = ((180.0f + Q1) * PI) / 180.0f;
	Q4 = ((180.0f - Q4) * PI) / 180.0f;

	cos_Q1 = cos(Q1);
	sin_Q1 = sin(Q1);
	cos_Q4 = cos(Q4);
	sin_Q4 = sin(Q4);
	xb = -L5 / 2.0f + L1 * cos_Q1;
	xd = L5 / 2.0f - L4 * cos_Q4;
	yb = L1 * sin_Q1;
	yd = L4 * sin_Q4;

	Lbd = (xd - xb) * (xd - xb) + (yd - yb) * (yd - yb);
	A0 = 2.0f * L2 * (xd - xb);
	B0 = 2.0f * L2 * (yd - yb);
	C0 = L2 * L2 + Lbd - L3 * L3;
	temp = A0 * A0 + B0 * B0 - C0 * C0;
	if(temp<0)
	{temp = 0;}
	Q2 = 2.0f * atan2((B0 + sqrt(temp)) , (A0 + C0));

	xc = xb + cos(Q2) * L2;
	yc = yb + sin(Q2) * L2;

	L0 = sqrt(xc * xc + yc * yc);
	Q0 = atan2(xc , yc);

	vxb = -S1 * L1 * sin_Q1;
	vyb = S1 * L1 * cos_Q1;
	vxd = -S4 * L4 * sin_Q4;
	/* Q4 = PI - q4, so dQ4/dt = -S4. */
	vyd = -S4 * L4 * cos_Q4;
	Q3 = atan2((yc - yd) , (xc - xd));
	fp32 velocity_denominator = L2 * sin(Q3 - Q2);
	if (fabs(velocity_denominator) > 1.0e-6f && L0 > 1.0e-6f)
	{
		S2 = ((vxd - vxb) * cos(Q3) + (vyd - vyb) * sin(Q3)) / velocity_denominator;
		vxc = vxb - S2 * L2 * sin(Q2);
		vyc = vyb + S2 * L2 * cos(Q2);
		/* Q0 = atan2(xc, yc), therefore dQ0/dt =
		 * (yc*vxc - xc*vyc)/(xc^2 + yc^2). */
		S0 = (cos(Q0) * vxc - sin(Q0) * vyc) / L0;
		dL0 = (xc * vxc + yc * vyc) / L0;
	}
	else
	{
		/* A singular/invalid five-bar pose must not inject NaN/Inf into LQR. */
		S0 = 0.0f;
		dL0 = 0.0f;
	}


	if (ce)
	{
		feedback_update->chassis_posture_info.last_leg_length_L = feedback_update->chassis_posture_info.leg_length_L;
		feedback_update->chassis_posture_info.leg_length_L = 0.9f*L0+0.1f*feedback_update->chassis_posture_info.last_leg_length_L;
		feedback_update->chassis_posture_info.leg_angle_L = Q0;
		feedback_update->chassis_posture_info.leg_gyro_L = S0;
		feedback_update->chassis_posture_info.leg_dlength_L = dL0;
		feedback_update->chassis_posture_info.leg_dlength_L_jacobian = dL0;
	}
	else
	{
		feedback_update->chassis_posture_info.last_leg_length_R = feedback_update->chassis_posture_info.leg_length_R;
		feedback_update->chassis_posture_info.leg_length_R = 0.9f*L0+0.1f*feedback_update->chassis_posture_info.last_leg_length_R;
		feedback_update->chassis_posture_info.leg_angle_R = -Q0;
		feedback_update->chassis_posture_info.leg_gyro_R = -S0;
		feedback_update->chassis_posture_info.leg_dlength_R = dL0;
		feedback_update->chassis_posture_info.leg_dlength_R_jacobian = dL0;
	}
}

// 计算多项式值
float evaluate_polynomial(float L0, float Q0, PolynomialCoefficients coeffs)
{
	return coeffs.c0 +
		   coeffs.c1 * L0 +
		   coeffs.c2 * Q0 +
		   coeffs.c3 * L0 * L0 +
		   coeffs.c4 * L0 * Q0 +
		   coeffs.c5 * Q0 * Q0;
}

// 计算雅可比矩阵
float get_jacobian_element(chassis_move_t *VMCJ, float L0, float Q0, uint8_t element_type)
{
	switch (element_type)
	{
	case 1: // N11
		return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N11);
	case 2: // N12
		return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N12);
	case 3: // N21
		return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N21);
	case 4: // N22
		return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N22);
	default:
		return 0.0f; // 或者返回错误值
	}
}
