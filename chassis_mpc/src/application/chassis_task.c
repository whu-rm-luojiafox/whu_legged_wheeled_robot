#include "chassis_task.h"
#include "chassis_follow.h"
#include "chassis_motion.h"
#include "chassis_balance.h"
#include "chassis_leg_math.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "detect_task.h"
#include "INS_task.h"
#include "vofa.h"
#include "LQR.h"
#include "usart.h"
#include "user_lib.h"
#include "uart_receive.h"
//功能类头文件包含
#include "chassis_power.h"
#include "chassis_contact.h"
#include "chassis_recovery.h"
#include "chassis_climb.h"

/* 阅读入口：
 * 本文件负责 RTOS 执行顺序、模式切换、传感器/电机接入和最终输出。
 * 调参量、模块状态和衔接留在本文件，编译参数集中在 chassis_config.h。
 * 腿部数学、运动、平衡、跟随按完整功能分组，见根目录 README.md。
 */
#define square(x) ((x) * (x))
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#define min(a, b) ((a) < (b) ? (a) : (b))

/* Verify that negative calibrated leg angles point toward the rear. */
#define FOOT_LAUNCH_LEG_TARGET    (-0.15f)
#define FOOT_LAUNCH_LEG_MIN       (-0.23f)
#define FOOT_LAUNCH_LEG_MAX       PI_2
#define FOOT_LAUNCH_LEG_GYRO_MAX  0.3f
#define FOOT_LAUNCH_LEG_LENGTH    0.30f

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

#define MODEL_PYH_WHEEL_DIR -1
int16_t cmd_L,cmd_R;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

float a1,b1,c1,d1;
float a2,b2,c2,d2;
int temp;

// 底盘运动数据
chassis_move_t chassis_move;
static TickType_t last_control_tick;

/* 在线调参请求：保留独立 section，供调试器从 ELF 解析类型。
 * 以下 PID 参数每周期在反馈阶段应用，复位恢复 chassis_config.h 中的默认值。 */
volatile fp32 tune_leg_ki __attribute__((section(".data.tune_leg_ki"))) = LEG_SET_PID_KI;
volatile fp32 tune_leg_i_limit __attribute__((section(".data.tune_leg_i_limit"))) = LEG_SET_PID_IOUT;
volatile fp32 tune_roll_ki __attribute__((section(".data.tune_roll_ki"))) = ROLL_PID_KI;
volatile fp32 tune_roll_i_limit __attribute__((section(".data.tune_roll_i_limit"))) = ROLL_PID_IOUT;

/* 云台跟随外环。 */
volatile fp32 tune_follow_kp __attribute__((section(".data.tune_follow_kp"))) = FOLLOW_KP_RAD_S_PER_DEG;
volatile fp32 tune_follow_rate_limit __attribute__((section(".data.tune_follow_rate_limit"))) = FOLLOW_MAX_RATE_RAD_S;
volatile fp32 tune_follow_accel __attribute__((section(".data.tune_follow_accel"))) = FOLLOW_RATE_ACCEL_RAD_S2;
volatile fp32 tune_follow_filter_s __attribute__((section(".data.tune_follow_filter_s"))) = FOLLOW_ERROR_FILTER_S;
volatile fp32 tune_follow_yaw_lead_s __attribute__((section(".data.tune_follow_yaw_lead_s"))) = FOLLOW_YAW_LEAD_S;

/* 实验前馈，默认关闭。 */
volatile uint8_t tune_com_ff_enable __attribute__((section(".data.tune_com_ff_enable"))) = 0u;
volatile fp32 tune_com_ff_moment __attribute__((section(".data.tune_com_ff_moment"))) = 0.0f;

/* 编码器零点和独立姿态参考。 */
volatile fp32 tune_gimbal_forward_deg __attribute__((section(".data.tune_gimbal_forward_deg"))) = CHASSIS_GIMBAL_FORWARD_DEG;
volatile fp32 tune_balance_pitch_rad __attribute__((section(".data.tune_balance_pitch_rad"))) = BALANCE_PITCH_DEFAULT_RAD;
volatile fp32 tune_balance_leg_ground_rad __attribute__((section(".data.tune_balance_leg_ground_rad"))) = BALANCE_LEG_GROUND_DEFAULT_RAD;

/* 原实现省略对侧腿反馈。默认保留，先观察 cross_leg 诊断量再单独试调。 */
volatile fp32 tune_lqr_cross_leg_scale __attribute__((section(".data.tune_lqr_cross_leg_scale"))) = 0.0f;

chassis_follow_t chassis_follow __attribute__((section(".bss.chassis_follow")));
chassis_odometry_t chassis_odometry __attribute__((section(".bss.chassis_odometry")));
chassis_position_t chassis_position __attribute__((section(".bss.chassis_position")));
chassis_feedforward_t chassis_feedforward __attribute__((section(".bss.chassis_feedforward")));
chassis_balance_reference_t chassis_balance_reference __attribute__((section(".bss.chassis_balance_reference")));
chassis_lqr_feedback_t chassis_lqr_feedback __attribute__((section(".bss.chassis_lqr_feedback")));
float chassis_control_dt_s __attribute__((section(".bss.chassis_control_dt_s")));

/* ------------------------PID info------------------------ */
fp32 jump_stand_PD_L[2] = {30000000.0f, 10.0f};
fp32 jump_stand_PD_R[2] = {30000000.0f, 10.0f};

fp32 jump_canstacting_PD_L[2] = {30000000.0f, 10.0f};
fp32 jump_canstacting_PD_R[2] = {30000000.0f, 10.0f};

/* ------------------------平步数据------------------------ */
fp32 delta;
float alpha_da = 0.7f;

fp32 IDEAL_PREPARING_STAND_JUMPING_ANGLE = 0.174532f;//0.0872f;
fp32 rotate_move_threshold = 45.0f;
fp32 rotate_speed_list[11] = {0.0, 5.0, 5.3, 5.6, 6.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0};
fp32 move_scale_list[11] = {0.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
fp32 rotate_move_scale_list[11] = {0.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8};

fp32 rollP, rollD, rollI, roll_angle_deadband = 0.004f, roll_gyro_deadband = 0.005f, leg_dlength_deadband = 0.0f;
fp32 normalized_speed,move_angle;
fp32 normal_move_scale __attribute__((section(".data.normal_move_scale"))) = 1.0f;
extern gimbal_control_t gimbal_control;

/* ----------------------function definition ------------------------ */
static void apply_integral_tuning(pid_type_def *pid, fp32 ki, fp32 limit);
static void chassis_apply_pid_tuning(chassis_move_t *chassis);
static uint8_t chassis_is_enabled_on_ground(const chassis_move_t *chassis);
static uint8_t chassis_is_balancing_on_ground(const chassis_move_t *chassis);
static void chassis_update_stop_position(chassis_move_t *chassis);
static void chassis_apply_com_feedforward(chassis_move_t *chassis);
static void chassis_update_yaw_follow(chassis_move_t *chassis, float move_deg);
static void chassis_update_wheel_odometry(chassis_move_t *fdb);
static void chassis_calculate_lqr_feedback(const chassis_move_t *chassis, const float gain[4][10]);
void Joint_Motor_to_Init_Pos(void);
void Motor_Zero_CMD_Send(void);
void HT_Motor_zero_set(void);
static void chassis_init(chassis_move_t *chassis_move_init);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
void Target_Value_Set(chassis_move_t *target_value_set);
void Chassis_Torque_Calculation(chassis_move_t *bl_ctrl);
void Chassis_Torque_Combine(chassis_move_t *bl_ctrl);
void Motor_CMD_Send(chassis_move_t *CMD_Send);
uint8_t Check_Jump_Preparation_Complete(chassis_move_t *chassis);

uint8_t tmp =0;	

void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);

	//初始化电机 传感器数据获取
	chassis_init(&chassis_move);
	last_control_tick = xTaskGetTickCount();

	while (1)
	{
		chassis_feedback_update(&chassis_move);
		Chassis_Status_Detect(&chassis_move);
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);

		/* 参考：平移、停车位置、yaw 跟随；特殊动作后更新姿态偏置。 */
		Target_Value_Set(&chassis_move);
		chassisClimbProcess(&chassis_move);

		/* 力矩：腿长/roll + LQR -> 虚拟腿前馈 -> VMC/电机限幅。 */
		Chassis_Torque_Calculation(&chassis_move);
		chassis_apply_com_feedforward(&chassis_move);
		Chassis_Torque_Combine(&chassis_move);

		/* 输出：自救接管 -> 功率限制 -> 电机指令。 */
		chassisRecoveryProcess(&chassis_move);
		chassis_power_limit(&chassis_move);
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
	
	vTaskDelay(3000);
	// HT_Motor_zero_set();
	// Motor_Zero_CMD_Send();
	// CAN_clear_dm_error();
	for(uint8_t i =1;i<=4;i++ )
	{
		CAN_dm_enable(i);
		vTaskDelay(2);
	}
	// CAN_dm_save_0_point(4);
	/* -------------------------param get---------------------------- */
	chassis_move_init->joint_motor_1.motor_measure = get_DM_motor_measure_point(0);
	chassis_move_init->joint_motor_2.motor_measure = get_DM_motor_measure_point(1);
	chassis_move_init->joint_motor_3.motor_measure = get_DM_motor_measure_point(2);
	chassis_move_init->joint_motor_4.motor_measure = get_DM_motor_measure_point(3);
	chassis_move_init->foot_motor_L.motor_measure = get_chassis_motor_measure_point(0);
	chassis_move_init->foot_motor_R.motor_measure = get_chassis_motor_measure_point(1);
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

	// 初始化yaw角度PID

	// 初始化腿长PID
	const static fp32 leg_length_pid[3] = {LEG_SET_PID_KP, LEG_SET_PID_KI, LEG_SET_PID_KD};
	PID_init(&chassis_move_init->leg_L_length_pid, PID_POSITION, leg_length_pid, LEG_SET_PID_OUT, LEG_SET_PID_IOUT);
	PID_init(&chassis_move_init->leg_R_length_pid, PID_POSITION, leg_length_pid, LEG_SET_PID_OUT, LEG_SET_PID_IOUT);
	//初始化roll控制pid
	const static fp32 chassis_roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	PID_init(&chassis_move_init->chassis_roll_pid, PID_POSITION, chassis_roll_pid, ROLL_PID_OUT, ROLL_PID_IOUT);

	chassis_move_init->flag_info.init_flag = 1;
	chassis_feedback_update(chassis_move_init);
	chassis_move_init->flag_info.init_flag = 0;

	chassis_move_init->gimbal_yaw_motor.relative_angle_init = CHASSIS_GIMBAL_FORWARD_DEG;
}
void chassis_feedback_update(chassis_move_t *fdb)
{
	if (fdb == NULL)
	{
		return;
	}

	/* 用实际两次反馈的间隔积分，包含电机发送等待；不能只按末尾 delay=2 ms。
	 * 先应用在线积分参数，再读取反馈，保持原控制周期内的执行顺序。 */
	const TickType_t now = xTaskGetTickCount();
	chassis_control_dt_s = (float)(TickType_t)(now - last_control_tick) /
		(float)configTICK_RATE_HZ;
	last_control_tick = now;
	chassis_apply_pid_tuning(fdb);

	/*------------------------------- Update mode info ----------------------------*/
	fdb->mode.last_chassis_mode = fdb->mode.chassis_mode;
	fdb->mode.last_chassis_balancing_mode = fdb->mode.chassis_balancing_mode;
	fdb->mode.last_sport_mode = fdb->mode.sport_mode;
	fdb->mode.last_jumping_mode = fdb->mode.jumping_mode;
	fdb->mode.last_jumping_stage = fdb->mode.jumping_stage;
	fdb->mode.last_chassis_high_mode = fdb->mode.chassis_high_mode;

	fdb->flag_info.last_overpower_warning_flag = fdb->flag_info.overpower_warning_flag;

	fdb->flag_info.last_suspend_flag_L = fdb->flag_info.suspend_flag_L;
	fdb->flag_info.last_suspend_flag_R = fdb->flag_info.suspend_flag_R;

	fdb->joint_motor_1.last_motor_mode = fdb->joint_motor_1.motor_mode;
	fdb->joint_motor_2.last_motor_mode = fdb->joint_motor_2.motor_mode;
	fdb->joint_motor_3.last_motor_mode = fdb->joint_motor_3.motor_mode;
	fdb->joint_motor_4.last_motor_mode = fdb->joint_motor_4.motor_mode;
	fdb->foot_motor_L.last_motor_mode = fdb->foot_motor_L.motor_mode;
	fdb->foot_motor_R.last_motor_mode = fdb->foot_motor_R.motor_mode;

	/*------------------------------- Update HT Motor info ------------------------------ */
	/* 1~4 号髋关节电机统一约定：从各自电机轴侧观察，逆时针位置、
	 * 速度和输出力矩为正。下式保留驱动反馈的原始正方向。
	 */
	fdb->joint_motor_1.position = fdb->joint_motor_1.motor_measure->pos;
	fdb->joint_motor_2.position = fdb->joint_motor_2.motor_measure->pos;
	fdb->joint_motor_3.position = fdb->joint_motor_3.motor_measure->pos;
	fdb->joint_motor_4.position = fdb->joint_motor_4.motor_measure->pos;

	fdb->joint_motor_1.velocity = fdb->joint_motor_1.motor_measure->vel;
	fdb->joint_motor_2.velocity = fdb->joint_motor_2.motor_measure->vel;
	fdb->joint_motor_3.velocity = fdb->joint_motor_3.motor_measure->vel;
	fdb->joint_motor_4.velocity = fdb->joint_motor_4.motor_measure->vel;

	//更新力矩反馈
	fdb->joint_motor_1.torque_get = fdb->joint_motor_1.motor_measure->torque;
	fdb->joint_motor_2.torque_get = fdb->joint_motor_2.motor_measure->torque;
	fdb->joint_motor_3.torque_get = fdb->joint_motor_3.motor_measure->torque;
	fdb->joint_motor_4.torque_get = fdb->joint_motor_4.motor_measure->torque;
	fdb->foot_motor_L.speed = -fdb->foot_motor_L.motor_measure->speed_rpm *  2*PI / 60 / (268.0f / 17.0f) * WHEEL_RADIUS; // dps -> m/s
	fdb->foot_motor_R.speed = fdb->foot_motor_R.motor_measure->speed_rpm  * 2*PI / 60 / (268.0f / 17.0f) * WHEEL_RADIUS;
	fdb->chassis_posture_info.foot_speed = (fdb->foot_motor_L.speed + fdb->foot_motor_R.speed) / 2.0f;

	/* 保留旧观察变量名，实际估计由轮速里程模块负责，没有 KF。 */
	chassis_update_wheel_odometry(fdb);

	//足端角度解算
	chassis_leg_forward_kinematics(&fdb->chassis_posture_info.chassis_posture_L,
							   fdb->joint_motor_1.position, fdb->joint_motor_1.velocity,
							   fdb->joint_motor_2.position, fdb->joint_motor_2.velocity);
	chassis_leg_forward_kinematics(&fdb->chassis_posture_info.chassis_posture_R,
							   fdb->joint_motor_4.position, fdb->joint_motor_4.velocity,
							   fdb->joint_motor_3.position, fdb->joint_motor_3.velocity);
	
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

	/* Publish only ground-frame feedback. Writing the local value first lets
	 * asynchronous debugger sampling observe an intermediate coordinate frame.
	 * The right leg's local angle/rate has the opposite model sign. */
	fdb->chassis_posture_info.chassis_posture_L.leg_angle =
		fdb->chassis_posture_info.chassis_posture_L.leg_angle_local + fdb->chassis_posture_info.pitch_angle;
	fdb->chassis_posture_info.chassis_posture_R.leg_angle =
		-fdb->chassis_posture_info.chassis_posture_R.leg_angle_local + fdb->chassis_posture_info.pitch_angle;
	fdb->chassis_posture_info.chassis_posture_L.leg_gyro =
		fdb->chassis_posture_info.chassis_posture_L.leg_gyro_local + fdb->chassis_posture_info.pitch_gyro;
	fdb->chassis_posture_info.chassis_posture_R.leg_gyro =
		-fdb->chassis_posture_info.chassis_posture_R.leg_gyro_local + fdb->chassis_posture_info.pitch_gyro;
	
	// 计算加速度
	if (fdb->flag_info.init_flag)
	{
		/* Establish a derivative baseline before off-ground detection starts. */
		fdb->chassis_posture_info.chassis_posture_L.leg_ddlength = 0.0f;
		fdb->chassis_posture_info.chassis_posture_R.leg_ddlength = 0.0f;
		fdb->chassis_posture_info.chassis_posture_L.last_leg_dlength_jacobian = fdb->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian;
		fdb->chassis_posture_info.chassis_posture_R.last_leg_dlength_jacobian = fdb->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian;
	}
	else
	{
		fp32 temp_a_L = (fdb->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian - fdb->chassis_posture_info.chassis_posture_L.last_leg_dlength_jacobian) / CHASSIS_CONTROL_TIME;
		fdb->chassis_posture_info.chassis_posture_L.leg_ddlength = alpha_da * temp_a_L + (1-alpha_da) * fdb->chassis_posture_info.chassis_posture_L.last_leg_ddlength;
		fp32 temp_a_R = (fdb->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian - fdb->chassis_posture_info.chassis_posture_R.last_leg_dlength_jacobian) / CHASSIS_CONTROL_TIME;
		fdb->chassis_posture_info.chassis_posture_R.leg_ddlength = alpha_da * temp_a_R + (1-alpha_da) * fdb->chassis_posture_info.chassis_posture_R.last_leg_ddlength;
	}
	
	fdb->chassis_posture_info.chassis_posture_L.last_leg_angle = fdb->chassis_posture_info.chassis_posture_L.leg_angle;
	fdb->chassis_posture_info.chassis_posture_R.last_leg_angle = fdb->chassis_posture_info.chassis_posture_R.leg_angle;
	fdb->chassis_posture_info.chassis_posture_L.last_leg_length = fdb->chassis_posture_info.chassis_posture_L.leg_length;
	fdb->chassis_posture_info.chassis_posture_R.last_leg_length = fdb->chassis_posture_info.chassis_posture_R.leg_length;	
	fdb->chassis_posture_info.chassis_posture_L.last_leg_dlength_jacobian = fdb->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian;
	fdb->chassis_posture_info.chassis_posture_R.last_leg_dlength_jacobian = fdb->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian;
	fdb->chassis_posture_info.chassis_posture_L.last_leg_ddlength = fdb->chassis_posture_info.chassis_posture_L.leg_ddlength;
	fdb->chassis_posture_info.chassis_posture_R.last_leg_ddlength = fdb->chassis_posture_info.chassis_posture_R.leg_ddlength;
	//云台相对角度更新
	fdb->gimbal_yaw_motor.relative_angle = theta_format(fdb->gimbal_yaw_motor.gimbal_motor_measure->angle);
	//合成 x y 方向速度以及限幅
	if(fdb->chassis_data_->vx_set>=0)normalized_speed = fp32_constrain(sqrt(fdb->chassis_data_->vx_set*fdb->chassis_data_->vx_set+fdb->chassis_data_->vy_set* fdb->chassis_data_->vy_set),-2.8f,2.8f);
	else normalized_speed = -fp32_constrain(sqrt(fdb->chassis_data_->vx_set*fdb->chassis_data_->vx_set+fdb->chassis_data_->vy_set* fdb->chassis_data_->vy_set),-2.8f,2.8f);
	
	move_angle = atan2f(fdb->chassis_data_->vy_set,fdb->chassis_data_->vx_set) * 180 /PI;
	if(move_angle>90) move_angle -= 180.0f;
	else if (move_angle<-90) move_angle += 180.0f;
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}
	/* --------------------------------set chassis mode -------------------------------- */
	/* The retired debug protocol value now requests zero force. */
	if (chassis_move_mode->chassis_data_->chassis_mode == CHASSIS_MODE_OFF ||
		chassis_move_mode->chassis_data_->chassis_mode == CHASSIS_MODE_DEBUG)
		chassis_move_mode->mode.chassis_mode = DISABLE_CHASSIS;
	else
		chassis_move_mode->mode.chassis_mode = ENABLE_CHASSIS;

	if (chassis_move_mode->mode.chassis_mode == ENABLE_CHASSIS)
	{
		
		if(tmp ==0)
		{
			CAN_clear_dm_error();
			CAN_dm_enable(1);
			CAN_dm_enable(2);
			CAN_dm_enable(3);
			CAN_dm_enable(4);
			tmp =1;
		}

		chassis_move_mode->joint_motor_1.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_2.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_3.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_4.motor_mode = MOTOR_FORCE;
		chassis_move_mode->foot_motor_L.motor_mode = MOTOR_FORCE;
		chassis_move_mode->foot_motor_R.motor_mode = MOTOR_FORCE;
		uint32_t current_tick = xTaskGetTickCount();
		uint32_t timeout_threshold = pdMS_TO_TICKS(100);  // 100ms超时

		// if (current_tick - chassis_move_mode->foot_motor_L.motor_measure->last_update_time > timeout_threshold ||
		// 	current_tick - chassis_move_mode->foot_motor_R.motor_measure->last_update_time > timeout_threshold)
		// {
		// 	// 轮毂电机离线，让髋关节失能
		// 	chassis_move_mode->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
		// 	chassis_move_mode->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
		// 	chassis_move_mode->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
		// 	chassis_move_mode->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
		// 	chassis_move_mode->foot_motor_L.motor_mode = MOTOR_NO_FORCE;
		// 	chassis_move_mode->foot_motor_R.motor_mode = MOTOR_NO_FORCE;
		// }
	}
	else
	{
		if(tmp ==1)
		{
			tmp =0;
		}
		chassis_move_mode->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->foot_motor_L.motor_mode = MOTOR_NO_FORCE;
		chassis_move_mode->foot_motor_R.motor_mode = MOTOR_NO_FORCE;
	}

	/*-------------------------- Sport Mode Update ----------------------------------*/
	if (chassis_move_mode->mode.chassis_balancing_mode == BALANCING_READY)
	{
		if (chassis_move_mode->mode.sport_mode == JUMPING_MODE && chassis_move_mode->mode.jumping_stage != FINISHED)
			chassis_move_mode->mode.sport_mode = JUMPING_MODE;
		else if (chassis_move_mode->chassis_data_->jump_flag)
			chassis_move_mode->mode.sport_mode = JUMPING_MODE;
		else
			chassis_move_mode->mode.sport_mode = NORMAL_MOVING_MODE;
	}
	else
		chassis_move_mode->mode.sport_mode = NONE;
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_mode_change)
{
	if (chassis_mode_change == NULL)
	{
		return;
	}
	/* --------------------------------使能/失能 后判断是否 进入/退出 平衡模式--------------------------------  */
	if (chassis_mode_change->mode.chassis_mode == DISABLE_CHASSIS)
		chassis_mode_change->mode.chassis_balancing_mode = NO_FORCE;
	else if (chassis_mode_change->mode.last_chassis_mode == DISABLE_CHASSIS)
	{
		chassis_mode_change->mode.chassis_balancing_mode = FOOT_LAUNCHING;
		chassis_mode_change->chassis_posture_info.yaw_angle_sett = chassis_mode_change->chassis_posture_info.yaw_angle_total;
	}
	else if (
		chassis_mode_change->mode.chassis_balancing_mode == FOOT_LAUNCHING)
	{
		chassis_leg_posture_t *L = &chassis_mode_change->chassis_posture_info.chassis_posture_L;
		chassis_leg_posture_t *R = &chassis_mode_change->chassis_posture_info.chassis_posture_R;

		/* Both legs must enter the allowed range and slow down. */
		if (L->leg_angle > FOOT_LAUNCH_LEG_MIN && L->leg_angle < FOOT_LAUNCH_LEG_MAX &&
			R->leg_angle > FOOT_LAUNCH_LEG_MIN && R->leg_angle < FOOT_LAUNCH_LEG_MAX &&
			fabsf(L->leg_gyro) < FOOT_LAUNCH_LEG_GYRO_MAX &&
			fabsf(R->leg_gyro) < FOOT_LAUNCH_LEG_GYRO_MAX &&
			fabsf(chassis_mode_change->chassis_posture_info.pitch_angle) < EXIT_PITCH_ANGLE &&
			fabsf(chassis_mode_change->chassis_posture_info.roll_angle) < 0.25f)
		{
			chassis_mode_change->mode.chassis_balancing_mode = JOINT_LAUNCHING;
		}
	}
	else if (chassis_mode_change->mode.chassis_balancing_mode == JOINT_LAUNCHING)
		chassis_mode_change->mode.chassis_balancing_mode = BALANCING_READY;
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
                if (chassis_mode_change->chassis_posture_info.chassis_posture_L.leg_length >= 0.34f && 
                    chassis_mode_change->chassis_posture_info.chassis_posture_R.leg_length >= 0.34f&&
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
                if (chassis_mode_change->chassis_posture_info.chassis_posture_L.leg_length <= 0.11f&&chassis_mode_change->chassis_posture_info.chassis_posture_R.leg_length <= 0.11f)
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

/* 普通位置锁与前馈共用控制权判定，特殊动作不沿用普通控制的参考。 */

void Target_Value_Set(chassis_move_t *target_value_set)
{

	//底盘正方向速度控制
	if (target_value_set->mode.sport_mode != NONE &&
	target_value_set->flag_info.suspend_flag_L == ON_GROUND &&
	target_value_set->flag_info.suspend_flag_R == ON_GROUND)
	{
		if(fabs(target_value_set->chassis_data_->wz_set) < CHASSIS_RC_WZ_DEADLINE)
		{
			target_value_set->chassis_posture_info.foot_speed_set = chassis_position_speed_request(
				fp32_constrain(normalized_speed * normal_move_scale, -2.2f, 2.2f));
			chassis_power_limit(target_value_set);
		}
		else
		{
			/* A manual spin must not reuse the previous translation command. */
			target_value_set->chassis_posture_info.foot_speed_set = 0.0f;
		}
	}
	else 
	{
		target_value_set->chassis_posture_info.foot_speed_set=0;
	}

	//位置环控制器
	chassis_update_stop_position(target_value_set);

	chassis_update_yaw_follow(target_value_set, move_angle);

	target_value_set->chassis_posture_info.roll_angle_set = 0.0f;

	// ---------------- Leg Angle Set ----------
	if (target_value_set->mode.jumping_stage == PREPARING_STAND_JUMPING)
	{
		target_value_set->chassis_posture_info.chassis_posture_L.leg_angle_set =  IDEAL_PREPARING_STAND_JUMPING_ANGLE;
		target_value_set->chassis_posture_info.chassis_posture_R.leg_angle_set =  IDEAL_PREPARING_STAND_JUMPING_ANGLE;
	}
	else
	{
		target_value_set->chassis_posture_info.chassis_posture_L.leg_angle_set = 0.0f;
		target_value_set->chassis_posture_info.chassis_posture_R.leg_angle_set = 0.0f;
	}

	// ----------------- Chassis High Mode Update -----------------
	target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	// --------- Leg Length Set ---------
	if (target_value_set->mode.chassis_high_mode == NORMAL_MODE)
		target_value_set->chassis_posture_info.ideal_high = fp32_constrain(target_value_set->chassis_data_->high_set,0.17,0.34);
	// 跳跃阶段优先于其他模式设定腿长
	if (target_value_set->mode.sport_mode == JUMPING_MODE)
	{
		switch(target_value_set->mode.jumping_stage)
		{
			case PREPARING_STAND_JUMPING:
				// 跳跃准备阶段：设定较低的腿长用于蓄力
				target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = 0.13f;
				target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = 0.13f;
				break;
				
			case EXTENDING_LEGS:
				// 起跳阶段：快速伸腿
				target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = 0.34f;
				target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = 0.34f;
				break;
				
			case CONSTACTING_LEGS_2:
				// 空中收缩阶段：准备落地
				target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = 0.10f;
				target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = 0.10f;
				break;
				
			case PREPARING_LANDING:
				// 落地准备阶段：保持较低腿长
				target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = 0.15f;
				target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = 0.15f;
				break;
				
			default:
				// 其他跳跃阶段使用理想高度
				target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = target_value_set->chassis_posture_info.ideal_high;
				target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = target_value_set->chassis_posture_info.ideal_high;
				break;
		}
	}
	// ==========================================
	else if (target_value_set->flag_info.suspend_flag_L == OFF_GROUND &&
		target_value_set->flag_info.suspend_flag_R == OFF_GROUND)
	{
		target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = target_value_set->chassis_posture_info.ideal_high;
		target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = target_value_set->chassis_posture_info.ideal_high;
	}
	else
	{
		//目前表现良好
		target_value_set->chassis_posture_info.foot_roll_angle =
			target_value_set->chassis_posture_info.roll_angle;
		target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set =
			target_value_set->chassis_posture_info.ideal_high - 0.24f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
		target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set =
			target_value_set->chassis_posture_info.ideal_high + 0.24f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
	}

	/* Keep the legs short while aligning; normal height resumes afterwards. */
	if (target_value_set->mode.chassis_balancing_mode == FOOT_LAUNCHING)
	{
		target_value_set->chassis_posture_info.chassis_posture_L.leg_length_set = FOOT_LAUNCH_LEG_LENGTH;
		target_value_set->chassis_posture_info.chassis_posture_R.leg_length_set = FOOT_LAUNCH_LEG_LENGTH;
	}
}
void Chassis_Torque_Calculation(chassis_move_t *bl_ctrl)
{
	chassis_balance_reference_step(&chassis_balance_reference,
		bl_ctrl->mode.chassis_mode == ENABLE_CHASSIS,
		tune_balance_pitch_rad, tune_balance_leg_ground_rad, chassis_control_dt_s);

	//LQR拟合矩阵数据更新
	LQR_Data_Update(bl_ctrl);
	if (bl_ctrl->mode.chassis_balancing_mode == FOOT_LAUNCHING)
	{
		chassis_leg_posture_t *L = &bl_ctrl->chassis_posture_info.chassis_posture_L;
		chassis_leg_posture_t *R = &bl_ctrl->chassis_posture_info.chassis_posture_R;
		torque_info_t *t = &bl_ctrl->torque_info;
		/* Right-leg mirroring is handled by Chassis_Torque_Combine. */
		if(L->leg_angle<-PI/10)t->joint_balancing_torque_L = -4;
		else if(L->leg_angle>PI/10)t->joint_balancing_torque_L = 0;
		if(R->leg_angle<-PI/10)t->joint_balancing_torque_R = -4;
		else if(R->leg_angle>PI/10)t->joint_balancing_torque_R = 0;
		t->joint_stand_torque_L = fp32_constrain(
			200.0f * (L->leg_length_set - L->leg_length) - 40.0f * L->leg_dlength_jacobian,
			-8.0f, 8.0f);
		t->joint_stand_torque_R = fp32_constrain(
			200.0f * (R->leg_length_set - R->leg_length) - 40.0f * R->leg_dlength_jacobian,
			-8.0f, 8.0f);

		/* Clear previous-cycle outputs before skipping the LQR branches. */
		t->joint_moving_torque_L = t->joint_moving_torque_R = 0.0f;
		t->joint_roll_torque_L = t->joint_roll_torque_R = 0.0f;
		t->foot_balancing_torque_L = t->foot_balancing_torque_R = 0.0f;
		t->foot_moving_torque_L = t->foot_moving_torque_R = 0.0f;
		return;
	}
	//不同情况下roll轴控制
	if (bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 )
	{
		bl_ctrl->torque_info.joint_roll_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_roll_torque_L = 0.0f;
	}
	else
	{
		PID_calc(&bl_ctrl->chassis_roll_pid, bl_ctrl->chassis_posture_info.roll_angle,bl_ctrl->chassis_posture_info.roll_angle_set);
		/* Both leg force coordinates use positive supporting force. Roll
		 * correction must redistribute support, preserving its total.
		 * Negative roll already requests a longer left leg in Target_Value_Set. */
		bl_ctrl->torque_info.joint_roll_torque_R = -bl_ctrl->chassis_roll_pid.out;
		bl_ctrl->torque_info.joint_roll_torque_L = bl_ctrl->chassis_roll_pid.out;
	}

	//不同情况下垂直方向力控制
	if( bl_ctrl->mode.jumping_stage == EXTENDING_LEGS )
	{
		bl_ctrl->torque_info.joint_stand_torque_L =
			+ jump_stand_PD_L[0] * ( bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length_set - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length )
			+ jump_stand_PD_L[1] * ( 0 - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian );

		bl_ctrl->torque_info.joint_stand_torque_R =
			+ jump_stand_PD_R[0] * ( bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length_set - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length )
			+ jump_stand_PD_R[1] * ( 0 - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian );
	}
	else if(bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2)
	{
		bl_ctrl->torque_info.joint_stand_torque_L =
			+ jump_canstacting_PD_L[0] * ( bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length_set - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length )
			+ jump_canstacting_PD_L[1] * ( 0 - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian );

		bl_ctrl->torque_info.joint_stand_torque_R =
			+ jump_canstacting_PD_R[0] * ( bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length_set - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length )
			+ jump_canstacting_PD_R[1] * ( 0 - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian );
	}
	else if( bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 )
	{
		if( bl_ctrl->flag_info.suspend_flag_L == 1 )
		{
			PID_calc(&bl_ctrl->leg_L_length_pid, bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length,bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length_set);
			bl_ctrl->torque_info.joint_stand_torque_L = 0.6*FEED_f + bl_ctrl->leg_L_length_pid.out;		
		}
		else{
			PID_calc(&bl_ctrl->leg_L_length_pid, bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length,bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length_set);
			bl_ctrl->torque_info.joint_stand_torque_L = FEED_f+bl_ctrl->leg_L_length_pid.out;
		}

		if( bl_ctrl->flag_info.suspend_flag_R == 1 )
		{ 
			PID_calc(&bl_ctrl->leg_R_length_pid, bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length,bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length_set);
			bl_ctrl->torque_info.joint_stand_torque_R = 0.6*FEED_f + bl_ctrl->leg_R_length_pid.out;
		} 
		else {
			PID_calc(&bl_ctrl->leg_R_length_pid, bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length,bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length_set);
			bl_ctrl->torque_info.joint_stand_torque_R = FEED_f+bl_ctrl->leg_R_length_pid.out;
		}
	}
	else
	{
		//普通情况下加重力补偿
		PID_calc(&bl_ctrl->leg_L_length_pid, bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length,bl_ctrl->chassis_posture_info.chassis_posture_L.leg_length_set);
		bl_ctrl->torque_info.joint_stand_torque_L = FEED_f+bl_ctrl->leg_L_length_pid.out;
		PID_calc(&bl_ctrl->leg_R_length_pid, bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length,bl_ctrl->chassis_posture_info.chassis_posture_R.leg_length_set);
		bl_ctrl->torque_info.joint_stand_torque_R = FEED_f+bl_ctrl->leg_R_length_pid.out;
	}
	//添加被动的检测到离地的相应操作
	/* 统一模型误差顺序，避免各输出复制公式时遗漏状态。 */
	chassis_calculate_lqr_feedback(bl_ctrl, LQR);
	if (bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2)
	{
		//此时处于空中，想要达到的效果是腿的度跟地面是保持垂直，
		//不在控制机身角度因为机身保持平衡的力矩其实与腿部保持竖直力矩相冲突，在没有地面支持力的情况下没有意义
		bl_ctrl->torque_info.joint_balancing_torque_L = (
			+ LQR[2][4] * (bl_ctrl->chassis_posture_info.chassis_posture_L.leg_angle_set - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_angle)
			+ LQR[2][5] * (0.0f - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_gyro) 
		);		
		bl_ctrl->torque_info.joint_balancing_torque_R = (
			+ LQR[3][6] * (bl_ctrl->chassis_posture_info.chassis_posture_R.leg_angle_set - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_angle) 
			+ LQR[3][7] * (0.0f - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_gyro) 
		);

		bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
	}

	else
	{
		bl_ctrl->torque_info.joint_balancing_torque_L = chassis_lqr_feedback.balance[2];
		bl_ctrl->torque_info.joint_balancing_torque_R = chassis_lqr_feedback.balance[3];
		bl_ctrl->torque_info.joint_moving_torque_L = chassis_lqr_feedback.motion[2];
		bl_ctrl->torque_info.joint_moving_torque_R = chassis_lqr_feedback.motion[3];
	}

	//轮毂控制
	/* 右轮电机侧镜像保留在任务层，模块输出始终为模型坐标。 */
	bl_ctrl->torque_info.foot_balancing_torque_L = chassis_lqr_feedback.balance[0];
	bl_ctrl->torque_info.foot_moving_torque_L = chassis_lqr_feedback.motion[0];
	bl_ctrl->torque_info.foot_balancing_torque_R = -chassis_lqr_feedback.balance[1];
	bl_ctrl->torque_info.foot_moving_torque_R = -chassis_lqr_feedback.motion[1];

	if (bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1)
	{
		// 离地状态统一处理
		Chassis_Contact_ApplyAirborne_Torque(bl_ctrl);
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
	/* N=d(phi1,phi2)/d(length,Q0) 是逆运动学雅可比。
	 * phi1=-q_first, phi2=q_second，故电机力矩为
	 * diag(-1,+1)*N^(-T)*[F,T_local]，不能遗漏电机坐标变换。
	 * 左腿电机顺序为 (1,2)，theta_L=Q0；右腿为 (4,3)，theta_R=-Q0。
	 * 因此 T_local_L=T_model_L，T_local_R=-T_model_R。
	 */
	const chassis_leg_jacobian_t left_jacobian = chassis_leg_inverse_jacobian(
        &bl_ctrl->chassis_posture_info.chassis_posture_L);
    const chassis_leg_jacobian_t right_jacobian = chassis_leg_inverse_jacobian(
        &bl_ctrl->chassis_posture_info.chassis_posture_R);
    bl_ctrl->mapping_info.invJ1_L = left_jacobian.n11;
    bl_ctrl->mapping_info.invJ2_L = left_jacobian.n21;
    bl_ctrl->mapping_info.invJ3_L = left_jacobian.n12;
    bl_ctrl->mapping_info.invJ4_L = left_jacobian.n22;
    bl_ctrl->mapping_info.invJ1_R = right_jacobian.n11;
    bl_ctrl->mapping_info.invJ2_R = right_jacobian.n21;
    bl_ctrl->mapping_info.invJ3_R = right_jacobian.n12;
    bl_ctrl->mapping_info.invJ4_R = right_jacobian.n22;

	bl_ctrl->torque_info.foot_horizontal_torque_L =
		bl_ctrl->torque_info.foot_balancing_torque_L + bl_ctrl->torque_info.foot_moving_torque_L;
	bl_ctrl->torque_info.foot_horizontal_torque_R =
		bl_ctrl->torque_info.foot_balancing_torque_R + bl_ctrl->torque_info.foot_moving_torque_R; // 足端轮子水平力矩

	bl_ctrl->foot_motor_L.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_L;
	bl_ctrl->foot_motor_R.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_R;
	LimitMax(bl_ctrl->foot_motor_L.torque_out, MAX_FOOT_OUTPUT);
	LimitMax(bl_ctrl->foot_motor_R.torque_out, MAX_FOOT_OUTPUT);
	bl_ctrl->torque_info.joint_horizontal_torque_L = 
		bl_ctrl->torque_info.joint_balancing_torque_L+bl_ctrl->torque_info.joint_moving_torque_L;
	bl_ctrl->torque_info.joint_horizontal_torque_R =
		bl_ctrl->torque_info.joint_balancing_torque_R+bl_ctrl->torque_info.joint_moving_torque_R;

	bl_ctrl->torque_info.joint_vertical_torque_L = 
		bl_ctrl->torque_info.joint_stand_torque_L + bl_ctrl->torque_info.joint_roll_torque_L;
	bl_ctrl->torque_info.joint_vertical_torque_R = 
		bl_ctrl->torque_info.joint_stand_torque_R + bl_ctrl->torque_info.joint_roll_torque_R;

    const chassis_leg_motor_torque_t left_torque = chassis_leg_map_wrench(left_jacobian,
        bl_ctrl->torque_info.joint_vertical_torque_L,
        bl_ctrl->torque_info.joint_horizontal_torque_L, 0);
    const chassis_leg_motor_torque_t right_torque = chassis_leg_map_wrench(right_jacobian,
        bl_ctrl->torque_info.joint_vertical_torque_R,
        bl_ctrl->torque_info.joint_horizontal_torque_R, 1);
    bl_ctrl->torque_info.joint_vertical_torque_temp1_L = left_torque.support_first_nm;
    bl_ctrl->torque_info.joint_vertical_torque_temp2_L = left_torque.support_second_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp1_L = left_torque.balance_first_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp2_L = left_torque.balance_second_nm;
    bl_ctrl->torque_info.joint_vertical_torque_temp1_R = right_torque.support_first_nm;
    bl_ctrl->torque_info.joint_vertical_torque_temp2_R = right_torque.support_second_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp1_R = right_torque.balance_first_nm;
    bl_ctrl->torque_info.joint_horizontal_torque_temp2_R = right_torque.balance_second_nm;

	/* Diagnostic components are motor-side N*m before the output clamp.
	 * temp1/temp2 follow FK input order: L=(1,2), R=(4,3). */
	bl_ctrl->joint_motor_1.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp1_L
		+ bl_ctrl->torque_info.joint_horizontal_torque_temp1_L;
	bl_ctrl->joint_motor_2.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp2_L
		+ bl_ctrl->torque_info.joint_horizontal_torque_temp2_L;
	bl_ctrl->joint_motor_4.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp1_R
		+ bl_ctrl->torque_info.joint_horizontal_torque_temp1_R;
	bl_ctrl->joint_motor_3.torque_out = bl_ctrl->torque_info.joint_vertical_torque_temp2_R
		+ bl_ctrl->torque_info.joint_horizontal_torque_temp2_R;

	/* Match the HT protocol's physical torque range before the CAN layer. */
	LimitOutput(bl_ctrl->joint_motor_1.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
	LimitOutput(bl_ctrl->joint_motor_2.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
	LimitOutput(bl_ctrl->joint_motor_3.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
	LimitOutput(bl_ctrl->joint_motor_4.torque_out, -JOINT_TORQUE_LIMIT, JOINT_TORQUE_LIMIT);
}
void Motor_CMD_Send(chassis_move_t *CMD_Send)
{
	//为保证轮毂电机高相应速度的要求，单独开任务负责给电机发力矩指令
	if (CMD_Send->foot_motor_R.motor_mode != MOTOR_FORCE)
		CMD_Send->foot_motor_R.torque_out = 0.0f;
	if (CMD_Send->foot_motor_L.motor_mode != MOTOR_FORCE)
		CMD_Send->foot_motor_L.torque_out = 0.0f;

	if (CMD_Send->joint_motor_1.motor_mode == MOTOR_FORCE)
	MIT_CtrlMotor(0,0,0,0,CMD_Send->joint_motor_1.torque_out,0x01);
	else
	MIT_CtrlMotor(0,0,0,0,0,0x01);
	if (CMD_Send->joint_motor_3.motor_mode == MOTOR_FORCE)
	MIT_CtrlMotor(0,0,0,0,CMD_Send->joint_motor_3.torque_out,0x03);
	else
	MIT_CtrlMotor(0,0,0,0,0,0x03);
	vTaskDelay(1);
	if (CMD_Send->joint_motor_2.motor_mode == MOTOR_FORCE)
	MIT_CtrlMotor(0,0,0,0,CMD_Send->joint_motor_2.torque_out,0x02);
	else
	MIT_CtrlMotor(0,0,0,0,0,0x02);
	if (CMD_Send->joint_motor_4.motor_mode == MOTOR_FORCE)
	MIT_CtrlMotor(0,0,0,0,CMD_Send->joint_motor_4.torque_out,0x04);
	else
	MIT_CtrlMotor(0,0,0,0,0,0x04);

	cmd_L = (int16_t)fp32_constrain(
		(MODEL_PYH_WHEEL_DIR * chassis_move.foot_motor_L.torque_out * 2598.20f),
		-16384.0f, 16384.0f);

	cmd_R = (int16_t)fp32_constrain(
		(MODEL_PYH_WHEEL_DIR * chassis_move.foot_motor_R.torque_out * 2598.20f),
		-16384.0f, 16384.0f);
	CAN_cmd_chassis(cmd_L,cmd_R,0,0);
}

uint8_t leg_length_ready,leg_angle_ready,leg_gyro_stable,pitch_stable,both_feet_on_ground ,speed_flag,prepare_complete;
uint8_t Check_Jump_Preparation_Complete(chassis_move_t *chassis)
{
	if (chassis == NULL) return 0;
    
    // 综合判断
    prepare_complete = 1;    
    return prepare_complete;
}

static void apply_integral_tuning(pid_type_def *pid, fp32 ki, fp32 limit)
{
	if (!isfinite(ki) || !isfinite(limit) || ki < 0.0f || limit < 0.0f)
		return;
	/* 积分限幅不需要超过整个 PID 的输出限幅。 */
	limit = fminf(limit, pid->max_out);
	pid->Ki = ki;
	pid->max_iout = limit;
	if (pid->Iout > limit) pid->Iout = limit;
	else if (pid->Iout < -limit) pid->Iout = -limit;
}

static void chassis_apply_pid_tuning(chassis_move_t *chassis)
{
    const float leg_ki = tune_leg_ki;
    const float leg_limit = tune_leg_i_limit;
    apply_integral_tuning(&chassis->leg_L_length_pid, leg_ki, leg_limit);
    apply_integral_tuning(&chassis->leg_R_length_pid, leg_ki, leg_limit);
    apply_integral_tuning(&chassis->chassis_roll_pid, tune_roll_ki, tune_roll_i_limit);
}

static uint8_t chassis_is_enabled_on_ground(const chassis_move_t *chassis)
{
	return chassis->mode.chassis_mode == ENABLE_CHASSIS &&
		chassis->flag_info.suspend_flag_L == ON_GROUND &&
		chassis->flag_info.suspend_flag_R == ON_GROUND;
}

static uint8_t chassis_is_balancing_on_ground(const chassis_move_t *chassis)
{
	return chassis_is_enabled_on_ground(chassis) &&
		chassis->mode.chassis_balancing_mode == BALANCING_READY &&
		chassis->mode.sport_mode == NORMAL_MOVING_MODE &&
		chassisClimbGetState() == CLIMB_IDLE && !chassisRecoveryIsActive() &&
		!chassis->chassis_data_->climb_flag && !chassis->chassis_data_->recovery_flag;
}

static void chassis_update_stop_position(chassis_move_t *chassis)
{
	chassis_position_input_t input = {0};
	input.allowed = chassis_is_balancing_on_ground(chassis) && chassis_odometry.sample_valid &&
		fabsf(chassis->chassis_data_->wz_set) < CHASSIS_RC_WZ_DEADLINE;
	input.dt_s = chassis_control_dt_s;
	input.command_speed_m_s = chassis->chassis_posture_info.foot_speed_set;
	input.measured_speed_m_s = chassis_odometry.speed_m_s;
	input.distance_m = chassis_odometry.distance_m;
	chassis_position_step(&chassis_position, &input);
	chassis->chassis_posture_info.position_lock_flag = chassis_position.locked;
	chassis->chassis_posture_info.position_lock_state = chassis_position.locked;
	chassis->chassis_posture_info.target_distance_set = chassis_position.reference_m;
	chassis->chassis_posture_info.foot_distance_set = chassis_position.reference_m;
}

static void chassis_apply_com_feedforward(chassis_move_t *chassis)
{
	chassis_feedforward_step(&chassis_feedforward, chassis_is_balancing_on_ground(chassis),
		tune_com_ff_enable == 1u, tune_com_ff_moment,
		chassis->chassis_posture_info.pitch_angle, chassis_control_dt_s);
	/* 模型坐标下左右同号，右腿镜像仍只在 VMC 内进行一次。
	 * 不向轮端加恒定驱动力，不绕过原有关节输出限幅。 */
	chassis->torque_info.joint_balancing_torque_L += chassis_feedforward.applied_per_leg_nm;
	chassis->torque_info.joint_balancing_torque_R += chassis_feedforward.applied_per_leg_nm;
}

static void chassis_update_yaw_follow(chassis_move_t *chassis, float move_deg)
{
	chassis_follow_input_t input = {0};
	const fp32 forward_deg = tune_gimbal_forward_deg;
	if (isfinite(forward_deg) && forward_deg >= -180.0f && forward_deg <= 180.0f)
		chassis->gimbal_yaw_motor.relative_angle_init = forward_deg;
	input.dt_s = chassis_control_dt_s;
	input.allowed = chassis_is_enabled_on_ground(chassis) && chassis->mode.sport_mode != NONE;
	input.speed_m_s = chassis->chassis_posture_info.foot_speed_set;
	input.relative_deg = chassis->gimbal_yaw_motor.relative_angle;
	input.zero_deg = chassis->gimbal_yaw_motor.relative_angle_init;
	input.move_deg = move_deg;
	input.yaw_rad = chassis->chassis_posture_info.yaw_angle_total;
	input.manual_rate_rad_s = chassis->chassis_data_->wz_set;
	input.kp_rad_s_per_deg = tune_follow_kp;
	input.rate_limit_rad_s = tune_follow_rate_limit;
	input.accel_rad_s2 = tune_follow_accel;
	input.filter_s = tune_follow_filter_s;
	input.yaw_lead_s = tune_follow_yaw_lead_s;
	chassis_follow_step(&chassis_follow, &input);
	chassis->chassis_posture_info.yaw_angle_sett = chassis_follow.angle_ref_rad;
	chassis->chassis_posture_info.yaw_gyro_set = chassis_follow.rate_ref_rad_s;
	chassis->chassis_posture_info.yaw_lock_flag = !chassis_follow.heading_locked;
	chassis->gimbal_yaw_motor.relative_limit = chassis_follow.error_deg;

}

static void chassis_update_wheel_odometry(chassis_move_t *fdb)
{
	/* 旧成员名带 KF，但当前仅为轮速均值及其积分，没有融合定位。 */
	chassis_odometry_step(&chassis_odometry, fdb->foot_motor_L.speed,
		fdb->foot_motor_R.speed, chassis_control_dt_s);
	fdb->chassis_posture_info.foot_speed_KF = chassis_odometry.speed_m_s;
	fdb->chassis_posture_info.foot_distance_K = chassis_odometry.distance_m;
}



static void chassis_calculate_lqr_feedback(const chassis_move_t *chassis, const float gain[4][10])
{
	const chassis_posture_info_t *posture = &chassis->chassis_posture_info;
	const chassis_leg_posture_t *left = &posture->chassis_posture_L;
	const chassis_leg_posture_t *right = &posture->chassis_posture_R;
	const float state_error[10] = {
		/* 平移位置、平移速度、连续 yaw、yaw 角速度。 */
		posture->foot_distance_set - posture->foot_distance_K,
		posture->foot_speed_set - posture->foot_speed_KF,
		posture->yaw_angle_sett - posture->yaw_angle_total,
		posture->yaw_gyro_set - posture->yaw_gyro,
		/* 左右对地腿角及角速度：独立于机身 pitch 参考。 */
		left->leg_angle_set + chassis_balance_reference.leg_ground_rad - left->leg_angle,
		-left->leg_gyro,
		right->leg_angle_set + chassis_balance_reference.leg_ground_rad - right->leg_angle,
		-right->leg_gyro,
		/* 机身 pitch 及角速度：参考偏置不改写传感器反馈。 */
		posture->pitch_angle_set + chassis_balance_reference.pitch_rad - posture->pitch_angle,
		posture->pitch_gyro_set - posture->pitch_gyro
	};
	chassis_lqr_feedback_calculate(gain, state_error,
		chassis_is_balancing_on_ground(chassis) ? tune_lqr_cross_leg_scale : 0.0f,
		&chassis_lqr_feedback);
}
