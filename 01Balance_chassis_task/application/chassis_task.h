/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "can_service.h"
#include "gim_cha_protocol.h"
#include "super_power_protocol.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_task.h"
// #include "kalman_filter.h"
//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode,
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define CHASSIS_RC_WZ_DEADLINE 0.5f


#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define OMNIDIRECTIONAL_WHEEL 1.41421356f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 50000.0f
//press the key, chassis will swing
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508 rmp change to chassis speed,
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR   0.001007427488972422478f                           //0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f

#define CHASSIS_WZ_SET_SCALE 0.0f

//when chassis is not set to move, swing max angle
//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 9000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 5000.0f


//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.2f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0006f//0.5
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 20.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f//2.8
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 10.0f
#define MOTOR_ECD_TO_RAD 0.000766990394f 
/* -----------------------------平步新增宏定义---------------------------- */

//腿长设定PID
#define LEG_SET_PID_KP 420
#define LEG_SET_PID_KI 2.0f
#define LEG_SET_PID_KD 4500.0f //350.0f
#define LEG_SET_PID_OUT 100.0f
#define LEG_SET_PID_IOUT 12.0f


// -------------  Limit info ------------- 
#define MAX_ACCL 13000.0f
#define MAX_ACCL_JOINT 20.0f
#define MAX_FOOT_OUTPUT 2048

// ------------- Mech info ------------- 
#define L1 0.15f
#define L2 0.27f
#define L3 0.27f
#define L4 0.15f
#define L5 0.15f
  
#define WHEEL_PERIMETER  0.446106f                        //0.56547
#define WHEEL_RADIUS 0.071f
#define LEG_OFFSET       30.0f// 水平位置到上限位的夹角
#define LOWER_SUPPORT_FORCE_FOR_JUMP 5.0f
#define LOWER_SUPPORT_FORCE 0.0f
#define MOVE_LOWER_BOUND 0.5f
#define EXIT_PITCH_ANGLE 0.2f
#define DANGER_PITCH_ANGLE 0.5f

#define FEED_f 50.0f



#define MOTOR_POS_UPPER_BOUND 29.0f
#define MOTOR_POS_LOWER_BOUND 64.0f
#define LIMITED_TORQUE 5.0f
#define UNLIMITED_TORQUE 200.0f

// ------------- Transfer info ------------- 
#define HALF_POSITION_RANGE    178.0f
#define TORQ_K            77.1604f
// ------------- Math info ------------- 
#define PI2					  6.28318530717959f
#define PI					  3.14159265358979f
#define PI_2				  1.57079632679489f
#define PI_4				  0.78539816339744f


 typedef enum
{
    ENABLE_CHASSIS = 0,
    DISABLE_CHASSIS,
    DEBUG_CHASSIS,
} chassis_mode_e;


typedef enum
{
    NO_FORCE,
    FOOT_LAUNCHING,
    JOINT_LAUNCHING,
    BALANCING_READY,
    JOINT_REDUCING,
} chassis_balancing_mode_e;

typedef enum
{
    NONE,
    NORMAL_MOVING_MODE,
    ABNORMAL_MOVING_MODE,
    JUMPING_MODE,
    FLY_MODE,
    TK_MODE,
} sport_mode_e;

typedef enum
{
    READY_TO_JUMP,
    PREPARING_LANDING,
    PREPARING_STAND_JUMPING,
    CONSTACTING_LEGS,
    EXTENDING_LEGS,
    CONSTACTING_LEGS_2,
    FINISHED,
} jumping_stage_e;

typedef enum
{
    NOT_DEFINE,
    STANDING_JUMP,
} jumping_mode_e;

typedef enum
{
    SIT_MODE = 0,
    NORMAL_MODE,
    HIGH_MODE,
    EXTREMELY_HIGH_MODE,
    CHANGING_HIGH,
} chassis_high_mode_e;

typedef enum
{
    MOTOR_NO_FORCE = 0,
    MOTOR_FORCE,
} chassis_motor_mode_e;

typedef enum
{
	ON_GROUND = 0,
	OFF_GROUND = 1,
} suspend_flag_e;

typedef struct
{
    chassis_mode_e           chassis_mode, last_chassis_mode;
    chassis_balancing_mode_e chassis_balancing_mode, last_chassis_balancing_mode;
    sport_mode_e             sport_mode, last_sport_mode; 
    
    jumping_mode_e           jumping_mode, last_jumping_mode;
    jumping_stage_e          jumping_stage, last_jumping_stage;

    chassis_high_mode_e       chassis_high_mode, last_chassis_high_mode;

} mode_t;
typedef struct
{
    const fp32 *chassis_INS_angle_point;
  	const fp32 *chassis_INS_gyro_point;
    const fp32 *chassis_INS_accel_point;
    fp32 yaw_angle,yaw_angle_last,yaw_angle_total, pitch_angle, roll_angle,last_pitch_angle,yaw_round_cnt;
    fp32 yaw_gyro, pitch_gyro, roll_gyro,last_pitch_gyro;
    fp32 yaw_accel, pitch_accel, roll_accel;
    fp32 x_accel,y_accel,z_accel;

    fp32 yaw_angle_sett, pitch_angle_set, roll_angle_set;
    fp32 yaw_gyro_set, pitch_gyro_set, roll_gyro_set;

    fp32 ideal_high;
    fp32 leg_length_L_set, leg_length_R_set;
    fp32 leg_length_L, last_leg_length_L;
    fp32 leg_length_R, last_leg_length_R;
    fp32 leg_dlength_L,leg_dlength_R,last_leg_dlength_L,last_leg_dlength_R,leg_dlength_L_jacobian, leg_dlength_R_jacobian,last_leg_dlength_L_jacobian, last_leg_dlength_R_jacobian;
    fp32 leg_ddlength_L,leg_ddlength_R,last_leg_ddlength_L,last_leg_ddlength_R;
    fp32 foot_roll_angle;
    fp32 leg_angle_L, last_leg_angle_L, leg_angle_L_set,leg_angle_L_kf;
    fp32 leg_angle_R, last_leg_angle_R, leg_angle_R_set,leg_angle_R_kf;
    fp32 leg_gyro_L, leg_gyro_R, last_leg_gyro_L, last_leg_gyro_R;
    fp32 leg_gyro_L_jacobian, leg_gyro_R_jacobian,last_leg_gyro_L_jacobian, last_leg_gyro_R_jacobian;
    fp32 leg_accel_L, leg_accel_R;

    /* ----------debug param-------- */
    fp32 xc,yc,xb,yb;
    fp32 Q2;

    uint16_t position_lock_flag,position_lock_state;
    uint16_t yaw_lock_flag;
    fp32 foot_distance, foot_distance_K, foot_distance_set,target_distance_set;
    fp32 foot_speed, foot_speed_KF, foot_speed_set;


    //车身加速度
    fp32 chassis_accel;
    //轮子加速度计算
    fp32 foot_accel_L, foot_accel_R;


} chassis_posture_info_t;


typedef struct
{
    // -------- horizontal force -------- 
    fp32 foot_balancing_torque_L,  foot_balancing_torque_R;
	  fp32 foot_moving_torque_L,  foot_moving_torque_R;
    fp32 foot_horizontal_torque_L,  foot_horizontal_torque_R;
    
    fp32 joint_balancing_torque_L, joint_balancing_torque_R;
    fp32 joint_moving_torque_L, joint_moving_torque_R;
    fp32 joint_prevent_splits_torque_L, joint_prevent_splits_torque_R;
    fp32 joint_horizontal_torque_L, joint_horizontal_torque_R;
    
    fp32 joint_roll_torque_L,  joint_roll_torque_R;
    
    
    fp32 joint_horizontal_torque_temp1_R, joint_horizontal_torque_temp2_R;
    fp32 joint_horizontal_torque_temp1_L, joint_horizontal_torque_temp2_L;
    
    fp32 yaw_torque;
    
    // -------- vertical force -------- 
    
    fp32 joint_stand_torque_L, joint_stand_torque_R;
    fp32 last_control_torque_L,last_control_torque_R;
    fp32 joint_vertical_torque_L,      joint_vertical_torque_R;
    fp32 joint_real_vertical_torque_L, joint_real_vertical_torque_R;

    fp32 joint_vertical_torque_temp1_R, joint_vertical_torque_temp2_R;
    fp32 joint_vertical_torque_temp1_L, joint_vertical_torque_temp2_L;

    fp32 forque_L, forque_R;
    fp32 supportive_force_L, supportive_force_R,last_supportive_force_L, last_supportive_force_R;
    fp32 balance_force_L, balance_force_R;

} torque_info_t;
typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
    HTmotor_measure_t *motor_measure;
    chassis_motor_mode_e motor_mode, last_motor_mode;

    bool_t offline_flag;

    fp32 position;
    fp32 init_position;
    fp32 position_offset;

    fp32 velocity;
    // fp32 velocity_kf; //滤波后的速度

    fp32 torque_out, torque_get;
    fp32 max_torque, min_torque;
} joint_motor_t;

typedef struct
{
    lkmotor_measure_t *motor_measure;
    chassis_motor_mode_e motor_mode, last_motor_mode;

    bool_t offline_flag;

    fp32 distance, distance_offset, last_position, position, turns;
    fp32 speed,speed_kf;
    fp32 torque_out, torque_get;
    fp32 last_control_torque;

} foot_motor_t;

/* -----------------------------平步标志位结构体---------------------------- */
typedef struct
{
    bool_t init_flag;
	suspend_flag_e suspend_flag_L, last_suspend_flag_L;
    suspend_flag_e suspend_flag_R, last_suspend_flag_R;
    bool_t Ignore_Off_Ground;
    bool_t abnormal_flag;
    bool_t rotation_flag;
    bool_t set_pos_after_moving;
    bool_t overpower_warning_flag;
    bool_t last_overpower_warning_flag;
    bool_t stablize_high_flag;
    bool_t last_stablize_high_flag;

    // 跳跃相关标志
    uint8_t jump_prepare_complete;    // 跳跃准备完成标志
    uint32_t jump_prepare_timer;      // 跳跃准备计时器
    uint32_t jump_extend_timer;       // 伸腿计时器
    uint32_t jump_contact_timer;      // 接触计时器
} flag_info_t;

/* -----------------------------VMC结构体---------------------------- */
typedef struct 
{
    fp32 J1_L,J2_L;
	fp32 J3_L,J4_L;
	fp32 J1_R,J2_R;
	fp32 J3_R,J4_R;
	fp32 invJ1_L,invJ2_L;
	fp32 invJ3_L,invJ4_L;
	fp32 invJ1_R,invJ2_R;
	fp32 invJ3_R,invJ4_R;
} mapping_info_t;

// 定义多项式系数结构体
typedef struct {
    float c0; // 常数项
    float c1; // L0系数
    float c2; // Q0系数
    float c3; // L0^2系数
    float c4; // L0*Q0系数
    float c5; // Q0^2系数
} PolynomialCoefficients;

// 定义逆雅可比矩阵系数
typedef struct {
    PolynomialCoefficients N11;
    PolynomialCoefficients N12;
    PolynomialCoefficients N21;
    PolynomialCoefficients N22;
} InverseJacobianCoefficients;

typedef struct
{

    mode_t mode;
    flag_info_t flag_info;
    chassis_posture_info_t chassis_posture_info;
    torque_info_t torque_info;
    mapping_info_t mapping_info;
    InverseJacobianCoefficients InverseJacobianCoefficient;
    const RC_ctrl_t *chassis_RC;
    const fp32 *chassis_INS_angle;
    const fp32 *chassis_INS_gyro;
    const fp32 *chassis_INS_accel;
    const chassis_data_t *chassis_data_; // 从云台接受到的底盘数据设定值

    chassis_motor_t motor_chassis[4]; // chassis motor data.底盘电机数据
    pid_type_def motor_speed_pid[4];  // motor speed PID.底盘电机速度pid
    pid_type_def chassis_yaw_pid;     // follow angle PID.底盘跟随角度pid
    pid_type_def leg_L_length_pid;    // 腿长设定PID
    pid_type_def leg_R_length_pid;    // 腿长设定PID

    fp32 vx;     // chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;     // chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;     // chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 vx_set; // chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set; // chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set; // chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
    fp32 chassis_yaw_set;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
    gimbal_motor_t gimbal_yaw_motor;
    joint_motor_t joint_motor_1, joint_motor_2, joint_motor_3, joint_motor_4;
    foot_motor_t foot_motor_L, foot_motor_R;
    fp32 distance;
    super_power_t super_power;

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern chassis_move_t chassis_move;
#endif
