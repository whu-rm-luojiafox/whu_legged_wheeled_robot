#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/* Imported control constants; units and defaults are preserved for migration.
 * Pitch/yaw feedback and commands in the gimbal application use degrees.
 * RC/keyboard selection belongs to command generation, not board selection. */
#define GIMBAL_RC_INPUT_ENABLED 1
#define GIMBAL_KEY_INPUT_ENABLED 1

// pitch speed close-loop PID params, max out and max iout
// pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_PID_KP 3800.0f//5000.0f
#define PITCH_GYRO_PID_KI 12.0f//10.0f
#define PITCH_GYRO_PID_KD 0.0f
#define PITCH_GYRO_PID_MAX_OUT 30000.0f
#define PITCH_GYRO_PID_MAX_IOUT 10000.0f
#define PITCH_ANGLE_PID_KP 2.20f//0.3f       
#define PITCH_ANGLE_PID_KI 0.0f      
#define PITCH_ANGLE_PID_KD 15.0f       
#define PITCH_ANGLE_PID_MAX_OUT 12.0f//3.0f  
#define PITCH_ANGLE_PID_MAX_IOUT 3.0f//1.0f 

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   0
#define PITCH_CHANNEL  1
#define HEIGHT_CHANNEL 
#define GIMBAL_MODE_CHANNEL 0
//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10


#define YAW_RC_SEN    0.0003f
#define PITCH_RC_SEN  -0.00006f //0.005

#define YAW_MOUSE_SEN   0.001f
#define PITCH_MOUSE_SEN -0.0007f

#define HIGH_SEN 0.0000005f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD           0.000095873799f                             // 0.000766990394f //      2*  PI  /8192
#endif
//有关底盘的定义

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.020f
// #define CHASSIS_VX_RC_SEN 0.012f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.020f
#define CHASSIS_RC_DEADLINE 10
//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 2
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 3
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_CONTROL_TIME 0.002f
#define CHASSIS_WZ_RC_SEN 0.02f
#define CHASSIS_WZ_CHANNEL 4
#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2



#endif
