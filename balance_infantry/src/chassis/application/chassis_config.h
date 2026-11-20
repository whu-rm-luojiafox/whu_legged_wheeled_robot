#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 2026-10-15：云台朝向车头时的编码器角度（deg）。 */
#define CHASSIS_GIMBAL_FORWARD_DEG 42.8f

#include "struct_typedef.h"

/* 编译默认 PID 参数。复位会恢复默认，在线 RAM 写入不会修改源码。
 * 跟随限速/滤波在 chassis_follow.h；姿态参考在 chassis_balance.h。
 * tune_* 保持为独立标量，便于调试器按 ELF/DWARF 名称安全读写。 */
/* 跟随外环为角差 P + 相对转速 D + 云台速度前馈，不维护额外 PID 对象。 */
#define FOLLOW_KP_RAD_S_PER_DEG 0.10f
//腿长设定PID
#define LEG_SET_PID_KP 350.0f
#define LEG_SET_PID_KI 0.8f
#define LEG_SET_PID_KD 300.0f
#define LEG_SET_PID_OUT 60.0f
#define LEG_SET_PID_IOUT 30.0f

//roll轴水平pid
#define ROLL_PID_KP 220.0f
#define ROLL_PID_KI 0.08f
#define ROLL_PID_KD 1.0f /* Measured-rate damping, N/(rad/s). */
#define ROLL_PID_OUT 10.0f
#define ROLL_PID_IOUT 2.5f
/* Keep Ki's original per-4ms tuning convention, integrate using measured dt.
 * D uses measured roll rate, N/(rad/s), instead of differencing the angle. */
#define ROLL_PID_NOMINAL_DT_S 0.004f
/* 起立/自救：双腿的对地角到位后开始缩腿。 */
#define LEG_SWING_MOTOR_SPEED_RAD_S 0.80f
#define STARTUP_SWING_DIRECTION (-1.0f) /* 手摆实测：前到后时机身局部角沿负方向跨过 ±π。 */
#define STARTUP_SWING_TARGET_ANGLE_RAD 1.4f
#define STARTUP_LEG_ANGLE_MIN_RAD 1.2f /* 摆腿到位窗口使用对地角。 */
#define STARTUP_LEG_ANGLE_MAX_RAD (STARTUP_SWING_TARGET_ANGLE_RAD + 0.05f)
#define STARTUP_SWING_STALL_PROGRESS_RAD 0.015f /* 检查局部腿角，排除机身 pitch 的假进展。 */
#define STARTUP_SWING_STALL_TIME_S 1.5f
#define STARTUP_SWING_TIMEOUT_S 17.0f /* 0.40 rad/s 下最长摆动约 14.1 s，另留跟踪余量。 */
#define STARTUP_RETRACT_SPEED_M_S 0.40f /* 三次曲线峰值速度：缩短 20 cm 约需 1.0 s。 */
#define STARTUP_RETRACT_MIN_TIME_S 0.3f
#define STARTUP_LENGTH_TOLERANCE_M 0.03f /* 轨迹结束且腿长到位后交接，不再要求腿角到位。 */
#define STARTUP_SETTLE_TIME_S 0.15f
#define STARTUP_PHASE_TIMEOUT_S 10.0f
#define STARTUP_ANGLE_KP 8.0f  /* 虚拟腿角 PD，N*m/rad */
#define STARTUP_ANGLE_KD 0.6f  /* N*m/(rad/s) */
#define STARTUP_LEG_TORQUE_LIMIT_NM 4.0f /* 到位等待/收腿角度 PD 总限幅，不叠加固定积分偏置。 */
#define STARTUP_JOINT_TORQUE_LIMIT_NM 8.0f
#define STARTUP_JOINT_TORQUE_SLEW_NM_S 20.0f
#define STARTUP_LQR_BLEND_TIME_S 2.0f /* 交接时才从后摆角平滑回到配平角。 */

/* 腿长、roll：每个控制周期同步到 PID，并将已有积分输出收回新限幅内。 */
extern volatile float tune_leg_ki;
extern volatile float tune_leg_i_limit;
extern volatile float tune_roll_ki;
extern volatile float tune_roll_i_limit;
/* 跟随：角差单位 deg，输出角速度 rad/s，加速度 rad/s²。 */
extern volatile float tune_follow_kp;
extern volatile float tune_follow_rate_limit;
extern volatile float tune_follow_accel;
extern volatile float tune_follow_filter_s;
/* 实验前馈：开关必须为 1 才生效；moment 单位 kg·m，默认关闭。 */
extern volatile uint8_t tune_com_ff_enable;
extern volatile float tune_com_ff_moment;
/* 标定：云台正前方编码器角度，以及独立的机身/对地腿角参考。 */
extern volatile float tune_gimbal_forward_deg;
extern volatile float tune_balance_pitch_rad;
extern volatile float tune_balance_leg_ground_rad;
/* 对侧腿反馈：默认 1 使用完整矩阵，0 可对比之前删减的反馈。 */
extern volatile float tune_lqr_cross_leg_scale;

/* 底盘硬件、协议换算和控制默认值。 */
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
#define M3508_MOTOR_RPM_TO_VECTOR 0.001007427488972422478f //0.000415809748903494517209f
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

#define MOTOR_ECD_TO_RAD 0.000766990394f
/* -----------------------------平步新增宏定义---------------------------- */

// -------------  Limit info -------------
#define MAX_ACCL 13000.0f
#define MAX_ACCL_JOINT 18.0f
#define MAX_FOOT_OUTPUT 6.3

// ------------- Mech info -------------

#define WHEEL_PERIMETER 0.446106f //0.56547
#define WHEEL_RADIUS 0.068f
#define LEG_OFFSET 30.0f // 标定姿态相对主动杆朝外水平的软件角偏置（轴侧逆时针为正）
#define LOWER_SUPPORT_FORCE_FOR_JUMP 5.0f
#define EXIT_PITCH_ANGLE 0.4f

#define FEED_f 0.0f

#define MOTOR_POS_UPPER_BOUND 29.0f
#define MOTOR_POS_LOWER_BOUND 64.0f
#define LIMITED_TORQUE 5.0f
#define UNLIMITED_TORQUE 200.0f

// ------------- Transfer info -------------
#define HALF_POSITION_RANGE 178.0f
// ------------- Math info -------------
#define PI2 6.28318530717959f
#define PI 3.14159265358979f
#define PI_2 1.57079632679489f
#define PI_4 0.78539816339744f

#endif
