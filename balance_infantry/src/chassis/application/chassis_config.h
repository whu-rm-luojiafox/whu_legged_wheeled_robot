#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "struct_typedef.h"

/* 编译默认 PID 参数。复位会恢复默认，在线 RAM 写入不会修改源码。
 * 跟随限速/滤波在 chassis_follow.h；姿态参考在 chassis_balance.h。
 * tune_* 保持为独立标量，便于调试器按 ELF/DWARF 名称安全读写。 */
/* 跟随外环只有 P，不再维护一套未参与计算的 PID 对象。 */
#define FOLLOW_KP_RAD_S_PER_DEG 0.16f
//腿长设定PID
#define LEG_SET_PID_KP 300.0f
#define LEG_SET_PID_KI 0.075f
#define LEG_SET_PID_KD 300.0f
#define LEG_SET_PID_OUT 50.0f
#define LEG_SET_PID_IOUT 2.5f

//roll轴水平pid
#define ROLL_PID_KP 300.0f
#define ROLL_PID_KI 0.08f
#define ROLL_PID_KD 10.0f
#define ROLL_PID_OUT 10.0f
#define ROLL_PID_IOUT 2.5f

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
extern volatile float tune_follow_yaw_lead_s;
/* 实验前馈：开关必须为 1 才生效；moment 单位 kg·m，默认关闭。 */
extern volatile uint8_t tune_com_ff_enable;
extern volatile float tune_com_ff_moment;
/* 标定：云台正前方编码器角度，以及独立的机身/对地腿角参考。 */
extern volatile float tune_gimbal_forward_deg;
extern volatile float tune_balance_pitch_rad;
extern volatile float tune_balance_leg_ground_rad;
/* 实验耦合：0 保留原反馈，1 使用完整矩阵；目前默认 0。 */
extern volatile float tune_lqr_cross_leg_scale;


#endif
