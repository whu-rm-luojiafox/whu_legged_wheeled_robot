#ifndef CHASSIS_CLIMB_H
#define CHASSIS_CLIMB_H

#include "chassis_fwd.h"

typedef enum
{
    CLIMB_IDLE = 0,      // 正常控制，等待云台攀爬指令
    CLIMB_READY,         // 检查底盘状态是否允许攀爬
    CLIMB_EXTEND,        // 自动升腿
    CLIMB_APPROACH,      // 等待操作手主动向前，检测台阶立面
    CLIMB_PULL_UP = 6,   // 撞阶确认后立即停车收腿；保留编号，便于对照历史波形
    CLIMB_WHEEL_CONFIRM, // 旧状态编号保留；正常流程收腿到位后直接起立
    CLIMB_STAND_UP,      // 轮子上台阶后重新伸腿到正常站立高度
    CLIMB_FINISHED,      // 攀爬完成，稳定后自动复位
    CLIMB_ABORT,         // 严重异常时保持，稳定后平滑退出
    CLIMB_RETURN_NORMAL  // 超时/再次按 F 后平滑交回普通高度控制
} climb_state_e;

/*
 * 必须放在 chassisMotionUpdate() 之后、
 * chassis_power_limit() 之前调用。
 */
void chassisClimbProcess(chassis_move_t *chassis);

/* 方便调试器或者上位机查看当前状态。 */
climb_state_e chassisClimbGetState(void);

/* Keep stop-position control available in IDLE, READY, EXTEND and RETURN.
 * Normal movement input still releases the captured position.
 */
uint8_t chassisClimbAllowsPositionHold(void);

/* 收腿/等待轮子落地时，只允许腿长控制；由控制模块执行输出策略。 */
uint8_t chassisClimbLengthOnly(void);

#endif
