#ifndef CHASSIS_CLIMB_H
#define CHASSIS_CLIMB_H

#include "chassis_fwd.h"

typedef enum
{
    CLIMB_IDLE = 0,     // 正常控制，等待云台攀爬指令
    CLIMB_READY,        // 检查底盘状态是否允许攀爬
    CLIMB_EXTEND,       // 自动升腿
    CLIMB_APPROACH,     // 等待操作手主动向前，检测台阶立面
    CLIMB_CROSS,        // 已碰到台阶，保持长腿越过棱边
    CLIMB_TOP_CONFIRM,  // 确认机身已经稳定搭到台阶上
    CLIMB_PULL_UP,      // 自动停车并收腿，将轮子拉过台阶棱边
    CLIMB_WHEEL_CONFIRM,// 保持短腿，确认左右轮已经落到台阶顶面
    CLIMB_STAND_UP,     // 轮子上台阶后重新伸腿到正常站立高度
    CLIMB_FINISHED,     // 攀爬完成，稳定后自动复位
    CLIMB_ABORT,        // 严重异常时保持，稳定后平滑退出
    CLIMB_RETURN_NORMAL // 超时/再次按 F 后平滑交回普通高度控制
} climb_state_e;

/*
 * 必须放在 Target_Value_Set() 之后、
 * chassis_power_limit() 之前调用。
 */
void chassisClimbProcess(chassis_move_t *chassis);



/* 方便调试器或者上位机查看当前状态。 */
climb_state_e chassisClimbGetState(void);

/* Keep stop-position control available in IDLE, READY, EXTEND and RETURN.
 * Normal movement input still releases the captured position.
 */
uint8_t chassisClimbAllowsPositionHold(void);

/* Immediately suppress wheel/hip feedback and roll correction during pull-up
 * and wheel-contact confirmation. Use radial length PID output without
 * standing gravity feedforward during these two stages.
 * Call before every VMC mapping; returns nonzero when suppression is active.
 */
uint8_t chassisClimbApplyTorquePolicy(chassis_move_t *chassis);

#endif
