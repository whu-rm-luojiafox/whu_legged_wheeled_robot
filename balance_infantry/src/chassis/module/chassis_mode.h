#ifndef CHASSIS_MODE_H
#define CHASSIS_MODE_H

#include "chassis_fwd.h"

/* 模式和跳跃/起立目标；普通运动目标之后、攀爬覆盖之前调用 SetTargets。 */
void chassisModeInit(chassis_move_t *chassis);
void chassisModeUpdate(chassis_move_t *chassis);
void chassisModeSetTargets(chassis_move_t *chassis);
uint8_t chassisIsEnabledOnGround(const chassis_move_t *chassis);
uint8_t chassisIsBalancingOnGround(const chassis_move_t *chassis);
#endif
