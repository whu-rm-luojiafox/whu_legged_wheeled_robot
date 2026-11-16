#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_fwd.h"

/* 计算虚拟力/力矩；起立/自救 Apply 后统一映射一次。 */
void chassisControlInit(chassis_move_t *chassis);
void chassisControlCalculate(chassis_move_t *chassis, float dt_s);
void chassisControlMap(chassis_move_t *chassis, float dt_s);
float chassisControlLegAngleReference(void);
#endif
