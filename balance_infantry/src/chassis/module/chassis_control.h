#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_fwd.h"

/* 计算虚拟力/力矩；自救覆盖后可以只重新调用 Map。 */
void chassisControlInit(chassis_move_t *chassis);
void chassisControlCalculate(chassis_move_t *chassis, float dt_s);
void chassisControlMap(chassis_move_t *chassis);
#endif
