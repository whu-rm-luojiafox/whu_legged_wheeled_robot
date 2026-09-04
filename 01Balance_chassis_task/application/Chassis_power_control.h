#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "struct_typedef.h"
#include "chassis_task.h"
extern float Plimit;
void chassis_power_limit(chassis_move_t *chassis_move_control);
#endif
