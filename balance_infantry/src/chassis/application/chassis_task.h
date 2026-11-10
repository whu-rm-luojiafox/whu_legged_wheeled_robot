#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "chassis_fwd.h"

#include "chassis_types.h"

void chassis_task(void const *pvParameters);
extern chassis_move_t chassis_move;
#endif
