#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H

#include "chassis_task.h"

/* Call once after the normal torque mapping and before Motor_CMD_Send. */
void chassisRecoveryProcess(chassis_move_t *chassis);

#endif
