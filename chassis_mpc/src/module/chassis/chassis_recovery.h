#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H

#include "chassis_task.h"

/* Call once after the normal torque mapping and before Motor_CMD_Send. */
void chassisRecoveryProcess(chassis_move_t *chassis);
/* 普通参考/前馈在自救期间让出控制权。 */
uint8_t chassisRecoveryIsActive(void);

#endif
