#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include "chassis_fwd.h"

void motor_cmd_task(void const *pvParameters);
void chassisMotorSend(chassis_move_t *chassis);
#endif
