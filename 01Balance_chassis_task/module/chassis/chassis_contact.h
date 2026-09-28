# ifndef _CHASSIS_CONTACT_H
# define _CHASSIS_CONTACT_H
#include "chassis_task.h"

//功能函数
void Chassis_Status_Detect(chassis_move_t *detect);
void Chassis_Contact_ApplyAirborne_Torque(chassis_move_t *bl_ctrl);
# endif
