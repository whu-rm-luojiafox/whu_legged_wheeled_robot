#include "chassis_types.h"
#include "chassis_power.h"

/* 沿用当前参考缩放；本模块尚未启用基于裁判功率的闭环控制。 */
float Plimit = 1.0f;
float Wlimit = 1.0f;

void chassis_power_limit(chassis_move_t *chassis)
{
    chassis->chassis_posture_info.foot_speed_set *= Plimit;
    chassis->chassis_posture_info.yaw_gyro_set *= Wlimit;
}
