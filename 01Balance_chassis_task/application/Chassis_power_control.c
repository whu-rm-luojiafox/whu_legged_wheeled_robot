#include "main.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "Chassis_power_control.h"
#include "referee.h"
float Plimit=1;
float Wlimit=1;
static float Power_Buffer;

extern power_heat_data_t power_heat_data;

static float power_buffer_limit(float buffer)
{
    if (buffer >= 60.0f) return 1.00f;
    if (buffer >= 50.0f) return 0.95f + (buffer - 50.0f) * (1.00f - 0.95f) / 10.0f;
    if (buffer >= 40.0f) return 0.90f + (buffer - 40.0f) * (0.95f - 0.90f) / 10.0f;
    if (buffer >= 35.0f) return 0.75f + (buffer - 35.0f) * (0.90f - 0.75f) / 5.0f;
    if (buffer >= 30.0f) return 0.50f + (buffer - 30.0f) * (0.75f - 0.50f) / 5.0f;
    if (buffer >= 20.0f) return 0.25f + (buffer - 20.0f) * (0.50f - 0.25f) / 10.0f;
    if (buffer >= 10.0f) return 0.125f + (buffer - 10.0f) * (0.25f - 0.125f) / 10.0f;
    if (buffer >= 0.0f) return 0.05f + buffer * (0.125f - 0.05f) / 10.0f;
    return 0.05f;
}

void chassis_power_limit(chassis_move_t *chassis_move_control)
{

Power_Buffer = power_heat_data.buffer_energy;

float target_limit = power_buffer_limit(Power_Buffer);

Plimit = 0.9f * Plimit + 0.1f * target_limit;
Wlimit = 0.9f * Wlimit + 0.1f * target_limit;


chassis_move_control->chassis_posture_info.foot_speed_set *= Plimit;
chassis_move_control->chassis_posture_info.yaw_gyro_set *= Wlimit;
}
