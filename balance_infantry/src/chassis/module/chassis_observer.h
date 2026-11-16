#ifndef CHASSIS_OBSERVER_H
#define CHASSIS_OBSERVER_H

#include "chassis_fwd.h"

/* 速度向前为正，单位 m/s；distance 是同一速度的相对积分。 */
typedef struct
{
    uint8_t sample_valid;
    uint8_t stationary_filter;
    float stationary_time_s;
    float forward_accel_m_s2;
    float speed_m_s;
    float distance_m;
} chassis_odometry_t;
extern chassis_odometry_t chassis_odometry;
void chassisObserverInit(chassis_move_t *chassis);
void chassisObserverUpdate(chassis_move_t *chassis, float dt_s);
#endif
