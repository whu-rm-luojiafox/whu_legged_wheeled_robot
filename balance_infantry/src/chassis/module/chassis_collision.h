#ifndef CHASSIS_COLLISION_H
#define CHASSIS_COLLISION_H
#include "chassis_fwd.h"

typedef enum { COLLISION_IDLE, COLLISION_BRAKE, COLLISION_RETREAT, COLLISION_HOLD } chassis_collision_state_e;
typedef struct
{
    chassis_collision_state_e state;
    uint8_t observer_valid;
    float external_force_n; /* Approximate longitudinal disturbance, not a contact sensor. */
    float force_filtered_n;
    float impact_delta_v_m_s; /* Signed short-pulse disturbance impulse / mass. */
    float ready_time_s, push_time_s, hit_time_s, stage_time_s, neutral_time_s;
    float retreat_direction, start_distance_m, heading_rad, speed_ref_m_s;
    uint32_t events;
} chassis_collision_t;
extern chassis_collision_t chassis_collision;
/* Run after operator speed selection, before stop-position and yaw references. */
void chassisCollisionProcess(chassis_move_t *chassis, float dt_s);
uint8_t chassisCollisionIsActive(void);
#endif
