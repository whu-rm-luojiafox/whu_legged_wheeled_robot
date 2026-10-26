#ifndef _CHASSIS_POWER_H
#define _CHASSIS_POWER_H
#include "struct_typedef.h"
#include "chassis_fwd.h"
extern float Plimit;
/* Legacy reference scaling, not an output-power limiter. Preserve call timing
 * until a separate power-control change has been validated on hardware. */
void chassis_power_limit(chassis_move_t *chassis_move_control);
#endif
