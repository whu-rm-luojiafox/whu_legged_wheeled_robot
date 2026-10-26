#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H

#include "chassis_fwd.h"

/* Call after normal torque mapping. Returns 1 when the virtual force/torque
 * was replaced (including handoff); the caller must then remap motor outputs. */
uint8_t chassisRecoveryProcess(chassis_move_t *chassis);
/* 普通参考/前馈在自救期间让出控制权。 */
uint8_t chassisRecoveryIsActive(void);

#endif
