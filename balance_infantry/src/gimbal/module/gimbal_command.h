#ifndef GIMBAL_COMMAND_H
#define GIMBAL_COMMAND_H
#include <stdint.h>
#include "gimbal_fwd.h"
#include "gim_cha_protocol.h"

/* Initialize the outgoing chassis command, then update it once per gimbal
 * iteration, before feedback refresh (the original scheduler order). */
void gimbal_command_init(chassis_data_t *command);
void gimbal_command_update(gimbal_control_t *control, chassis_data_t *command);
uint8_t gimbal_command_enabled(void);
uint8_t gimbal_command_auto_aim_enabled(void);

#endif
