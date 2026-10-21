/* Both boards compile this file with the same short-enum ABI. Do not pack the
 * legacy UART payload: its existing padding is part of the on-wire layout. */
#include <stddef.h>
#include "robot_link_frames.h"
#include "remote_control.h"

_Static_assert(sizeof(RC_chassis_mode_e) == 1, "board link requires short enums");
_Static_assert(sizeof(shoot_mode_e) == 1, "shoot mode ABI changed");
_Static_assert(sizeof(chassis_data_t) == 48, "chassis command wire size changed");
_Static_assert(offsetof(chassis_data_t, chassis_mode) == 32, "mode offset changed");
_Static_assert(offsetof(chassis_data_t, shoot_mode_rc) == 33, "shoot offset changed");
_Static_assert(offsetof(chassis_data_t, recovery_flag) == 40, "recovery offset changed");
_Static_assert(offsetof(chassis_data_t, fric_speed_set) == 44, "friction speed offset changed");
_Static_assert(CHASSIS_MODE_OFF == 0 && CHASSIS_MOVE_ON == 1 &&
               CHASSIS_MODE_DEBUG == 2 && CHASSIS_MODE_INIT == 3,
               "board mode values changed");
_Static_assert(sizeof(uart_data_t) == 52, "command frame size changed");
_Static_assert(offsetof(uart_data_t, command) == 4, "command frame padding changed");
_Static_assert(sizeof(referee_data_t) == 8, "referee frame size changed");
_Static_assert(offsetof(referee_data_t, shoot_speed) == 4, "shoot speed padding changed");
_Static_assert(sizeof(RC_ctrl_t) == 26, "remote input must remain packed");
_Static_assert(offsetof(RC_ctrl_t, mouse) == 15, "remote mouse offset changed");
_Static_assert(offsetof(RC_ctrl_t, key) == 24, "remote key offset changed");
