/* Build-time guards for existing packed layouts during compiler migration.
 * These sizes describe this repository's protocol version.
 */
#include <stddef.h>
#include "referee.h"
#include "remote_control.h"

#define CHECK_LAYOUT(name, condition) typedef char name[(condition) ? 1 : -1]
CHECK_LAYOUT(game_status_size, sizeof(game_status_t) == 11);
CHECK_LAYOUT(game_status_timestamp_offset, offsetof(game_status_t, SyncTimeStamp) == 3);
CHECK_LAYOUT(robot_status_size, sizeof(robot_status_t) == 13);
CHECK_LAYOUT(power_heat_size, sizeof(power_heat_data_t) == 14);
CHECK_LAYOUT(shoot_data_size, sizeof(shoot_data_t) == 7);
CHECK_LAYOUT(shoot_speed_offset, offsetof(shoot_data_t, inital_speed) == 3);
CHECK_LAYOUT(remote_control_size, sizeof(RC_ctrl_t) == 26);
CHECK_LAYOUT(remote_mouse_offset, offsetof(RC_ctrl_t, mouse) == 15);
CHECK_LAYOUT(remote_key_offset, offsetof(RC_ctrl_t, key) == 24);
