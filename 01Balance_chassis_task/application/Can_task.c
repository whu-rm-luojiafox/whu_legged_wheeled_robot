#include "main.h"
#include "cmsis_os.h"
#include "Can_task.h"
#include "referee.h"
#include "gimbal_task.h"
extern gimbal_control_t gimbal_control;

void robot_id_to_enemy_color(uint8_t robot_id, uint8_t *enemy_color)
{
    switch(robot_id)
    {
        case UI_Data_RobotID_RHero:         
        case UI_Data_RobotID_REngineer:
        case UI_Data_RobotID_RStandard1:
        case UI_Data_RobotID_RStandard2:
        case UI_Data_RobotID_RStandard3:
        case UI_Data_RobotID_RAerial:
        case UI_Data_RobotID_RSentry:
        case UI_Data_RobotID_RRadar:
            *enemy_color = 0;
            break;
        case UI_Data_RobotID_BHero:
        case UI_Data_RobotID_BEngineer:
        case UI_Data_RobotID_BStandard1:
        case UI_Data_RobotID_BStandard2:
        case UI_Data_RobotID_BStandard3:
        case UI_Data_RobotID_BAerial:
        case UI_Data_RobotID_BSentry:
        case UI_Data_RobotID_BRadar:
            *enemy_color = 1;
            break;
        default:
            break;
    }
}
void Can_task(void const *pvParameters)
{
	uint8_t color;
	while(1)
	{
		robot_id_to_enemy_color(get_robot_id(),&color);
		CAN_cmd_referee_data(color);
//        CAN_INIT_STATUS(gimbal_control.YAW_INIT_FLAG);
		osDelay(10);
	}
}