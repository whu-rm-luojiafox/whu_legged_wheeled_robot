#include "motor_cmd.h"
#include "chassis_task.h"
#include "can_service.h"
#include "cmsis_os.h"
#include "referee.h"

extern chassis_move_t chassis_move;
extern robot_status_t robot_state;
extern power_heat_data_t power_heat_data;

void motor_cmd_task(void const *pvParameters)
{
    osDelay(500);
    chassis_move.super_power.enableDCDC =1;
    chassis_move.super_power.systemRestart =0;
    chassis_move.super_power.useNewFeedbackMessage =1;
    chassis_move.super_power.enableActiveChargingLimit = 1;
    while(1)
    {
        //超电
        chassis_move.super_power.refereePowerLimit = robot_state.chassis_power_limit;
        chassis_move.super_power.refereeEnergyBuffer = power_heat_data.buffer_energy;
        chassis_move.super_power.activeChargingLimitRatio = 80;
        CAN_SuperPower_Control(chassis_move.super_power);
        //轮毂
        CAN_LK_Boradcast_Control(chassis_move.foot_motor_L.torque_out,chassis_move.foot_motor_R.torque_out,0,0);
        osDelay(1);
    }
}