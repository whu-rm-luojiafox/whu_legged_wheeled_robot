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
    chassis_move.super_power.enableDCDC = 1;
    chassis_move.super_power.systemRestart = 0;
    chassis_move.super_power.useNewFeedbackMessage = 1;
    chassis_move.super_power.enableActiveChargingLimit = 1;
    while (1)
    {
        //超电
        chassis_move.super_power.refereePowerLimit = robot_state.chassis_power_limit;
        chassis_move.super_power.refereeEnergyBuffer = power_heat_data.buffer_energy;
        chassis_move.super_power.activeChargingLimitRatio = 80;
        // CAN_SuperPower_Control(chassis_move.super_power);
        //轮毂

        osDelay(1);
    }
}

void chassisMotorSend(chassis_move_t *CMD_Send)
{
    /* 发送关节 1/3 -> 等待 1 tick -> 关节 2/4 -> 轮毂，保持原顺序。 */
    if (CMD_Send->foot_motor_R.motor_mode != MOTOR_FORCE) CMD_Send->foot_motor_R.torque_out = 0.0f;
    if (CMD_Send->foot_motor_L.motor_mode != MOTOR_FORCE) CMD_Send->foot_motor_L.torque_out = 0.0f;

    if (CMD_Send->joint_motor_1.motor_mode == MOTOR_FORCE)
        MIT_CtrlMotor(0, 0, 0, 0, CMD_Send->joint_motor_1.torque_out, 0x01);
    else
        MIT_CtrlMotor(0, 0, 0, 0, 0, 0x01);
    if (CMD_Send->joint_motor_3.motor_mode == MOTOR_FORCE)
        MIT_CtrlMotor(0, 0, 0, 0, CMD_Send->joint_motor_3.torque_out, 0x03);
    else
        MIT_CtrlMotor(0, 0, 0, 0, 0, 0x03);
    vTaskDelay(1);
    if (CMD_Send->joint_motor_2.motor_mode == MOTOR_FORCE)
        MIT_CtrlMotor(0, 0, 0, 0, CMD_Send->joint_motor_2.torque_out, 0x02);
    else
        MIT_CtrlMotor(0, 0, 0, 0, 0, 0x02);
    if (CMD_Send->joint_motor_4.motor_mode == MOTOR_FORCE)
        MIT_CtrlMotor(0, 0, 0, 0, CMD_Send->joint_motor_4.torque_out, 0x04);
    else
        MIT_CtrlMotor(0, 0, 0, 0, 0, 0x04);

    const int16_t cmd_L = (int16_t)fp32_constrain(((-1.0f) * CMD_Send->foot_motor_L.torque_out * 2598.20f),
                                                  -16384.0f, 16384.0f);

    const int16_t cmd_R = (int16_t)fp32_constrain(((-1.0f) * CMD_Send->foot_motor_R.torque_out * 2598.20f),
                                                  -16384.0f, 16384.0f);
    CAN_cmd_chassis(cmd_L, cmd_R, 0, 0);
}
