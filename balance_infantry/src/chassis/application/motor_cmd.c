#include "motor_cmd.h"
#include "chassis_task.h"
#include "can_service.h"
#include "cmsis_os.h"
#include "referee.h"

extern chassis_move_t chassis_move;
extern robot_status_t robot_state;
extern power_heat_data_t power_heat_data;

static uint8_t transition_target[4];

static uint8_t swing_drives_ready(void)
{
    for (uint16_t id = 1; id <= 4; ++id)
    {
        if (CAN_dm_control_mode(id) != DM_CONTROL_SPEED ||
            CAN_dm_control_mode_pending(id) || transition_target[id - 1u])
            return 0u;
    }
    return 1u;
}

static void send_joint(const joint_motor_t *motor, uint16_t id, uint8_t swing_ready)
{
    const uint8_t mode = CAN_dm_control_mode(id);
    const uint16_t mode_id = id + (mode == DM_CONTROL_SPEED ? 0x200u : 0u);
    const uint8_t force_enabled = motor->motor_mode == MOTOR_FORCE;
    /* 遥控失能 = MIT 零力矩卸力：退出速度闭环，但不向驱动器发送失能帧。
     * 不能一直 speed_ctrl(0)，否则速度环会主动抗住外力。 */
    const uint8_t target = force_enabled && motor->velocity_native ? DM_CONTROL_SPEED : DM_CONTROL_MIT;
    const uint8_t pending = CAN_dm_control_mode_pending(id);
    if (mode != target || pending != 0u)
    {
        /* One frame per motor per tick: stop, request mode, then enable on ACK.
         * No delay or extra burst into the three hardware CAN mailboxes. */
        if (transition_target[id - 1u] != target)
        {
            if (mode == DM_CONTROL_SPEED) speed_ctrl(0.0f, id);
            else MIT_CtrlMotor(0, 0, 0, 0, 0, id);
            transition_target[id - 1u] = target;
        }
        else
            CAN_dm_request_control_mode(id, target);
        return;
    }
    if (transition_target[id - 1u])
    {
        CAN_dm_enable(mode_id);
        transition_target[id - 1u] = 0u;
        return;
    }
    if (!force_enabled) MIT_CtrlMotor(0, 0, 0, 0, 0, id);
    else if (motor->velocity_native) speed_ctrl(swing_ready ? motor->velocity_set : 0.0f, id);
    else MIT_CtrlMotor(0.0f, 0.0f, 0.0f, 0.0f, motor->torque_out, id);
}

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

    /* 等四个关节切完模式，再下发非零摆腿速度。 */
    const uint8_t swing_ready = CMD_Send->swing_velocity_active && swing_drives_ready();
    send_joint(&CMD_Send->joint_motor_1, 0x01, swing_ready);
    send_joint(&CMD_Send->joint_motor_3, 0x03, swing_ready);
    vTaskDelay(1);
    send_joint(&CMD_Send->joint_motor_2, 0x02, swing_ready);
    send_joint(&CMD_Send->joint_motor_4, 0x04, swing_ready);

    const int16_t cmd_L = (int16_t)fp32_constrain(((-1.0f) * CMD_Send->foot_motor_L.torque_out * 2598.20f),
                                                  -16384.0f, 16384.0f);

    const int16_t cmd_R = (int16_t)fp32_constrain(((-1.0f) * CMD_Send->foot_motor_R.torque_out * 2598.20f),
                                                  -16384.0f, 16384.0f);
    CAN_cmd_chassis(cmd_L, cmd_R, 0, 0);
}
