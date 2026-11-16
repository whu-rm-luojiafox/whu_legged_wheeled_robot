#include "chassis_types.h"
#include "chassis_mode.h"
#include "chassis_climb.h"
#include "chassis_recovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

static uint8_t motors_enabled;
fp32 IDEAL_PREPARING_STAND_JUMPING_ANGLE = 0.174532f; //0.0872f;

static void chassis_set_mode(chassis_move_t *c)
{
    uint8_t enabled = c->chassis_data_->chassis_mode != CHASSIS_MODE_OFF &&
                      c->chassis_data_->chassis_mode != CHASSIS_MODE_DEBUG;
    if (enabled && !motors_enabled)
    {
        CAN_clear_dm_error();
        for (uint8_t id = 1; id <= 4; ++id)
            CAN_dm_enable(id);
    }
    motors_enabled = enabled;
    c->mode.chassis_mode = enabled ? ENABLE_CHASSIS : DISABLE_CHASSIS;
    chassis_motor_mode_e motor_mode = enabled ? MOTOR_FORCE : MOTOR_NO_FORCE;
    c->joint_motor_1.motor_mode = c->joint_motor_2.motor_mode = motor_mode;
    c->joint_motor_3.motor_mode = c->joint_motor_4.motor_mode = motor_mode;
    c->foot_motor_L.motor_mode = c->foot_motor_R.motor_mode = motor_mode;

    if (c->mode.chassis_balancing_mode != BALANCING_READY)
        c->mode.sport_mode = NONE;
    else if ((c->mode.sport_mode == JUMPING_MODE && c->mode.jumping_stage != FINISHED) ||
             c->chassis_data_->jump_flag)
        c->mode.sport_mode = JUMPING_MODE;
    else
        c->mode.sport_mode = NORMAL_MOVING_MODE;
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_mode_change)
{
    /* --------------------------------使能/失能 后判断是否 进入/退出 平衡模式--------------------------------  */
    if (chassis_mode_change->mode.chassis_mode == DISABLE_CHASSIS)
        chassis_mode_change->mode.chassis_balancing_mode = NO_FORCE;
    else if (chassis_mode_change->mode.last_chassis_mode == DISABLE_CHASSIS)
    {
        chassis_mode_change->mode.chassis_balancing_mode = FOOT_LAUNCHING;
        chassis_mode_change->chassis_posture_info.yaw_angle_sett =
            chassis_mode_change->chassis_posture_info.yaw_angle_total;
    }
    /* 起立阶段由 chassis_recovery.c 统一推进，不在这里直接跳入 LQR。 */
    //跳跃模式流程
    if (chassis_mode_change->mode.sport_mode == JUMPING_MODE &&
        chassis_mode_change->mode.last_sport_mode != JUMPING_MODE)
    {
        chassis_mode_change->mode.jumping_mode = STANDING_JUMP;
        chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
        chassis_mode_change->flag_info.jump_prepare_complete = 0;
    }
    else if (chassis_mode_change->mode.sport_mode != JUMPING_MODE)
    {
        chassis_mode_change->mode.jumping_mode = NOT_DEFINE;
        chassis_mode_change->mode.jumping_stage = FINISHED;
    }

    else if (chassis_mode_change->mode.jumping_mode == STANDING_JUMP)
    {
        // 站立跳跃状态机
        switch (chassis_mode_change->mode.jumping_stage)
        {
            case READY_TO_JUMP:
                // 进入准备阶段
                chassis_mode_change->mode.jumping_stage = PREPARING_STAND_JUMPING;
                chassis_mode_change->flag_info.jump_prepare_timer = xTaskGetTickCount();
                break;

            case PREPARING_STAND_JUMPING:
                chassis_mode_change->flag_info.jump_prepare_complete = 1;
                chassis_mode_change->mode.jumping_stage = EXTENDING_LEGS;
                chassis_mode_change->flag_info.jump_extend_timer = xTaskGetTickCount();
                break;

            case EXTENDING_LEGS:
                // 腿伸长阶段
                if (chassis_mode_change->chassis_posture_info.chassis_posture_L.leg_length >= 0.34f &&
                    chassis_mode_change->chassis_posture_info.chassis_posture_R.leg_length >= 0.34f &&
                    (xTaskGetTickCount() - chassis_mode_change->flag_info.jump_extend_timer) >
                        pdMS_TO_TICKS(50))
                {
                    chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS_2;
                    chassis_mode_change->flag_info.jump_contact_timer = xTaskGetTickCount();
                }
                else if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_extend_timer) >
                         pdMS_TO_TICKS(500))
                {
                    // 伸腿超时，认为失败
                    chassis_mode_change->mode.jumping_stage = FINISHED;
                    chassis_mode_change->mode.sport_mode = NORMAL_MOVING_MODE;
                }
                break;

            case CONSTACTING_LEGS_2:
                // 再次收缩腿准备落地
                if (chassis_mode_change->chassis_posture_info.chassis_posture_L.leg_length <= 0.11f &&
                    chassis_mode_change->chassis_posture_info.chassis_posture_R.leg_length <= 0.11f)
                {
                    chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
                }
                else if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_contact_timer) >
                         pdMS_TO_TICKS(300))
                {
                    // 收缩超时，强制进入落地准备
                    chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
                }
                break;

            case PREPARING_LANDING:
                // 等待落地检测
                if (chassis_mode_change->flag_info.suspend_flag_R == ON_GROUND &&
                    chassis_mode_change->flag_info.suspend_flag_L == ON_GROUND &&
                    (xTaskGetTickCount() - chassis_mode_change->flag_info.jump_contact_timer) >
                        pdMS_TO_TICKS(500))
                {
                    chassis_mode_change->mode.jumping_stage = FINISHED;
                }
                // 落地超时保护
                else if ((xTaskGetTickCount() - chassis_mode_change->flag_info.jump_contact_timer) >
                         pdMS_TO_TICKS(2000))
                {
                    chassis_mode_change->mode.jumping_stage = FINISHED;
                }
                break;

            case CONSTACTING_LEGS:
                break;

            case FINISHED:
                // 跳跃完成，重置状态
                chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
                chassis_mode_change->mode.sport_mode = NORMAL_MOVING_MODE;
                chassis_mode_change->flag_info.jump_prepare_complete = 0;
                break;
        }
    }
    else
    {
        chassis_mode_change->mode.jumping_stage = FINISHED;
    }
}

void chassisModeInit(chassis_move_t *c)
{
    c->mode.chassis_mode = c->mode.last_chassis_mode = DISABLE_CHASSIS;
    c->mode.chassis_balancing_mode = c->mode.last_chassis_balancing_mode = NO_FORCE;
    c->mode.sport_mode = c->mode.last_sport_mode = NORMAL_MOVING_MODE;
    c->joint_motor_1.motor_mode = c->joint_motor_1.last_motor_mode = MOTOR_NO_FORCE;
    c->joint_motor_2.motor_mode = c->joint_motor_2.last_motor_mode = MOTOR_NO_FORCE;
    c->joint_motor_3.motor_mode = c->joint_motor_3.last_motor_mode = MOTOR_NO_FORCE;
    c->joint_motor_4.motor_mode = c->joint_motor_4.last_motor_mode = MOTOR_NO_FORCE;
    c->foot_motor_L.motor_mode = c->foot_motor_L.last_motor_mode = MOTOR_NO_FORCE;
    c->foot_motor_R.motor_mode = c->foot_motor_R.last_motor_mode = MOTOR_NO_FORCE;
}

void chassisModeUpdate(chassis_move_t *c)
{
    c->mode.last_chassis_mode = c->mode.chassis_mode;
    c->mode.last_chassis_balancing_mode = c->mode.chassis_balancing_mode;
    c->mode.last_sport_mode = c->mode.sport_mode;
    c->mode.last_jumping_mode = c->mode.jumping_mode;
    c->mode.last_jumping_stage = c->mode.jumping_stage;
    c->mode.last_chassis_high_mode = c->mode.chassis_high_mode;

    c->flag_info.last_overpower_warning_flag = c->flag_info.overpower_warning_flag;

    c->joint_motor_1.last_motor_mode = c->joint_motor_1.motor_mode;
    c->joint_motor_2.last_motor_mode = c->joint_motor_2.motor_mode;
    c->joint_motor_3.last_motor_mode = c->joint_motor_3.motor_mode;
    c->joint_motor_4.last_motor_mode = c->joint_motor_4.motor_mode;
    c->foot_motor_L.last_motor_mode = c->foot_motor_L.motor_mode;
    c->foot_motor_R.last_motor_mode = c->foot_motor_R.motor_mode;

    chassis_set_mode(c);
    chassis_mode_change_control_transit(c);
    c->mode.chassis_high_mode = NORMAL_MODE;
}

void chassisModeSetTargets(chassis_move_t *c)
{
    chassis_leg_posture_t *left = &c->chassis_posture_info.chassis_posture_L;
    chassis_leg_posture_t *right = &c->chassis_posture_info.chassis_posture_R;
    float angle =
        c->mode.jumping_stage == PREPARING_STAND_JUMPING ? IDEAL_PREPARING_STAND_JUMPING_ANGLE : 0.0f;
    left->leg_angle_set = right->leg_angle_set = angle;

    if (c->mode.sport_mode == JUMPING_MODE)
    {
        float length = c->chassis_posture_info.ideal_high;
        switch (c->mode.jumping_stage)
        {
            case PREPARING_STAND_JUMPING:
                length = 0.13f;
                break;
            case EXTENDING_LEGS:
                length = 0.34f;
                break;
            case CONSTACTING_LEGS_2:
                length = 0.10f;
                break;
            case PREPARING_LANDING:
                length = 0.15f;
                break;
            default:
                break;
        }
        left->leg_length_set = right->leg_length_set = length;
    }

}

uint8_t chassisIsEnabledOnGround(const chassis_move_t *chassis)
{
    return chassis->mode.chassis_mode == ENABLE_CHASSIS && chassis->flag_info.suspend_flag_L == ON_GROUND &&
           chassis->flag_info.suspend_flag_R == ON_GROUND;
}

uint8_t chassisIsBalancingOnGround(const chassis_move_t *chassis)
{
    return chassisIsEnabledOnGround(chassis) && chassis->mode.chassis_balancing_mode == BALANCING_READY &&
           chassis->mode.sport_mode == NORMAL_MOVING_MODE && chassisClimbGetState() == CLIMB_IDLE &&
           !chassisRecoveryIsActive() && !chassis->chassis_data_->climb_flag &&
           !chassis->chassis_data_->recovery_flag;
}
