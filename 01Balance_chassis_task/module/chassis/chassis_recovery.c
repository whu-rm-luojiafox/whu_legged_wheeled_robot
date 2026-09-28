#include "chassis_recovery.h"
#include "struct_typedef.h"
#include "uart_receive.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    REC_IDLE,
    REC_WAIT,
    REC_ROTATE
} RecoveryState;

typedef struct
{
    RecoveryState state;
    uint32_t startTick;
    uint32_t stableTick;
    fp32 holdLength;
    bool lastKey;
} RecoveryContext;

RecoveryContext recovery = { REC_IDLE, 0u, 0u, 0.0f, true };

/* Existing virtual-force mapper in chassis_task.c. */
extern void Chassis_Torque_Combine(chassis_move_t *chassis);

/* Tune these limits only after checking the leg direction on a stand. */
#define FALL_PITCH       0.65f
#define FALL_ROLL        0.70f
#define SIDE_LIMIT       0.50f
#define UPRIGHT_PITCH    0.18f
#define LOCAL_ANGLE_MAX  0.35f
#define MOTOR_TORQUE_MAX 2.0f

static fp32 clamp(fp32 value, fp32 limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static bool valid(fp32 value)
{
    return value == value && fabsf(value) < 1000.0f;
}

static void chassisRecoveryInit(RecoveryContext *r)
{
    r->state = REC_IDLE;
    r->startTick = 0u;
    r->stableTick = 0u;
    r->holdLength = 0.0f;
    r->lastKey = true; // First observe R released before accepting a press.
}

/* 返回当前反倒状态 */
static bool chassisRecoveryUpdate(RecoveryContext *r, const chassis_move_t *c,
                                  bool commandFresh)
{
    const uint32_t now = (uint32_t)xTaskGetTickCount();
    const fp32 pitch = c->chassis_posture_info.pitch_angle;
    const fp32 roll = c->chassis_posture_info.roll_angle;
    const bool key = c->chassis_data_->recovery_flag == 1u;
    const bool pressed = commandFresh && key && !r->lastKey;
    const bool upright = pitch < UPRIGHT_PITCH && pitch > -UPRIGHT_PITCH &&
                         fabsf(roll) < 0.25f &&
                         fabsf(c->chassis_posture_info.pitch_gyro) < 0.40f;

    // A lost UART link must be followed by an observed release and a new press.
    r->lastKey = commandFresh ? key : true;
    if (c->mode.chassis_mode != ENABLE_CHASSIS)
    {
        chassisRecoveryInit(r);
        return false;
    }
    //检查角度值合理性
    if (!valid(pitch) || !valid(roll))
    {
        r->state = REC_WAIT;
        r->stableTick = 0u;
        return false;
    }

    if (r->state == REC_IDLE)
    {
        if (c->mode.sport_mode != JUMPING_MODE &&
            (fabsf(pitch) > FALL_PITCH || fabsf(roll) > FALL_ROLL))
            r->state = REC_WAIT;
    }
    else if (r->state == REC_WAIT)
    {
        if (upright)
        {
            if (r->stableTick == 0u) r->stableTick = now;
            if (now - r->stableTick >= pdMS_TO_TICKS(300))
            {
                chassisRecoveryInit(r);
                return true;
            }
        }
        else
            r->stableTick = 0u;

        if (pressed && fabsf(pitch) > FALL_PITCH &&
            fabsf(roll) < SIDE_LIMIT &&
            valid(c->chassis_posture_info.chassis_posture_L.leg_length) &&
            valid(c->chassis_posture_info.chassis_posture_R.leg_length))
        {
            r->holdLength =
                (c->chassis_posture_info.chassis_posture_L.leg_length +
                 c->chassis_posture_info.chassis_posture_R.leg_length) * 0.5f;
            r->startTick = now;
            r->stableTick = 0u;
            r->state = REC_ROTATE;
        }
    }
    else if (r->state == REC_ROTATE)
    {
        if (!commandFresh || !key || fabsf(roll) >= SIDE_LIMIT ||
            now - r->startTick >= pdMS_TO_TICKS(2500))
        {
            r->state = REC_WAIT;
            r->stableTick = 0u;
        }
        else if (upright)
        {
            if (r->stableTick == 0u) r->stableTick = now;
            if (now - r->stableTick >= pdMS_TO_TICKS(300))
            {
                chassisRecoveryInit(r);
                return true;
            }
        }
        else
            r->stableTick = 0u;
    }
    return false;
}

static void chassisRecoveryApply(const RecoveryContext *r, chassis_move_t *c)
{
    torque_info_t *t = &c->torque_info;
    const chassis_leg_posture_t *left = &c->chassis_posture_info.chassis_posture_L;
    const chassis_leg_posture_t *right = &c->chassis_posture_info.chassis_posture_R;
    const fp32 pitch = c->chassis_posture_info.pitch_angle;

    // WAIT and the handoff cycle send zero torque through the existing mapper.
    t->foot_balancing_torque_L = t->foot_balancing_torque_R = 0.0f;
    t->foot_moving_torque_L = t->foot_moving_torque_R = 0.0f;
    t->joint_balancing_torque_L = t->joint_balancing_torque_R = 0.0f;
    t->joint_moving_torque_L = t->joint_moving_torque_R = 0.0f;
    t->joint_roll_torque_L = t->joint_roll_torque_R = 0.0f;
    t->joint_stand_torque_L = t->joint_stand_torque_R = 0.0f;

    if (r->state != REC_ROTATE || !valid(pitch) || !valid(r->holdLength) ||
        !valid(left->leg_length) || !valid(right->leg_length) ||
        !valid(left->leg_angle_local) || !valid(right->leg_angle_local) ||
        !valid(left->leg_gyro_local) || !valid(right->leg_gyro_local))
        return;

    // Hold the length found when R was pressed; the gas springs provide support.
    t->joint_stand_torque_L = clamp(
        40.0f * (r->holdLength - left->leg_length) -
        3.0f * left->leg_dlength_jacobian, 8.0f);
    t->joint_stand_torque_R = clamp(
        40.0f * (r->holdLength - right->leg_length) -
        3.0f * right->leg_dlength_jacobian, 8.0f);

    if (fabsf(pitch) < UPRIGHT_PITCH) return;

    // Local angles: left target = -pitch; right target = +pitch.
    if(fabsf(left->leg_angle-right->leg_angle)>PI/2)
    {
        if(pitch<PI/2 && pitch>0)
        {
            if(left->leg_angle>0) t->joint_balancing_torque_L = 2;
            if(right->leg_angle>0) t->joint_balancing_torque_R = -2;
        }
        else
        {
            if(left->leg_angle<0) t->joint_balancing_torque_L = -2;
            if(right->leg_angle<0) t->joint_balancing_torque_R = 2;
        }
    }
    else
    {
        if(pitch<PI/2)
        {
            t->joint_balancing_torque_L = 2;
            t->joint_balancing_torque_R = -2;
        }
        else
        {
            t->joint_balancing_torque_L = -2;
            t->joint_balancing_torque_R = 2;
        }
    }
}

static void chassisRecoveryLimitMotorOutput(chassis_move_t *c)
{
    c->joint_motor_1.torque_out = clamp(c->joint_motor_1.torque_out, MOTOR_TORQUE_MAX);
    c->joint_motor_2.torque_out = clamp(c->joint_motor_2.torque_out, MOTOR_TORQUE_MAX);
    c->joint_motor_3.torque_out = clamp(c->joint_motor_3.torque_out, MOTOR_TORQUE_MAX);
    c->joint_motor_4.torque_out = clamp(c->joint_motor_4.torque_out, MOTOR_TORQUE_MAX);
}

void chassisRecoveryProcess(chassis_move_t *c)
{
    const bool finished = chassisRecoveryUpdate(
        &recovery, c, uartChassisCommandFresh(100u) != 0u);

    if (recovery.state == REC_IDLE && !finished) return;

    if (finished)
    {
        c->mode.chassis_balancing_mode = FOOT_LAUNCHING;
        c->chassis_posture_info.position_lock_state = 0u;
        c->chassis_posture_info.foot_speed_set = 0.0f;
        c->chassis_posture_info.yaw_angle_sett =
            c->chassis_posture_info.yaw_angle_total;
        PID_clear(&c->leg_L_length_pid);
        PID_clear(&c->leg_R_length_pid);
    }

    c->mode.sport_mode = NONE;
    c->mode.jumping_stage = FINISHED;
    chassisRecoveryApply(&recovery, c);
    Chassis_Torque_Combine(c);
    chassisRecoveryLimitMotorOutput(c);
}
