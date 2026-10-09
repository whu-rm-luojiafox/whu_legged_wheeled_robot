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

typedef enum
{
    REC_DIRECTION_BACKWARD = -1,
    REC_DIRECTION_NONE = 0,
    REC_DIRECTION_FORWARD = 1
} RecoveryDirection;

typedef struct
{
    RecoveryState state;
    uint32_t startTick;
    uint32_t stableTick;
    fp32 holdLength;
    bool lastKey;
    bool rotatePending;
    uint32_t rotateRequestTick;
    uint32_t alignTick;
    bool alignReady;
    RecoveryDirection direction;
} RecoveryContext;

RecoveryContext recovery = {
    REC_IDLE, 0u, 0u, 0.0f, true, false, 0u,
    0u, false, REC_DIRECTION_NONE
};

/* Existing virtual-force mapper in chassis_task.c. */
extern void Chassis_Torque_Combine(chassis_move_t *chassis);

/* Tune these limits only after checking the leg direction on a stand. */
#define FALL_PITCH       0.80f
#define FALL_ROLL        0.70f
#define SIDE_LIMIT       1.0f
#define UPRIGHT_PITCH    0.18f
#define LOCAL_ANGLE_MAX  0.35f
#define MOTOR_TORQUE_MAX 4.0f
#define ROTATE_REQUEST_TIMEOUT_MS 3000u
#define LEG_ALIGN_ERROR_RAD  0.0873f  // 约5°
#define LEG_ALIGN_WAIT_MS    200u     // 连续对齐200ms

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

static fp32 wrappedAngleDifference(fp32 left, fp32 right)
{
    fp32 difference = left - right;

    while (difference > PI) difference -= PI2;
    while (difference < -PI) difference += PI2;
    return difference;
}

static void chassisRecoveryInit(RecoveryContext *r)
{
    r->state = REC_IDLE;
    r->startTick = 0u;
    r->stableTick = 0u;
    r->holdLength = 0.0f;
    r->lastKey = true; // First observe R released before accepting a press.
    r->rotatePending = false;
    r->rotateRequestTick = 0u;
    r->alignTick = 0u;
    r->alignReady = false;
    r->direction = REC_DIRECTION_NONE;
}
bool fallenForward,fallenBackward;
/* 返回当前反倒状态 */
static bool chassisRecoveryUpdate(RecoveryContext *r, const chassis_move_t *c)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    fp32 pitch = c->chassis_posture_info.pitch_angle;
    fp32 roll = c->chassis_posture_info.roll_angle;
    bool key = c->chassis_data_->recovery_flag == 1u;
    bool pressed =  key && !r->lastKey;
    r->lastKey = key;
    bool upright = pitch < UPRIGHT_PITCH && pitch > -UPRIGHT_PITCH &&
                         fabsf(roll) < 0.25f &&
                         fabsf(c->chassis_posture_info.pitch_gyro) < 0.40f;
    fallenForward = pitch > PI_2 && pitch < PI;
    fallenBackward = pitch > -PI && pitch < -PI_2;

    if (c->mode.chassis_mode != ENABLE_CHASSIS)
    {
        chassisRecoveryInit(r);
        return false;
    }

    /* Latch the one-cycle key edge until REC_WAIT consumes it. */
    if (pressed)
    {
        r->rotatePending = true;
        r->rotateRequestTick = now;
    }

    /* Do not let an old key press trigger recovery at a later time. */
    if (r->rotatePending &&
        now - r->rotateRequestTick >= pdMS_TO_TICKS(ROTATE_REQUEST_TIMEOUT_MS))
    {
        r->rotatePending = false;
        r->rotateRequestTick = 0u;
    }

    //检查角度值合理性
    if (!valid(pitch) || !valid(roll))
    {
        r->state = REC_WAIT;
        r->stableTick = 0u;
        r->alignTick = 0u;
        r->alignReady = false;
        r->direction = REC_DIRECTION_NONE;
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

        if (r->rotatePending && (fallenForward || fallenBackward) )
        {
            r->rotatePending = false;
            r->rotateRequestTick = 0u;
            r->holdLength =(c->chassis_posture_info.chassis_posture_L.leg_length +
                 c->chassis_posture_info.chassis_posture_R.leg_length) * 0.5f;
            r->startTick = now;
            r->stableTick = 0u;
            r->alignTick = 0u;
            r->alignReady = false;
            r->direction = fallenForward ?
                REC_DIRECTION_FORWARD : REC_DIRECTION_BACKWARD;
            r->state = REC_ROTATE;
        }
    }
    else if (r->state == REC_ROTATE)
    {
        if ( now - r->startTick >= pdMS_TO_TICKS(2500))
        {
            r->state = REC_WAIT;
            r->stableTick = 0u;
            r->alignTick = 0u;
            r->alignReady = false;
            r->direction = REC_DIRECTION_NONE;
        }
        else if (upright)
        {
            if (r->stableTick == 0u) r->stableTick = now;
            if (now - r->stableTick >= pdMS_TO_TICKS(1500))
            {
                chassisRecoveryInit(r);
                return true;
            }
        }
        else
            r->stableTick = 0u;

        if (r->state == REC_ROTATE && !upright && !r->alignReady )
        {
            const fp32 angleError = fabsf(wrappedAngleDifference(
                c->chassis_posture_info.chassis_posture_L.leg_angle,
                c->chassis_posture_info.chassis_posture_R.leg_angle));

            if (angleError <= LEG_ALIGN_ERROR_RAD)
            {
                if (r->alignTick == 0u) r->alignTick = now;
                if (now - r->alignTick >= pdMS_TO_TICKS(LEG_ALIGN_WAIT_MS))
                    r->alignReady = true;
            }
            else r->alignTick = 0u;
        }
        else if (!r->alignReady) r->alignTick = 0u;
    }
    return false;
}
fp32 angleDifference ;
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
    angleDifference = wrappedAngleDifference(left->leg_angle, right->leg_angle);

    if (r->state != REC_ROTATE || r->direction == REC_DIRECTION_NONE ||
        !valid(pitch) || !valid(r->holdLength) ||
        !valid(left->leg_length) || !valid(right->leg_length) ||
        !valid(left->leg_angle_local) || !valid(right->leg_angle_local) ||
        !valid(left->leg_gyro_local) || !valid(right->leg_gyro_local))
        return;

    // Hold the length found when R was pressed; the gas springs provide support.
    t->joint_stand_torque_L = clamp(
        200.0f * (r->holdLength - left->leg_length) -
        40.0f * left->leg_dlength_jacobian, 8.0f);
    t->joint_stand_torque_R = clamp(
        200.0f * (r->holdLength - right->leg_length) -
        40.0f * right->leg_dlength_jacobian, 8.0f);

    if (fabsf(pitch) < UPRIGHT_PITCH) return;

    // 先对齐双腿；进入角差容差后保持静止，等待稳定计时完成。
    if (!r->alignReady)
    {
        angleDifference = wrappedAngleDifference(left->leg_angle, right->leg_angle);

        if (r->alignTick != 0u) return;

        if (r->direction == REC_DIRECTION_FORWARD)
        {
            if (angleDifference > 0.0f)
                t->joint_balancing_torque_R = MOTOR_TORQUE_MAX;
            else
                t->joint_balancing_torque_L = MOTOR_TORQUE_MAX;
        }
        else
        {
            if (angleDifference > 0.0f)
                t->joint_balancing_torque_L = -MOTOR_TORQUE_MAX;
            else
                t->joint_balancing_torque_R = -MOTOR_TORQUE_MAX;
        }
        return;
    }

    // 角差连续稳定200ms后，双腿同时撑起机身。
    t->joint_balancing_torque_L = t->joint_balancing_torque_R =
        r->direction == REC_DIRECTION_FORWARD ?
            MOTOR_TORQUE_MAX : -MOTOR_TORQUE_MAX;
}



void chassisRecoveryProcess(chassis_move_t *c)
{
    const bool finished = chassisRecoveryUpdate(&recovery, c);

    if (recovery.state == REC_IDLE && !finished) return;

    if (finished)
    {
        c->mode.chassis_balancing_mode = FOOT_LAUNCHING;
        c->chassis_posture_info.position_lock_state = 0u;
        c->chassis_posture_info.foot_speed_set = 0.0f;
        c->chassis_posture_info.yaw_angle_sett = c->chassis_posture_info.yaw_angle_total;
        PID_clear(&c->leg_L_length_pid);
        PID_clear(&c->leg_R_length_pid);
    }

    c->mode.sport_mode = NONE;
    c->mode.jumping_stage = FINISHED;
    chassisRecoveryApply(&recovery, c);
    Chassis_Torque_Combine(c);
}
