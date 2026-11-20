#include "chassis_types.h"
#include "chassis_climb.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* -------------------------- 腿长参数 -------------------------- */

#define CLIMB_HIGH_LENGTH_M 0.40f
#define CLIMB_PULL_LENGTH_M 0.14f
#define CLIMB_LENGTH_ERROR_M 0.06f
#define CLIMB_EXTEND_LENGTH_ERROR_M 0.040f

/* -------------------------- 速度参数 -------------------------- */

/* 攀爬阶段限制操作手的最大前进速度 */
#define CLIMB_APPROACH_SPEED_MAX 1.2f

/* -------------------------- 姿态参数 -------------------------- */

#define CLIMB_READY_PITCH_RAD 0.20f
#define CLIMB_READY_ROLL_RAD 0.10f

/* 向前接近时，机身正向俯仰超过门限并持续确认，才判定撞阶。 */
#define CLIMB_HIT_PITCH_RAD 0.22f

#define CLIMB_ABORT_PITCH_RAD 0.55f
#define CLIMB_ABORT_ROLL_RAD 0.30f

#define CLIMB_PITCH_GYRO_STABLE 0.5f

/* -------------------------- 时间参数 -------------------------- */

#define CLIMB_READY_STABLE_MS 20u
#define CLIMB_READY_TIMEOUT_MS 8000u

#define CLIMB_EXTEND_TIME_MS 900u
#define CLIMB_EXTEND_STABLE_MS 100u
#define CLIMB_EXTEND_TIMEOUT_MS 10000u

#define CLIMB_HIT_CONFIRM_MS 100u

#define CLIMB_PULL_TIME_MS 300u
#define CLIMB_PULL_STABLE_MS 10u
#define CLIMB_PULL_TIMEOUT_MS 5000u

#define CLIMB_STAND_TIME_MS 500u
#define CLIMB_STAND_STABLE_MS 200u
#define CLIMB_STAND_TIMEOUT_MS 6000u

#define CLIMB_FINISHED_STABLE_MS 200u
#define CLIMB_ABORT_RESET_MS 300u
#define CLIMB_RETURN_TIME_MS 1500u

typedef struct
{
    climb_state_e state;

    uint32_t stateTick;
    uint32_t conditionTick;

    fp32 normalLegLength;
    fp32 motionStartLength;
    fp32 abortHoldLength;

    fp32 lastLengthSetL;
    fp32 lastLengthSetR;
    fp32 returnStartLengthL;
    fp32 returnStartLengthR;
    bool lengthTargetValid;

    bool riserDetected;
    bool lastCommand;
} ClimbContext;

/*
 * 模块内部保存状态，因此 chassis_task 不需要调用初始化函数。
 * 初次必须先看到 climb_flag 释放，才接受下一次上升沿。
 */
ClimbContext climbContext = {.state = CLIMB_IDLE,
                             .normalLegLength = 0.19f,
                             .motionStartLength = 0.19f,
                             .abortHoldLength = 0.19f,
                             .lastCommand = true};

static fp32 climbClamp(fp32 value, fp32 minimum, fp32 maximum)
{
    if (value < minimum) return minimum;

    if (value > maximum) return maximum;

    return value;
}

static bool climbTimePassed(uint32_t now, uint32_t start, uint32_t durationMs)
{
    return start != 0u && now - start >= pdMS_TO_TICKS(durationMs);
}

static bool climbConditionHold(ClimbContext *ctx, bool condition, uint32_t now, uint32_t durationMs)
{
    if (!condition)
    {
        ctx->conditionTick = 0u;
        return false;
    }

    if (ctx->conditionTick == 0u) ctx->conditionTick = now;

    return climbTimePassed(now, ctx->conditionTick, durationMs);
}

static void climbEnterState(ClimbContext *ctx, climb_state_e state, uint32_t now)
{
    ctx->state = state;
    ctx->stateTick = now;
    ctx->conditionTick = 0u;
}

static void climbReset(ClimbContext *ctx)
{
    ctx->state = CLIMB_IDLE;
    ctx->stateTick = 0u;
    ctx->conditionTick = 0u;

    ctx->riserDetected = false;
    ctx->lengthTargetValid = false;

    /*
     * Do not overwrite lastCommand here. It continuously tracks the real
     * command level, so a flag stuck high cannot retrigger after completion.
     * A new climb requires a genuine 0 -> 1 edge.
     */
}

static void climbEnterAbort(ClimbContext *ctx, const chassis_move_t *chassis, uint32_t now)
{
    ctx->abortHoldLength = 0.5f * (chassis->chassis_posture_info.chassis_posture_L.leg_length +
                                   chassis->chassis_posture_info.chassis_posture_R.leg_length);

    if (!isfinite(ctx->abortHoldLength)) ctx->abortHoldLength = ctx->normalLegLength;

    climbEnterState(ctx, CLIMB_ABORT, now);
}

/*
 * 攀爬过程中关闭普通位置锁，避免旧的位置目标阻止机器人向前。
 */
static void climbReleasePositionLock(chassis_move_t *chassis)
{
    chassis->chassis_posture_info.position_lock_flag = 0u;
    chassis->chassis_posture_info.position_lock_state = 0u;

    chassis->chassis_posture_info.foot_distance_set = chassis->chassis_posture_info.foot_distance_K;
}

/*
 * 设置左右腿相同的目标长度。
 */
static void climbSetLegTargets(chassis_move_t *chassis, fp32 left, fp32 right)
{
    chassis->chassis_posture_info.chassis_posture_L.leg_length_set = left;

    chassis->chassis_posture_info.chassis_posture_R.leg_length_set = right;

    climbContext.lastLengthSetL = left;
    climbContext.lastLengthSetR = right;
    climbContext.lengthTargetValid = true;
}

static void climbSetLegLength(chassis_move_t *chassis, fp32 length)
{
    climbSetLegTargets(chassis, length, length);
}

static void climbEnterReturn(ClimbContext *ctx, chassis_move_t *chassis, uint32_t now)
{
    /* chassisMotionUpdate has already published ordinary targets for this cycle.
     * Start from the last climb targets, not those overwritten ordinary ones,
     * and not measured lengths: changing PID error at handoff causes a kick.
     */
    ctx->returnStartLengthL = ctx->lengthTargetValid
                                  ? ctx->lastLengthSetL
                                  : chassis->chassis_posture_info.chassis_posture_L.leg_length;
    ctx->returnStartLengthR = ctx->lengthTargetValid
                                  ? ctx->lastLengthSetR
                                  : chassis->chassis_posture_info.chassis_posture_R.leg_length;
    if (!isfinite(ctx->returnStartLengthL) || ctx->returnStartLengthL <= 0.0f)
        ctx->returnStartLengthL = ctx->normalLegLength;
    if (!isfinite(ctx->returnStartLengthR) || ctx->returnStartLengthR <= 0.0f)
        ctx->returnStartLengthR = ctx->normalLegLength;
    climbEnterState(ctx, CLIMB_RETURN_NORMAL, now);
    climbSetLegTargets(chassis, ctx->returnStartLengthL, ctx->returnStartLengthR);
}

/*
 * 在两个腿长之间使用三次平滑曲线插值。
 * 起点和终点的一阶导数均为0，避免目标腿长突然改变。
 */
static fp32 climbSmoothLength(fp32 startLength, fp32 targetLength, uint32_t now, uint32_t startTick,
                              uint32_t durationMs)
{
    fp32 progress;
    fp32 smooth;

    progress = (fp32)(now - startTick) / (fp32)pdMS_TO_TICKS(durationMs);

    progress = climbClamp(progress, 0.0f, 1.0f);
    smooth = progress * progress * (3.0f - 2.0f * progress);

    return startLength + (targetLength - startLength) * smooth;
}

climb_state_e chassisClimbGetState(void)
{
    return climbContext.state;
}

uint8_t chassisClimbAllowsPositionHold(void)
{
    return climbContext.state == CLIMB_IDLE || climbContext.state == CLIMB_READY ||
           climbContext.state == CLIMB_EXTEND || climbContext.state == CLIMB_RETURN_NORMAL;
}

uint8_t chassisClimbLengthOnly(void)
{
    return climbContext.state == CLIMB_PULL_UP || climbContext.state == CLIMB_WHEEL_CONFIRM;
}
bool hitRiser;
ClimbContext *ctx;
void chassisClimbProcess(chassis_move_t *chassis)
{
    ctx = &climbContext;
    chassis_posture_info_t *posture = &chassis->chassis_posture_info;
    const chassis_leg_posture_t *left = &posture->chassis_posture_L;
    const chassis_leg_posture_t *right = &posture->chassis_posture_R;

    uint32_t now;

    bool command;
    bool commandPressed;

    bool bothGround;
    bool postureReady;
    bool legsExtended;
    bool legsPulledUp;
    bool legsAtNormalHeight;
    bool standingStable;

    fp32 requestedSpeed;
    fp32 legLength;
    fp32 normalTargetL;
    fp32 normalTargetR;

    now = (uint32_t)xTaskGetTickCount();
    command = chassis->chassis_data_->climb_flag == 1u;
    commandPressed = command && !ctx->lastCommand;
    normalTargetL = left->leg_length_set;
    normalTargetR = right->leg_length_set;

    ctx->lastCommand = command;

    if (commandPressed && ctx->state != CLIMB_IDLE && ctx->state != CLIMB_ABORT &&
        ctx->state != CLIMB_RETURN_NORMAL)
        climbEnterReturn(ctx, chassis, now);

    requestedSpeed = posture->foot_speed_set;
    legLength = 0.5f * (left->leg_length + right->leg_length);

    bothGround =
        chassis->flag_info.suspend_flag_L == ON_GROUND && chassis->flag_info.suspend_flag_R == ON_GROUND;

    postureReady = fabsf(posture->pitch_angle) < CLIMB_READY_PITCH_RAD &&
                   fabsf(posture->roll_angle) < CLIMB_READY_ROLL_RAD;

    legsExtended = left->leg_length >= CLIMB_HIGH_LENGTH_M - CLIMB_EXTEND_LENGTH_ERROR_M &&
                   right->leg_length >= CLIMB_HIGH_LENGTH_M - CLIMB_EXTEND_LENGTH_ERROR_M;

    legsPulledUp = fabsf(left->leg_length - CLIMB_PULL_LENGTH_M) < CLIMB_LENGTH_ERROR_M &&
                   fabsf(right->leg_length - CLIMB_PULL_LENGTH_M) < CLIMB_LENGTH_ERROR_M;

    legsAtNormalHeight = fabsf(left->leg_length - ctx->normalLegLength) < CLIMB_LENGTH_ERROR_M &&
                         fabsf(right->leg_length - ctx->normalLegLength) < CLIMB_LENGTH_ERROR_M;

    hitRiser = ctx->state == CLIMB_APPROACH && requestedSpeed > 0.0f &&
               posture->pitch_angle > CLIMB_HIT_PITCH_RAD;

    standingStable = legsAtNormalHeight && bothGround && postureReady &&
                     fabsf(posture->pitch_gyro) < CLIMB_PITCH_GYRO_STABLE;

    if (ctx->state != CLIMB_IDLE && ctx->state != CLIMB_FINISHED && ctx->state != CLIMB_ABORT)
    {
        bool sensorInvalid = !isfinite(posture->pitch_angle) ||
                             !isfinite(posture->roll_angle) || !isfinite(legLength);

        bool dangerousPosture = fabsf(posture->pitch_angle) > CLIMB_ABORT_PITCH_RAD ||
                                fabsf(posture->roll_angle) > CLIMB_ABORT_ROLL_RAD;

        bool modeConflict = chassis->mode.chassis_mode != ENABLE_CHASSIS ||
                            chassis->mode.sport_mode == JUMPING_MODE ||
                            chassis->chassis_data_->recovery_flag == 1u;

        if (sensorInvalid || dangerousPosture || modeConflict)
        {
            climbEnterAbort(ctx, chassis, now);
        }
    }

    switch (ctx->state)
    {
        case CLIMB_IDLE:

            if (!commandPressed) return;

            ctx->normalLegLength = posture->ideal_high;

            if (!isfinite(ctx->normalLegLength)) ctx->normalLegLength = 0.19f;

            ctx->normalLegLength = climbClamp(ctx->normalLegLength, 0.17f, CLIMB_HIGH_LENGTH_M);

            climbSetLegTargets(chassis, normalTargetL, normalTargetR);
            posture->foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX, CLIMB_APPROACH_SPEED_MAX);

            ctx->riserDetected = false;

            climbEnterState(ctx, CLIMB_READY, now);
            break;

        case CLIMB_READY:
        {
            bool ready = chassis->mode.chassis_mode == ENABLE_CHASSIS &&
                         chassis->mode.chassis_balancing_mode == BALANCING_READY &&
                         chassis->mode.sport_mode == NORMAL_MOVING_MODE;

            climbSetLegTargets(chassis, normalTargetL, normalTargetR);
            posture->foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX, CLIMB_APPROACH_SPEED_MAX);

            if (climbConditionHold(ctx, ready, now, CLIMB_READY_STABLE_MS))
            {
                ctx->motionStartLength = legLength;
                climbEnterState(ctx, CLIMB_EXTEND, now);
            }
            else if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_READY_TIMEOUT_MS))
            {
                climbEnterReturn(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_EXTEND:

            posture->foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX, CLIMB_APPROACH_SPEED_MAX);

            climbSetLegLength(chassis, climbSmoothLength(ctx->motionStartLength, CLIMB_HIGH_LENGTH_M, now,
                                                         ctx->stateTick, CLIMB_EXTEND_TIME_MS));

            if (climbConditionHold(
                    ctx, now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_EXTEND_TIME_MS) && legsExtended, now,
                    CLIMB_EXTEND_STABLE_MS))
            {
                climbEnterState(ctx, CLIMB_APPROACH, now);
            }
            else if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_EXTEND_TIMEOUT_MS))
            {
                climbEnterReturn(ctx, chassis, now);
            }

            break;

        case CLIMB_APPROACH:

            climbSetLegLength(chassis, CLIMB_HIGH_LENGTH_M);

            posture->foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX, CLIMB_APPROACH_SPEED_MAX);

            if (climbConditionHold(ctx, hitRiser, now, CLIMB_HIT_CONFIRM_MS))
            {
                ctx->riserDetected = true;
                ctx->motionStartLength = legLength;
                posture->foot_speed_set = 0.0f;
                /* 本周期立即切换收腿输出策略，从实测腿长开始平滑收缩。 */
                climbEnterState(ctx, CLIMB_PULL_UP, now);
                climbSetLegLength(chassis, ctx->motionStartLength);
            }

            break;

        case CLIMB_PULL_UP:
        {
            fp32 lengthSet;

            posture->foot_speed_set = 0.0f;

            lengthSet = climbSmoothLength(ctx->motionStartLength, CLIMB_PULL_LENGTH_M, now, ctx->stateTick,
                                          CLIMB_PULL_TIME_MS);
            climbSetLegLength(chassis, lengthSet);

            /* 收腿到位的确认与轨迹并行，不再串行等待机身静止。
             * 轨迹完成后直接恢复平衡并起立；俯仰/横滚异常仍由全局保护中止。 */
            bool pulledStable = climbConditionHold(ctx, legsPulledUp, now, CLIMB_PULL_STABLE_MS);
            if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_PULL_TIME_MS) && pulledStable)
            {
                /* 延续本周期目标，避免从实测腿长重置 PID 误差造成切换冲击。 */
                ctx->motionStartLength = lengthSet;
                climbEnterState(ctx, CLIMB_STAND_UP, now);
            }
            else if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_PULL_TIMEOUT_MS))
            {
                climbEnterReturn(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_WHEEL_CONFIRM:
            /* 保留旧状态编号供历史波形使用，恢复到此状态时也不额外停等。 */
            posture->foot_speed_set = 0.0f;
            climbSetLegLength(chassis, CLIMB_PULL_LENGTH_M);
            ctx->motionStartLength = CLIMB_PULL_LENGTH_M;
            climbEnterState(ctx, CLIMB_STAND_UP, now);
            break;

        case CLIMB_STAND_UP:
        {
            fp32 lengthSet;

            posture->foot_speed_set = 0.3f;

            lengthSet = climbSmoothLength(ctx->motionStartLength, ctx->normalLegLength, now, ctx->stateTick,
                                          CLIMB_STAND_TIME_MS);

            climbSetLegLength(chassis, lengthSet);

            if (climbConditionHold(ctx, now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_STAND_TIME_MS) &&
                                           standingStable, now, CLIMB_STAND_STABLE_MS))
            {
                climbEnterState(ctx, CLIMB_FINISHED, now);
            }
            else if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_STAND_TIMEOUT_MS))
            {
                climbEnterReturn(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_FINISHED:

            posture->foot_speed_set = 0.0f;
            climbSetLegLength(chassis, ctx->normalLegLength);

            if (climbConditionHold(ctx, standingStable, now, CLIMB_FINISHED_STABLE_MS))
            {
                posture->position_lock_state = 0u;

                climbEnterReturn(ctx, chassis, now);
            }

            break;

        case CLIMB_ABORT:

            posture->foot_speed_set = 0.0f;
            climbSetLegLength(chassis, ctx->abortHoldLength);

            if (climbConditionHold(ctx,
                                   postureReady && isfinite(legLength) &&
                                       chassis->mode.chassis_mode == ENABLE_CHASSIS &&
                                       chassis->mode.chassis_balancing_mode == BALANCING_READY &&
                                       chassis->mode.sport_mode == NORMAL_MOVING_MODE &&
                                       chassis->chassis_data_->recovery_flag == 0u,
                                   now, CLIMB_ABORT_RESET_MS))
            {
                posture->position_lock_state = 0u;

                climbEnterReturn(ctx, chassis, now);
            }

            break;

        case CLIMB_RETURN_NORMAL:

            if (!isfinite(normalTargetL) || normalTargetL <= 0.0f)
                normalTargetL = ctx->normalLegLength;
            if (!isfinite(normalTargetR) || normalTargetR <= 0.0f)
                normalTargetR = ctx->normalLegLength;
            posture->foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX, CLIMB_APPROACH_SPEED_MAX);
            climbSetLegTargets(chassis,
                               climbSmoothLength(ctx->returnStartLengthL, normalTargetL, now, ctx->stateTick,
                                                 CLIMB_RETURN_TIME_MS),
                               climbSmoothLength(ctx->returnStartLengthR, normalTargetR, now, ctx->stateTick,
                                                 CLIMB_RETURN_TIME_MS));
            if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_RETURN_TIME_MS)) climbReset(ctx);
            break;

        default:

            climbEnterAbort(ctx, chassis, now);
            break;
    }

    if (!chassisClimbAllowsPositionHold()) climbReleasePositionLock(chassis);
}
