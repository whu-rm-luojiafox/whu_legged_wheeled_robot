#include "chassis_climb.h"
#include "uart_receive.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* -------------------------- 腿长参数 -------------------------- */

#define CLIMB_HIGH_LENGTH_M             0.34f
#define CLIMB_PULL_LENGTH_M             0.14f
#define CLIMB_LENGTH_ERROR_M            0.008f

/* -------------------------- 速度参数 -------------------------- */

/* 开始判断碰撞所要求的最小操作手前进指令 */
#define CLIMB_FORWARD_COMMAND_MIN       0.08f

/* 攀爬阶段限制操作手的最大前进速度 */
#define CLIMB_APPROACH_SPEED_MAX        0.35f
#define CLIMB_CROSS_SPEED_MAX           0.25f

/* 碰到台阶时，实际轮速应低于此值 */
#define CLIMB_STALL_SPEED_MPS           0.08f

/* 越过台阶棱边后，轮速应恢复到此值以上 */
#define CLIMB_RECOVER_SPEED_MPS         0.15f

/* -------------------------- 电流参数 -------------------------- */

/*
 * 下面两个数只是示例值。
 * given_current 是电机反馈原始值，必须根据实车日志重新标定。
 */
#define CLIMB_HIT_CURRENT_RAW           5000.0f
#define CLIMB_RELEASE_CURRENT_RAW       3000.0f

/* -------------------------- 姿态参数 -------------------------- */

#define CLIMB_READY_PITCH_RAD           0.12f
#define CLIMB_READY_ROLL_RAD            0.08f

#define CLIMB_ABORT_PITCH_RAD           0.55f
#define CLIMB_ABORT_ROLL_RAD            0.30f

#define CLIMB_PITCH_GYRO_STABLE         0.25f
#define CLIMB_LEG_ANGLE_ERROR_RAD       0.10f

/* -------------------------- 时间参数 -------------------------- */

#define CLIMB_COMMAND_TIMEOUT_MS        100u

#define CLIMB_READY_STABLE_MS           100u
#define CLIMB_READY_TIMEOUT_MS          2000u

#define CLIMB_EXTEND_STABLE_MS          100u
#define CLIMB_EXTEND_TIMEOUT_MS         1500u

#define CLIMB_HIT_CONFIRM_MS            30u
#define CLIMB_CROSS_TIMEOUT_MS          3000u

#define CLIMB_TOP_DETECT_MS             80u
#define CLIMB_TOP_STABLE_MS             150u
#define CLIMB_TOP_CONFIRM_TIMEOUT_MS     1200u

#define CLIMB_PULL_TIME_MS              600u
#define CLIMB_PULL_STABLE_MS            120u
#define CLIMB_PULL_TIMEOUT_MS            1500u

#define CLIMB_WHEEL_CONTACT_STABLE_MS    100u
#define CLIMB_WHEEL_CONTACT_TIMEOUT_MS   1000u

#define CLIMB_STAND_TIME_MS             700u
#define CLIMB_STAND_STABLE_MS           200u
#define CLIMB_STAND_TIMEOUT_MS          1800u

#define CLIMB_FINISHED_STABLE_MS        200u
#define CLIMB_ABORT_RESET_MS            300u

typedef struct
{
    climb_state_e state;

    uint32_t stateTick;
    uint32_t conditionTick;

    fp32 normalLegLength;
    fp32 motionStartLength;
    fp32 abortHoldLength;
    fp32 approachLegAngle;

    fp32 currentFiltered;

    bool riserDetected;
    bool lastCommand;
} ClimbContext;

/*
 * 模块内部保存状态，因此 chassis_task 不需要调用初始化函数。
 * 初次必须先看到 climb_flag 释放，才接受下一次上升沿。
 */
static ClimbContext climbContext =
{
    CLIMB_IDLE,
    0u,
    0u,
    0.25f,
    0.25f,
    0.25f,
    0.0f,
    0.0f,
    false,
    true
};

static fp32 climbClamp(fp32 value, fp32 minimum, fp32 maximum)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}

static bool climbValueValid(fp32 value)
{
    return value == value && fabsf(value) < 1000.0f;
}

static bool climbTimePassed(uint32_t now,
                            uint32_t start,
                            uint32_t durationMs)
{
    return start != 0u &&
           now - start >= pdMS_TO_TICKS(durationMs);
}

static bool climbConditionHold(ClimbContext *ctx,
                               bool condition,
                               uint32_t now,
                               uint32_t durationMs)
{
    if (!condition)
    {
        ctx->conditionTick = 0u;
        return false;
    }

    if (ctx->conditionTick == 0u)
        ctx->conditionTick = now;

    return climbTimePassed(now,
                           ctx->conditionTick,
                           durationMs);
}

static void climbEnterState(ClimbContext *ctx,
                            climb_state_e state,
                            uint32_t now)
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
    ctx->currentFiltered = 0.0f;

    /*
     * Do not overwrite lastCommand here. It continuously tracks the real
     * command level, so a flag stuck high cannot retrigger after completion.
     * A new climb requires a genuine 0 -> 1 edge.
     */
}

static void climbEnterAbort(ClimbContext *ctx,
                            const chassis_move_t *chassis,
                            uint32_t now)
{
    ctx->abortHoldLength =
        0.5f *
        (chassis->chassis_posture_info
                    .chassis_posture_L.leg_length +
         chassis->chassis_posture_info
                    .chassis_posture_R.leg_length);

    if (!climbValueValid(ctx->abortHoldLength))
        ctx->abortHoldLength = ctx->normalLegLength;

    climbEnterState(ctx, CLIMB_ABORT, now);
}

/*
 * 攀爬过程中关闭普通位置锁，避免旧的位置目标阻止机器人向前。
 */
static void climbReleasePositionLock(chassis_move_t *chassis)
{
    chassis->chassis_posture_info.position_lock_flag = 0u;
    chassis->chassis_posture_info.position_lock_state = 0u;

    chassis->chassis_posture_info.foot_distance_set =
        chassis->chassis_posture_info.foot_distance_K;
}

/*
 * 设置左右腿相同的目标长度。
 */
static void climbSetLegLength(chassis_move_t *chassis,
                              fp32 length)
{
    chassis->chassis_posture_info
        .chassis_posture_L.leg_length_set = length;

    chassis->chassis_posture_info
        .chassis_posture_R.leg_length_set = length;
}

/*
 * 在两个腿长之间使用三次平滑曲线插值。
 * 起点和终点的一阶导数均为0，避免目标腿长突然改变。
 */
static fp32 climbSmoothLength(fp32 startLength,
                              fp32 targetLength,
                              uint32_t now,
                              uint32_t startTick,
                              uint32_t durationMs)
{
    fp32 progress;
    fp32 smooth;

    if (durationMs == 0u)
        return targetLength;

    progress =
        (fp32)(now - startTick) /
        (fp32)pdMS_TO_TICKS(durationMs);

    progress = climbClamp(progress, 0.0f, 1.0f);
    smooth = progress * progress * (3.0f - 2.0f * progress);

    return startLength +
           (targetLength - startLength) * smooth;
}

climb_state_e chassisClimbGetState(void)
{
    return climbContext.state;
}

void chassisClimbProcess(chassis_move_t *chassis)
{
    ClimbContext *ctx = &climbContext;

    uint32_t now;


    bool command;
    bool commandPressed;

    bool bothGround;
    bool postureReady;
    bool legsExtended;
    bool legsPulledUp;
    bool legsAtNormalHeight;
    bool hitRiser;
    bool topDetected;
    bool bodyOnStepStable;
    bool standingStable;

    fp32 requestedSpeed;
    fp32 wheelSpeed;
    fp32 currentLeft;
    fp32 currentRight;
    fp32 currentAverage;
    fp32 legLength;
    fp32 localLegAngle;

    if (chassis == NULL || chassis->chassis_data_ == NULL)
        return;
    now = (uint32_t)xTaskGetTickCount();
    command = chassis->chassis_data_->climb_flag == 1u;
    commandPressed = command && !ctx->lastCommand;

    /*
     * 通信失联时将 lastCommand 置为true。
     * 恢复通信后必须先释放按键，防止意外重新触发。
     */
    ctx->lastCommand = command;
    /*
     * 保存 Target_Value_Set() 生成的操作手速度指令。
     * 后续攀爬状态只对它限幅，不主动产生前进速度。
     */
    requestedSpeed =
        chassis->chassis_posture_info.foot_speed_set;

    wheelSpeed =
        0.5f *
        (chassis->foot_motor_L.speed +
         chassis->foot_motor_R.speed);

    currentLeft =
        fabsf((fp32)chassis->foot_motor_L
                          .motor_measure->given_current);

    currentRight =
        fabsf((fp32)chassis->foot_motor_R
                           .motor_measure->given_current);

    currentAverage =
        0.5f * (currentLeft + currentRight);

    /*
     * 一阶低通滤波。
     * 控制周期约2ms时，0.10系数可滤掉部分单周期尖峰。
     */
    ctx->currentFiltered =
        0.90f * ctx->currentFiltered +
        0.10f * currentAverage;

    legLength =
        0.5f *
        (chassis->chassis_posture_info
                    .chassis_posture_L.leg_length +
         chassis->chassis_posture_info
                    .chassis_posture_R.leg_length);

    /*
     * 当前工程右腿局部角方向与左腿相反，
     * 所以使用 left - right 统一左右腿方向。
     */
    localLegAngle =
        0.5f *
        (chassis->chassis_posture_info
                    .chassis_posture_L.leg_angle_local -
         chassis->chassis_posture_info
                    .chassis_posture_R.leg_angle_local);

    bothGround =
        chassis->flag_info.suspend_flag_L == ON_GROUND &&
        chassis->flag_info.suspend_flag_R == ON_GROUND;

    postureReady =
        fabsf(chassis->chassis_posture_info.pitch_angle)
            < CLIMB_READY_PITCH_RAD &&
        fabsf(chassis->chassis_posture_info.roll_angle)
            < CLIMB_READY_ROLL_RAD;

    legsExtended =
        fabsf(chassis->chassis_posture_info
                   .chassis_posture_L.leg_length -
               CLIMB_HIGH_LENGTH_M) < CLIMB_LENGTH_ERROR_M &&
        fabsf(chassis->chassis_posture_info
                   .chassis_posture_R.leg_length -
               CLIMB_HIGH_LENGTH_M) < CLIMB_LENGTH_ERROR_M;

    legsPulledUp =
        fabsf(chassis->chassis_posture_info
                   .chassis_posture_L.leg_length -
               CLIMB_PULL_LENGTH_M) < CLIMB_LENGTH_ERROR_M &&
        fabsf(chassis->chassis_posture_info
                   .chassis_posture_R.leg_length -
               CLIMB_PULL_LENGTH_M) < CLIMB_LENGTH_ERROR_M;

    legsAtNormalHeight =
        fabsf(chassis->chassis_posture_info
                   .chassis_posture_L.leg_length -
               ctx->normalLegLength) < CLIMB_LENGTH_ERROR_M &&
        fabsf(chassis->chassis_posture_info
                   .chassis_posture_R.leg_length -
               ctx->normalLegLength) < CLIMB_LENGTH_ERROR_M;

    /*
     * 碰撞立面：
     * 操作手正在给前进指令；
     * 两侧轮速都明显下降；
     * 两侧轮毂电流都升高。
     */
    hitRiser =
        requestedSpeed > CLIMB_FORWARD_COMMAND_MIN &&
        fabsf(chassis->foot_motor_L.speed)
            < CLIMB_STALL_SPEED_MPS &&
        fabsf(chassis->foot_motor_R.speed)
            < CLIMB_STALL_SPEED_MPS &&
        currentLeft > CLIMB_HIT_CURRENT_RAW &&
        currentRight > CLIMB_HIT_CURRENT_RAW;

    /*
     * 顶部判断采用投票，防止单个传感器误触发。
     */
    {
        bool speedRecovered;
        bool currentReleased;
        bool legAngleRecovered;
        bool pitchRecovered;
        uint8_t votes = 0u;

        speedRecovered =
            wheelSpeed > CLIMB_RECOVER_SPEED_MPS;

        currentReleased =
            ctx->currentFiltered <
            CLIMB_RELEASE_CURRENT_RAW;

        legAngleRecovered =
            fabsf(localLegAngle -
                   ctx->approachLegAngle)
                < CLIMB_LEG_ANGLE_ERROR_RAD;

        pitchRecovered =
            fabsf(chassis->chassis_posture_info.pitch_angle)
                < CLIMB_READY_PITCH_RAD &&
            fabsf(chassis->chassis_posture_info.pitch_gyro)
                < CLIMB_PITCH_GYRO_STABLE;

        votes += speedRecovered ? 1u : 0u;
        votes += currentReleased ? 1u : 0u;
        votes += legAngleRecovered ? 1u : 0u;
        votes += pitchRecovered ? 1u : 0u;
        votes += bothGround ? 1u : 0u;

        topDetected =
            ctx->riserDetected &&
            speedRecovered &&
            (currentReleased || legAngleRecovered) &&
            votes >= 4u;
    }

    /*
     * topDetected 已经通过轮速、电流、腿角和姿态投票确认跨过棱边。
     * 这里不要求双轮已经接地，否则轮子尚未被收腿拉上台阶时，
     * 状态机会一直卡在 TOP_CONFIRM。
     */
    bodyOnStepStable =
        ctx->riserDetected &&
        postureReady &&
        fabsf(chassis->chassis_posture_info.pitch_gyro)
            < CLIMB_PITCH_GYRO_STABLE;

    standingStable =
        legsAtNormalHeight &&
        bothGround &&
        postureReady &&
        fabsf(chassis->chassis_posture_info.pitch_gyro)
            < CLIMB_PITCH_GYRO_STABLE;

    /*
     * 攀爬过程中的公共安全判断。
     */
    if (ctx->state != CLIMB_IDLE &&
        ctx->state != CLIMB_FINISHED &&
        ctx->state != CLIMB_ABORT)
    {
        bool sensorInvalid =
            !climbValueValid(
                chassis->chassis_posture_info.pitch_angle) ||
            !climbValueValid(
                chassis->chassis_posture_info.roll_angle) ||
            !climbValueValid(legLength);

        bool dangerousPosture =
            fabsf(chassis->chassis_posture_info.pitch_angle)
                > CLIMB_ABORT_PITCH_RAD ||
            fabsf(chassis->chassis_posture_info.roll_angle)
                > CLIMB_ABORT_ROLL_RAD;

        bool modeConflict =
            chassis->mode.chassis_mode != ENABLE_CHASSIS ||
            chassis->mode.sport_mode == JUMPING_MODE ||
            chassis->chassis_data_->recovery_flag == 1u;

        /*
         * climb_flag is an event trigger, not a hold-to-run command. Once its
         * rising edge starts the state machine, returning the flag to zero
         * must not abort the climb. Communication freshness is still checked.
         */
        if (sensorInvalid || dangerousPosture || modeConflict)
        {
            climbEnterAbort(ctx, chassis, now);
        }
    }

    switch (ctx->state)
    {
        case CLIMB_IDLE:

            /*
             * IDLE直接返回，不修改正常控制逻辑。
             */
            if (!commandPressed)
                return;

            ctx->normalLegLength =
                chassis->chassis_posture_info.ideal_high;

            ctx->normalLegLength =
                climbClamp(ctx->normalLegLength,
                           0.22f,
                           CLIMB_HIGH_LENGTH_M);

            ctx->riserDetected = false;

            climbEnterState(ctx, CLIMB_READY, now);
            break;

        case CLIMB_READY:
        {
            bool ready =
                bothGround &&
                postureReady &&
                fabsf(wheelSpeed) < 0.10f &&
                chassis->mode.chassis_mode == ENABLE_CHASSIS &&
                chassis->mode.sport_mode != JUMPING_MODE;

            /*
             * 状态检查期间不允许机器人移动。
             */
            chassis->chassis_posture_info.foot_speed_set = 0.0f;

            if (climbConditionHold(ctx,
                                   ready,
                                   now,
                                   CLIMB_READY_STABLE_MS))
            {
                climbEnterState(ctx, CLIMB_EXTEND, now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_READY_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_EXTEND:

            chassis->chassis_posture_info.foot_speed_set = 0.0f;
            climbSetLegLength(chassis, CLIMB_HIGH_LENGTH_M);

            if (climbConditionHold(ctx,
                                   legsExtended && postureReady,
                                   now,
                                   CLIMB_EXTEND_STABLE_MS))
            {
                ctx->approachLegAngle = localLegAngle;
                climbEnterState(ctx, CLIMB_APPROACH, now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_EXTEND_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;

        case CLIMB_APPROACH:

            climbSetLegLength(chassis, CLIMB_HIGH_LENGTH_M);

            /*
             * 速度来自操作手，只允许低速向前。
             * 操作手不推摇杆时，速度设定就是0。
             */
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed,
                           0.0f,
                           CLIMB_APPROACH_SPEED_MAX);

            if (climbConditionHold(ctx,
                                   hitRiser,
                                   now,
                                   CLIMB_HIT_CONFIRM_MS))
            {
                ctx->riserDetected = true;
                climbEnterState(ctx, CLIMB_CROSS, now);
            }

            break;

        case CLIMB_CROSS:

            climbSetLegLength(chassis, CLIMB_HIGH_LENGTH_M);

            /*
             * 已碰到台阶后进一步限制速度，
             * 但仍然由操作手决定是否向前。
             */
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed,
                           0.0f,
                           CLIMB_CROSS_SPEED_MAX);

            if (climbConditionHold(ctx,
                                   topDetected,
                                   now,
                                   CLIMB_TOP_DETECT_MS))
            {
                climbEnterState(ctx,
                                CLIMB_TOP_CONFIRM,
                                now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_CROSS_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;

        case CLIMB_TOP_CONFIRM:

            /*
             * 检测到顶部后，模块暂时接管速度并停车。
             */
            chassis->chassis_posture_info.foot_speed_set = 0.0f;
            climbSetLegLength(chassis, CLIMB_HIGH_LENGTH_M);

            if (climbConditionHold(ctx,
                                   bodyOnStepStable,
                                   now,
                                   CLIMB_TOP_STABLE_MS))
            {
                ctx->motionStartLength = legLength;

                climbEnterState(ctx,
                                CLIMB_PULL_UP,
                                now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_TOP_CONFIRM_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;

        case CLIMB_PULL_UP:
        {
            fp32 lengthSet;

            chassis->chassis_posture_info.foot_speed_set = 0.0f;

            /*
             * 机身已经搭到台阶上后收腿，把仍在棱边下方的轮子
             * 拉到台阶顶面。整个阶段禁止轮子继续主动前进。
             */
            lengthSet =
                climbSmoothLength(ctx->motionStartLength,
                                  CLIMB_PULL_LENGTH_M,
                                  now,
                                  ctx->stateTick,
                                  CLIMB_PULL_TIME_MS);

            climbSetLegLength(chassis, lengthSet);

            if (climbConditionHold(ctx,
                                   legsPulledUp &&
                                   fabsf(chassis->chassis_posture_info
                                                    .pitch_gyro) <
                                       CLIMB_PITCH_GYRO_STABLE,
                                   now,
                                   CLIMB_PULL_STABLE_MS))
            {
                climbEnterState(ctx,
                                CLIMB_WHEEL_CONFIRM,
                                now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_PULL_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_WHEEL_CONFIRM:

            chassis->chassis_posture_info.foot_speed_set = 0.0f;
            climbSetLegLength(chassis, CLIMB_PULL_LENGTH_M);

            /*
             * 保持短腿等待左右轮真正落到台阶顶面。
             * 未确认接地前禁止重新伸腿，避免轮子仍卡在立面时撑车。
             */
            if (climbConditionHold(ctx,
                                   bothGround &&
                                   fabsf(chassis->chassis_posture_info
                                                    .pitch_gyro) <
                                       CLIMB_PITCH_GYRO_STABLE,
                                   now,
                                   CLIMB_WHEEL_CONTACT_STABLE_MS))
            {
                ctx->motionStartLength = legLength;

                climbEnterState(ctx,
                                CLIMB_STAND_UP,
                                now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_WHEEL_CONTACT_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;

        case CLIMB_STAND_UP:
        {
            fp32 lengthSet;

            chassis->chassis_posture_info.foot_speed_set = 0.0f;

            /*
             * 轮子被拉过棱边后，平滑伸回攀爬开始前保存的正常高度。
             * 只有实际腿长到位、双轮接地且姿态稳定才允许完成。
             */
            lengthSet =
                climbSmoothLength(ctx->motionStartLength,
                                  ctx->normalLegLength,
                                  now,
                                  ctx->stateTick,
                                  CLIMB_STAND_TIME_MS);

            climbSetLegLength(chassis, lengthSet);

            if (climbConditionHold(ctx,
                                   standingStable,
                                   now,
                                   CLIMB_STAND_STABLE_MS))
            {
                climbEnterState(ctx,
                                CLIMB_FINISHED,
                                now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_STAND_TIMEOUT_MS))
            {
                climbEnterAbort(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_FINISHED:

            chassis->chassis_posture_info.foot_speed_set = 0.0f;
            climbSetLegLength(chassis, ctx->normalLegLength);

            /*
             * Finish automatically after the chassis is stable. lastCommand
             * remains unchanged, so a command held high cannot retrigger.
             */
            if (climbConditionHold(ctx,
                                   standingStable,
                                   now,
                                   CLIMB_FINISHED_STABLE_MS))
            {
                chassis->chassis_posture_info
                    .position_lock_state = 0u;

                climbReset(ctx);
            }

            break;

        case CLIMB_ABORT:

            /*
             * 异常时停车并保持当前腿长。
             * 不自动收腿，避免机器人还顶在台阶上时倾倒。
             */
            chassis->chassis_posture_info.foot_speed_set = 0.0f;
            climbSetLegLength(chassis, ctx->abortHoldLength);

            /*
             * Abort also exits automatically after the robot is safe and
             * stable; it does not wait for climb_flag to change.
             */
            if (climbConditionHold(ctx,
                                   postureReady,
                                   now,
                                   CLIMB_ABORT_RESET_MS))
            {
                chassis->chassis_posture_info
                    .position_lock_state = 0u;

                climbReset(ctx);
            }

            break;

        default:

            climbEnterAbort(ctx, chassis, now);
            break;
    }

    /*
     * IDLE之外都关闭普通位置锁。
     */
    if (ctx->state != CLIMB_IDLE)
        climbReleasePositionLock(chassis);
}
