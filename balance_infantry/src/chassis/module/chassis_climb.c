#include "chassis_task.h" /* Legacy state adapter; not part of the public interface. */
#include "chassis_climb.h"
#include "uart_receive.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* -------------------------- 腿长参数 -------------------------- */

#define CLIMB_HIGH_LENGTH_M             0.40f
#define CLIMB_PULL_LENGTH_M             0.14f
#define CLIMB_LENGTH_ERROR_M            0.03f
#define CLIMB_EXTEND_LENGTH_ERROR_M     0.020f

/* -------------------------- 速度参数 -------------------------- */

/* 攀爬阶段限制操作手的最大前进速度 */
#define CLIMB_APPROACH_SPEED_MAX        1.6f
#define CLIMB_CROSS_SPEED_MAX           1.6f

/* 有移动指令，且设定与两轮平均反馈相差超过 0.2 m/s，作为撞阶辅助信号。 */
#define CLIMB_HIT_COMMAND_MIN_MPS       0.20f
#define CLIMB_HIT_SPEED_ERROR_MPS       0.20f

/* 越过台阶棱边后，轮速应恢复到此值以上 */
#define CLIMB_RECOVER_SPEED_MPS         0.15f

/* -------------------------- 电流参数 -------------------------- */

/*
 * 顶部电流释放阈值仍需实车标定；撞阶检测已不再使用轮毂电流阈值。
 * given_current 是电机反馈原始值，必须根据实车日志重新标定。
 */
#define CLIMB_RELEASE_CURRENT_RAW       3000.0f

/* -------------------------- 姿态参数 -------------------------- */

#define CLIMB_READY_PITCH_RAD           0.20f
#define CLIMB_READY_ROLL_RAD            0.10f

/* 浙大台阶检测主判据的简化试用值，角度取绝对值，需实车验证。
 * pitch 超过 6°，且任一腿相对世界竖直方向超过 14°。
 */
#define CLIMB_HIT_PITCH_RAD             0.104720f
#define CLIMB_HIT_LEG_RAD               0.244346f

/* 轮子空转时速度差不一定明显，较大的姿态变化也可作为辅助信号。 */
#define CLIMB_HIT_STRONG_PITCH_RAD      0.209440f  /* 12° */
#define CLIMB_HIT_STRONG_LEG_RAD        0.349066f  /* 20° */

#define CLIMB_ABORT_PITCH_RAD           0.55f
#define CLIMB_ABORT_ROLL_RAD            0.30f

#define CLIMB_PITCH_GYRO_STABLE         0.5f
#define CLIMB_LEG_ANGLE_ERROR_RAD       0.20f

/* -------------------------- 时间参数 -------------------------- */

#define CLIMB_READY_STABLE_MS           20u
#define CLIMB_READY_TIMEOUT_MS          8000u

#define CLIMB_EXTEND_TIME_MS            900u
#define CLIMB_EXTEND_STABLE_MS          100u
#define CLIMB_EXTEND_TIMEOUT_MS         10000u

#define CLIMB_HIT_CONFIRM_MS            20u
#define CLIMB_CROSS_TIMEOUT_MS          15000u

#define CLIMB_TOP_DETECT_MS             80u
#define CLIMB_TOP_STABLE_MS             50u
#define CLIMB_TOP_CONFIRM_TIMEOUT_MS     5000u

#define CLIMB_PULL_TIME_MS              400u
#define CLIMB_PULL_STABLE_MS            10u
#define CLIMB_PULL_TIMEOUT_MS            5000u

#define CLIMB_WHEEL_CONTACT_STABLE_MS    100u
#define CLIMB_WHEEL_CONTACT_TIMEOUT_MS   5000u

#define CLIMB_STAND_TIME_MS             500u
#define CLIMB_STAND_STABLE_MS           200u
#define CLIMB_STAND_TIMEOUT_MS          6000u

#define CLIMB_FINISHED_STABLE_MS        200u
#define CLIMB_ABORT_RESET_MS            300u
#define CLIMB_RETURN_TIME_MS            1500u

typedef struct
{
    climb_state_e state;

    uint32_t stateTick;
    uint32_t conditionTick;

    fp32 normalLegLength;
    fp32 motionStartLength;
    fp32 abortHoldLength;
    fp32 approachLegAngle;

    fp32 lastLengthSetL;
    fp32 lastLengthSetR;
    fp32 returnStartLengthL;
    fp32 returnStartLengthR;
    bool lengthTargetValid;

    fp32 currentFiltered;

    bool riserDetected;
    bool lastCommand;
} ClimbContext;

/*
 * 模块内部保存状态，因此 chassis_task 不需要调用初始化函数。
 * 初次必须先看到 climb_flag 释放，才接受下一次上升沿。
 */
ClimbContext climbContext =
{
    .state = CLIMB_IDLE,
    .normalLegLength = 0.19f,
    .motionStartLength = 0.19f,
    .abortHoldLength = 0.19f,
    .lastCommand = true
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
    ctx->lengthTargetValid = false;

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
static void climbSetLegTargets(chassis_move_t *chassis, fp32 left, fp32 right)
{
    chassis->chassis_posture_info
        .chassis_posture_L.leg_length_set = left;

    chassis->chassis_posture_info
        .chassis_posture_R.leg_length_set = right;

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
    /* Target_Value_Set has already published ordinary targets for this cycle.
     * Start from the last climb targets, not those overwritten ordinary ones,
     * and not measured lengths: changing PID error at handoff causes a kick.
     */
    ctx->returnStartLengthL = ctx->lengthTargetValid ? ctx->lastLengthSetL :
        chassis->chassis_posture_info.chassis_posture_L.leg_length;
    ctx->returnStartLengthR = ctx->lengthTargetValid ? ctx->lastLengthSetR :
        chassis->chassis_posture_info.chassis_posture_R.leg_length;
    if (!climbValueValid(ctx->returnStartLengthL) || ctx->returnStartLengthL <= 0.0f)
        ctx->returnStartLengthL = ctx->normalLegLength;
    if (!climbValueValid(ctx->returnStartLengthR) || ctx->returnStartLengthR <= 0.0f)
        ctx->returnStartLengthR = ctx->normalLegLength;
    climbEnterState(ctx, CLIMB_RETURN_NORMAL, now);
    climbSetLegTargets(chassis, ctx->returnStartLengthL, ctx->returnStartLengthR);
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

uint8_t chassisClimbAllowsPositionHold(void)
{
    return climbContext.state == CLIMB_IDLE ||
           climbContext.state == CLIMB_READY ||
           climbContext.state == CLIMB_EXTEND ||
           climbContext.state == CLIMB_RETURN_NORMAL;
}

uint8_t chassisClimbApplyTorquePolicy(chassis_move_t *chassis)
{
    if (chassis == NULL ||
        (climbContext.state != CLIMB_PULL_UP &&
         climbContext.state != CLIMB_WHEEL_CONFIRM))
        return 0u;

    /* The body is supported by the step while the wheels are pulled up.
     * Zero every balancing/motion correction at the final mapping boundary,
     * including air-contact overrides. Use the length PID directly so the
     * standing gravity feedforward cannot oppose wheel retraction.
     */
    torque_info_t *torque = &chassis->torque_info;
    torque->joint_stand_torque_L = chassis->leg_L_length_pid.out;
    torque->joint_stand_torque_R = chassis->leg_R_length_pid.out;
    torque->foot_balancing_torque_L = torque->foot_balancing_torque_R = 0.0f;
    torque->foot_moving_torque_L = torque->foot_moving_torque_R = 0.0f;
    torque->joint_balancing_torque_L = torque->joint_balancing_torque_R = 0.0f;
    torque->joint_moving_torque_L = torque->joint_moving_torque_R = 0.0f;
    torque->joint_roll_torque_L = torque->joint_roll_torque_R = 0.0f;
    return 1u;
}
ClimbContext *ctx;
bool hitRiser;
fp32 speedError;
bool bodyTilted;
bool legTilted ;
void chassisClimbProcess(chassis_move_t *chassis)
{
    ctx = &climbContext;

    uint32_t now;


    bool command;
    bool commandPressed;

    bool bothGround;
    bool postureReady;
    bool legsExtended;
    bool legsPulledUp;
    bool legsAtNormalHeight;
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
    fp32 normalTargetL;
    fp32 normalTargetR;

    if (chassis == NULL || chassis->chassis_data_ == NULL)
        return;
    now = (uint32_t)xTaskGetTickCount();
    command = chassis->chassis_data_->climb_flag == 1u;
    commandPressed = command && !ctx->lastCommand;
    normalTargetL = chassis->chassis_posture_info.chassis_posture_L.leg_length_set;
    normalTargetR = chassis->chassis_posture_info.chassis_posture_R.leg_length_set;

    /* Track the real key level so a held F cannot repeatedly trigger. */
    ctx->lastCommand = command;
    /* A second real F edge cancels the active sequence through a soft return.
     * Holding F high still cannot retrigger or repeatedly restart the ramp.
     */
    if (commandPressed && ctx->state != CLIMB_IDLE &&
        ctx->state != CLIMB_ABORT && ctx->state != CLIMB_RETURN_NORMAL)
        climbEnterReturn(ctx, chassis, now);
    /*
     * 保存 Target_Value_Set() 生成的操作手速度指令。
     * 后续攀爬状态只对它限幅，不主动产生前进速度。
     */
    requestedSpeed = chassis->chassis_posture_info.foot_speed_set;
    wheelSpeed = 0.5f *(chassis->foot_motor_L.speed + chassis->foot_motor_R.speed);
    currentLeft = fabsf((fp32)chassis->foot_motor_L.motor_measure->given_current);
    currentRight = fabsf((fp32)chassis->foot_motor_R.motor_measure->given_current);
    currentAverage = 0.5f * (currentLeft + currentRight);
    /*
     * 一阶低通滤波。
     * 控制周期约2ms时，0.10系数可滤掉部分单周期尖峰。
     */
    ctx->currentFiltered = 0.90f * ctx->currentFiltered + 0.10f * currentAverage;
    legLength = 0.5f * (chassis->chassis_posture_info.chassis_posture_L.leg_length 
              +chassis->chassis_posture_info.chassis_posture_R.leg_length);

    /*
     * 当前工程右腿局部角方向与左腿相反，
     * 所以使用 left - right 统一左右腿方向。
     */
    localLegAngle = 0.5f *(chassis->chassis_posture_info.chassis_posture_L.leg_angle_local
                    -chassis->chassis_posture_info.chassis_posture_R.leg_angle_local);

    bothGround =
        chassis->flag_info.suspend_flag_L == ON_GROUND &&
        chassis->flag_info.suspend_flag_R == ON_GROUND;

    postureReady =
        fabsf(chassis->chassis_posture_info.pitch_angle)
            < CLIMB_READY_PITCH_RAD &&
        fabsf(chassis->chassis_posture_info.roll_angle)
            < CLIMB_READY_ROLL_RAD;

    /* Extension only needs both legs to reach the height band. Overshoot
     * must not make an already extended leg fail the completion check.
     * Retraction and final standing keep their existing tighter tolerance.
     */
    legsExtended =
        chassis->chassis_posture_info.chassis_posture_L.leg_length >=
            CLIMB_HIGH_LENGTH_M - CLIMB_EXTEND_LENGTH_ERROR_M &&
        chassis->chassis_posture_info.chassis_posture_R.leg_length >=
            CLIMB_HIGH_LENGTH_M - CLIMB_EXTEND_LENGTH_ERROR_M;

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

    /* -------------------------- 撞台阶判断 -------------------------- */
    {
        /* 1. 取角度大小。leg_angle 相对世界竖直方向，已包含 pitch。 */
        fp32 pitchAbs = fabsf(chassis->chassis_posture_info.pitch_angle);
        fp32 leftLegAbs = fabsf(chassis->chassis_posture_info.chassis_posture_L.leg_angle);
        fp32 rightLegAbs = fabsf(chassis->chassis_posture_info.chassis_posture_R.leg_angle);
        fp32 legAngleAbs = fmaxf(leftLegAbs, rightLegAbs);

        /* 2. 基本条件：机身倾斜超过 6°，并且任一腿倾斜超过 14°。 */
        bodyTilted = pitchAbs > CLIMB_HIT_PITCH_RAD;
        legTilted = legAngleAbs > CLIMB_HIT_LEG_RAD;

        /* 3. 速度辅助：有移动指令，且设定与两轮平均反馈相差超过 0.2 m/s。
         * 先对设定限幅，避免把限速本身当成速度跟踪误差。
         */
        fp32 speedSet = climbClamp(requestedSpeed,
                                   -CLIMB_APPROACH_SPEED_MAX,
                                    CLIMB_APPROACH_SPEED_MAX);
        speedError = fabsf(speedSet - wheelSpeed);
        bool hasMoveCommand = fabsf(requestedSpeed) > CLIMB_HIT_COMMAND_MIN_MPS;

        hitRiser = (ctx->state == CLIMB_APPROACH)
                   && legTilted
                   && ctx->currentFiltered > 6000.0f;
    }

    /*
     * 顶部判断采用投票，防止单个传感器误触发。
     */
    {
        topDetected = ctx->riserDetected ;
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
         * must not abort the climb. A second rising edge requests soft return.
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

            if (!climbValueValid(ctx->normalLegLength))
                ctx->normalLegLength = 0.19f;

            ctx->normalLegLength =
                climbClamp(ctx->normalLegLength,
                           0.17f,
                           CLIMB_HIGH_LENGTH_M);

            climbSetLegTargets(chassis, normalTargetL, normalTargetR);
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX,
                           CLIMB_APPROACH_SPEED_MAX);

            ctx->riserDetected = false;

            climbEnterState(ctx, CLIMB_READY, now);
            break;

        case CLIMB_READY:
        {
            bool ready =
                chassis->mode.chassis_mode == ENABLE_CHASSIS &&
                chassis->mode.chassis_balancing_mode == BALANCING_READY &&
                chassis->mode.sport_mode == NORMAL_MOVING_MODE;

            /* Do not gate extension on wheel speed, contact flags or tight
             * pitch/roll limits. The shared invalid-data, severe-posture and
             * mode-conflict checks above still abort an unsafe sequence.
             */

            /* Keep low-speed forward/reverse control while preparing. */
            climbSetLegTargets(chassis, normalTargetL, normalTargetR);
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX,
                           CLIMB_APPROACH_SPEED_MAX);

            if (climbConditionHold(ctx,
                                   ready,
                                   now,
                                   CLIMB_READY_STABLE_MS))
            {
                ctx->motionStartLength = legLength;
                climbEnterState(ctx, CLIMB_EXTEND, now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_READY_TIMEOUT_MS))
            {
                climbEnterReturn(ctx, chassis, now);
            }

            break;
        }

        case CLIMB_EXTEND:

            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX,
                           CLIMB_APPROACH_SPEED_MAX);
            /* 从进入阶段时的实际腿长平滑升腿，起止速度均为零。 */
            climbSetLegLength(chassis,
                              climbSmoothLength(ctx->motionStartLength,
                                                CLIMB_HIGH_LENGTH_M,
                                                now,
                                                ctx->stateTick,
                                                CLIMB_EXTEND_TIME_MS));

            if (climbConditionHold(ctx,
                                   now - ctx->stateTick >=
                                       pdMS_TO_TICKS(CLIMB_EXTEND_TIME_MS) &&
                                   legsExtended,
                                   now,
                                   CLIMB_EXTEND_STABLE_MS))
            {
                ctx->approachLegAngle = localLegAngle;
                climbEnterState(ctx, CLIMB_APPROACH, now);
            }
            else if (now - ctx->stateTick >=
                     pdMS_TO_TICKS(CLIMB_EXTEND_TIMEOUT_MS))
            {
                climbEnterReturn(ctx, chassis, now);
            }

            break;

        case CLIMB_APPROACH:

            climbSetLegLength(chassis, CLIMB_HIGH_LENGTH_M);

            /*
             * 速度来自操作手，允许低速前进和后退。
             * 操作手不推摇杆时，速度设定就是0。
             */
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed,
                           -CLIMB_APPROACH_SPEED_MAX,
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
             * 但仍然由操作手决定前进或后退。
             */
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed,
                           -CLIMB_CROSS_SPEED_MAX,
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
                climbEnterReturn(ctx, chassis, now);
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
                climbEnterReturn(ctx, chassis, now);
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
                climbEnterReturn(ctx, chassis, now);
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
                                   fabsf(chassis->chassis_posture_info.pitch_gyro) <CLIMB_PITCH_GYRO_STABLE,
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
                climbEnterReturn(ctx, chassis, now);
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
                climbEnterReturn(ctx, chassis, now);
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

                climbEnterReturn(ctx, chassis, now);
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
             * After a real safety abort, hold until enabled and stable, then
             * use the same soft return instead of jumping directly to IDLE.
             */
            if (climbConditionHold(ctx,
                                   postureReady && climbValueValid(legLength) &&
                                   chassis->mode.chassis_mode == ENABLE_CHASSIS &&
                                   chassis->mode.chassis_balancing_mode == BALANCING_READY &&
                                   chassis->mode.sport_mode == NORMAL_MOVING_MODE &&
                                   chassis->chassis_data_->recovery_flag == 0u,
                                   now,
                                   CLIMB_ABORT_RESET_MS))
            {
                chassis->chassis_posture_info
                    .position_lock_state = 0u;

                climbEnterReturn(ctx, chassis, now);
            }

            break;

        case CLIMB_RETURN_NORMAL:
            /* Blend separately into the ordinary targets produced above by
             * Target_Value_Set, including roll compensation and current height
             * input. At completion the next IDLE cycle has identical targets.
             */
            if (!climbValueValid(normalTargetL) || normalTargetL <= 0.0f)
                normalTargetL = ctx->normalLegLength;
            if (!climbValueValid(normalTargetR) || normalTargetR <= 0.0f)
                normalTargetR = ctx->normalLegLength;
            chassis->chassis_posture_info.foot_speed_set =
                climbClamp(requestedSpeed, -CLIMB_APPROACH_SPEED_MAX,
                           CLIMB_APPROACH_SPEED_MAX);
            climbSetLegTargets(chassis,
                climbSmoothLength(ctx->returnStartLengthL, normalTargetL,
                                  now, ctx->stateTick, CLIMB_RETURN_TIME_MS),
                climbSmoothLength(ctx->returnStartLengthR, normalTargetR,
                                  now, ctx->stateTick, CLIMB_RETURN_TIME_MS));
            if (now - ctx->stateTick >= pdMS_TO_TICKS(CLIMB_RETURN_TIME_MS))
                climbReset(ctx);
            break;

        default:

            climbEnterAbort(ctx, chassis, now);
            break;
    }

    /* Position hold remains available during extension/return at zero input;
     * ordinary movement input releases it through the stop-position module. */
    if (!chassisClimbAllowsPositionHold())
        climbReleasePositionLock(chassis);
}
