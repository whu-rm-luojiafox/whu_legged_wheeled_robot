#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H

#include "chassis_fwd.h"

typedef enum {
    REC_IDLE,       /* 正常控制 */
    REC_WAIT,       /* 倒地/动作超时，卸力等待恢复键 */
    REC_SWING,      /* 关节驱动器速度环摆腿，按对地角判定到位 */
    REC_RETRACT,    /* 保持后摆角缩腿，交接只判断轨迹和腿长 */
    REC_BLEND      /* 从缩腿控制平滑交给 LQR，同时腿角回站立参考 */
} chassis_recovery_stage_e;

/* 调度：Update 规划目标 -> ControlCalculate -> Apply 覆盖/混合 -> Map 一次。 */
void chassisRecoveryUpdate(chassis_move_t *chassis, float dt_s);
void chassisRecoveryApply(chassis_move_t *chassis);
uint8_t chassisRecoveryIsActive(void);
float chassisRecoveryBalanceScale(void);
float chassisRecoveryJointTorqueLimit(void);

#endif
