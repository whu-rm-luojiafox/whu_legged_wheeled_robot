#include "chassis_task.h"
#include "chassis_config.h"
#include "chassis_observer.h"
#include "chassis_mode.h"
#include "chassis_motion.h"
#include "chassis_control.h"
#include "chassis_climb.h"
#include "chassis_recovery.h"
#include "chassis_power.h"
#include "motor_cmd.h"
#include "uart_receive.h"
#include "FreeRTOS.h"
#include "task.h"

chassis_move_t chassis_move;
float chassis_control_dt_s __attribute__((section(".bss.chassis_control_dt_s")));
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

static void chassis_init(chassis_move_t *chassis)
{
    vTaskDelay(3000);
    for (uint8_t id = 1; id <= 4; ++id)
    {
        CAN_dm_enable(id);
        vTaskDelay(2);
    }
    chassis->chassis_data_ = get_Uart_Chassisdata_point();
    chassisModeInit(chassis);
    chassisControlInit(chassis);
    chassisObserverInit(chassis);
    // CAN_dm_save_0_point(4);
}

/* 调度顺序固定；每个模块只处理本阶段的数据。 */
void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);
    TickType_t last_tick = xTaskGetTickCount();

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        chassis_control_dt_s = (float)(TickType_t)(now - last_tick) / configTICK_RATE_HZ;
        last_tick = now;

        chassisObserverUpdate(&chassis_move, chassis_control_dt_s);
        chassisModeUpdate(&chassis_move);
        chassisMotionUpdate(&chassis_move, chassis_control_dt_s);
        chassisModeSetTargets(&chassis_move);
        chassisClimbProcess(&chassis_move);

        chassisControlCalculate(&chassis_move, chassis_control_dt_s);
        chassisControlMap(&chassis_move);
        if (chassisRecoveryProcess(&chassis_move)) chassisControlMap(&chassis_move);

        chassis_power_limit(&chassis_move);
        chassisMotorSend(&chassis_move);
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
