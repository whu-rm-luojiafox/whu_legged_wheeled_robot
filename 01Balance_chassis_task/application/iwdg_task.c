#include "iwdg_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "bsp_buzzer.h"
void iwdg_task(void const *pvParamter)
{
    while(1)
    {
        if(chassis_move.chassis_data_->reset_flag && (chassis_move.mode.chassis_mode != ENABLE_CHASSIS))
        {
            uint32_t tick;
            tick = xTaskGetTickCount();
            buzzer_on(25,4000);
            osDelay(100);
            buzzer_off();
            buzzer_on(25,4000);
            osDelay(100);
            buzzer_off();        
            buzzer_on(25,8399);
            osDelay(100);
            buzzer_off();  
            while((xTaskGetTickCount() - tick) < pdMS_TO_TICKS(1000))
            {
                if(chassis_move.chassis_posture_info.pitch_angle>0)
                {
                    CAN_LK_Boradcast_Control(-1000,1000,0,0);
                }
                else
                {
                    CAN_LK_Boradcast_Control(1000,-1000,0,0);
                } 
            }
             CAN_LK_Boradcast_Control(0,0,0,0);
            // __disable_irq();              // 关闭全局中断
            // HAL_NVIC_SystemReset();       // 执行系统复位        
        }
        osDelay(1000);
    }
}