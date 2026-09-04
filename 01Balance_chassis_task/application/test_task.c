/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "vofa.h"
#include "chassis_task.h"
#include "can_service.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "ui.h"
#include "referee.h"
static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;

extern __IO uint32_t uwTick;
uint32_t sendTick =0;

/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();
	
    while(1)
    {
//	ui_self_id = (int)get_robot_id();
//	ui_init_default_Ungroup();
//	ui_default_Ungroup_NewFloat->number=chassis_move.chassis_relative_angle*1000;
//	_ui_update_default_Ungroup_5();
////	if(chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
////	{
////		ui_default_Ungroup_NewRound->color = 1;
////	}
////    else if(chassis_move.chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
////	{
////		ui_default_Ungroup_NewRound->color=5;
////	}
////	 else if(chassis_move.chassis_mode == CHASSIS_VECTOR_RAW)
////	{
////		ui_default_Ungroup_NewRound->color=7;
////	}
//	switch (chassis_move.chassis_data_->shoot_mode & 0x01) {
//		case 0x01:  // 当 shoot_mode 的最低位为 1 时
//			ui_default_Ungroup_NewRound2->color = 5;
//			_ui_update_default_Ungroup_3();
//			break;
//		case 0x00:  // 当 shoot_mode 的最低位为 0 时
//			ui_default_Ungroup_NewRound2->color= 7;
//			_ui_update_default_Ungroup_3();
//			break;
//		default:
//			// 默认情况，可以根据需要处理
//			break;
//}
//	switch (chassis_move.chassis_data_->shoot_mode & 0x04) {
//		case 0x04:  // 当 shoot_mode 的第三位为 1 时
//			ui_default_Ungroup_NewRound1->color = 5;
//			_ui_update_default_Ungroup_3();
//        break;
//		case 0x00:  // 当 shoot_mode 的第三位为 0 时
//			ui_default_Ungroup_NewRound1->color = 7;
//			_ui_update_default_Ungroup_3();
//			break;
//		default:
//        // 默认情况，可以根据需要处理
//			break;
//}
//	
//	_ui_update_default_Ungroup_5();
//	ui_update_default_Ungroup();
//	
//		
////			sendfloatdata(data,2);
			osDelay(1);
    }

}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          使得蜂鸣器响
  * @param[in]      num:响声次数
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


