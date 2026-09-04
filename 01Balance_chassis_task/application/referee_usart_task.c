/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
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
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "uart_receive.h"
#include "ui.h"
#include "chassis_task.h"
#include "shoot.h"


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

uint32_t ui_fpsTick; // 刷新计数
uint32_t ui_period = 100;
uint32_t ui_initTick_1; // 重新初始化绘制计数（解决丢包导致静态图案丢失）
uint32_t ui_initTick_2;
uint32_t ui_initTick_3;
extern __IO uint32_t uwTick; // 系统时钟

//裁判系统
extern robot_status_t robot_state; //机器人状态
extern power_heat_data_t power_heat_data;  //机器人功率与发射热量
extern projectile_allowance_t bullet_remaining;  //剩余弹量
extern game_status_t game_state;   //比赛信息
extern int ui_self_id;  //机器人ID

//云台数据
extern uart_data_t uart_data;
//拨弹盘结构体数据
extern shoot_control_t shoot_control;

// 相机参数
#define HORIZONTAL_FOV     139.0f    // 水平视角
#define FOCAL_LENGTH_MM   12.0f      // 等效焦距 mm
#define CAMERA_HEIGHT      0.6f     // 相机安装高度（米）

// 像素焦距计算
// f_像素 = 图像宽度 / (2 * tan(视角/2))
#define FOCAL_LENGTH      (1920.0f / (2.0f * tanf(HORIZONTAL_FOV * 3.14159f / 360.0f)))  // ≈ 835 像素
#define CX                960.0f     // 光心X
#define CY                540.0f     // 光心Y

fp32 pitch_rad;
// ================== 动态起跳线计算 ==================
static int32_t calculate_jump_line_position(void)
{

    fp32 jump_distance = 0.8f;  // 0.6米，可根据速度调整
    

    pitch_rad = uart_data.receive_chassis_data.pitch_angle*PI/180.0f;
    fp32 cos_p = cosf(pitch_rad);
    fp32 sin_p = sinf(pitch_rad);
    
    fp32 world_x = jump_distance;
    fp32 world_z = 0.0f;  // 地面高度
    

    fp32 P_c_x = world_x;
    fp32 P_c_z = world_x * sin_p + (CAMERA_HEIGHT - world_z) * cos_p;
    fp32 P_c_y = -world_x * cos_p + (CAMERA_HEIGHT - world_z) * sin_p;
    

    fp32 u = CX + FOCAL_LENGTH * P_c_x / P_c_z;
    fp32 v = CY + FOCAL_LENGTH * P_c_y / P_c_z;
    

    if (v < 0) v = 0;
    if (v > 1080) v = 1080;

    
    return (int32_t)v;  // 返回线的高度像素值
}
// ==================================================

void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
		osDelay(10);

    ui_init_g_DynamicGroup();
    ui_init_g_StaticGraphicGroup();
    ui_init_g_StaticTextGroup();

    while (1)
    {
        ui_self_id =robot_state.robot_id;        
        referee_unpack_fifo_data();
        osDelay(10);
        //定时初始化防止图形未进行初始化
        if(uwTick - ui_initTick_1 >= 2*ui_period)
        {
          ui_initTick_1 =uwTick;
          ui_init_g_DynamicGroup();
        }
        if(uwTick - ui_initTick_2 >= 3*ui_period)
        {
          ui_initTick_2 =uwTick;
          ui_init_g_StaticGraphicGroup();        
        }
        if(uwTick - ui_initTick_3 >= 4*ui_period)
        {
          ui_initTick_3 =uwTick;
          ui_init_g_StaticTextGroup();
        }
        //设置按键触发重新初始化
        if(uart_data.receive_chassis_data.ui_init_flag==1)
        {
          ui_init_g_DynamicGroup();
          ui_init_g_StaticGraphicGroup();
          ui_init_g_StaticTextGroup();
          osDelay(ui_period);
        }
        //开始伟大的编程吧
        //自瞄和摩擦轮
        if(uart_data.receive_chassis_data.fric_flag==1)
        {
          ui_g_DynamicGroup_FricRound->color = UI_Color_Pink;
        }
        else
        {
          ui_g_DynamicGroup_FricRound->color = UI_Color_White;
        }
        if(uart_data.receive_chassis_data.auto_flag==1)
        {
          ui_g_DynamicGroup_AutoRound->color = UI_Color_Pink;
        }
        else
        {
          ui_g_DynamicGroup_AutoRound->color = UI_Color_White;
        }
        ui_g_DynamicGroup_DirectionArc->start_angle = 30-loop_fp32_constrain(chassis_move.chassis_posture_info.yaw_angle_total,-PI,PI)*180/PI;
        ui_g_DynamicGroup_DirectionArc->end_angle = 330-loop_fp32_constrain(chassis_move.chassis_posture_info.yaw_angle_total,-PI,PI)*180/PI;
        if(chassis_move.chassis_data_->fric_speed_set>5900)
        {
          ui_g_DynamicGroup_FricNum->color = UI_Color_Purplish_red;
        }
        else if (chassis_move.chassis_data_->fric_speed_set<5900)
        {
          ui_g_DynamicGroup_FricNum->color = UI_Color_Green;
        }
        else
        {
          ui_g_DynamicGroup_FricNum->color = UI_Color_Orange;
        }
        ui_g_DynamicGroup_FricNum->number = chassis_move.chassis_data_->fric_speed_set;
        ui_g_DynamicGroup_HightNum->number = chassis_move.chassis_posture_info.leg_length_L*100;
        ui_g_DynamicGroup_JumpLine->end_y = calculate_jump_line_position();
        ui_g_DynamicGroup_JumpLine->start_y = ui_g_DynamicGroup_JumpLine->end_y;
        ui_update_g_DynamicGroup();

    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART6_IRQHandler(void)
{
   static volatile uint8_t res;
   if(USART6->SR & UART_FLAG_IDLE)
   {
       __HAL_UART_CLEAR_PEFLAG(&huart6);

       static uint16_t this_time_rx_len = 0;

       if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
       {
           __HAL_DMA_DISABLE(huart6.hdmarx);
           this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
           __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
           huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
           __HAL_DMA_ENABLE(huart6.hdmarx);
           fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
           detect_hook(REFEREE_TOE);
       }
       else
       {
           __HAL_DMA_DISABLE(huart6.hdmarx);
           this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
           __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
           huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
           __HAL_DMA_ENABLE(huart6.hdmarx);
           fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
           detect_hook(REFEREE_TOE);
       }
   }
}


