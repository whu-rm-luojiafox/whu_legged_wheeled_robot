/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"



//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;


//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//串口中断
void USART6_IRQHandler(void)
{
    if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //判断是遥控器的帧头才处理，否则直接丢弃
                if(sbus_rx_buf[0][0] == 0xA9 && sbus_rx_buf[0][1] == 0x53)
                {
                    if(verify_CRC16_check_sum(sbus_rx_buf[0],RC_FRAME_LENGTH))
                    {
                        sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                        //记录数据接收时间
                        detect_hook(DBUS_TOE);
                    }
                }
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //判断是遥控器的帧头才处理，否则直接丢弃
                if(sbus_rx_buf[1][0] == 0xA9 && sbus_rx_buf[1][1] == 0x53)
                {
                    if(verify_CRC16_check_sum(sbus_rx_buf[1],RC_FRAME_LENGTH))
                    {
                        sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                        //记录数据接收时间
                        detect_hook(DBUS_TOE);
                    }
                }
            }
        }
    }

}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    /* ------------------------新图传数据接收协议------------------------ */
    rc_ctrl->rc.ch[0] = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) |          //!< Channel 2
                         (sbus_buf[6] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x07ff;  //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[7]   >> 4) & 0x0003);                        //!< Switch mode
    rc_ctrl->rc.s[1] = ((sbus_buf[7] >> 4) & 0x0004)>>2;                     //!< suspend_key
    rc_ctrl->rc.s[2] = ((sbus_buf[7] >> 4) & 0x0008)>>3;                     //!< left_key
    rc_ctrl->rc.s[3] = (sbus_buf[8] & 0x0001);                               //!< right_key
    rc_ctrl->rc.ch[4] = ((sbus_buf[8] >> 1) | (sbus_buf[9] << 7)) & 0x07ff;  //!< Channel 4
    rc_ctrl->rc.s[4] = ((sbus_buf[9]>>4) & 0x0001);                               //!< left_key

    rc_ctrl->mouse.x = sbus_buf[10] | (sbus_buf[11] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[12] | (sbus_buf[13] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[14] | (sbus_buf[15] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = (sbus_buf[16]&0x0003);                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = ((sbus_buf[16]>>2)&0x0003);                                  //!< Mouse Right Is Press ?
    rc_ctrl->mouse.press_m = ((sbus_buf[16]>>4)&0x0003);                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[17] | (sbus_buf[18] << 8);                    //!< KeyBoard value

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
// 根据说明书重新编写的解析函数
//void sbus_to_rc(uint8_t* vtm_buf, RC_ctrl_t * rc_ctrl) {
//    // 1. 验证帧头
//    if (vtm_buf[0] != 0x09 || vtm_buf[1] != 0x53) {
//        return; // 帧头错误
//    }    
//    // 方法2：手动解析（更可靠）
//    uint32_t* data = (uint32_t*)(vtm_buf + 1); // 跳过帧头
//    
//    // 通道0 (位0-10)
//    rc_ctrl->rc.ch[0] = (data[0] & 0x07FF);
//    
//    // 通道1 (位11-21)
//    rc_ctrl->rc.ch[1] = ((data[0] >> 11) & 0x07FF);
//    
//    // 通道2 (位22-32) - 跨字节边界
//    rc_ctrl->rc.ch[2] = ((data[0] >> 22) & 0x07FF) | ((data[1] & 0x0003) << 10);
//    
//    // 通道3 (位33-43)
//    rc_ctrl->rc.ch[3] = ((data[1] >> 2) & 0x07FF);
//    
//    // 挡位切换开关 (位60-61, 2位)
//    rc_ctrl->rc.s[0] = ((data[1] >> 13) & 0x03);  // 0:C, 1:N, 2:S
//    
//    // 暂停按键 (位62, 1位)
//    rc_ctrl->rc.s[1] = ((data[1] >> 15) & 0x01);
//    
//    // 自定义按键(左) (位63, 1位)
//    rc_ctrl->rc.s[2] = ((data[1] >> 16) & 0x01);
//    
//    // 自定义按键(右) (位64, 1位)
//    rc_ctrl->rc.s[3] = ((data[1] >> 17) & 0x01);
//    
//    // 拨轮 (位65-75, 11位) - 需要跨多个字节
//    uint32_t dial_low = (data[1] >> 18) & 0x3FFF;  // 低14位
//    uint32_t dial_high = (data[2] & 0x0007) << 14; // 高3位
//    rc_ctrl->rc.ch[4] = (dial_low | dial_high) & 0x07FF;
//    
//    // 扳机键 (位76, 1位)
//    rc_ctrl->rc.s[4] = ((data[2] >> 3) & 0x01);
//    
//    // 鼠标X轴 (位80-95, 16位，有符号)
//    // 位偏移：80位 = 10字节 + 0位
//    int16_t mouse_x = (vtm_buf[10] | (vtm_buf[11] << 8));
//    rc_ctrl->mouse.x = mouse_x;
//    
//    // 鼠标Y轴 (位96-111, 16位，有符号)
//    int16_t mouse_y = (vtm_buf[12] | (vtm_buf[13] << 8));
//    rc_ctrl->mouse.y = mouse_y;
//    
//    // 鼠标Z轴 (位112-127, 16位，有符号)
//    int16_t mouse_z = (vtm_buf[14] | (vtm_buf[15] << 8));
//    rc_ctrl->mouse.z = mouse_z;
//    
//    // 鼠标左键 (位128-129, 2位)
//    rc_ctrl->mouse.press_l = (vtm_buf[16] & 0x03);
//    
//    // 鼠标右键 (位130-131, 2位)
//    rc_ctrl->mouse.press_r = ((vtm_buf[16] >> 2) & 0x03);
//    
//    // 鼠标中键 (位132-133, 2位)
//    rc_ctrl->mouse.press_m = ((vtm_buf[16] >> 4) & 0x03);
//    
//    // 键盘 (位136-151, 16位)
//    rc_ctrl->key.v = (vtm_buf[17] | (vtm_buf[18] << 8));
//    
//    // 3. 通道值偏移（根据说明书的364-1024-1684范围）
//    const uint16_t RC_MIN = 364;
//    const uint16_t RC_MID = 1024;
//    const uint16_t RC_MAX = 1684;
//    
//    // 注意：这里可能需要减去中间值1024，而不是固定的偏移
//    // rc_ctrl->rc.ch[0] -= 1024; // 如果需要转换为-660到+660的范围
//}

void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

