#include "UART_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"

uart_data_t uart_data;
chassis_data_t *chassis_data_get;
/*------接收相关数据------*/
static uint8_t buffer[sizeof(uart_data_t)];//用于存储完整的数据包
static uint8_t Count=0;//接收状态标签
static uint16_t received = 0;//当前接收到的数据长度
static uint8_t Usart_Receive[1];//用于接收单个字节的数据
referee_data_t referee_data;


	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t received_byte = Usart_Receive[0]; // 读取接收到的字节
        // buffer_single = received_byte;
        // 读取该字节，检查是否为帧头的sof
        if (Count == 0)
        {
            if (received_byte == 0xAA)
            {
                buffer[received++] = received_byte; // 存储帧头起始标志
                Count = 1;                          // 进入帧头接收阶段
            }
            else
            {
                received = 0; // 不是帧头标志则丢弃数据
            }
        }
        else if (Count == 1) 
        {
            if (received_byte == 0xFF)
            {
                buffer[received++] = received_byte;
                Count = 2; 
            }
            else
            {
                received = 0; // 帧头不匹配，重置接收
                Count = 0;

            }
        }
        else if (Count == 2) // 接收数据部分以及帧尾
        {
            buffer[received++] = received_byte;

            // 检查数据包是否接收完整
            if (received ==  sizeof(referee_data_t) )
            {
                // 无论是否校验成功，都重置接收状态
                memcpy(&referee_data, buffer, sizeof(referee_data_t));
                received = 0;
                Count = 0;
            }
        }
        // 重新开始接收
        HAL_UART_Receive_IT(&huart1, (uint8_t *)Usart_Receive, 1);
    }
}

void uart_task(void const *pvParameters)
{
    uart_data.header.sof1=0xAA;
    uart_data.header.sof2=0xFF;
	chassis_data_get=get_chassis_data_point();
    HAL_UART_Receive_IT(&huart1, (uint8_t *)Usart_Receive, 1);
    while(1)
    {
        memcpy((uint8_t *)&(uart_data.send_chassis_data),chassis_data_get,sizeof(chassis_data_t));
        HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&uart_data,sizeof(uart_data_t));
        osDelay(10);
    }
}

referee_data_t* get_referee_data()
{
    return &referee_data;
}