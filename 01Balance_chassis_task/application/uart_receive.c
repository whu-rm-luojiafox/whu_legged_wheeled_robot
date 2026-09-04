#include "uart_receive.h"
#include "referee.h"

uint8_t buffer[sizeof(uart_data_t)];//用于存储完整的数据包
uint8_t Count=0;//接收状态标签
uint16_t received = 0;//当前接收到的数据长度
uint8_t Usart_Receive[1];//用于接收单个字节的数据
uart_data_t uart_data;

shoot_data_t* shoot_data_get;   
/*发送数据结构体*/
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
            if (received ==  sizeof(uart_data_t) )
            {
                // 无论是否校验成功，都重置接收状态
                memcpy(&uart_data, buffer, sizeof(uart_data_t));
                received = 0;
                Count = 0;
            }
        }
        // 重新开始接收
        HAL_UART_Receive_IT(&huart1, (uint8_t *)Usart_Receive, 1);
    }
}
void uart_start_task(void const  *pvParameters)
{
    referee_data.header.sof1 =0xAA;
    referee_data.header.sof2 =0xFF;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)Usart_Receive, 1);
    while(1)
    {
        get_shoot_speed(&referee_data.shoot_speed);
        HAL_UART_Transmit(&huart1,(uint8_t *)&referee_data,sizeof(referee_data_t),100);
       osDelay(100);
    }
}


const chassis_data_t *get_Uart_Chassisdata_point()
{
  return &uart_data.receive_chassis_data;
}
