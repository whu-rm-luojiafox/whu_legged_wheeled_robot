#ifndef __UART_TASK_H
#define __UART_TASK_H

#include "main.h"
#include "usart.h"
#include "gimbal_task.h"
#include "shoot.h"

typedef struct  {
    uint8_t sof1;
    uint8_t sof2;
} frameHeader;


/*发送数据结构体*/
typedef struct
{
    frameHeader header;
    chassis_data_t send_chassis_data;
}uart_data_t;



typedef struct 
{
    frameHeader header;
    float shoot_speed;    
}referee_data_t;


void uart_task(void const *pvParameters);

extern referee_data_t* get_referee_data();
#endif