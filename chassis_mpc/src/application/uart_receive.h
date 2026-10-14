#ifndef __UART_RECEIVE_H__
#define __UART_RECEIVE_H__

#include "main.h"
#include "usart.h"
#include "gim_cha_protocol.h"
#include "referee.h"

typedef struct  {
    uint8_t sof1;
    uint8_t sof2;
} frameHeader;

/*接收数据结构体*/
typedef struct
{
    frameHeader header;    
    chassis_data_t receive_chassis_data;
}uart_data_t;

/*发送数据结构体*/

typedef struct 
{
    frameHeader header;
    float shoot_speed;   
}referee_data_t;


void uart_start_task(void const  *pvParameters);

const chassis_data_t *get_Uart_Chassisdata_point();

#endif


