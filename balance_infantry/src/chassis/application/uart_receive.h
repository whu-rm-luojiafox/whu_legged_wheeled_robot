#ifndef __UART_RECEIVE_H__
#define __UART_RECEIVE_H__

#include "main.h"
#include "usart.h"
#include "gim_cha_protocol.h"
#include "referee.h"

#include "robot_link_frames.h"



void uart_start_task(void const  *pvParameters);

const chassis_data_t *get_Uart_Chassisdata_point();

#endif


