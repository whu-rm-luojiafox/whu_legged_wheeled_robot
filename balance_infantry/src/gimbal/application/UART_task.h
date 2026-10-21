#ifndef __UART_TASK_H
#define __UART_TASK_H

#include "main.h"
#include "usart.h"
#include "gimbal_task.h"
#include "shoot.h"

#include "robot_link_frames.h"



void uart_task(void const *pvParameters);

extern referee_data_t* get_referee_data();
#endif