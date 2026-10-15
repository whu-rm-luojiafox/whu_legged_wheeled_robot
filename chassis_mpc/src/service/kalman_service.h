#ifndef _KALMAN_SERVICE_H
#define _KALMAN_SERVICE_H 

#include "main.h"
#include "kalman_filter.h"

typedef enum
{
    WHEEL_LEFT = 0,
    WHEEL_RIGHT = 1,
    WHEEL_NUM = 2,
}wheel_index_t;

//=========对外接口函数=============
void WheelSpeedKF_Init(void);
void WheelSpeedKF_Update(float torque1,float torque2);
float *WheelSpeedKF_Get_point(uint8_t index);

#endif
