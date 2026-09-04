#ifndef _LQR_H
#define _LQR_H
#include "chassis_task.h"




/*----------------返回数组和更新K矩阵函数------------------------*/
extern float LQR[4][10];

void LQR_Data_Update(chassis_move_t *chassis_move);

#endif
