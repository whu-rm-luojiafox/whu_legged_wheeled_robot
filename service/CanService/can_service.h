#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include "struct_typedef.h"
#include "detect_task.h"
#include "motor_protocol.h"
#include "super_power_protocol.h"

#define CHASSIS_CAN hcan1
#define REFEREE_CAN hcan2

extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_referee_data(uint8_t color);
extern void CAN_INIT_STATUS(uint8_t status);
/* -----------------------HT_func------------------------- */
void CAN_HT_CMD( uint8_t id, fp32 f_t );
void CAN_CMD_HT_Enable(uint8_t id, uint8_t unterleib_motor_send_data[8] );
/* -----------------------LK_func------------------------ */
extern void CAN_read_lkmotor_state(void);
extern void CAN_LK_START_control(uint16_t id);
extern void CAN_LK_CLOSE_control(uint16_t id);
extern void CAN_LK_POSITION_Control(int32_t angleControl);
extern void CAN_LK_SPEED_Control(int16_t iqControl,int32_t speedControl);
extern void CAN_LK_Torque_Control(uint16_t id,int16_t iqControl);
void CAN_LK_Boradcast_Control(int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4);
/* -----------------------DM_func------------------------ */
extern void CAN_clear_dm_error(void);
extern void CAN_dm_enable(uint16_t motor_ID);
extern void speed_ctrl(float vel,uint16_t motor_ID);
extern void pos_sped_ctrl(float p_des,float v_limit,uint16_t motor_ID);
extern void MIT_CtrlMotor(float _pos, float _vel,float _KP, float _KD, float _torq,uint16_t motor_ID);
extern void CAN_dm_save_0_point(uint16_t motor_ID);
extern void CAN_dm_disable(uint16_t motor_ID);
/* -----------------------Setpower------------------------ */
extern void CAN_SuperPower_Control(super_power_t super_power_data);
//==========���ݷ�������ָ��=============
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

HTmotor_measure_t *get_HT_motor_measure_point(uint8_t i);
lkmotor_measure_t *get_LK_motor_measure_point(uint8_t i);


float get_wheel_velocity_point(uint8_t index);
#endif
