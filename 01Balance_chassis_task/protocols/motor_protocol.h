#ifndef _MOTOR_PROTOCOL_H
#define _MOTOR_PROTOCOL_H
 
#include "struct_typedef.h"
//========海泰电机参数===========
#define P_MIN -95.5f// Radians
#define P_MAX 95.5f
#define V_MIN -45.0f// Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f// N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f// N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
//========dm电机参数===========
#define P_MIN   -12.5  
#define P_MAX   12.5   
#define V_MIN   -30.0    
#define V_MAX   30.0     
#define KP_MIN  0      
#define KP_MAX  500    
#define KD_MIN  0      
#define KD_MAX  5      
#define Tor_MIN   -18.0f  
#define Tor_MAX   18.0f
//=======can_id================
/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  // CAN_3508_M4_ID = 0x204,

  CAN_YAW_MOTOR_ID = 0x205,
  CAN_PIT_MOTOR_ID = 0x206,
  CAN_TRIGGER_MOTOR_ID = 0x207,
  CAN_GIMBAL_ALL_ID = 0x1FF,
  CAN_gmbial_chassis_data1 = 0x01,
  CAN_gmbial_chassis_data2 = 0x02,
  CAN_gmbial_chassis_data3 = 0x03,
  CAN_gmbial_chassis_data4 = 0x04,
  CAN_chassis_gimbal_ID = 0x123,
  CAN_referee_data = 0x05,

  CAN_HT_MOTOR_ID1 = 0x01,
  CAN_HT_MOTOR_ID2 = 0x02,
  CAN_HT_MOTOR_ID3 = 0x03,
  CAN_HT_MOTOR_ID4 = 0x04,

  CAN_LK_MOTOR_ID1 = 0x141,
  CAN_LK_MOTOR_ID2 = 0x142,
  CAN_LK_MOTOR_ID3 = 0x143,

  CAN_DM_CLEAR_ERROR_ID = 0x7FF,
  CAN_DM_IMU_ID = 0x11,
  CAN_DM_MOTOR_ID4 = 0x204,

  CAN_SUPER_CAP_ID = 0x211,
  CAN_SUPER_CAP_SET_ID = 0x210,
  CAN_DISTANCE_ID = 0x208,
  CAN_SuperPower_ID = 0x52,

} can_msg_id_e;

//=======电机数据结构体==========
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	fp32 angle;
    int32_t ecd_count;
} motor_measure_t;

typedef struct
{
	int8_t temp;
	int16_t iq;
	int16_t speed;
	uint16_t encoder;
	uint16_t last_encoder;
	float angle;
  uint32_t last_update_time;
}lkmotor_measure_t;

typedef struct 
{
  float last_ecd;
  float ecd;
  float velocity_rad_s;
  float real_torque;
}HTmotor_measure_t;
//
typedef struct
{
    uint8_t ID;
	uint8_t state;
	int p_int;
    int v_int;
    int t_int;
    float pos;
    float vel;
    float torque;
    uint8_t mos_temperate;
	uint8_t rotor_temperate;
} dm_motor_measure_t;

//=======解包宏/函数=======
#define get_motor_measure(ptr, data)                                             \
  do                                                                             \
  {                                                                              \
    (ptr)->last_ecd = (ptr)->ecd;                                                \
    (ptr)->ecd = (uint16_t)(((data)[0] << 8) | (data)[1]);                       \
    (ptr)->speed_rpm = (int16_t)(((data)[2] << 8) | (data)[3]);                  \
    (ptr)->given_current = (int16_t)(((data)[4] << 8) | (data)[5]);              \
    (ptr)->temperate = (data)[6];                                                \
    if ((ptr)->ecd - (ptr)->last_ecd > 4096)                                     \
    {                                                                            \
      (ptr)->ecd_count--;                                                        \
    }                                                                            \
    else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                               \
    {                                                                            \
      (ptr)->ecd_count++;                                                        \
    }                                                                            \
    (ptr)->angle = (ptr)->ecd_count * 360.0f +                                   \
                   (ptr)->ecd * 360.0f / 8192.0f;                                \
  } while (0)

#define get_lkmotor_measure(ptr, data)                                           \
  do                                                                             \
  {                                                                              \
    (ptr)->temp = (int8_t)(data)[1];                                             \
    (ptr)->iq = (int16_t)(((data)[3] << 8) | (data)[2]);                         \
    (ptr)->speed = (int16_t)(((data)[5] << 8) | (data)[4]);                      \
    (ptr)->last_encoder = (ptr)->encoder;                                        \
    (ptr)->encoder = (uint16_t)(((data)[7] << 8) | (data)[6]);                   \
    (ptr)->angle = (float)(ptr)->encoder * 360.0f / 65536.0f;                    \
  } while (0)

#define get_HT_motor_measure(ptr, data)                                          \
  do                                                                             \
  {                                                                              \
    (ptr)->last_ecd = (ptr)->ecd;                                                \
    (ptr)->ecd = uint_to_float(                                                  \
        (uint16_t)(((data)[1] << 8) | (data)[2]), P_MIN, P_MAX, 16) * 180.0f / PI;   \
    if ((ptr)->ecd > 180.0f)                                                     \
    {                                                                            \
      (ptr)->ecd -= 360.0f;                                                      \
    }                                                                            \
    if ((ptr)->ecd < -180.0f)                                                    \
    {                                                                            \
      (ptr)->ecd += 360.0f;                                                      \
    }                                                                            \
    (ptr)->velocity_rad_s = uint_to_float(                                       \
        (uint16_t)(((data)[3] << 4) | ((data)[4] >> 4)), V_MIN, V_MAX, 12);      \
    (ptr)->real_torque = uint_to_float(                                          \
        (uint16_t)((((data)[4] & 0x0FU) << 8) | (data)[5]),                      \
        T_MIN, T_MAX, 12);                                                       \
  } while (0)

#define get_superpower_measure(ptr, data)                                        \
  do                                                                             \
  {                                                                              \
    (ptr)->statusCode = (uint8_t)(data)[0];                                      \
    (ptr)->chassisPower = (uint16_t)((                                           \
        (((data)[2] << 8) | (data)[1]) - 16384.0f) / 64.0f);                    \
    (ptr)->refereePower = (uint16_t)((                                           \
        (((data)[4] << 8) | (data)[3]) - 16384.0f) / 64.0f);                    \
    (ptr)->chassisPowerLimit =                                                   \
        (uint16_t)(((data)[6] << 8) | (data)[5]);                                \
    (ptr)->capEnergy = (uint8_t)(data)[7];                                       \
  } while (0)

#define get_dm_measure(ptr, data)                                             \
  do                                                                           \
  {                                                                            \
    (ptr)->ID = (uint8_t)((data)[0] & 0x0F);                                   \
    (ptr)->state = (uint8_t)((data)[0] >> 4);                                  \
    (ptr)->p_int = (uint16_t)(((data)[1] << 8) | (data)[2]);                   \
    (ptr)->v_int = (uint16_t)(((data)[3] << 4) | ((data)[4] >> 4));            \
    (ptr)->t_int = (uint16_t)((((data)[4] & 0x0F) << 8) | (data)[5]);          \
    (ptr)->pos = uint_to_float((ptr)->p_int, P_MIN, P_MAX, 16);                \
    (ptr)->vel = uint_to_float((ptr)->v_int, V_MIN, V_MAX, 12);                \
    (ptr)->torque = uint_to_float((ptr)->t_int, Tor_MIN, Tor_MAX, 12);         \
    (ptr)->rotor_temperate = (data)[7];                                       \
  } while (0)

//===========数学工具宏/函数============= 
static inline float TOFSense_ParseDistanceM(uint8_t d0, uint8_t d1, uint8_t d2)
{
  uint32_t raw_u24 = (uint32_t)d0 |
                     ((uint32_t)d1 << 8) |
                     ((uint32_t)d2 << 16);

  if ((raw_u24 & 0x00800000U) != 0U)
  {
    raw_u24 |= 0xFF000000U;
  }

  return (int32_t)raw_u24 / 1000.0f;
};
static inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;

  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
};
static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
};
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))




#endif // 
