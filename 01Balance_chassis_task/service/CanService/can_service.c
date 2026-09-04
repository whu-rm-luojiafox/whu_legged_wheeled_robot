#include "can_service.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef referee_tx_message;
static uint8_t referee_can_send_data[1];

motor_measure_t motor_chassis[7];
lkmotor_measure_t lkmotor_data[2];
HTmotor_measure_t htmotor_data[4];
super_power_receive_t super_power_data;





void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  if (hcan == &hcan1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      return;
    }
    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
    {
      return;
    }
    switch (rx_data[0])
    {
      case CAN_HT_MOTOR_ID1:
      case CAN_HT_MOTOR_ID2:
      case CAN_HT_MOTOR_ID3:
      case CAN_HT_MOTOR_ID4:
      {
        uint8_t i = rx_data[0] - CAN_HT_MOTOR_ID1;
        get_HT_motor_measure(&htmotor_data[i], rx_data);
        break;
      }
      default:
      {
        break;
      }
    }
    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
      uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }
    case CAN_SuperPower_ID:
    {
      get_superpower_measure(&super_power_data, rx_data);
      break;
    }
    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hcan2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      return;
    }
    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
    {
      return;
    }
    switch (rx_header.StdId)
    {
      case CAN_LK_MOTOR_ID1:
      {
        get_lkmotor_measure(&lkmotor_data[0], rx_data);
        lkmotor_data[0].last_update_time = xTaskGetTickCountFromISR();
        break;
      }
      case CAN_LK_MOTOR_ID2:
      {
        get_lkmotor_measure(&lkmotor_data[1], rx_data);
        lkmotor_data[1].last_update_time = xTaskGetTickCountFromISR();
        break;
      }
      case CAN_YAW_MOTOR_ID:
      {
        uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], rx_data);
        break;
      }
      case CAN_TRIGGER_MOTOR_ID:
      {
        uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + i);
        break;
      }
      // case CAN_DISTANCE_ID:
      // {
      //   uint8_t dis_status = rx_data[3];

      //   if (dis_status == 0)
      //   {
      //     chassis_move.distance = TOFSense_ParseDistanceM(
      //         rx_data[0], rx_data[1], rx_data[2]);
      //   }
      //   break;
      // }
      default:
      {
        break;
      }
    }
  }
}

void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&REFEREE_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_referee_data(uint8_t color)
{
  uint32_t send_mail_box;
  referee_tx_message.StdId = CAN_referee_data;
  referee_tx_message.IDE = CAN_ID_STD;
  referee_tx_message.RTR = CAN_RTR_DATA;
  referee_tx_message.DLC = 0x01;
  referee_can_send_data[0] = color;
  HAL_CAN_AddTxMessage(&REFEREE_CAN, &referee_tx_message, referee_can_send_data, &send_mail_box);
}
void CAN_INIT_STATUS(uint8_t status)
{
  uint32_t send_mail_box;
  referee_tx_message.StdId = CAN_chassis_gimbal_ID;
  referee_tx_message.IDE = CAN_ID_STD;
  referee_tx_message.RTR = CAN_RTR_DATA;
  referee_tx_message.DLC = 0x01;
  referee_can_send_data[0] = status;
  HAL_CAN_AddTxMessage(&REFEREE_CAN, &referee_tx_message, referee_can_send_data, &send_mail_box);
}
/*------------------------6020云台电机---------------------*/
void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = motor1 >> 8;
  gimbal_can_send_data[1] = motor1;
  gimbal_can_send_data[2] = motor2 >> 8;
  gimbal_can_send_data[3] = motor2;
  gimbal_can_send_data[4] = motor3 >> 8;
  gimbal_can_send_data[5] = motor3;
  gimbal_can_send_data[6] = motor4 >> 8;
  gimbal_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
/* ------------------------海泰电机------------------------- */
void CAN_HT_CMD(uint8_t id, fp32 f_t)
{
  float t1,t2,t3,t4;
  uint32_t can_tx_mailbox;

  fp32 f_p = 0.0f, f_v = 0.0f, f_kp = 0.0f, f_kd = 0.0f;
  uint16_t p, v, kp, kd, t;
  uint8_t buf[8];
  LIMIT_MIN_MAX(f_p, P_MIN, P_MAX);
  LIMIT_MIN_MAX(f_v, V_MIN, V_MAX);
  LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
  LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
  LIMIT_MIN_MAX(f_t, T_MIN, T_MAX);

  p = float_to_uint(f_p, P_MIN, P_MAX, 16);
  v = float_to_uint(f_v, V_MIN, V_MAX, 12);
  kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
  kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
  t = float_to_uint(f_t, T_MIN, T_MAX, 12);

  switch (id)
  {
    case 0x01:
    {
      t1=f_t;
      break;
    }
    case 0x02:
      {
      t2=f_t;
      break;
    }
    case 0x03:
    {
      t3=f_t;
      break;
    }
    case 0x04:
    {
      t4=f_t;
      break;
    }
  }
  buf[0] = p >> 8;
  buf[1] = p & 0xFF;
  buf[2] = v >> 4;
  buf[3] = ((v & 0xF) << 4) | (kp >> 8);
  buf[4] = kp & 0xFF;
  buf[5] = kd >> 4;
  buf[6] = ((kd & 0xF) << 4) | (t >> 8);
  buf[7] = t & 0xff;

  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;

  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, buf, &can_tx_mailbox);
}
void CAN_CMD_HT_Enable(uint8_t id, uint8_t unterleib_motor_send_data[8])
{
  uint32_t can_tx_mailbox;

  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;

  HAL_CAN_AddTxMessage(
      &hcan1, &chassis_tx_message, unterleib_motor_send_data, &can_tx_mailbox);
}
/* ------------------------瓴控电机------------------------- */
void CAN_LK_START_control(uint16_t id)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0x88;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_CLOSE_control(uint16_t id)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0x80;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_read_lkmotor_state(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_LK_MOTOR_ID1;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0x9C;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_POSITION_Control(int32_t angleControl)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_LK_MOTOR_ID1;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0xA3;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = *((uint8_t *)(&angleControl));
  chassis_can_send_data[5] = *((uint8_t *)(&angleControl) + 1);
  chassis_can_send_data[6] = *((uint8_t *)(&angleControl) + 2);
  chassis_can_send_data[7] = *((uint8_t *)(&angleControl) + 3);
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_SPEED_Control(int16_t iqControl, int32_t speedControl)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_LK_MOTOR_ID1;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0xA2;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = *((uint8_t *)(&iqControl));
  chassis_can_send_data[3] = *((uint8_t *)(&iqControl) + 1);
  chassis_can_send_data[4] = *((uint8_t *)(&speedControl));
  chassis_can_send_data[5] = *((uint8_t *)(&speedControl) + 1);
  chassis_can_send_data[6] = *((uint8_t *)(&speedControl) + 2);
  chassis_can_send_data[7] = *((uint8_t *)(&speedControl) + 3);
  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_Torque_Control(uint16_t id,int16_t iqControl)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0xA1;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = *((uint8_t *)(&iqControl));
  chassis_can_send_data[5] = *((uint8_t *)(&iqControl) + 1);
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_Boradcast_Control(int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x280;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = *(uint8_t *)(&iqControl_1);
  chassis_can_send_data[1] = *((uint8_t *)(&iqControl_1)+1);
  chassis_can_send_data[2] = *(uint8_t *)(&iqControl_2);
  chassis_can_send_data[3] = *((uint8_t *)(&iqControl_2)+1);
  chassis_can_send_data[4] = *(uint8_t *)(&iqControl_3);
  chassis_can_send_data[5] = *((uint8_t *)(&iqControl_3) + 1);
  chassis_can_send_data[6] = *(uint8_t *)(&iqControl_4);
  chassis_can_send_data[7] = *((uint8_t *)(&iqControl_4)+1);
  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/* -----------------超级电容功率控制----------------- */
void CAN_SuperPower_Control(super_power_t super_power_data)
{
  uint32_t send_mail_box;
  uint8_t *p_data = (uint8_t *)&super_power_data;
  uint8_t i;

  chassis_tx_message.StdId = 0x61;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  // 直接复制整个结构体
  for (i = 0; i < 8U; i++)
  {
    chassis_can_send_data[i] = p_data[i];
  }
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//===============数据反馈函数指针=================
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

HTmotor_measure_t *get_HT_motor_measure_point(uint8_t i)
{
  return &htmotor_data[i & 0x03U];
}

lkmotor_measure_t *get_LK_motor_measure_point(uint8_t i)
{
  return &lkmotor_data[i & 0x01U];
}



float get_wheel_velocity_point(uint8_t index)
{
  return  (float)motor_chassis[index].speed_rpm*2*PI/60.0f;  //rad/s
}


