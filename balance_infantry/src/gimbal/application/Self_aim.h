#include "struct_typedef.h"
#include "gimbal_task.h"

typedef struct {
    uint8_t     mode;
    float       q[4];
    float       yaw;
    float       yaw_vel;
    float       pitch;
    float       pitch_vel;
    float       bullet_speed;
    uint16_t    bullet_count;

} __attribute__((packed)) OutputData;


typedef struct {
    uint8_t       mode;
    float         yaw;
    float         yaw_vel;
    float         yaw_acc;
    float         pitch;
    float         pitch_vel;
    float         pitch_acc;
} __attribute__((packed)) InputData;

typedef struct  {
    uint8_t     s;
    uint8_t     p;
} __attribute__((packed))FrameHeader;

typedef struct {
    uint16_t    crc16;
} __attribute__((packed))FrameTailer ;

typedef struct  {            
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;   
} __attribute__((packed))RECEIVE_DATA;

typedef struct  {         
    FrameHeader frame_header;
    OutputData  output_data;
    FrameTailer frame_tailer;
	
} __attribute__((packed))SEND_DATA;

#define RECEIVE_DATA_SIZE 12
#define SEND_DATA_SIZE 19
#define FRAME_HEADER      0X7B //Frame_header 
#define FRAME_TAIL        0X7D //Frame_tail 

#define FRAME_HEADER_S  'S'     //0x53
#define FRAME_HEADER_P  'P'     //0x50
// #define FRAME_HEADER_SOF  0xA5  
#define FRAME_HEADER_CRC8 0xFF  
#define FRAME_TAILER_CRC16_init 0xFFFF  
void Self_aim_task(void const *pvParameters);
InputData *get_selfaim_data(void);


extern void usbd_cdc_receive_handle(void);
extern uint8_t Usart_Receive[40]; //用于接收单个字节的数据




