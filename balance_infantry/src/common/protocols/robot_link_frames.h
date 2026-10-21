#ifndef ROBOT_LINK_FRAMES_H
#define ROBOT_LINK_FRAMES_H
#include "gim_cha_protocol.h"

#define ROBOT_LINK_SOF1 0xAAu
#define ROBOT_LINK_SOF2 0xFFu

typedef struct {
    uint8_t sof1;
    uint8_t sof2;
} frameHeader;

/* Preserve the existing natural alignment and UART padding. Both boards must
 * use -fshort-enums. Changing framing/packing requires a two-board migration. */
typedef struct {
    frameHeader header;
    union {
        chassis_data_t command;
        /* Compatibility names retained for existing UI/debug expressions. */
        chassis_data_t receive_chassis_data;
        chassis_data_t send_chassis_data;
    };
} robot_command_frame_t;

typedef struct {
    frameHeader header;
    float shoot_speed;
} robot_referee_frame_t;

typedef robot_command_frame_t uart_data_t;
typedef robot_referee_frame_t referee_data_t;
#endif
