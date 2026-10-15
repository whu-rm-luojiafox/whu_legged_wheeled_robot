#ifndef _SUPER_POWER_PROTOCOL_H
#define _SUPER_POWER_PROTOCOL_H

#include "struct_typedef.h"

typedef struct
{
    uint8_t statusCode;
    uint16_t chassisPower;
    uint16_t refereePower;
    uint16_t chassisPowerLimit;
    uint8_t capEnergy;
} __attribute__((packed))super_power_receive_t;
typedef struct{
    uint8_t enableDCDC: 1;
    uint8_t systemRestart: 1;
    uint8_t resv0: 3;
    uint8_t clearError: 1;
    uint8_t enableActiveChargingLimit: 1;
    uint8_t useNewFeedbackMessage: 1;

    uint16_t refereePowerLimit;
    uint16_t refereeEnergyBuffer;
    uint8_t activeChargingLimitRatio; 
    int16_t resv2;
} __attribute__((packed)) super_power_t ; 

#endif // !
