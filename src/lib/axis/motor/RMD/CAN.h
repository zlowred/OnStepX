#pragma once

#ifndef CAN_INTERAFCE
#include <ACAN_T4.h>
#define CAN_INTERAFCE ACAN_T4::can1
#define CAN_MESSAGE CANMessage
#define CAN_SETTINGS ACAN_T4_Settings
#endif

#include "../../../../Common.h"

class RmdCan {
    public:
        bool init();
        double getPosition(int addr);
        void setPosition(int addr, double position, double maxSpeed);
        void enable(int addr);
        void disable(int addr);
    private:
        bool sendReceive();
        
        bool initialized = false;
        CAN_MESSAGE message;
        CAN_MESSAGE dump;

};
