#pragma once

#ifndef CAN_INTERAFCE
#include <ACAN.h>
#define CAN_INTERAFCE ACAN::can0
#define CAN_MESSAGE CANMessage
#define CAN_SETTINGS ACANSettings
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
