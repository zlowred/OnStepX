#include "CAN.h"

bool RmdCan::init() {
    if (!initialized) {
        CAN_SETTINGS settings(1000 * 1000) ; // 1 mbit/s

        const uint32_t errorCode = CAN_INTERAFCE.begin(settings);
        if (0 == errorCode) {
            V("CAN init ok");
        } else {
            V("CAN init error ");
            VL(errorCode);
            return false;
        }
        initialized = true;
    }

    return true;
}

double RmdCan::getPosition(int addr) {
    V("CAN: get position @ ");VL(addr);
    return 0;
}

void RmdCan::setPosition(int addr, double position, double maxSpeed) {
    // V("CAN: set position ");V(position);V(" with max speed ");V(maxSpeed);V(" @ ");VL(addr);

    long maxSpeedInt = maxSpeed;
    long positionInt = position;

    message.id = 0x140 + addr;
    message.len = 8;
    message.data[0] = 0xA4;
    message.data[1] = 0;
    message.data[2] = maxSpeedInt & 0xFF; 
    message.data[3] = (maxSpeedInt >> 8) & 0xFF;
    message.data[4] = positionInt & 0xFF; 
    message.data[5] = (positionInt >> 8) & 0xFF;
    message.data[6] = (positionInt >> 16) & 0xFF;
    message.data[7] = (positionInt >> 24) & 0xFF;

    sendReceive();
}

void RmdCan::enable(int addr) {
    V("CAN: enable @ ");VL(addr);
}

void RmdCan::disable(int addr) {
    V("CAN: disable @ ");VL(addr);
}

bool RmdCan::sendReceive() {
    while (CAN_INTERAFCE.available()) {
        CAN_INTERAFCE.receive(dump);
    }

    if (!CAN_INTERAFCE.tryToSend(message)) {
        return false;
    }

    unsigned long timer = micros();
    while (!CAN_INTERAFCE.available() && micros() - timer < 10000) {
        delayMicroseconds(100);
    }

    if (CAN_INTERAFCE.available()) {
        return CAN_INTERAFCE.receive(message);
    }

    return false;
}
