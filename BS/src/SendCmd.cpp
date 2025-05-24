// SendCmd.cpp
#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>

#include "SendCmd.h"

extern RFM69 txRadio;
extern bool txRadioReady;

void sendCommand(const Slope& slope, uint8_t commandId, const uint8_t* params, uint8_t paramLen) {
    if (!txRadioReady) return;
    uint8_t packet[64];

    // Sikkerhetssjekk: ikke overskrid bufferstørrelse
    if (paramLen + 2 > sizeof(packet)) return;

    packet[0] = 0xA0;          // Prefix
    packet[1] = commandId;     // Command ID

    if (params && paramLen > 0) {
        memcpy(&packet[2], params, paramLen);
    }

    txRadio.send(slope.getGliderId(), packet, paramLen + 2);
}

void sendFlightSettings(const Slope& slope) {

    int offsetN, offsetE, offsetD;
    slope.getPilotOffsetNED(offsetN, offsetE, offsetD);

    uint16_t angle10 = static_cast<uint16_t>(slope.getSlopeAngle() * 10);
    int16_t offsetN_cm = static_cast<int16_t>(offsetN * 100.0f);
    int16_t offsetE_cm = static_cast<int16_t>(offsetE * 100.0f);

    uint8_t params[8];
    params[0] = angle10 & 0xFF;
    params[1] = angle10 >> 8;
    params[2] = slope.getABaseLeft() ? 1 : 0;
    params[3] = slope.getGliderId();
    params[4] = offsetN_cm & 0xFF;
    params[5] = offsetN_cm >> 8;
    params[6] = offsetE_cm & 0xFF;
    params[7] = offsetE_cm >> 8;

    sendCommand(slope, MSG_FLIGHT_SETTINGS, params, sizeof(params));
}

void sendPosRequest(const Slope& slope) {
    sendCommand(slope, MSG_REQ_POS);
}