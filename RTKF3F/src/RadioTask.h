// RadioTask.h
#pragma once

#include <Arduino.h>

struct RadioInitParams {
    int8_t pinMISO;
    int8_t pinMOSI;
    int8_t pinSCK;
    int8_t pinCS;
    int8_t pinIRQ;
    uint8_t nodeId;
    uint8_t networkId;
    uint32_t frequency;
};

enum class RadioMessageType {
    RTCM,
    STATUS
};

struct RxPacket {
    uint8_t  from = 0;
    uint8_t  len = 0;
    int16_t  rssi = 0;     // optional
    uint8_t  data[1024];     // adjust to your max payload
};

struct RadioMessage {
    RadioMessageType type;
    union {
        struct {
            const uint8_t* data;
            size_t length;
        } rtcm;
        struct {
            uint8_t destNode;
            uint32_t destFreq;
            uint32_t returnFreq;
            uint8_t msgType;
            uint8_t code;
        } status;
    };
};

bool radioStartTask(int8_t cs, int8_t irq, int8_t miso, int8_t mosi, int8_t sck,
                    uint8_t nodeId, uint8_t networkId, uint32_t frequency);
bool radioSendRTCM(const uint8_t* data, size_t len);
bool radioSendMsg(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq, uint8_t msgType, uint8_t code);
bool radioReceive(RxPacket& out, TickType_t timeoutTicks = 0);
