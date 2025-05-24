// MessageTypes.h
#pragma once

#ifndef MESSAGETYPES_H
#define MESSAGETYPES_H

#include <stdint.h>
#include "Const.h"

// Message Types
#define MSG_TYPE_G2B_EVENT  0b00
#define MSG_TYPE_G2B_RELPOS 0b01
#define MSG_TYPE_G2B_MISC   0b10

// Decoding helpers
inline uint8_t decodeMsgType(uint8_t header) {
    return (header >> 6) & 0x03;
}

inline uint8_t decodeGliderId(uint8_t header) {
    return header & 0x3F;
}

// Encoding functions
inline uint8_t encodeHeader(uint8_t msgType, uint8_t gliderId) {
    return ((msgType & 0x03) << 6) | (gliderId & 0x3F);
}

inline void encodeEventMessage(uint8_t* msg, uint8_t gliderId, uint8_t event, uint8_t status) {
    msg[0] = encodeHeader(MSG_TYPE_G2B_EVENT, gliderId);
    msg[1] = (event << 4) | (status & 0x0F);
    for (int i = 2; i < 6; ++i) msg[i] = 0;  // Reserved
}

inline void decodeEventMessage(const uint8_t* msg, uint8_t& gliderId, uint8_t& event, uint8_t& status) {
    gliderId = decodeGliderId(msg[0]);
    event    = (msg[1] >> 4) & 0x0F;
    status   = msg[1] & 0x0F;
}

inline void encodeRelPosMessage(uint8_t* msg, uint8_t gliderId, int16_t n, int16_t e, int16_t d, uint8_t status) {
    msg[0] = encodeHeader(MSG_TYPE_G2B_RELPOS, gliderId);
    uint32_t packed = ((n & 0x3FF) << 20) | ((e & 0x3FF) << 10) | (d & 0x3FF);
    msg[1] = (packed >> 24) & 0xFF;
    msg[2] = (packed >> 16) & 0xFF;
    msg[3] = (packed >> 8) & 0xFF;
    msg[4] = packed & 0xFF;
    msg[5] = status;
}

inline void decodeRelPos(const uint8_t* msg, int16_t& n, int16_t& e, int16_t& d) {
    uint32_t packed = (msg[1] << 24) | (msg[2] << 16) | (msg[3] << 8) | msg[4];
    n = (packed >> 20) & 0x3FF;
    e = (packed >> 10) & 0x3FF;
    d = packed & 0x3FF;
    // Sign extend 10-bit values
    if (n & 0x200) n |= 0xFC00;
    if (e & 0x200) e |= 0xFC00;
    if (d & 0x200) d |= 0xFC00;
}

inline void encodeMiscMessage(uint8_t* msg, uint8_t gliderId, uint8_t d) {
    msg[0] = encodeHeader(MSG_TYPE_G2B_MISC, gliderId);
    msg[1] = d & 0x0F;
    for (int i = 2; i < 6; ++i) msg[i] = 0;
}

inline uint8_t decodeMiscMessage(const uint8_t* msg, uint8_t& gliderId, uint8_t& d) {
    gliderId = decodeGliderId(msg[0]);
    d = msg[1] & 0x0F;
    return decodeMsgType(msg[0]); // Optional: return type for confirmation
}

#endif