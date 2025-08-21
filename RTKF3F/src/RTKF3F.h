//RTKF3F.h

#pragma once

#include "Const.h"  
#include "Structs.h"
#include "GNSSModule.h"
#include "RadioModule.h"
#include "ConfigMgr.h"
#include "Arena.h"
#include "Glider.h"

const char* getMessageName(MessageType type);

// BASE TRANSITION CODES
/*
#define STATE_STARTING   0X00
#define STATE_GETTINGFIX 0x01 
#define STATE_SURVEYING  0x02
#define STATE_OPERATING  0x03
*/

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40

enum class F3FTaskState {
	TASK_UNKNOWN,
	TASK_OUTSIDE_A,
	TASK_STARTED,
	TASK_TURN1,
	TASK_TURN2,
	TASK_TURN3,
	TASK_TURN4,
	TASK_TURN5,
	TASK_TURN6,
	TASK_TURN7,
	TASK_TURN8,
	TASK_TURN9,
	TASK_FINISHED
};


// Define ANSI color codes (disable if not supported)
#define ANSI_GREEN  "\x1b[32m"
#define ANSI_RED    "\x1b[31m"
#define ANSI_RESET  "\x1b[0m"

// === ACK codes ===
enum class AckCode : uint8_t {
	ACK_OK = 0x00,
	ACK_ERROR = 0x01,  // Optional future use
};

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

inline void encodeEventMessage(uint8_t* msg, uint8_t gliderId, MessageType event, uint8_t status) {
    msg[0] = static_cast<uint8_t>(event) | (gliderId & 0x0F);
    msg[1] = (event << 4) | (status & 0x0F);
    for (int i = 2; i < 6; ++i) msg[i] = 0;  // Reserved
}

inline void decodeEventMessage(const uint8_t* msg, uint8_t& gliderId, MessageType &event, uint8_t& status) {
    gliderId = decodeGliderId(msg[0]);
    event = static_cast<MessageType>((msg[1] >> 4) & 0x0F);
    status = msg[1] & 0x0F;
}

inline void encodeRelPosMessage(uint8_t* msg, uint8_t gliderId, int16_t n, int16_t e, int16_t d, uint8_t status) {
    if (!msg) return; // Safety check for null buffer

    msg[0] = static_cast<uint8_t>(MessageType::MSG_G2B_RELPOS) | (gliderId & 0x0F);

    const uint32_t packed = ((static_cast<uint32_t>(n & 0x3FF) << 20) |
        (static_cast<uint32_t>(e & 0x3FF) << 10) |
        (static_cast<uint32_t>(d & 0x3FF)));

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
    if (!msg) return; // Safety check for null buffer

    // Encode header with message type and glider ID
    msg[0] = static_cast<uint8_t>(MessageType::MSG_G2B_MISC) | (gliderId & 0x0F);

    // Store 4-bit data value
    msg[1] = d & 0x0F;

    // Zero out reserved bytes
    memset(&msg[2], 0, 4);
}

inline uint8_t decodeMiscMessage(const uint8_t* msg, uint8_t& gliderId, uint8_t& d) {
    gliderId = decodeGliderId(msg[0]);
    d = msg[1] & 0x0F;
    return decodeMsgType(msg[0]); // Optional: return type for confirmation
}
