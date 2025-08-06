#pragma once

//RTKF3F.h

#include "GNSSModule.h"
#include "RadioModule.h"

#include "Slope.h"

#define MAX_GU_UNITS 5
#define SLOPELENGTH 100
#define SURVEYINTIME 15

// Const for airborne detection
#define DETECTOR_BUFFER_SIZE 30  // 3 secs with 0.1s interval
#define THRESHOLD_AIRBORNE 9.0f  // 3 m/s avg over 3 secs
#define THRESHOLD_LANDED 1.0f    // 1 m/s avg over 3 secs

// Message codes
#define	MSG_RTCM             0xD3
#define	MSG_RTCMFRAGMENT     0x0A
#define	MSG_RTCM_NUMSENT     0x0B

#define	MSG_FLIGHT_SETTINGS  0xA0
#define	MSG_REQ_POS          0xA1
#define	MSG_GU_GPSSETTINGS   0xA2

#define MSG_INFORMATION      0xF0
#define MSG_ERROR		     0xF1
#define MSG_SIV			     0xF2

#define MSG_TYPE_G2B_EVENT   0xB1
#define MSG_TYPE_G2B_RELPOS  0xB2
#define MSG_TYPE_G2B_MISC    0xB3

// Radio errors
#define ERROR_RADIO_INIT     0
#define ERROR_RADIO_VERIFY   1

// GNSS errors
#define ERROR_UART           10
#define ERROR_COM            11

// Unknown error code
#define ERROR_UNKNOWN        99 

// INFORMATION MESSGAGE CODES
// BASE TRANSITION CODES
#define INFO_TRANSITION_GETTINGFIX 0x01 
#define INFO_TRANSITION_SURVEYING  0x02
#define INFO_TRANSITION_OPERATING  0x03
#define INFO_FIX_NOFIX	           0x04  
#define INFO_FIX_GPS	           0x05
#define INFO_FIX_DGPS			   0x06
#define INFO_FIX_PPS		       0x07
#define INFO_FIX_RTK_FLOAT		   0x08
#define INFO_FIX_RTK_FIX		   0x09
#define INFO_FIX_DEAD_RECKONING    0x0A
#define INFO_FIX_MANUAL        	   0x0B
#define INFO_FIX_SIM		       0x0C
#define INFO_FIX_OTHER		       0x0D
#define INFO_DEVICE_STARTING       0x10

#define RTCM_TX_FREQ 868100000 
#define GU_TX_FREQ   868200000

#define NODEID_RTKBASE   1
#define NODEID_CD		 2
#define NODEID_GU		 3
#define NETWORK_ID		 100

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40

enum class F3FTaskState {
	TASK_UNKNOWN,
	//  TASK_INSIDE_A,
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

// === Event codes (Bits 7–4 of Byte 1) ===
// Events sent from Glider Unit (GU) to Base Station (BS)
enum class EventCode : uint8_t {
	EVT_NONE = 0x0,  // Reserved / no event
	EVT_CROSS_A_IN = 0x1,  // Cross A base into task
	EVT_CROSS_A_OUT = 0x2,  // Cross A base out of task
	EVT_CROSS_B_IN = 0x3,  // Cross B base into task
	EVT_CROSS_B_OUT = 0x4,  // Cross B base out of task
	EVT_SAFETY_IN_TO = 0x5,  // Cross into safety area (danger zone)
	EVT_SAFETY_OUT_OF = 0x6,  // Cross out of safety area (safe zone)
	EVT_AIRBORNE = 0x7,  // Glider airborne (takeoff)
	EVT_LANDED = 0x8,  // Glider landed
	EVT_ACK = 0x9,  // Acknowledgment of flight settings
};

// === Status bit flags (Bits 3–0 of Byte 1 + bit 4 in Byte 2) ===
//constexpr uint8_t STATUS_LINK_OK = 0x01;  // Or define separately if needed
//constexpr uint8_t STATUS_RTK_FLOAT = 0x02;
//constexpr uint8_t STATUS_RTK_FIX = 0x04;
//constexpr uint8_t STATUS_GPS_FIX = 0x08;
//constexpr uint8_t STATUS_DGNSS_USED = 0x10;

// === Packet format for messages from GU to BS ===
// Byte 0: GLIDER_ID       (uint8_t, 0–255)
// Byte 1: EVENT_STATUS    ((event << 4) | (status & 0x0F))
// Byte 2: STATUS_DGNSS_USED (bit 4) + TIMESTAMP[23:17] (7 bits)
// Byte 3: TIMESTAMP[16:9]
// Byte 4: TIMESTAMP[8:0]

// === Packet format for ACK message from GU to BS ===
// Byte 0: GLIDER_ID
// Byte 1: EVENT_STATUS (EVENT = EVT_ACK, status as needed)
// Byte 2: ACK_CODE (e.g., 0x00 = OK)
// Byte 3: RESERVED = 0x00
// Byte 4: RESERVED = 0x00

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

inline void encodeEventMessage(uint8_t* msg, uint8_t gliderId, uint8_t event, uint8_t status) {
    msg[0] = encodeHeader(MSG_TYPE_G2B_EVENT, gliderId);
    msg[1] = (event << 4) | (status & 0x0F);
    for (int i = 2; i < 6; ++i) msg[i] = 0;  // Reserved
}

inline void decodeEventMessage(const uint8_t* msg, uint8_t& gliderId, uint8_t& event, uint8_t& status) {
    gliderId = decodeGliderId(msg[0]);
    event = (msg[1] >> 4) & 0x0F;
    status = msg[1] & 0x0F;
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
