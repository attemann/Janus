//RTKF3F.h

#pragma once

#ifndef RTKF3F_H
#define RTKF3F_H

#include "GNSSModule.h"
#include "RadioModule.h"

#include "MessageTypes.h"
#include "Slope.h"

#define MAX_GU_UNITS 5

#define SLOPELENGTH 100


#define MSG_RTCM              0xD3  // Full RTCM message	
#define MSG_RTCMFRAGMENT      0x0A  // Fragmented RTCM

#define MSG_GU_GPSSETTINGS    0x02  // GUGPS settings
#define MSG_FLIGHT_SETTINGS   0x03  // Settings for the slope
#define MSG_REQ_POS           0x04  // BS request to GU for position

#define RTCM_TX_FREQ 868100000 
#define GU_TX_FREQ   868200000

#define NODEID_RTKBASE   1
#define NODEID_BS		 2
#define NODEID_GU		 3
#define NETWORK_ID		 100

// MISC
#define SURVEYINTIME 15


// Const for airborne detection
#define DETECTOR_BUFFER_SIZE 30           // 3 secs with 0.1s interval
#define THRESHOLD_AIRBORNE 9.0f  // 3 m/s avg over 3 secs
#define THRESHOLD_LANDED 1.0f    // 1 m/s avg over 3 secs

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40


enum class BSTaskState {
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
constexpr uint8_t STATUS_LINK_OK = 0x01;  // Or define separately if needed
constexpr uint8_t STATUS_RTK_FLOAT = 0x02;
constexpr uint8_t STATUS_RTK_FIX = 0x04;
constexpr uint8_t STATUS_GPS_FIX = 0x08;
constexpr uint8_t STATUS_DGNSS_USED = 0x10;

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

#endif
