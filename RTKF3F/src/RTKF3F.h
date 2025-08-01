//RTKF3F.h

#pragma once

#ifndef RTKF3F_H
#define RTKF3F_H

#include <HardwareSerial.h>
#include <RFM69.h>

#include "MessageTypes.h"
#include "Slope.h"

#define MAX_GU_UNITS 5

#define SLOPELENGTH 100

#define MSG_RTCM              0x01  // RTCM3 correction stream
#define MSG_GU_GPSSETTINGS    0x02  // GUGPS settings
#define MSG_FLIGHT_SETTINGS   0x03  // Settings for the slope
#define MSG_REQ_POS           0x04  // BS request to GU for position

#define RTCM_TX_FREQ 868100000 
#define GU_TX_FREQ   868200000

#define NODEID_RTKBASE   1
#define NODEID_BS		 2
#define NODEID_GU		 3
#define NETWORK_ID		 100
// Const for airborne detection
#define DETECTOR_BUFFER_SIZE 30           // 3 secs with 0.1s interval
#define THRESHOLD_AIRBORNE 9.0f  // 3 m/s avg over 3 secs
#define THRESHOLD_LANDED 1.0f    // 1 m/s avg over 3 secs

extern HardwareSerial SerialGNSS;  // Declaration only
extern RFM69 radio;  // Declaration only

struct GNSSFix {
	// ⬅️ Add these for time parsing and printing
	int hour = 0;
	int minute = 0;
	float second = 0;

	// From GNGGA
	float lat;         // Decimal degrees
	float lon;         // Decimal degrees
	float alt;         // Altitude in meters

	// From UBX-RELPOPNED (optional/legacy)
	float relNorth;    // Relative North (m)
	float relEast;     // Relative East (m)
	float relDown;     // Relative Down (m)

	float adjNorth;    // Adjusted North (m), relative to pilot or reference
	float adjEast;
	float adjDown;

	int numSV = 0;      // Satellites in view
	int HDOP = 0;        // Horizontal Dilution of Precision (HDOP) in cm
	int fixType = 0;    // Raw fix type

	// Fix flags
	bool gpsFix;       // True if GNSS fix is valid (GGA fix quality > 0)
	bool diffUsed;     // Differential corrections used (GGA fix quality >= 2)
	bool rtkFloat;     // RTK float (GGA fix quality == 5)
	bool rtkFix;       // RTK fixed (GGA fix quality == 4)
};

void monitorSerial(HardwareSerial& gnssSerial, String c,  int wait);
int detectUM980Port(HardwareSerial& gnssSerial);
const char* getRTCMMessageName(uint16_t type);
uint16_t getBits(const uint8_t* buffer, int startBit, int bitLen);
bool readGNSSData(GNSSFix& fix);
bool verifyRadio(RFM69& radio);
void updateRTCMForwarder();
bool isValidRTCM(const uint8_t* rtcm, size_t totalLen);

enum class BSState {
	BS_WAITING,
	BS_ONGROUND,
	BS_AIRBORNE,
	BS_SEL_GLIDER,
	BS_SEL_A_RIGHT,
	BS_SEL_SLOPEANGLE,
	BS_SET_PLOC
};

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

class RTCM_Fragmenter {
public:
	static const uint8_t MAX_PAYLOAD = 61; // Safe max per RFM69
	static const uint8_t MAX_TOTAL_LEN = 255; // 8-bit length

	// Send a long RTCM message, broken into chunks
	static void sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len) {
		if (len > MAX_TOTAL_LEN) return;  // too big

		uint8_t packet[MAX_PAYLOAD];
		uint8_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

		for (uint8_t i = 0; i < totalChunks; ++i) {
			size_t offset = i * MAX_PAYLOAD;
			size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);

			packet[0] = 0xA0;              // Fragment marker
			packet[1] = totalChunks;      // Total
			packet[2] = i;                // Index

			memcpy(packet + 3, data + offset, chunkLen);
			radio.send(destId, packet, chunkLen + 3);
			delay(5);  // throttle
		}
	}
};

class RTCM_Reassembler {
public:
	static const uint8_t MAX_FRAGMENTS = 10;
	static const uint16_t MAX_TOTAL_LEN = 610; // MAX_FRAGMENTS * MAX_PAYLOAD

	RTCM_Reassembler() : receivedCount(0), totalExpected(0), complete(false) {
		memset(fragmentReceived, 0, sizeof(fragmentReceived));
	}

	// Feed each incoming fragment here
	void acceptFragment(const uint8_t* data, size_t len) {
		if (len < 4 || data[0] != 0xA0) return; // invalid

		uint8_t total = data[1];
		uint8_t index = data[2];
		if (total > MAX_FRAGMENTS || index >= total) return;

		if (index == 0) {
			totalExpected = total;
			receivedCount = 0;
			complete = false;
			memset(fragmentReceived, 0, sizeof(fragmentReceived));
		}

		size_t fragLen = len - 3;
		memcpy(buffer + index * RTCM_Fragmenter::MAX_PAYLOAD, data + 3, fragLen);
		fragmentReceived[index] = fragLen;
		receivedCount++;

		if (receivedCount == totalExpected) {
			size_t totalLen = 0;
			for (uint8_t i = 0; i < totalExpected; ++i) {
				totalLen += fragmentReceived[i];
			}
			this->length = totalLen;
			complete = true;
		}
	}

	bool isComplete() const { return complete; }
	const uint8_t* getData() const { return buffer; }
	size_t getLength() const { return length; }
	void reset() { complete = false; receivedCount = 0; }

private:
	uint8_t buffer[MAX_TOTAL_LEN];
	uint8_t fragmentReceived[MAX_FRAGMENTS];
	uint8_t receivedCount;
	uint8_t totalExpected;
	size_t length;
	bool complete;
};

#endif
