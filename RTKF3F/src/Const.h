//Const.h
#pragma once

#ifndef CONST_H
#define CONST_H

	#define MAX_GU_UNITS 5

	#define BTN_MNU 32
	#define BTN_INC 25
	#define BTN_DEC 33
	#define BTN_ESC 26

	#define SLOPELENGTH 100

	#define MSG_RTCM              0x01  // RTCM3 correction stream
	#define MSG_GU_GPSSETTINGS    0x02  // GUGPS settings
	#define MSG_FLIGHT_SETTINGS   0x03  // Settings for the slope
	#define MSG_REQ_POS           0x04  // BS request to GU for position

	#define BS_TX_FREQ 868100000
	#define BS_RX_FREQ 868200000

	#define BASE_NODE_ID   1
	#define NODE_ID_INIT   2
	#define NETWORK_ID   100

	#define RTCM_INTERVAL 1000 // 1 second

	// Const for airborne detection
	#define DETECTOR_BUFFER_SIZE 30           // 3 secs with 0.1s interval
	#define THRESHOLD_AIRBORNE 9.0f  // 3 m/s avg over 3 secs
	#define THRESHOLD_LANDED 1.0f    // 1 m/s avg over 3 secs

#endif