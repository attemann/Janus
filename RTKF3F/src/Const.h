//Const.h
#pragma once
#include <Arduino.h>

    // --------- Constants ---------
    // Node IDs
static constexpr uint8_t  NODEID_RTKBASE = 1;
static constexpr uint8_t  NODEID_CD = 2;
static constexpr uint8_t  NODEID_GU = 3;
static constexpr uint16_t NETWORK_ID = 100;

// Frequencies (Hz)
static constexpr uint32_t FREQUENCY_RTCM = 868100000;
static constexpr uint32_t FREQUENCY_CD = 868200000;

#define MAX_GU_UNITS 5
#define SLOPELENGTH  100
#define SURVEYINTIME 120000

// Const for airborne detection
#define DETECTOR_BUFFER_SIZE 30  // 3 secs with 0.1s interval
#define THRESHOLD_AIRBORNE 9.0f  // 3 m/s avg over 3 secs
#define THRESHOLD_LANDED 1.0f    // 1 m/s avg over 3 secs

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40
