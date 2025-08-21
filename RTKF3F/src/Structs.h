// Structs.h
#pragma once

#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>


// Message Types
enum  MessageType : uint8_t {
    MSG_RTCM = 0xD3,              // Standard RTCM message
    MSG_RTCM_FRAGMENT = 0x0A,     // Fragmented RTCM
    MSG_ARENA_SETTINGS = 0xA0,    // Arena configuration
    MSG_GLIDER_SETTINGS = 0xA1,   // Glider configuration
    MSG_REQ_POS = 0xA2,           // Request position
    MSG_GU_GPS_SETTINGS = 0xA3,   // GPS settings for GU
    MSG_INFORMATION = 0xF0,       // General information
    MSG_ERROR = 0xF1,             // Error message
    MSG_SIV = 0xF2,               // Satellites in view
	MSG_DEVICESTATE = 0xF3,       // Device status
	MSG_FIXTYPE = 0xF4,           // Fix type information
	MSG_SURVEY = 0xF5,            // Surveying status
    MSG_G2B_EVENT = 0xB1,         // Glider-to-base event
    MSG_G2B_RELPOS = 0xB2,        // Glider-to-base relative position
    MSG_G2B_MISC = 0xB3           // Miscellaneous Glider-to-base
};

enum DeviceState {
    DEVICE_STARTING = 0,
    DEVICE_GETTINGFIX = 1,
    DEVICE_SURVEYING = 2,
    DEVICE_OPERATING = 3,
    DEVICE_MENU = 4
};

// Fix types
enum  FixType : uint8_t {
    NO_FIX = 0x04,
    GPS_FIX = 0x05,
    DGPS_FIX = 0x06,
    PPS_FIX = 0x07,
    RTK_FLOAT = 0x08,
    RTK_FIX = 0x09,
    DEAD_RECKONING = 0x0A,
    MANUAL_FIX = 0x0B,
    SIM_FIX = 0x0C,
    OTHER_FIX = 0x0D
};

// Error Codes
enum  ErrorCode : uint8_t {
    ERR_RADIOINIT = 0x00,
    ERR_UART = 0x01,
    ERR_COM = 0x02,
    ERR_UNKNOWN = 0x03
};

#endif

