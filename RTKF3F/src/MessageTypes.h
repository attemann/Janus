// MessageTypes.h
#pragma once

#ifndef MESSAGETYPES_H
#define MESSAGETYPES_H


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
    MSG_G2B_EVENT = 0xB1,         // Glider-to-base event
    MSG_G2B_RELPOS = 0xB2,        // Glider-to-base relative position
    MSG_G2B_MISC = 0xB3           // Miscellaneous Glider-to-base
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
    ERROR_RADIO_INIT = 0x00,
    ERROR_RADIO_VERIFY = 0x01,
    ERROR_UART = 0x10,
    ERROR_COM = 0x11,
    ERROR_UNKNOWN = 0x63
};

#endif

