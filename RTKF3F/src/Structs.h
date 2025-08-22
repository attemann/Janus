// Structs.h
#pragma once

#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

// Message Types
enum MessageType : uint8_t {
    // --- Core data (0x00–0x1F) ---
    MSG_RTCM = 0xD3,              // Standard RTCM message
    MSG_RTCM_FRAGMENT = 0x0A,     // Fragmented RTCM

    // --- Configuration & requests (0x20–0x3F) ---
    MSG_ARENA_SETTINGS = 0x20,    // Arena configuration
    MSG_GLIDER_SETTINGS = 0x21,   // Glider configuration
    MSG_REQ_POS = 0x22,           // Request position
    MSG_GU_GPS_SETTINGS = 0x23,   // GPS settings for GU

    // --- Status & info (0x40–0x5F) ---
    MSG_INFORMATION = 0x40,       // General information
    MSG_ERROR = 0x41,             // Error message
    MSG_SIV = 0x42,               // Satellites in view
    MSG_DEVICESTATE = 0x43,       // Device status
    MSG_FIXTYPE = 0x44,           // Fix type information
    MSG_SURVEY = 0x45,            // Surveying status

    // --- Glider-to-base messages (0x60–0x6F) ---
    MSG_G2B_EVENT = 0x60,         // Glider-to-base event
    MSG_G2B_RELPOS = 0x61,        // Glider-to-base relative position
    MSG_G2B_MISC = 0x62,          // Miscellaneous Glider-to-base

    // --- Event system (0x80–0x9F) ---
    MSG_EVT_NONE = 0x80,          // Reserved / no event
    MSG_EVT_CROSS_A_IN = 0x81,    // Cross A base into task
    MSG_EVT_CROSS_A_OUT = 0x82,   // Cross A base out of task
    MSG_EVT_CROSS_B_IN = 0x83,    // Cross B base into task
    MSG_EVT_CROSS_B_OUT = 0x84,   // Cross B base out of task
    MSG_EVT_SAFETY_IN_TO = 0x85,  // Cross into safety area (danger zone)
    MSG_EVT_SAFETY_OUT_OF = 0x86, // Cross out of safety area (safe zone)
    MSG_EVT_AIRBORNE = 0x87,      // Glider airborne (takeoff)
    MSG_EVT_LANDED = 0x88,        // Glider landed
    MSG_EVT_ACK = 0x89            // Acknowledgment of flight settings
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
	NO_FIX     = 0x04, 
	GPS_FIX    = 0x05, 
	DGPS_FIX   = 0x06, 
	PPS_FIX    = 0x07, 
	RTK_FLOAT  = 0x08, 
	RTK_FIX    = 0x09,
	DEAD_RECK  = 0x0A, 
	MANUAL_FIX = 0x0B, 
	SIM_FIX    = 0x0C, 
	OTHER_FIX  = 0x0D 
};

// String table (indexes match FIXTYPE values)
static constexpr const char* kFixTypeName[] = {
  "None",
  "GPS",
  "DGPS",
  "PPS",
  "RTK Float",
  "RTK Fixed",
  "Dead Reckoning",
  "Manual",
  "Simulation",
  "Other"
};

// Minimal helpers
inline const char* fixTypeName(FixType t) {
    const unsigned i = static_cast<unsigned>(t);
    return (i < (sizeof(kFixTypeName) / sizeof(kFixTypeName[0])))
        ? kFixTypeName[i]
        : "Unknown";
}

inline String fixTypeToString(FixType t) { return String(fixTypeName(t)); }

// Error Codes
enum  ErrorCode : uint8_t {
    ERR_RADIOINIT = 0x00,
    ERR_UART = 0x01,
    ERR_UNKNOWN = 0x02
};

#endif

