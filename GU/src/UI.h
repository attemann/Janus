// ======================================================
// UI.h  —  RTK Rover Web UI Header
// ======================================================
#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "RTKF3F.h"   // for GNSSFix definition

// ------------------- GLOBALS (extern only) -------------------
extern WebServer webserver;

extern const char* ap_ssid;
extern const char* ap_password;
extern IPAddress local_IP, gateway, subnet;

extern int rtcmLogIndex;

struct RTCMLogEntry {
    unsigned long time;
    uint16_t type;
    uint16_t bytes;
};

#define MAX_RTCM_LOG 20
extern RTCMLogEntry rtcmLog[MAX_RTCM_LOG];

struct WebStatus {
    unsigned long lastUpdate = 0;
    uint32_t totalRTCMMessages = 0;
    uint32_t totalRTCMBytes = 0;
    uint8_t  lastRSSI = 0;
    bool     radioConnected = false;
    String   lastError;
};

extern WebStatus webStatus;
extern GNSSFix fix;  // declared elsewhere (e.g. GNSS module)

// ------------------- FUNCTION DECLARATIONS -------------------
void handleRoot();
void handleAPIStatus();
void handleAPIPosition();
void handleAPIRTCM();
void setupWebServer();
void logRTCMMessage(uint16_t type, uint16_t bytes);

