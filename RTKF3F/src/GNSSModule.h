// GNSSModule.h
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

//--------------------------------------
// Timings & fix-type constants
//--------------------------------------
#define COMMANDDELAY 500u
#define RESETDELAY   5000u

//--------------------------------------
// FIXTYPE
//--------------------------------------
enum class FIXTYPE : uint8_t {
    NOFIX     = 0,
    GPS       = 1,
    DGPS      = 2,
    PPS       = 3,
    RTK_FLOAT = 4,
    RTK_FIX   = 5,
    DEAD_RECK = 6,
    MANUAL    = 7,
    SIM       = 8,
    OTHER     = 9
};

//--------------------------------------
// GNSSModule
//--------------------------------------
class GNSSModule {
public:
    // -------- Message framing --------
    enum class GNSSMessageType {
        NONE,
        NMEA,
        RTCM,
        COMMAND_RESPONSE,
        UBX,
        UBX_RELPOSNED,
        UBX_NAV_PVT,
        UBX_ACK_ACK,
        UBX_ACK_NAK,
        NODATA
    };

    struct GNSSMessage {
        GNSSMessageType type = GNSSMessageType::NONE;
        const uint8_t*  data = nullptr;
        size_t          length = 0;
    };

    // -------- Parsed fix snapshot --------
    struct GNSSFix {
        // UTC time
        int   hour   = 0;
        int   minute = 0;
        float second = 0;

        // Position from GGA
        float lat    = 0.0f;
        float lon    = 0.0f;
        float alt    = 0.0f;

        // Optional RELPOSNED (if you use it elsewhere)
        float relNorth = 0.0f;
        float relEast  = 0.0f;
        float relDown  = 0.0f;

        float adjNorth = 0.0f;
        float adjEast  = 0.0f;
        float adjDown  = 0.0f;

        int SIV  = 0;  // Satellites used (from GGA field 7)
        int HDOP = 0;  // HDOP * 100 (dimensionless, not "cm")
        FIXTYPE type = FIXTYPE::NOFIX;   // GGA fix quality code
    };

    // -------- Lifecycle & I/O --------
    explicit GNSSModule(HardwareSerial& serial);
    void     begin(uint32_t baud, int rx, int tx);
    uint8_t  detectGPS();
    void     sendCommand(const String& command, int eatResponseTime);

    // -------- Unified reader & parsers --------
    GNSSMessage readGPS(); // Returns one message (NMEA/RTCM/command/UBX) if available
    bool        parseGGA(const uint8_t* buf, size_t len, GNSSFix& fix);
  
    // -------- RTCM utilities --------
    static uint16_t getRTCMType(const uint8_t* buf, size_t len);
    static bool     isValidRTCM(const uint8_t* data, size_t len);

private:

    HardwareSerial& _serial;

    // Single shared read buffer for readGPS()/printAscii()
    uint8_t _gpsBuf[1024];
    size_t  _gpsBufLen = 0;
};
