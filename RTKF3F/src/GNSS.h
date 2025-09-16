// GNSS.h - Minimal Ring Buffer Fix
#pragma once
#ifndef GNSS_H
#define GNSS_H

#include <HardwareSerial.h>

#define UARTBUFFERSIZE 2048 // Adjust if needed

// Forward declaration
class RadioModule;

// ---------------- UART Ring Buffer ----------------
class UARTBuffer {
private:
    uint8_t _data[UARTBUFFERSIZE];
    static const size_t _size = UARTBUFFERSIZE;
    volatile size_t _head = 0;
    volatile size_t _tail = 0;

public:
    bool write(uint8_t byte) {
        size_t next = (_tail + 1) % _size;
        if (next == _head) return false;  // Buffer full
        _data[_tail] = byte;
        _tail = next;
        return true;
    }

    bool available() const {
        return _head != _tail;
    }

    bool read(uint8_t& out) {
        if (_head == _tail) return false;
        out = _data[_head];
        _head = (_head + 1) % _size;
        return true;
    }

    // Peek uten å forbruke bytes
    bool peek(size_t index, uint8_t& out) const {
        if (index >= count()) return false;
        size_t pos = (_head + index) % _size;
        out = _data[pos];
        return true;
    }

    // Drop n bytes
    bool skip(size_t n) {
        if (n > count()) return false;
        _head = (_head + n) % _size;
        return true;
    }

    size_t count() const {
        return (_tail - _head + _size) % _size;
    }

    size_t size() const {
        return _size;
    }

    void clear() {
        _head = 0;
        _tail = 0;
    }
};

// ---------------- GNSS Fix Struct ----------------
struct GNSSFix {
    float lat = 0.0f, lon = 0.0f;
    float relNorth = 0.0f, relEast = 0.0f, relDown = 0.0f;
    float adjNorth = 0.0f, adjEast = 0.0f, adjDown = 0.0f;
    uint8_t hour = 0, minute = 0;
    float second = 0.0f;
    uint8_t SIV = 0;
    uint8_t type = 0;
    float HDOP = 0.0f;
    float elevation = 0.0f;
};

// Unicore Common Header (28 bytes)
struct UnicoreHeader {
    uint8_t sync[3];        // 0-2: AA 44 B5
    uint8_t headerLength;   // 3: Header length (28)
    uint16_t messageID;     // 4-5: Message ID (2118 for BESTNAVB)
    uint8_t messageType;    // 6: Message type
    uint8_t portAddress;    // 7: Port address
    uint16_t messageLength; // 8-9: Message length (payload only)
    uint16_t sequence;      // 10-11: Sequence number
    uint8_t idleTime;       // 12: Idle time
    uint8_t timeStatus;     // 13: Time status
    uint16_t week;          // 14-15: GPS week number
    uint32_t milliseconds;  // 16-19: Milliseconds into week (but stored as 4 bytes from 16-19)
    uint32_t receiverStatus;// 20-23: Receiver status
    uint16_t reserved;      // 24-25: Reserved
    uint16_t receiverVersion; // 26-27: Receiver software version
} __attribute__((packed));

// BESTNAVB Message Payload (72 bytes)
struct BestNavBPayload {
    uint32_t solStat;       // 0-3: Solution status
    uint32_t posType;       // 4-7: Position type
    double lat;             // 8-15: Latitude (degrees)
    double lon;             // 16-23: Longitude (degrees)  
    double hgt;             // 24-31: Height above MSL (meters)
    float undulation;       // 32-35: Undulation (meters)
    uint32_t datumID;       // 36-39: Datum ID
    float latStdDev;        // 40-43: Latitude standard deviation (meters)
    float lonStdDev;        // 44-47: Longitude standard deviation (meters)
    float hgtStdDev;        // 48-51: Height standard deviation (meters)
    uint8_t stnID[4];       // 52-55: Base station ID
    float diffAge;          // 56-59: Differential age (seconds)
    float solAge;           // 60-63: Solution age (seconds)
    uint8_t numObs;         // 64: Number of observations
    uint8_t numL1;          // 65: Number of L1 observations
    uint8_t numL1L2;        // 66: Number of L1/L2 observations
    uint8_t reserved;       // 67: Reserved
    uint8_t extSolStat;     // 68: Extended solution status
    uint8_t galBdsSignals;  // 69: Galileo/BDS signals mask
    uint8_t gpsGloSignals;  // 70: GPS/GLONASS signals mask
    uint8_t reserved2;      // 71: Reserved
} __attribute__((packed));

// Complete BESTNAVB Message
struct BestNavBMessage {
    UnicoreHeader header;
    BestNavBPayload payload;
    uint32_t crc32;
} __attribute__((packed));

// ---------------- GNSS Module ----------------
class GNSSModule {
public:
    GNSSModule() : _ser(&Serial2) {}
    GNSSModule(HardwareSerial& serial) : _ser(&serial) {}

    bool begin(long baud, int rxPin = -1, int txPin = -1);
    void setRadio(RadioModule* radio);

    void sendCommand(const char* cmd);
    bool sendWait(const char* cmd, const char* expected = "response: OK", uint32_t timeoutMs = 2000);

    bool pumpGGA(GNSSFix& fix);
    bool pumpBestNavA(GNSSFix& fix);
    void pumpRTCM();
    uint32_t crc32(const uint8_t* data, size_t len);

    void printFix(const GNSSFix& fix);
    void clearUARTBuffer();

    // Buffer monitoring
    size_t getBufferUsage() const { return _buffer.count(); }
    size_t getBufferFree() const { return _buffer.size() - _buffer.count(); }

    // RTCM utilities
    static uint16_t getRTCMType(const uint8_t* data, size_t length);
    bool verifyRTCMCRC24(uint8_t* data, size_t length);
    void sendRTCMToRadio(uint8_t* data, size_t length);
    bool isValidRTCM(const uint8_t* frame, size_t len);

private:
    HardwareSerial* _ser;
    RadioModule* _radio = nullptr;
    UARTBuffer _buffer;

    // Parsing helpers
    uint32_t crc24q(const uint8_t* data, size_t len);
    bool parseGGA(const char* line, GNSSFix& fix);
    static bool parseLatLon(const char* dm, char hemi, float& outDeg);

    // Line/frame helpers
    void fillBufferFromUART();
    bool findCompleteGGA(char* line, size_t maxLen);
    bool findCompleteBestNavA(char* line, size_t maxLen);
    bool parseBestNavA(const char* line, GNSSFix& fix);
    //void logRTCMMessageType(uint16_t messageType, size_t frameSize);
};

#endif
