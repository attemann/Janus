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
    void pumpRTCM();

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
    void logRTCMMessageType(uint16_t messageType, size_t frameSize);
};

#endif
