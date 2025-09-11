// GNSS.h - Minimal Ring Buffer Fix
#ifndef GNSS_H
#define GNSS_H

#include <HardwareSerial.h>

// Forward declaration
class RadioModule;

// Simple Ring Buffer for UART data
class UARTBuffer {
private:
    uint8_t _data[4096];  // Fixed size array instead of dynamic allocation
    volatile size_t _head = 0;
    volatile size_t _tail = 0;
    static const size_t _size = 4096;

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

    uint8_t read() {
        if (_head == _tail) return 0;
        uint8_t byte = _data[_head];
        _head = (_head + 1) % _size;
        return byte;
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

// Keep your existing GNSSFix structure
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

class GNSSModule {
public:
    // Default constructor
    GNSSModule() : _ser(&Serial2), _ownSerial(false) {}  // Fixed: removed *
    GNSSModule(HardwareSerial& serial) : _ser(&serial), _ownSerial(false) {}  // Fixed: removed *

    bool begin(long baud, int rxPin = -1, int txPin = -1, RadioModule* radio = nullptr);

    // Keep your existing API
    void sendCommand(const char* cmd);
    bool sendWait(const char* cmd, const char* expected = "response: OK", uint32_t timeoutMs = 2000);
    bool pumpGGA(GNSSFix& fix);
    void pumpRTCM();
    void printFix(const GNSSFix& fix);
    void clearUARTBuffer();

    // Add buffer monitoring
    size_t getBufferUsage() const {
        return _buffer.count();
    }

    size_t getBufferFree() const {
        return _buffer.size() - _buffer.count();
    }
    bool verifyRTCMCRC24(uint8_t* data, size_t length);
    void sendRTCMToRadio(uint8_t* data, size_t length);

private:
    HardwareSerial* _ser;
    bool _ownSerial;
    RadioModule* _radio = nullptr;
    UARTBuffer _buffer;
    //RTCMParser* _rtcmParser = nullptr;  // Heap-allocated parser

    // Your existing private methods

    uint32_t crc24q(const uint8_t* data, size_t len);
    bool isValidRTCM(const uint8_t* frame, size_t len);
    bool parseGGA(const char* line, GNSSFix& fix);
    static bool parseLatLon(const char* dm, char hemi, float& outDeg);

    // New helper methods
    void fillBufferFromUART();
    bool findCompleteGGA(char* line, size_t maxLen);
    void logRTCMMessageType(uint16_t messageType, size_t frameSize);
};

#endif