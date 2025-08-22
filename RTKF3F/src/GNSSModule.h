// GNSSModule.h
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

//--------------------------------------
// Timings & fix-type constants
//--------------------------------------
#define COMMANDDELAY 1000u
#define RESETDELAY   5000u

//--------------------------------------
// FIXTYPE
//--------------------------------------
enum class FIXTYPE : uint8_t {
    NOFIX = 0,
    GPS = 1,
    DGPS = 2,
    PPS = 3,
    RTK_FLOAT = 4,
    RTK_FIX = 5,
    DEAD_RECK = 6,
    MANUAL = 7,
    SIM = 8,
    OTHER = 9
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
        UBX_ACK_NAK
    };

    struct GNSSMessage {
        GNSSMessageType type = GNSSMessageType::NONE;
        const uint8_t* data = nullptr;
        size_t          length = 0;
    };

    // -------- Parsed fix snapshot --------
    struct GNSSFix {
        // UTC time
        int   hour = 0;
        int   minute = 0;
        float second = 0;

        // Position from GGA
        float lat = 0.0f;
        float lon = 0.0f;
        float alt = 0.0f;

        // Optional RELPOSNED (if you use it elsewhere)
        float relNorth = 0.0f;
        float relEast = 0.0f;
        float relDown = 0.0f;

        float adjNorth = 0.0f;
        float adjEast = 0.0f;
        float adjDown = 0.0f;

        int SIV = 0;   // Satellites used (from GGA field 7)
        int HDOP = 0;   // HDOP * 100 (dimensionless, not "cm")
        FIXTYPE type = FIXTYPE::NOFIX;   // GGA fix quality code
    };

    // -------- Lifecycle & I/O --------
    explicit GNSSModule(HardwareSerial& serial);
    void   begin(uint32_t baud, int rx, int tx);
    bool   gpsDataAvailable();
    uint8_t    detectUARTPort();
    void   sendCommand(const String& command);
    void   sendReset();

    // -------- Unified reader & parsers --------
    GNSSMessage readGPS(); // Returns one message (NMEA/RTCM/command/UBX) if available
    bool        readNMEA(uint8_t* buffer, size_t& len);
    bool        readCommandResponse(uint8_t* buffer, size_t& len);
    bool        parseGGA(const uint8_t* buf, size_t len, GNSSFix& fix);
    void        printAscii();

    // -------- RTCM utilities --------
    uint16_t    getRTCMType(const uint8_t* buf, size_t len);
    const char* getRTCMName(uint16_t type);
    uint16_t    getRTCMBits(const uint8_t* buffer, int startBit, int bitLen);
    bool        isValidRTCM(const uint8_t* data, size_t len);

    // -------- Misc helpers --------
    void showFix(const GNSSFix& fix);
    static String   fixTypeToString(FIXTYPE fix);
    const uint8_t* getBuffer()  const { return _gpsBuf; }
    size_t          getBufLen()  const { return _gpsBufLen; }

    // ------------------------------------------
    // Nested RTCM handler (configure/log messages)
    // ------------------------------------------
    class RTCMHandler {
    public:
        struct Entry {
            const char* name;
            uint16_t    id;
            float       frequency;       // Hz
            bool        enabled;
            const char* description;     // Human-readable description
            uint32_t    txCount = 0;     // Sent counter (optional)
        };

        static constexpr size_t MAX_MSGS = 20;

        explicit RTCMHandler(GNSSModule* gnss);

        void add(const char* name, uint16_t id, float freq, bool enabled, const char* desc);

        void enable(int index, bool value);
        void setFrequency(int index, float freq);

        void enableById(uint16_t id, bool value);
        void setFrequencyById(uint16_t id, float freq);

        void sendConfig(int index);
        void sendAllConfig();

        void printList(bool showOnlyEnabled);

        int  findById(uint16_t id) const;
        int  findByName(const char* name) const;

        void getNextRTCMCount(uint16_t* rtcmId, uint32_t* count);
        void incrementSentCount(uint16_t type);

        // Expose list for status UIs, if needed
        Entry  messages[MAX_MSGS];
        size_t count = 0;

        GNSSModule* parent = nullptr;

    private:
        size_t _lastStatusIdx = 0; // Index of last iterated message
    };

    RTCMHandler rtcmHandler;

private:
    // CRC24Q is only used internally by isValidRTCM()
    uint32_t calculateCRC24Q(const uint8_t* data, size_t len);

    HardwareSerial& _serial;

    // Single shared read buffer for readGPS()/printAscii()
    uint8_t _gpsBuf[1024];
    size_t  _gpsBufLen = 0;

    // Legacy/unused placeholders kept out to avoid confusion:
    // struct RTCMMessage { const char* name; float frequencyHz; bool enabled; };
    // static constexpr int maxRTCMs = 10;
    // RTCMMessage _rtcmList[maxRTCMs];
    // int _rtcmCount = 0;
};
