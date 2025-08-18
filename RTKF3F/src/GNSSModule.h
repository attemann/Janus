// GNSSModule.h (updated)
#pragma once
#include <Arduino.h>  
#include <HardwareSerial.h>  

#define COMMANDDELAY 100

class GNSSModule {
public:

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
        size_t length = 0;
    };

    struct GNSSFix {
        // UTC time
        int hour = 0;
        int minute = 0;
        float second = 0;

        // Position from GGA
        float lat = 0.0f;
        float lon = 0.0f;
        float alt = 0.0f;

        // Optional: RELPOSNED
        float relNorth = 0.0f;
        float relEast = 0.0f;
        float relDown = 0.0f;

        float adjNorth = 0.0f;
        float adjEast = 0.0f;
        float adjDown = 0.0f;

        int SIV = 0;     // Satelites in view
        int HDOP = 0;      // HDOP in cm
        int fixType = 0;   // GGA fix quality

        // Flags
        bool gpsFix = false;
        bool diffUsed = false;
        bool rtkFloat = false;
        bool rtkFix = false;
    };

    struct RTCMMessage {
        const char* name;
        float frequencyHz;
        bool enabled;
    };

    GNSSModule(HardwareSerial& serial);
    void begin(uint32_t baud,int rx, int tx);
	bool gpsDataAvailable();
    int detectUARTPort();
    void sendCommand(const String& command);
    void sendReset();
    bool parseGGA(const uint8_t* buf, size_t len, GNSSFix& fix);
    void printAscii();
    bool readRTCM(uint8_t* buffer, size_t& len);
    bool readNMEA();
    bool readCommandResponse();
    uint16_t getRTCMType(const uint8_t* buf, size_t len);
    const char* getRTCMName(uint16_t type);
    uint16_t getRTCMBits(const uint8_t* buffer, int startBit, int bitLen);
    static String fixTypeToString(int fixType);
    int parseField(const String& line, int num);
    void showFix(const GNSSFix& fix);
    bool isValidRTCM(const uint8_t* data, size_t len);
    const uint8_t* getBuffer() const { return _gpsBuf; }
    size_t getBufLen() const { return _gpsBufLen; }
    GNSSMessage readGPS();

    class RTCMHandler {
    public:
        struct Entry {
            const char* name;
            uint16_t id;
            float frequency;
            bool enabled;
            const char* description;  // Human-readable description
            uint32_t txCount = 0;     // Transmit counter (optional)
        };

        static constexpr size_t MAX_MSGS = 20;
        Entry messages[MAX_MSGS];
        size_t count = 0;

        GNSSModule* parent = nullptr;

        RTCMHandler(GNSSModule* gnss);

        void add(const char* name, uint16_t id, float freq, bool enabled, const char* desc);

        void enable(int index, bool value);
        void setFrequency(int index, float freq);

        void enableById(uint16_t id, bool value);
        void setFrequencyById(uint16_t id, float freq);

        void sendConfig(int index);
        void sendAllConfig();

        void printList(bool showOnlyEnabled);

        int findById(uint16_t id) const;
        int findByName(const char* name) const;
        void getNextRTCMCount(uint16_t* rtcmId, uint32_t* count);
        void incrementSentCount(uint16_t type);

    private:
        size_t _lastStatusIdx = 0; // Index of last shown message
    };

    RTCMHandler rtcmHandler;

private:
    HardwareSerial& _serial;
    char _nmeaBuffer[100];
    size_t _nmeaIdx;
    uint32_t calculateCRC24Q(const uint8_t* data, size_t len);
    static constexpr int maxRTCMs = 10;
    RTCMMessage _rtcmList[maxRTCMs];
    int _rtcmCount = 0;
    uint8_t _gpsBuf[128];
    size_t _gpsBufLen = 0;
};


