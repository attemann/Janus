// GNSSModule.h (updated)
#pragma once
#include <Arduino.h>  
#include <HardwareSerial.h>  

#define COMMANDDELAY 100

class GNSSModule {
public:
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
    int detectUARTPort();
    bool init();
    void sendCommand(const String& command);
    void sendReset();
    bool readGNSSData(GNSSFix& fix, bool showRaw);
    bool readFix(GNSSFix& fix);
    bool readRTCM(uint8_t* buffer, size_t& len);
    const char* getRTCMName(uint16_t type);
    uint16_t getRTCMBits(const uint8_t* buffer, int startBit, int bitLen);
    static String fixTypeToString(int fixType);
    int parseField(const String& line, int num);
    void showFix(const GNSSFix& fix);
    bool parseGGA(const char* line, GNSSFix& fix);
    bool isValidRTCM(const uint8_t* data, size_t len);
    void sendConfiguredRTCMs();  // Sender alle aktiverte meldinger
    void setDefaultRTCMs();      // Setter default liste
    void printRTCMConfig();
    const RTCMMessage& getRTCM(int index) const;
    int getRTCMCount() const;
    void toggleRTCM(int index);
    void setRTCMFrequency(int index, float hz);
    uint8_t fixStatus(GNSSModule::GNSSFix fix);


private:
    HardwareSerial& _serial;
    char _nmeaBuffer[100];
    size_t _nmeaIdx;
    uint32_t calculateCRC24Q(const uint8_t* data, size_t len);
    static constexpr int maxRTCMs = 10;
    RTCMMessage _rtcmList[maxRTCMs];
    int _rtcmCount = 0;
};


