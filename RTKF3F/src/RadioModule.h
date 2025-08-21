#pragma once
#include <Arduino.h>
#include <RFM69.h>
#include <RFM69registers.h>

// RadioModule.h - Header for RFM69-based radio communication module
// Notes:
// - Organized constants into logical groups with clear naming
// - Improved type safety and consistency
// - Added documentation for clarity
// - Removed unused ANSI color codes (can be re-added if needed)

class RadioModule {
public:

    // --------- Lifecycle ---------
    RadioModule(int csPin, int irqPin, bool isHighPower);

    // Initialize SPI and RFM69 radio
    bool init(int8_t pMISO, int8_t pMOSI, int8_t pSCK, uint16_t nodeId, uint8_t networkId, uint32_t frequencyHz);

    // --------- Basic I/O ---------
    RFM69& getRadio();
    void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK = false);
    bool receive(uint8_t* data, size_t& len);
    uint8_t getSenderId() const;
    uint8_t getTargetId() const;
    uint8_t getLastRSSI() const;


    // --------- Config / Debug ---------
    void setNetworkId(uint8_t networkId);
    bool checkRadioSettings(uint8_t expectedNodeId, uint8_t expectedNetworkId, uint32_t expectedFreqHz);
    void debugRFM69(RFM69& radio);

    // --------- Frequency Control ---------
    bool setFrequencyBlocking(uint32_t freqHz);
    void forceSetFrequency(uint32_t freqHz);

    // --------- Application Helpers ---------
    bool sendWithReturnFreq(uint8_t destNode, uint32_t destFreqHz, uint32_t returnFreqHz,
        const uint8_t* msg, uint8_t len);
    void sendMessageCode(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq,
        uint8_t msgType, uint8_t msgCode);

    // --------- RTCM Helpers ---------
    void sendRTCM(const uint8_t* data, size_t len);
    void sendFragmentedRTCM(const uint8_t* data, size_t len);
    void printRTCMSegment(const uint8_t* data, size_t len);

    // --------- RTCM Fragmentation / Reassembly ---------
    struct RTCM_Fragmenter {
        static constexpr size_t MAX_PAYLOAD = 57;    // Bytes per fragment payload (after header)
        static constexpr size_t MAX_TOTAL_LEN = 1024; // Max total RTCM message size

        static void sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len, uint8_t msgId);
    };

    class RTCM_Reassembler {
    public:
        RTCM_Reassembler();

        void acceptFragment(const uint8_t* data, size_t len);
        bool isComplete() const;
        const uint8_t* getData() const;
        size_t getLength() const;
        void reset();

    private:
        static constexpr size_t FRAGMENT_PAYLOAD_SIZE = 57;
        static constexpr size_t BUFFER_SIZE = 1024;

        uint8_t buffer[BUFFER_SIZE] = { 0 };
        size_t length = 0;
        uint8_t fragmentReceived[BUFFER_SIZE / FRAGMENT_PAYLOAD_SIZE + 1] = { 0 };
        uint8_t expectedMsgId = 0;
        uint8_t receivedCount = 0;
        uint8_t totalExpected = 0;
        bool complete = false;
    };

private:
    // Configuration
    const int _csPin;
    const int _irqPin;
    const bool _isHighPower;

    // State
    uint16_t _nodeId = 0;
    uint8_t _networkId = 0;
    uint8_t _currentMsgId = 0;

    // Radio instance
    RFM69 _radio;
};