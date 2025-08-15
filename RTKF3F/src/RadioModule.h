//RadioModule.h
#pragma once
#include <Arduino.h>
#include <RFM69.h>
#include <RFM69registers.h>

#define NODEID_RTKBASE   1
#define NODEID_CD		 2
#define NODEID_GU		 3
#define NETWORK_ID		 100

#define FREQUENCY_RTCM 868100000
#define FREQUENCY_CD   868200000

// Define ANSI color codes (disable if not supported)
#define ANSI_GREEN  "\x1b[32m"
#define ANSI_RED    "\x1b[31m"
#define ANSI_RESET  "\x1b[0m"

// Message codes (all unique, hex format)
#define MSG_RTCM                0xD3  // Standard RTCM message
#define MSG_RTCM_FRAGMENT       0x0A  // Fragmented RTCM

#define MSG_ARENA_SETTINGS      0xA0
#define MSG_GLIDER_SETTINGS     0xA1
#define MSG_REQ_POS             0xA2
#define MSG_GU_GPS_SETTINGS     0xA3

#define MSG_INFORMATION         0xF0
#define MSG_ERROR               0xF1
#define MSG_SIV                 0xF2

#define MSG_TYPE_G2B_EVENT      0xB1
#define MSG_TYPE_G2B_RELPOS     0xB2
#define MSG_TYPE_G2B_MISC       0xB3

// BASE TRANSITION CODES
#define INFO_TRANSITION_GETTINGFIX 0x01 
#define INFO_TRANSITION_SURVEYING  0x02
#define INFO_TRANSITION_OPERATING  0x03
#define INFO_FIX_NOFIX	           0x04
#define INFO_FIX_GPS	           0x05
#define INFO_FIX_DGPS			   0x06
#define INFO_FIX_PPS		       0x07
#define INFO_FIX_RTK_FLOAT		   0x08
#define INFO_FIX_RTK_FIX		   0x09
#define INFO_FIX_DEAD_RECKONING    0x0A
#define INFO_FIX_MANUAL        	   0x0B
#define INFO_FIX_SIM		       0x0C
#define INFO_FIX_OTHER		       0x0D
#define INFO_DEVICE_STARTING       0x10

// Radio errors
#define ERROR_RADIO_INIT        0x00
#define ERROR_RADIO_VERIFY      0x01

// GNSS errors
#define ERROR_UART              0x10
#define ERROR_COM               0x11

// Unknown error code
#define ERROR_UNKNOWN           0x63  // 99 decimal = 0x63

class RadioModule {
public:
    // ---- Lifecycle ----
    RadioModule(int csPin, int irqPin, bool isHighPower);

    // Initialize SPI + RFM69
    bool init(int8_t pMISO,
        int8_t pMOSI,
        int8_t pSCK,
        uint16_t nodeid,
        uint8_t networkid,
        uint32_t frequencyHz);

    // ---- Basic I/O ----
    RFM69& getRadio();
    void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
    bool receive(uint8_t* data, size_t& len);
    int  getSenderId();
    int  getTargetId();

    // ---- Config / Debug ----
    void joeSetNetwork(uint8_t networkid);
    bool checkRadioSettings(uint8_t expectedNodeId, uint8_t expectedNetworkId, uint32_t expectedFreqHz);
    void debugRFM69(RFM69& radio);

    // ---- Frequency control ----
    bool setFrequencyBlocking(uint32_t freqHz);
    void forceSetFrequency(uint32_t freqHz);

    // ---- App helpers ----
    bool sendWithReturnFreq(uint8_t destNode,
        uint32_t destFreqHz,
        uint32_t returnFreqHz,
        const uint8_t* msg,
        uint8_t len);

    void sendMessageCode(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq,
        uint8_t msgType, uint8_t msgCode);

    // ---- RTCM helpers ----
    void sendRTCM(const uint8_t* data, size_t len);
    void sendFragmentedRTCM(const uint8_t* data, size_t len);
    void sendRTCMTest(int len);
    void printRTCMSegment(const uint8_t* data, size_t len);

    // --------- Fragmentation / Reassembly ---------
    struct RTCM_Fragmenter {
        static constexpr size_t MAX_PAYLOAD = 57;    // bytes per fragment payload (after 4‑byte header)
        static constexpr size_t MAX_TOTAL_LEN = 1024;  // safety cap for one RTCM logical message

        static void sendFragmented(RFM69& radio,
            uint8_t destId,
            const uint8_t* data,
            size_t len,
            uint8_t msgId);
    };

    class RTCM_Reassembler {
    public:
        RTCM_Reassembler();

        void   acceptFragment(const uint8_t* data, size_t len);
        bool   isComplete() const;
        const uint8_t* getData() const;
        size_t getLength() const;
        void   reset();

    private:
        static constexpr size_t FRAGMENT_PAYLOAD_SIZE = 57;
        static constexpr size_t BUFFER_SIZE = 1024;   // total reassembled cap

        uint8_t  buffer[BUFFER_SIZE] = { 0 };
        size_t   length = 0;

        uint8_t  fragmentReceived[BUFFER_SIZE / FRAGMENT_PAYLOAD_SIZE + 1] = { 0 };
        uint8_t  expectedMsgId = 0;
        uint8_t  receivedCount = 0;
        uint8_t  totalExpected = 0;
        bool     complete = false;
    };

private:
    // Pins / config
    int   _csPin;
    int   _irqPin;
    bool  _isHighPower;

    // State
    uint16_t _nodeid = 0;
    uint8_t  _networkid = 0;
    uint8_t  currentMsgId = 0;

    // Radio instance (LowPowerLabs compatible)
    RFM69 _radio;
};
