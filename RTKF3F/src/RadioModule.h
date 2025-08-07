//RadioModule.h
#pragma once
#include <Arduino.h>
#include <RFM69.h>

#define REG_BITRATEMSB 0x03
#define REG_BITRATELSB 0x04

class RadioModule {
public:
    struct HWPins {
        int sck   = -1;
        int miso  = -1;
        int mosi  = -1;
        int cs    = -1;
        int irq   = -1;
        int reset = -1;
		int irqn  = -1;  
	};

    RadioModule(RFM69& radio);
    bool init(RadioModule::HWPins pins, int nodeid, int networkid, int frequency);
    bool verify();
    void sendMessageCode(int destNode, int destFreq, int returnFreq, int msgType, int msgCode);
    void sendRTCM(const uint8_t* data, size_t len);
    void sendRTCMTest(int len);
    void printRTCMSegment(const uint8_t* data, size_t len);
    void sendFragmentedRTCM(const uint8_t* data, size_t len);
    bool receive(uint8_t*& data, uint8_t& len);
    void sendWithReturnFreq(uint8_t destNode, int destFreq, int returnFreq, const uint8_t* msg, uint8_t len);
	int getSenderId();
	int getTargetId();	
	void setBitrate(uint16_t bitrate);
    uint16_t getBitrate();

    // Nested RTCM Fragmenter class
    class RTCM_Fragmenter {
    public:
        static const uint8_t MAX_PAYLOAD = 61;
        static const uint16_t MAX_TOTAL_LEN = 1029;

        static void sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len, uint8_t msgId);
    };

    // Nested RTCM Reassembler class
    class RTCM_Reassembler {
    public:
        static const uint8_t MAX_FRAGMENTS = 10;
        static const uint16_t MAX_TOTAL_LEN = 610;

        RTCM_Reassembler();
        void acceptFragment(const uint8_t* data, size_t len);
        bool isComplete() const;
        const uint8_t* getData() const;
        size_t getLength() const;
        void reset();
        uint8_t getReceivedCount() const { return receivedCount; }

    private:
        uint8_t buffer[MAX_TOTAL_LEN];
        uint8_t fragmentReceived[MAX_FRAGMENTS];
        uint8_t receivedCount;
        uint8_t totalExpected;
        size_t length;
        bool complete;
        uint8_t expectedMsgId = 0xFF; // 0xFF means not initialized
        static constexpr size_t FRAGMENT_HEADER_SIZE = 4;
        static constexpr size_t FRAGMENT_PAYLOAD_SIZE = 61 - FRAGMENT_HEADER_SIZE; // = 57
 
    };

private:
    RFM69& _radio;
    const uint8_t expectedVersion = 0x24;
    void setBitrate(RFM69& radio, uint16_t bitrate);

    uint32_t calculateCRC24Q(const uint8_t* data, size_t len);
    uint8_t currentMsgId = 0;

};