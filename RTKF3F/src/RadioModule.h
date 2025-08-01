//RadioModule.h
#pragma once
#include <Arduino.h>
#include <RFM69.h>

class RadioModule {
public:
    struct HWPins {
        int sck = -1;
        int miso = -1;
        int mosi = -1;
        int cs = -1;
        int irq = -1;
        int reset = -1;
	};

    RadioModule(RFM69& radio);
    bool init(RadioModule::HWPins pins, int nodeid, int networkid, int frequency);
    bool verify();
    void sendRTCM(const uint8_t* data, size_t len);
    void sendFragmentedRTCM(const uint8_t* data, size_t len);
    bool receive(uint8_t*& data, uint8_t& len);
    void sendWithReturnFreq(uint8_t destNode, int destFreq, int returnFreq, const uint8_t* msg, uint8_t len);

    // Nested RTCM Fragmenter class
    class RTCM_Fragmenter {
    public:
        static const uint8_t MAX_PAYLOAD = 61;
        static const uint8_t MAX_TOTAL_LEN = 255;

        static void sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len);
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


    private:
        uint8_t buffer[MAX_TOTAL_LEN];
        uint8_t fragmentReceived[MAX_FRAGMENTS];
        uint8_t receivedCount;
        uint8_t totalExpected;
        size_t length;
        bool complete;
    };

private:
    RFM69& _radio;
    const uint8_t expectedVersion = 0x24;
};