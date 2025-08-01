#pragma once
#include <Arduino.h>
#include <RFM69.h>

class RadioModule {
public:
    RadioModule(RFM69& radio);
    bool init(int sck, int miso, int mosi, int cs, int irq, int reset);
    bool verify();
    void sendRTCM(const uint8_t* data, size_t len);
    void sendFragmentedRTCM(const uint8_t* data, size_t len);

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