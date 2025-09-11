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

#ifndef RADIO_MAX_PAYLOAD
    #define RADIO_MAX_PAYLOAD  61   // typical for RFM69 payload (adjust to your driver)
#endif

class RadioUtilizationMonitor {
private:
    struct TransmissionStats {
        unsigned long totalTransmissions = 0;
        unsigned long totalBytes = 0;
        unsigned long fragmentedTransmissions = 0;
        unsigned long lastTransmissionTime = 0;
        unsigned long transmissionTimeSum = 0;
        unsigned long maxTransmissionTime = 0;
        float avgBytesPerSecond = 0.0f;
    };

    TransmissionStats stats;
    unsigned long windowStart = 0;
    static const unsigned long WINDOW_DURATION = 60000; // 1 minute window

public:
    void recordTransmission(size_t bytes, bool wasFragmented = false) {
        unsigned long now = millis();

        // Reset window if needed
        if (now - windowStart > WINDOW_DURATION) {
            windowStart = now;
            // Keep running totals but reset rate calculations
        }

        stats.totalTransmissions++;
        stats.totalBytes += bytes;
        if (wasFragmented) stats.fragmentedTransmissions++;

        // Calculate transmission time (estimate based on byte count)
        unsigned long txTime = estimateTransmissionTime(bytes);
        stats.transmissionTimeSum += txTime;
        stats.maxTransmissionTime = max(stats.maxTransmissionTime, txTime);

        // Calculate throughput
        unsigned long elapsed = now - windowStart;
        if (elapsed > 0) {
            stats.avgBytesPerSecond = (float)stats.totalBytes / (elapsed / 1000.0f);
        }

        stats.lastTransmissionTime = now;
    }

    void printUtilization() {
        unsigned long now = millis();
        unsigned long timeSinceLastTx = now - stats.lastTransmissionTime;

        Serial.println("=== Radio Utilization Report ===");
        Serial.print("Total Transmissions: ");
        Serial.println(stats.totalTransmissions);
        Serial.print("Total Bytes Sent: ");
        Serial.println(stats.totalBytes);
        Serial.print("Fragmented TXs: ");
        Serial.print(stats.fragmentedTransmissions);
        Serial.print(" (");
        Serial.print(stats.totalTransmissions > 0 ?
            (stats.fragmentedTransmissions * 100) / stats.totalTransmissions : 0);
        Serial.println("%)");

        Serial.print("Avg Throughput: ");
        Serial.print(stats.avgBytesPerSecond, 1);
        Serial.println(" bytes/sec");

        Serial.print("Avg TX Time: ");
        Serial.print(stats.totalTransmissions > 0 ?
            stats.transmissionTimeSum / stats.totalTransmissions : 0);
        Serial.println(" ms");

        Serial.print("Max TX Time: ");
        Serial.print(stats.maxTransmissionTime);
        Serial.println(" ms");

        Serial.print("Last TX: ");
        Serial.print(timeSinceLastTx);
        Serial.println(" ms ago");

        // Radio load percentage (estimated)
        float radioLoad = calculateRadioLoad();
        Serial.print("Radio Load: ");
        Serial.print(radioLoad, 1);
        Serial.println("%");

        Serial.println("==============================");
    }

private:
    unsigned long estimateTransmissionTime(size_t bytes) {
        // Estimate based on RFM69 typical bitrate (~50kbps effective)
        const float EFFECTIVE_BITRATE = 6250.0f; // bytes per second
        return (unsigned long)((bytes / EFFECTIVE_BITRATE) * 1000); // ms
    }

    float calculateRadioLoad() {
        unsigned long elapsed = millis() - windowStart;
        if (elapsed == 0) return 0.0f;

        float activeTime = stats.transmissionTimeSum;
        return (activeTime / elapsed) * 100.0f;
    }
};

class RTCMTypeCounter {
private:
    struct MessageTypeData {
        uint16_t messageType;
        uint32_t count;
        unsigned long lastSeen;
    };

    static const int MAX_MESSAGE_TYPES = 10;
    MessageTypeData messageData[MAX_MESSAGE_TYPES];
    int messageTypeCount = 0;
    unsigned long lastReport = 0;
    const unsigned long REPORT_INTERVAL = 60000; // Report every 60 seconds

    int findMessageTypeIndex(uint16_t messageType) {
        for (int i = 0; i < messageTypeCount; i++) {
            if (messageData[i].messageType == messageType) {
                return i;
            }
        }
        return -1; // Not found
    }

public:
    RTCMTypeCounter() {
        for (int i = 0; i < MAX_MESSAGE_TYPES; i++) {
            messageData[i].messageType = 0;
            messageData[i].count = 0;
            messageData[i].lastSeen = 0;
        }
    }

    void recordMessage(uint16_t messageType) {
        int index = findMessageTypeIndex(messageType);

        if (index >= 0) {
            // Message type already exists
            messageData[index].count++;
            messageData[index].lastSeen = millis();
        }
        else if (messageTypeCount < MAX_MESSAGE_TYPES) {
            // New message type
            messageData[messageTypeCount].messageType = messageType;
            messageData[messageTypeCount].count = 1;
            messageData[messageTypeCount].lastSeen = millis();
            messageTypeCount++;
        }
    }

    void printReport() {
        if (millis() - lastReport < REPORT_INTERVAL) return;

        Serial.println("=== RTCM Message Type Report ===");
        for (int i = 0; i < messageTypeCount; i++) {
            uint16_t msgType = messageData[i].messageType;
            uint32_t count = messageData[i].count;
            unsigned long lastSeen = messageData[i].lastSeen;

            Serial.print("RTCM ");
            Serial.print(msgType);
            Serial.print(": ");
            Serial.print(count);
            Serial.print(" sent");

            // Add message type description
            switch (msgType) {
            case 1006: Serial.print(" (ref station pos)"); break;
            case 1033: Serial.print(" (antenna info)"); break;
            case 1074: Serial.print(" (GPS MSM4)"); break;
            case 1084: Serial.print(" (GLONASS MSM4)"); break;
            case 1094: Serial.print(" (Galileo MSM4)"); break;
            case 1124: Serial.print(" (BeiDou MSM4)"); break;
            default: Serial.print(" (unknown)"); break;
            }

            unsigned long timeSinceLastSeen = millis() - lastSeen;
            Serial.print(" | Last: ");
            Serial.print(timeSinceLastSeen / 1000);
            Serial.println("s ago");
        }
        Serial.println("==============================");
        lastReport = millis();
    }
};



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

    void enterSleepMode();
    void wakeFromSleep();

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

    void printRadioUtilization() {
        utilizationMonitor.printUtilization();
    }
    void printRTCMTypeReport() {
        typeCounter.printReport();
    }

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
    RadioUtilizationMonitor utilizationMonitor;
    RTCMTypeCounter typeCounter;
};