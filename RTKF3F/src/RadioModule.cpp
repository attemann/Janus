#include <Arduino.h>    
#include "RadioModule.h"
#include "RTKF3F.h"

RadioModule::RadioModule(RFM69& radio) : _radio(radio) {}

bool RadioModule::init(RadioModule::HWPins pins, int nodeid, int networkid, int frequency) {
    SPI.begin(pins.sck, pins.miso, pins.mosi, pins.cs);

    if (pins.reset != -1) {
        pinMode(pins.reset, OUTPUT);
        digitalWrite(pins.reset, LOW);
        delay(10);
        digitalWrite(pins.reset, HIGH);
        delay(10);
    }

    pinMode(pins.irq, INPUT);

    if (!_radio.initialize(RF69_868MHZ, nodeid, networkid)) {
        Serial.println("Radio init failed");
        return false;
    }

    _radio.setFrequency(frequency);
    _radio.setHighPower();
    _radio.encrypt(NULL);

    return true;
}

bool RadioModule::verify() {
    uint8_t version = _radio.readReg(0x10);
    return version == expectedVersion;
}

void RadioModule::sendRTCM(const uint8_t* data, size_t len) {
    uint8_t packet[1025];
    packet[0] = MSG_RTCM;
    memcpy(&packet[1], data, len);
    _radio.send(0, packet, len + 1);
}

void RadioModule::sendFragmentedRTCM(const uint8_t* data, size_t len) {
    RTCM_Fragmenter::sendFragmented(_radio, 0, data, len);
}

// RTCM_Fragmenter implementation
void RadioModule::RTCM_Fragmenter::sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len) {
    if (len > MAX_TOTAL_LEN) return;

    uint8_t packet[MAX_PAYLOAD + 3];
    uint8_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

    for (uint8_t i = 0; i < totalChunks; ++i) {
        size_t offset = i * MAX_PAYLOAD;
        size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);

        packet[0] = 0xA0;
        packet[1] = totalChunks;
        packet[2] = i;

        memcpy(packet + 3, data + offset, chunkLen);
        radio.send(destId, packet, chunkLen + 3);
        delay(5);
    }
}

// RTCM_Reassembler implementation
RadioModule::RTCM_Reassembler::RTCM_Reassembler()
    : receivedCount(0), totalExpected(0), complete(false) {
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

void RadioModule::RTCM_Reassembler::acceptFragment(const uint8_t* data, size_t len) {
    if (len < 4 || data[0] != 0xA0) return;

    uint8_t total = data[1];
    uint8_t index = data[2];
    if (total > MAX_FRAGMENTS || index >= total) return;

    if (index == 0) {
        totalExpected = total;
        receivedCount = 0;
        complete = false;
        memset(fragmentReceived, 0, sizeof(fragmentReceived));
    }

    size_t fragLen = len - 3;
    memcpy(buffer + index * RTCM_Fragmenter::MAX_PAYLOAD, data + 3, fragLen);
    fragmentReceived[index] = fragLen;
    receivedCount++;

    if (receivedCount == totalExpected) {
        size_t totalLen = 0;
        for (uint8_t i = 0; i < totalExpected; ++i) {
            totalLen += fragmentReceived[i];
        }
        this->length = totalLen;
        complete = true;
    }
}

bool RadioModule::RTCM_Reassembler::isComplete() const { return complete; }
const uint8_t* RadioModule::RTCM_Reassembler::getData() const { return buffer; }
size_t RadioModule::RTCM_Reassembler::getLength() const { return length; }
void RadioModule::RTCM_Reassembler::reset() { complete = false; receivedCount = 0; }