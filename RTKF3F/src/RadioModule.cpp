//RadioModule.cpp

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

    _radio.setHighPower();
    _radio.setPowerLevel(31);
    _radio.encrypt(NULL);
    _radio.setFrequency(frequency);

    //setBitrate(_radio, 19200);  // Setter bitrate til 19200 bps
    //Serial.printf("Radio bitrate: %u bps\n", _radio.getPower());
    //Serial.printf("Radio bitrate: %u bps\n", _radio.getBitrate());

    _numRTCMSent = 0;

    return true;
}

void RadioModule::setBitrate(RFM69& radio, uint16_t bitrate) {
    uint16_t val = 32000000UL / bitrate;
    radio.writeReg(REG_BITRATEMSB, val >> 8);
    radio.writeReg(REG_BITRATELSB, val & 0xFF);
}

bool RadioModule::verify() {
    uint8_t version = _radio.readReg(0x10);
    return version == expectedVersion;
}

bool RadioModule::receive(uint8_t*& data, uint8_t& len) {
    if (_radio.receiveDone()) {
        data = _radio.DATA;
        len  = _radio.DATALEN;
        return true;
    }
    return false;
}

void RadioModule::sendWithReturnFreq(uint8_t destNode, int destFreq, int returnFreq, const uint8_t* msg, uint8_t len) {
    _radio.setMode(RF69_MODE_SLEEP);
    delay(2);
    _radio.setFrequency(destFreq);
    delay(2);
    _radio.setMode(RF69_MODE_TX);
    delay(2);
    _radio.send(destNode, msg, len);
    delay(2);
    _radio.setMode(RF69_MODE_SLEEP);
    delay(2);
    _radio.setFrequency(returnFreq);
    delay(2);
    _radio.setMode(RF69_MODE_RX);
    delay(2);
}

void RadioModule::sendRTCMNumMessages() {
    uint8_t packet[5];
    packet[0] = MSG_RTCM_NUMSENT;  // tag

    // Store numSent as big endian (network order)
    packet[1] = (_numRTCMSent >> 24) & 0xFF;
    packet[2] = (_numRTCMSent >> 16) & 0xFF;
    packet[3] = (_numRTCMSent >> 8) & 0xFF;
    packet[4] = _numRTCMSent & 0xFF;

    _radio.send(0, packet, sizeof(packet));
}

int RadioModule::getRTCMNumMessages() {
	return _numRTCMSent;
}

void RadioModule::sendRTCM(const uint8_t* data, size_t len) {
    if (len == 0) return;

    if (len <= RTCM_Fragmenter::MAX_PAYLOAD) {
        // Send full RTCM message directly (starts with 0xD3)
        _radio.send(0, data, len);
    }
    else {
        // Too large → fragment
        sendFragmentedRTCM(data, len);
    }

    _numRTCMSent++;
}

void RadioModule::sendFragmentedRTCM(const uint8_t* data, size_t len) {
    RTCM_Fragmenter::sendFragmented(_radio, 0, data, len);
}

// RTCM_Fragmenter implementation
void RadioModule::RTCM_Fragmenter::sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len) {
    const uint8_t MAX_PAYLOAD = 58;  // adjusted for 61 byte total
    if (len > MAX_TOTAL_LEN) return;

    uint8_t packet[61];  // max total length
    uint8_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

    for (uint8_t i = 0; i < totalChunks; ++i) {
        size_t offset = i * MAX_PAYLOAD;
        size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);

        packet[0] = MSG_RTCMFRAGMENT;
        packet[1] = i;              // fragment index
        packet[2] = totalChunks;    // total fragments

        memcpy(packet + 3, data + offset, chunkLen);
        radio.send(destId, packet, chunkLen + 3);  // total = header + payload
        delay(5);  // allow radio time to settle
    }
}

// RTCM_Reassembler implementation
RadioModule::RTCM_Reassembler::RTCM_Reassembler()
    : receivedCount(0), totalExpected(0), complete(false) {
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

void RadioModule::RTCM_Reassembler::acceptFragment(const uint8_t* data, size_t len) {
    if (len < 4 || data[0] != MSG_RTCMFRAGMENT) return;

    uint8_t total = data[1];
    uint8_t index = data[2];
    if (total > MAX_FRAGMENTS || index >= total) return;

	Serial.printf("RTCM_Reassembler: Accepting fragment %u of %u, length %zu\n", index, total, len);

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

bool RadioModule::RTCM_Reassembler::isComplete() const {
    return complete;
}

const uint8_t* RadioModule::RTCM_Reassembler::getData() const {
    return buffer;
}

size_t RadioModule::RTCM_Reassembler::getLength() const {
    return length;
}

void RadioModule::RTCM_Reassembler::reset() {
    complete = false;
    receivedCount = 0;
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

