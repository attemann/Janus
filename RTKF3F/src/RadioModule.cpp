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

    //_numRTCMSent = 0;

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

int RadioModule::getSenderId() {
	return _radio.SENDERID;
}

int RadioModule::getTargetId() {
	return _radio.TARGETID;
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
    if (destFreq != returnFreq) {
        _radio.setMode(RF69_MODE_SLEEP);
        delay(5);
        _radio.setFrequency(destFreq);
        delay(5);
        _radio.setMode(RF69_MODE_TX);
        delay(5);
    }

    _radio.send(destNode, msg, len);

    if (destFreq != returnFreq) {
        delay(5);
        _radio.setMode(RF69_MODE_SLEEP);
        delay(5);
        _radio.setFrequency(returnFreq);
        delay(5);
        _radio.setMode(RF69_MODE_RX);
    }

    delay(5);
}

void RadioModule::sendMessageCode(int destNode, int destFreq, int returnFreq, int msgType, int msgCode) {
    uint8_t packet[2];
    packet[0] = msgType;  // tag
    packet[1] = msgCode;

    sendWithReturnFreq(destNode, destFreq, returnFreq, packet, sizeof(packet));
}


void RadioModule::sendRTCM(const uint8_t* data, size_t len) {
    if (len == 0) return;

    if (len <= RTCM_Fragmenter::MAX_PAYLOAD) {
        //printRTCMSegment(data, len);  // print header + payload  
        _radio.send(0, data, len);
    }
    else {
        sendFragmentedRTCM(data, len);
    }
}

void RadioModule::sendFragmentedRTCM(const uint8_t* data, size_t len) {
    currentMsgId = (currentMsgId + 1) % 256;
	Serial.printf("Sending fragmented RTCM msgId " ANSI_GREEN "%d" ANSI_RESET ", length % u\n", currentMsgId, len);
    RTCM_Fragmenter::sendFragmented(_radio, 0, data, len, currentMsgId);
}

void RadioModule::sendRTCMTest(int len) {
//#define BYTESINTEST 100
    uint8_t rtcm[len];
    rtcm[0] = 0xD3;
    rtcm[1] = 0x01;
    uint8_t val = 0x02;
    for (int i = 2; i < len; ++i) {
        rtcm[i] = val++;
        if (val > 0xFF) val = 0x00;
    }
	printRTCMSegment(rtcm, len);
	sendRTCM(rtcm, len);
}

void RadioModule::printRTCMSegment(const uint8_t* data, size_t len) {

    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    if (len % 16 != 0) printf("\n");
}

void RadioModule::RTCM_Fragmenter::sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len, uint8_t msgId) {
    const uint8_t MAX_PAYLOAD = 57;  // adjust down by 1 for new msgId byte
    if (len > MAX_TOTAL_LEN) return;

    uint8_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

    for (uint8_t i = 0; i < totalChunks; ++i) {
        size_t offset = i * MAX_PAYLOAD;
        size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);

        uint8_t packet[4 + MAX_PAYLOAD];
        packet[0] = MSG_RTCM_FRAGMENT;
        packet[1] = i;
        packet[2] = totalChunks;
        packet[3] = msgId;             // NEW: Add message id
        memcpy(packet + 4, data + offset, chunkLen);

        radio.send(destId, packet, chunkLen + 4);
        delay(20);
    }
}

// RTCM_Reassembler implementation
RadioModule::RTCM_Reassembler::RTCM_Reassembler()
    : receivedCount(0), totalExpected(0), complete(false) {
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

void RadioModule::RTCM_Reassembler::acceptFragment(const uint8_t* data, size_t len) {
    constexpr size_t FRAGMENT_HEADER_SIZE = 4;
    constexpr size_t FRAGMENT_PAYLOAD_SIZE = 57;
    if (len < FRAGMENT_HEADER_SIZE + 1 || data[0] != MSG_RTCM_FRAGMENT) return;

    uint8_t index = data[1];
    uint8_t total = data[2];
    uint8_t msgId = data[3];
    if (total > MAX_FRAGMENTS || index >= total) return;

    if (index == 0) {
        totalExpected = total;
        receivedCount = 0;
        complete = false;
        expectedMsgId = msgId;
        memset(fragmentReceived, 0, sizeof(fragmentReceived));
        memset(buffer, 0, sizeof(buffer));
    }
    if (msgId != expectedMsgId) return;

    size_t fragLen = len - FRAGMENT_HEADER_SIZE;
    size_t offset = index * FRAGMENT_PAYLOAD_SIZE;
    if ((offset + fragLen) > sizeof(buffer)) return;

    memcpy(buffer + offset, data + FRAGMENT_HEADER_SIZE, fragLen);
    if (fragmentReceived[index] == 0) receivedCount++;
    fragmentReceived[index] = fragLen;

    if (receivedCount == totalExpected) {
        size_t totalLen = 0;
        for (uint8_t i = 0; i < totalExpected; ++i)
            totalLen += fragmentReceived[i];
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


