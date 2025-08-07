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
    _radio.setMode(RF69_MODE_SLEEP);
    delay(5);
    _radio.setFrequency(destFreq);
    delay(5);
    _radio.setMode(RF69_MODE_TX);
    delay(5);
    _radio.send(destNode, msg, len);
    delay(5);
    _radio.setMode(RF69_MODE_SLEEP);
    delay(5);
    _radio.setFrequency(returnFreq);
    delay(5);
    _radio.setMode(RF69_MODE_RX);
    delay(5);
}

void RadioModule::sendMessageCode(int destNode, int destFreq, int returnFreq, int msgType, int msgCode) {
    uint8_t packet[2];
    packet[0] = msgType;  // tag
    packet[1] = msgCode;

    sendWithReturnFreq(destNode, destFreq, returnFreq, packet, sizeof(packet));
}

void RadioModule::sendRTCMNumMessages() {
    uint8_t packet[5];
    packet[0] = MSG_RTCM_NUM_SENT;  // tag

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
		Serial.println("Sending full RTCM message");
        printRTCMSegment(data, len);  // print header + payload  
        _radio.send(0, data, len);
    }
    else {
        // Too large → fragment
        Serial.println("Sending fragmented RTCM message");
        sendFragmentedRTCM(data, len);
    }

    _numRTCMSent++;
}

void RadioModule::sendFragmentedRTCM(const uint8_t* data, size_t len) {
    RTCM_Fragmenter::sendFragmented(_radio, 0, data, len);
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

/*
Radio verified
D3 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F
30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F
40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F
50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F
60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F
70 71 72 73 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F
80 81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8E 8F
90 91 92 93 94 95 96 97 98 99 9A 9B 9C 9D 9E 9F
A0 A1 A2 A3 A4 A5 A6 A7 A8 A9 AA AB AC AD AE AF
B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 BA BB BC BD BE BF
C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA CB CC CD CE CF
D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 DA DB DC DD DE DF
E0 E1 E2 E3 E4 E5 E6 E7 E8 E9 EA EB EC ED EE EF
F0 F1 F2 F3 F4 F5 F6 F7 F8 F9 FA FB FC FD FE FF
00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
20 21 22 23 24 25 26 27 28 29 2A 2B
Sending fragmented RTCM message
RTCM_Fragmenter: Sending 300 bytes in fragments
RTCM_Fragmenter: Total fragments = 6
RTCM_Fragmenter: Sending fragment 1 of 6
0A 00 06 D3 01 02 03 04 05 06 07 08 09 0A 0B 0C
0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C
1D 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C
2D 2E 2F 30 31 32 33 34 35 36 37 38 39
RTCM_Fragmenter: Sending fragment 2 of 6
0A 01 06 3A 3B 3C 3D 3E 3F 40 41 42 43 44 45 46
47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 53 54 55 56
57 58 59 5A 5B 5C 5D 5E 5F 60 61 62 63 64 65 66
67 68 69 6A 6B 6C 6D 6E 6F 70 71 72 73
RTCM_Fragmenter: Sending fragment 3 of 6
0A 02 06 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F 80
81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8E 8F 90
91 92 93 94 95 96 97 98 99 9A 9B 9C 9D 9E 9F A0
A1 A2 A3 A4 A5 A6 A7 A8 A9 AA AB AC AD
RTCM_Fragmenter: Sending fragment 4 of 6
0A 03 06 AE AF B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 BA
BB BC BD BE BF C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA
CB CC CD CE CF D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 DA
DB DC DD DE DF E0 E1 E2 E3 E4 E5 E6 E7
RTCM_Fragmenter: Sending fragment 5 of 6
0A 04 06 E8 E9 EA EB EC ED EE EF F0 F1 F2 F3 F4
F5 F6 F7 F8 F9 FA FB FC FD FE FF 00 01 02 03 04
05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14
15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21
RTCM_Fragmenter: Sending fragment 6 of 6
0A 05 06 22 23 24 25 26 27 28 29 2A 2B
*/
void RadioModule::RTCM_Fragmenter::sendFragmented(RFM69& radio, uint8_t destId, const uint8_t* data, size_t len) {  
	Serial.printf("RTCM_Fragmenter: Sending %zu bytes in fragments\n", len);
    const uint8_t MAX_PAYLOAD = 58;  // adjusted for 61 byte total  
    if (len > MAX_TOTAL_LEN) return;  

    uint8_t packet[61];  // max total length  
    uint8_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;  
	Serial.printf("RTCM_Fragmenter: Total fragments = %u\n", totalChunks);

    for (uint8_t i = 0; i < totalChunks; ++i) { 
		Serial.println("RTCM_Fragmenter: Sending fragment " + String(i + 1) + " of " + String(totalChunks));
        size_t offset = i * MAX_PAYLOAD;  
        size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);  

        packet[0] = MSG_RTCM_FRAGMENT;  
        packet[1] = i;              // fragment index  s
        packet[2] = totalChunks;    // total fragments  

        memcpy(packet + 3, data + offset, chunkLen);  

        // Use a specific object reference for the non-static member function  
        RadioModule radioModule(radio);  
        radioModule.printRTCMSegment(packet, chunkLen + 3);  // print header + payload  

        radio.send(destId, packet, chunkLen + 3);  // total = header + payload  
        delay(10);  // allow radio time to settle  
    }  
}

// RTCM_Reassembler implementation
RadioModule::RTCM_Reassembler::RTCM_Reassembler()
    : receivedCount(0), totalExpected(0), complete(false) {
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

void RadioModule::RTCM_Reassembler::acceptFragment(const uint8_t* data, size_t len) {
    if (len < 4 || data[0] != MSG_RTCM_FRAGMENT) return;

    uint8_t index = data[1];
    uint8_t total = data[2];
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
    if (fragmentReceived[index] == 0) receivedCount++;
    fragmentReceived[index] = fragLen;

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


