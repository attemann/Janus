
// RadioModule.cpp — Refactored & optimized
// Notes:
// - Removed stray `while(true);` in init()
// - Fixed sendWithReturnFreq() to enter RX mode (not STANDBY) after hop back
// - Safer Serial.printf usage with bounds checking
// - Enhanced error handling and consistent debug logging
// - Improved code organization with clear separation of concerns

#include <Arduino.h>
#include <SPI.h>
#include "RadioModule.h"
#include "RFM69registers.h"
#include "MessageTypes.h"

// --------- Constants ---------
namespace {
    constexpr uint32_t FXOSC = 32000000UL; // Oscillator frequency (Hz)
    constexpr float FSTEP = FXOSC / 524288.0f; // Frequency step (Hz)
    constexpr size_t MAX_PAYLOAD = 57; // Max bytes per frame
    constexpr size_t MAX_TOTAL_LEN = 1024; // Max total RTCM length
    constexpr uint8_t MSG_RTCM_FRAGMENT = 0x01; // Fragment message type
}

// --------- File-local Helpers ---------
static bool waitForModeReady(RFM69& radio, uint32_t timeoutMs = 50) {
    const uint32_t start = millis();
    while (!(radio.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY)) {
        if (millis() - start > timeoutMs) {
            Serial.println("Timeout waiting for ModeReady");
            return false;
        }
    }
    return true;
}

static bool waitForPllLock(RFM69& radio, uint32_t timeoutMs = 5) {
    const uint32_t start = millis();
    while (!(radio.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_PLLLOCK)) {
        if (millis() - start > timeoutMs) {
            Serial.println("Timeout waiting for PLL lock");
            return false;
        }
    }
    return true;
}

static bool waitForPacketSent(RFM69& radio, uint32_t timeoutMs = 1000) {
    const uint32_t start = millis();
    while (!(radio.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT)) {
        if (millis() - start > timeoutMs) {
            Serial.println("Timeout waiting for packet sent");
            return false;
        }
    }
    return true;
}

static void printFrequency(RFM69& radio) {
    const uint32_t frf = ((uint32_t)radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)radio.readReg(REG_FRFMID) << 8) |
        radio.readReg(REG_FRFLSB);
    const float mhz = frf * FSTEP / 1e6f;
    Serial.printf("Frequency: FRF=0x%06lX (%.3f MHz)\r\n", (unsigned long)frf, mhz);
}

// --------- RadioModule Implementation ---------
RadioModule::RadioModule(int csPin, int irqPin, bool isHighPower)
    : _csPin(csPin), _irqPin(irqPin), _isHighPower(isHighPower), _radio(csPin, irqPin, isHighPower) {
}

bool RadioModule::init(int8_t pMISO, int8_t pMOSI, int8_t pSCK, uint16_t nodeId, uint8_t networkId, uint32_t frequencyHz) {
    SPI.begin(pSCK, pMISO, pMOSI, _csPin);
    _nodeId = nodeId;
    _networkId = networkId;

    if (!_radio.initialize(RF69_868MHZ, nodeId)) {
        Serial.println("❌ RFM69 initialization failed");
        return false;
    }

    _radio.setHighPower(_isHighPower);
    _radio.setFrequency(frequencyHz);
    _radio.encrypt(nullptr);
    _radio.setAddress(nodeId);
    setNetworkId(networkId);

    Serial.printf("Initialized: NodeID=%u, NetworkID=%u, HighPower=%s\r\n",
        nodeId, networkId, _isHighPower ? "true" : "false");
    debugRFM69(_radio);
    return checkRadioSettings(nodeId, networkId, frequencyHz);
}

void RadioModule::setNetworkId(uint8_t networkId) {
    _radio.writeReg(REG_SYNCVALUE1, networkId);
    const uint8_t actual = _radio.readReg(REG_SYNCVALUE1);
    Serial.printf("Set NetworkID: 0x%02X\r\n", actual);
}

RFM69& RadioModule::getRadio() {
    return _radio;
}

void RadioModule::send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK) {
    _radio.send(toAddress, buffer, bufferSize, requestACK);
}

uint8_t RadioModule::getSenderId() const { return _radio.SENDERID; }
uint8_t RadioModule::getTargetId() const { return _radio.TARGETID; }

bool RadioModule::receive(uint8_t* data, size_t& len) {
    if (!_radio.receiveDone()) return false;

    const size_t available = _radio.DATALEN;
    if (available > len) {
        Serial.printf("Receive buffer too small: %u > %u\r\n", available, len);
        return false;
    }
    memcpy(data, _radio.DATA, available);
    len = available;
    return true;
}

bool RadioModule::checkRadioSettings(uint8_t expectedNodeId, uint8_t expectedNetworkId, uint32_t expectedFreqHz) {
    bool valid = true;

    // Check Node ID
    const uint8_t nodeId = _radio.readReg(REG_NODEADRS);
    if (nodeId != expectedNodeId) {
        Serial.printf("⚠️ Node ID mismatch: actual=%u, expected=%u\r\n", nodeId, expectedNodeId);
        valid = false;
    }

    // Check Network ID
    const uint8_t networkId = _radio.readReg(REG_SYNCVALUE1);
    if (networkId != expectedNetworkId) {
        Serial.printf("⚠️ Network ID mismatch: actual=%u, expected=%u\r\n", networkId, expectedNetworkId);
        valid = false;
    }

    // Check Frequency
    const uint32_t frf = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        _radio.readReg(REG_FRFLSB);
    const float actualFreqHz = frf * FSTEP;
    if (abs(actualFreqHz - expectedFreqHz) > 1000) {
        Serial.printf("⚠️ Frequency mismatch: actual=%.1f Hz, expected=%lu Hz\r\n",
            actualFreqHz, (unsigned long)expectedFreqHz);
        valid = false;
    }

    return valid;
}

bool RadioModule::setFrequencyBlocking(uint32_t freqHz) {
    // Enter STANDBY mode
    _radio.setMode(RF69_MODE_STANDBY);
    if (!waitForModeReady(_radio)) return false;

    // Calculate FRF: FRF = freqHz / FSTEP
    const uint32_t frf = static_cast<uint32_t>(static_cast<uint64_t>(freqHz) / FSTEP);
    _radio.writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    _radio.writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    _radio.writeReg(REG_FRFLSB, frf & 0xFF);

    // Verify frequency
    const uint32_t actualFrf = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        _radio.readReg(REG_FRFLSB);
    const uint32_t actualHz = static_cast<uint32_t>(actualFrf * FSTEP);
    if (abs(static_cast<int32_t>(actualHz - freqHz)) > 1000) {
        Serial.printf("Frequency set failed: target=%lu Hz, actual=%lu Hz\r\n",
            (unsigned long)freqHz, (unsigned long)actualHz);
        return false;
    }

    // Nudge PLL via SYNTH mode
    _radio.setMode(RF69_MODE_SYNTH);
    if (!waitForModeReady(_radio) || !waitForPllLock(_radio)) return false;

    return true;
}

bool RadioModule::sendWithReturnFreq(uint8_t destNode, uint32_t destFreqHz, uint32_t returnFreqHz,
    const uint8_t* msg, uint8_t len) {
    if (!msg || len == 0 || len > MAX_PAYLOAD) {
        Serial.printf("Invalid message: ptr=%p, len=%u\r\n", msg, len);
        return false;
    }

    // Switch to destination frequency
    if (!setFrequencyBlocking(destFreqHz)) {
        Serial.println("Failed to set destination frequency");
        return false;
    }

    // Send message
    _radio.setMode(RF69_MODE_TX);
    if (!waitForModeReady(_radio)) {
        Serial.println("TX mode timeout");
        return false;
    }
    _radio.send(destNode, msg, len, false);
    if (!waitForPacketSent(_radio)) {
        Serial.println("Packet send timeout");
        return false;
    }

    // Return to listening frequency
    if (!setFrequencyBlocking(returnFreqHz)) {
        Serial.println("Failed to set return frequency");
        return false;
    }

    // Enter RX mode
    _radio.setMode(RF69_MODE_RX);
    if (!waitForModeReady(_radio)) {
        Serial.println("RX mode timeout");
        return false;
    }

    return true;
}

void RadioModule::forceSetFrequency(uint32_t freqHz) {
    const uint32_t frf = static_cast<uint32_t>(static_cast<uint64_t>(freqHz) / FSTEP);
    _radio.writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    _radio.writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    _radio.writeReg(REG_FRFLSB, frf & 0xFF);

    printFrequency(_radio);
}

void RadioModule::sendMessageCode(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq,
    uint8_t msgType, uint8_t msgCode) {
    uint8_t packet[2] = { msgType, msgCode };
    if (!sendWithReturnFreq(destNode, destFreq, returnFreq, packet, sizeof(packet))) {
        Serial.println("Failed to send message code");
    }
}

void RadioModule::sendRTCM(const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        Serial.println("Invalid RTCM data");
        return;
    }

    if (len <= MAX_PAYLOAD) {
        printRTCMSegment(data, len);
        _radio.send(0, data, static_cast<uint8_t>(len));
    }
    else {
        sendFragmentedRTCM(data, len);
    }
}

void RadioModule::sendFragmentedRTCM(const uint8_t* data, size_t len) {
    _currentMsgId = (_currentMsgId + 1) & 0xFF;
    Serial.printf("Sending fragmented RTCM: msgId=%u, len=%u\r\n", _currentMsgId, len);
    RTCM_Fragmenter::sendFragmented(_radio, 0, data, len, _currentMsgId);
}

void RadioModule::printRTCMSegment(const uint8_t* data, size_t len) {
    Serial.print("RTCM: ");
    for (size_t i = 0; i < len; ++i) {
        Serial.printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    if (len % 16 != 0) Serial.println();
}

void RadioModule::debugRFM69(RFM69& radio) {
    Serial.println("=== RFM69 Debug Dump ===");

    const uint8_t opMode = radio.readReg(REG_OPMODE);
    Serial.printf("OpMode: 0x%02X\r\n", opMode);

    printFrequency(radio);

    const uint16_t bitrate = ((uint16_t)radio.readReg(REG_BITRATEMSB) << 8) | radio.readReg(REG_BITRATELSB);
    Serial.printf("Bitrate: %.2f bps\r\n", FXOSC / static_cast<float>(bitrate));

    const uint16_t fdev = ((uint16_t)radio.readReg(REG_FDEVMSB) << 8) | radio.readReg(REG_FDEVLSB);
    Serial.printf("Freq Deviation: %.1f Hz\r\n", fdev * FSTEP);

    const uint8_t paLevel = radio.readReg(REG_PALEVEL);
    Serial.printf("PA Level: 0x%02X (raw=%u)\r\n", paLevel, paLevel & 0x1F);

    const uint8_t rssiThresh = radio.readReg(REG_RSSITHRESH);
    Serial.printf("RSSI Threshold: %d dB\r\n", -(rssiThresh / 2));

    Serial.printf("SyncConfig: 0x%02X\r\n", radio.readReg(REG_SYNCCONFIG));
    Serial.printf("SyncValue1 (NetworkID): 0x%02X\r\n", radio.readReg(REG_SYNCVALUE1));
    Serial.printf("PacketConfig1: 0x%02X\r\n", radio.readReg(REG_PACKETCONFIG1));
    Serial.printf("PacketConfig2: 0x%02X\r\n", radio.readReg(REG_PACKETCONFIG2));
    Serial.printf("NodeAddr: 0x%02X\r\n", radio.readReg(REG_NODEADRS));
    Serial.printf("BroadcastAddr: 0x%02X\r\n", radio.readReg(REG_BROADCASTADRS));
    Serial.printf("Version: 0x%02X\r\n", radio.readReg(REG_VERSION));

    Serial.println("=== End of Dump ===");
}

// --------- RTCM_Fragmenter Implementation ---------
void RadioModule::RTCM_Fragmenter::sendFragmented(RFM69& radio, uint8_t destId,
    const uint8_t* data, size_t len,
    uint8_t msgId) {
    if (!data || len == 0 || len > MAX_TOTAL_LEN) {
        Serial.println("Invalid RTCM fragment data");
        return;
    }

    const uint8_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
    for (uint8_t i = 0; i < totalChunks; ++i) {
        const size_t offset = i * MAX_PAYLOAD;
        const size_t chunkLen = min(MAX_PAYLOAD, len - offset);

        uint8_t packet[MAX_PAYLOAD + 4];
        packet[0] = static_cast<uint8_t>(MessageType::MSG_RTCM_FRAGMENT);
        packet[1] = i;
        packet[2] = totalChunks;
        packet[3] = msgId;
        memcpy(packet + 4, data + offset, chunkLen);

        radio.send(destId, packet, chunkLen + 4);
        delay(20); // Prevent overwhelming receiver
    }
}

// --------- RTCM_Reassembler Implementation ---------
RadioModule::RTCM_Reassembler::RTCM_Reassembler() : receivedCount(0), totalExpected(0), complete(false) {
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

void RadioModule::RTCM_Reassembler::acceptFragment(const uint8_t* data, size_t len) {
    constexpr size_t HEADER_SIZE = 4;
    constexpr size_t MAX_FRAGMENTS = 255;

    if (!data || len < HEADER_SIZE + 1 || data[0] != static_cast<uint8_t>(MessageType::MSG_RTCM_FRAGMENT)) return;

    const uint8_t index = data[1];
    const uint8_t total = data[2];
    const uint8_t msgId = data[3];

    if (total == 0 || total > MAX_FRAGMENTS || index >= total) return;

    if (index == 0) {
        reset();
        totalExpected = total;
        expectedMsgId = msgId;
    }
    if (msgId != expectedMsgId) return;

    const size_t fragLen = len - HEADER_SIZE;
    const size_t offset = index * MAX_PAYLOAD;
    if (offset + fragLen > sizeof(buffer)) return;

    memcpy(buffer + offset, data + HEADER_SIZE, fragLen);
    if (!fragmentReceived[index]) receivedCount++;
    fragmentReceived[index] = static_cast<uint8_t>(fragLen);

    if (receivedCount == totalExpected) {
        size_t totalLen = 0;
        for (uint8_t i = 0; i < totalExpected; ++i) totalLen += fragmentReceived[i];
        length = totalLen;
        complete = true;
    }
}

bool RadioModule::RTCM_Reassembler::isComplete() const { return complete; }
const uint8_t* RadioModule::RTCM_Reassembler::getData() const { return buffer; }
size_t RadioModule::RTCM_Reassembler::getLength() const { return length; }

void RadioModule::RTCM_Reassembler::reset() {
    complete = false;
    receivedCount = 0;
    totalExpected = 0;
    expectedMsgId = 0;
    length = 0;
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
    memset(buffer, 0, sizeof(buffer));
}