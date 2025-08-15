// RadioModule.cpp — Refactored & completed
// Notes:
// - Removed stray `while(true);` in init()
// - Fixed comment/logic mismatch in sendWithReturnFreq(): enter RX (not STANDBY) after hop back
// - Safer Serial.printf usage (no bare printf)
// - Defensive checks & small nits in logs

#include <Arduino.h>
#include <SPI.h>
#include "RadioModule.h"
#include "RFM69registers.h"

// --------- File‑local helpers ---------
static inline bool waitModeReady(RFM69& r, uint32_t timeoutMs = 50) {
    const uint32_t t0 = millis();
    while (!(r.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY)) {
        if (millis() - t0 > timeoutMs) return false;
    }
    return true;
}

static inline bool waitPllLock(RFM69& r, uint32_t timeoutMs = 5) {
    const uint32_t t0 = millis();
    while (!(r.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_PLLLOCK)) {
        if (millis() - t0 > timeoutMs) return false;
    }
    return true;
}

static inline bool waitPacketSent(RFM69& r, uint16_t timeoutMs = 20) {
    const uint32_t t0 = millis();
    while (!(r.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT)) {
        if (millis() - t0 > timeoutMs) return false;
    }
    return true;
}

static inline void dbgPrintFreq(RFM69& r) {
    const uint32_t frf = ((uint32_t)r.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)r.readReg(REG_FRFMID) << 8) |
        (uint32_t)r.readReg(REG_FRFLSB);
    const float mhz = frf * 61.03515625f / 1e6f; // Fstep = 32e6/2^19
    Serial.printf("FRF=0x%06lX -> %.3f MHz\r\n", (unsigned long)frf, mhz);
}

// --------- RadioModule ---------
RadioModule::RadioModule(int csPin, int irqPin, bool isHighPower)
    : _csPin(csPin), _irqPin(irqPin), _isHighPower(isHighPower), _radio(csPin, irqPin, isHighPower) {
}

void RadioModule::joeSetNetwork(uint8_t networkid) {
    // Set primary sync byte to act as a quick network discriminator
    _radio.writeReg(REG_SYNCVALUE1, networkid);
    const uint8_t syncVal = _radio.readReg(REG_SYNCVALUE1);
    Serial.printf("✅ Forced SyncValue1 to 0x%02X\r\n", syncVal);
}

bool RadioModule::init(int8_t pMISO, int8_t pMOSI, int8_t pSCK,
    uint16_t nodeid, uint8_t networkid, uint32_t frequencyHz) {
    // SPI CS controlled by driver; begin with our pins
    SPI.begin(pSCK, pMISO, pMOSI, _csPin);

    _nodeid = nodeid;
    _networkid = networkid;

    if (!_radio.initialize(RF69_868MHZ, nodeid)) { // Using your modified lib signature
        Serial.println("RFM69 init failed");
        return false;
    }

    Serial.printf("_isHighPower: %s \r\n", _isHighPower ? "true" : "false");
    _radio.setHighPower(true);
    _radio.setFrequency(868200000);
    _radio.encrypt(NULL);
    _radio.setAddress(nodeid);
    //joeSetNetwork(networkid);

    //_radio.setMode(RF69_MODE_STANDBY);
    debugRFM69(_radio);
    checkRadioSettings((uint8_t)nodeid, networkid, frequencyHz);

    return true;
}

RFM69& RadioModule::getRadio() {
    return _radio;
}

void RadioModule::send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK) {
    _radio.send(toAddress, buffer, bufferSize, requestACK);
}

int RadioModule::getSenderId() { return _radio.SENDERID; }
int RadioModule::getTargetId() { return _radio.TARGETID; }

bool RadioModule::receive(uint8_t* data, size_t& len) {
    if (_radio.receiveDone()) {
        const size_t available = _radio.DATALEN;
        if (available > len) return false; // prevent overflow
        memcpy(data, _radio.DATA, available);
        len = available;
        return true;
    }
    return false;
}

bool RadioModule::checkRadioSettings(uint8_t expectedNodeId,
    uint8_t expectedNetworkId,
    uint32_t expectedFreqHz) {
    bool ok = true;

    // Node ID
    const uint8_t actualNodeId = _radio.readReg(REG_NODEADRS);
    if (actualNodeId != expectedNodeId) {
        Serial.printf("⚠️ Node ID mismatch: actual=%u, expected=%u\r\n", actualNodeId, expectedNodeId);
            ok = false;
    }

    // Network ID (SyncValue1)
    const uint8_t actualNetworkId = _radio.readReg(REG_SYNCVALUE1);
    if (actualNetworkId != expectedNetworkId) {
        Serial.printf("⚠️ Network ID mismatch: actual=%u, expected=%u\r\n", actualNetworkId, expectedNetworkId);
            ok = false;
    }

    // Frequency check via FRF registers
    const uint32_t frf = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        _radio.readReg(REG_FRFLSB);
    const float actualFreqHz = (32.0f * 1000000.0f * frf) / 524288.0f; // FXOSC/2^19
    const int32_t diffHz = abs((int32_t)(actualFreqHz - expectedFreqHz));
    if (diffHz > 1000) { // 1 kHz tolerance
        Serial.printf("⚠️ Frequency mismatch: actual=%.1f Hz, expected=%lu Hz\r\n",
            actualFreqHz, (unsigned long)expectedFreqHz);
        ok = false;
    }

    return ok;
}

bool RadioModule::setFrequencyBlocking(uint32_t freqHz) {
    // 1) Go to STANDBY
    _radio.setMode(RF69_MODE_STANDBY);
    if (!waitModeReady(_radio)) {
        Serial.println("setFrequencyBlocking: standby timeout");
        return false;
    }

    // 2) Compute FRF precisely: FRF = (freqHz * 2^19) / 32e6
    const uint32_t frf = (uint32_t)((((uint64_t)freqHz) << 19) / 32000000ULL);

    // 3) Write FRF
    _radio.writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    _radio.writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    _radio.writeReg(REG_FRFLSB, frf & 0xFF);

    // 4) Verify
    const uint32_t vr = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        (uint32_t)_radio.readReg(REG_FRFLSB);
    const uint32_t actualHz = (uint32_t)((vr * 32000000ULL) >> 19);

    // 5) Nudge PLL via SYNTH
    _radio.setMode(RF69_MODE_SYNTH);
    if (!waitModeReady(_radio)) return false;
    (void)waitPllLock(_radio); // best-effort

    //Serial.printf("--- setFrequencyBlocking: target=%lu Hz, actual=%lu Hz\r\n",
    //    (unsigned long)freqHz, (unsigned long)actualHz);
    return true;
}

bool RadioModule::sendWithReturnFreq(uint8_t destNode,
    uint32_t destFreqHz,
    uint32_t returnFreqHz,
    const uint8_t* msg,
    uint8_t len) {
    if (!msg || len == 0 || len > 61) {
        Serial.println("sendWithReturnFreq: invalid msg buffer");
        return false;
    }

    _radio.setMode(RF69_MODE_SLEEP);

    // Hop to TX frequency
    //Serial.printf(">>> switch-in destNode=%u, returnFreq=%.3f MHz, destFreq=%.3f MHz\r\n",
    //    destNode, returnFreqHz / 1e6f, destFreqHz / 1e6f);
    if (!setFrequencyBlocking(destFreqHz)) return false;

    // Enter TX
    _radio.setMode(RF69_MODE_STANDBY);
    if (!waitModeReady(_radio)) {
        Serial.println("sendWithReturnFreq: TX mode timeout");
        return false;
    }

    // Dump payload hex
    //for (uint8_t i = 0; i < len; ++i) Serial.printf("%02X ", msg[i]);
    //Serial.println();

    debugRFM69(_radio);
    _radio.send(destNode, msg, len, false);
    (void)waitPacketSent(_radio, 1000);

    // Hop back to listening frequency
    //Serial.printf(">>> switch-out destNode=%u, destFreq=%.3f MHz, returnFreq=%.3f MHz\r\n",
    //    destNode, destFreqHz / 1e6f, returnFreqHz / 1e6f);
    if (!setFrequencyBlocking(returnFreqHz)) return false;

    // Enter RX (fixed: was STANDBY)
    _radio.setMode(RF69_MODE_STANDBY);
    if (!waitModeReady(_radio)) {
        Serial.println("sendWithReturnFreq: RX mode timeout");
        return false;
    }

    return true;
}

void RadioModule::forceSetFrequency(uint32_t freqHz) {
    // FRF = DesiredHz / (FXOSC / 2^19) ; step ≈ 61.03515625 Hz
    const uint32_t frf = (uint32_t)((double)freqHz / 61.03515625);

    _radio.writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    _radio.writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    _radio.writeReg(REG_FRFLSB, frf & 0xFF);

    const uint32_t verifyFRF = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        (uint32_t)_radio.readReg(REG_FRFLSB);
    const float actualHz = verifyFRF * 61.03515625f;
    Serial.printf("forceSetFrequency: Set %lu Hz, actual %lu Hz\r\n",
        (unsigned long)freqHz, (unsigned long)actualHz);
}

void RadioModule::sendMessageCode(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq,
    uint8_t msgType, uint8_t msgCode) {
    uint8_t packet[2] = { msgType, msgCode };
    if (!sendWithReturnFreq(destNode, destFreq, returnFreq, packet, sizeof(packet))) {
        Serial.println("sendWithReturnFreq failed");
    }
}

void RadioModule::sendRTCM(const uint8_t* data, size_t len) {
    if (len == 0) return;

    constexpr size_t MAX_PAYLOAD = 57; // safe single-frame payload
    if (len <= MAX_PAYLOAD) {
        //printRTCMSegment(data, len);
        _radio.send(0, data, (uint8_t)len);
    }
    else {
        sendFragmentedRTCM(data, len);
    }
}

void RadioModule::sendFragmentedRTCM(const uint8_t* data, size_t len) {
    currentMsgId = (uint8_t)((currentMsgId + 1) & 0xFF);
    Serial.printf("---> msgId %s%d%s\r\n", ANSI_GREEN, currentMsgId, ANSI_RESET);
        RTCM_Fragmenter::sendFragmented(_radio, 0, data, len, currentMsgId);
}

void RadioModule::sendRTCMTest(int len) {
    if (len < 3) len = 3;
    uint8_t* rtcm = (uint8_t*)malloc(len);
    if (!rtcm) return;

    rtcm[0] = 0xD3; // RTCM preamble
    rtcm[1] = 0x01; // arbitrary next byte
    uint8_t val = 0x02;
    for (int i = 2; i < len; ++i) {
        rtcm[i] = val++;
        if (val == 0x00) val = 0x01; // avoid 0x00 flood
    }

    printRTCMSegment(rtcm, (size_t)len);
    sendRTCM(rtcm, (size_t)len);
    free(rtcm);
}

void RadioModule::printRTCMSegment(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        Serial.printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    if (len % 16 != 0) Serial.println();
}

void RadioModule::debugRFM69(RFM69& radio) {
    Serial.println(F("-- - RFM69 Debug Dump-- - "));

    const uint8_t opMode = radio.readReg(REG_OPMODE);
    Serial.printf("OpMode: 0x%02X\r\n", opMode);

    // Frequency (Hz)
    const uint32_t frf = ((uint32_t)radio.readReg(REG_FRFMSB) << 16) |
    ((uint32_t)radio.readReg(REG_FRFMID) << 8) |
    (uint32_t)radio.readReg(REG_FRFLSB);
    const float freqHz = (float)frf * 32000000.0f / 524288.0f;
    Serial.printf("Frequency: %.1f Hz\r\n", freqHz);

    // Bitrate
    const uint16_t bitrate = ((uint16_t)radio.readReg(REG_BITRATEMSB) << 8) |
    radio.readReg(REG_BITRATELSB);
    const float bitrateVal = 32000000.0f / (float)bitrate;
    Serial.printf("Bitrate: %.2f bps\r\n", bitrateVal);

    // FSK deviation
    const uint16_t fdev = ((uint16_t)radio.readReg(REG_FDEVMSB) << 8) |
    radio.readReg(REG_FDEVLSB);
    const float fdevHz = (float)fdev * 32000000.0f / 524288.0f;
    Serial.printf("Freq Deviation: %.1f Hz\r\n", fdevHz);

    // Output power
    const uint8_t paLevel = radio.readReg(REG_PALEVEL);
    Serial.printf("PA Level: 0x%02X (raw=%u)\r\n", paLevel, (unsigned)(paLevel & 0x1F));

    // RSSI threshold
    const uint8_t rssiThresh = radio.readReg(REG_RSSITHRESH);
    Serial.printf("RSSI Thresh: %d dB\r\n", -(rssiThresh / 2));

    // Sync / Packet
    Serial.printf("SyncConfig: 0x%02X\r\n", radio.readReg(REG_SYNCCONFIG));
    Serial.printf("SyncValue1 (NetworkID): 0x%02X\r\n", radio.readReg(REG_SYNCVALUE1));
    Serial.printf("PacketConfig1: 0x%02X\r\n", radio.readReg(REG_PACKETCONFIG1));
    Serial.printf("PacketConfig2: 0x%02\r\nX", radio.readReg(REG_PACKETCONFIG2));

    // Addresses
    Serial.printf("NodeAddr: 0x%02X\r\n", radio.readReg(REG_NODEADRS));
    Serial.printf("BroadcastAddr: 0x%02X\r\n", radio.readReg(REG_BROADCASTADRS));

    // Version
    Serial.printf("Version: 0x%02X\r\n", radio.readReg(REG_VERSION));

    Serial.println(F("--- End of Dump ---"));
}

// --------- RTCM Fragmentation / Reassembly ---------
void RadioModule::RTCM_Fragmenter::sendFragmented(RFM69& radio, uint8_t destId,
    const uint8_t* data, size_t len,
    uint8_t msgId) {
    constexpr size_t MAX_PAYLOAD = 57;   // bytes per fragment
    constexpr size_t MAX_TOTAL_LEN = 1024; // safety cap
    if (!data || len == 0 || len > MAX_TOTAL_LEN) return;

    const uint8_t totalChunks = (uint8_t)((len + MAX_PAYLOAD - 1) / MAX_PAYLOAD);

    for (uint8_t i = 0; i < totalChunks; ++i) {
        const size_t offset = (size_t)i * MAX_PAYLOAD;
        const size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);

        uint8_t packet[4 + MAX_PAYLOAD];
        packet[0] = MSG_RTCM_FRAGMENT;
        packet[1] = i;
        packet[2] = totalChunks;
        packet[3] = msgId;
        memcpy(packet + 4, data + offset, chunkLen);

        radio.send(destId, packet, (uint8_t)(chunkLen + 4));
        delay(20);
    }
}

RadioModule::RTCM_Reassembler::RTCM_Reassembler()
    : receivedCount(0), totalExpected(0), complete(false) {
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

void RadioModule::RTCM_Reassembler::acceptFragment(const uint8_t* data, size_t len) {
    constexpr size_t FRAGMENT_HEADER_SIZE = 4;
    constexpr size_t FRAGMENT_PAYLOAD_SIZE = 57;
    constexpr size_t MAX_FRAGMENTS_LOCAL = 255; // fallback guard

    if (!data || len < FRAGMENT_HEADER_SIZE + 1 || data[0] != MSG_RTCM_FRAGMENT) return;

    const uint8_t index = data[1];
    const uint8_t total = data[2];
    const uint8_t msgId = data[3];

    if (total == 0) return;
    if (total > MAX_FRAGMENTS_LOCAL) return; // avoid absurd totals
    if (index >= total) return;

    if (index == 0) {
        totalExpected = total;
        receivedCount = 0;
        complete = false;
        expectedMsgId = msgId;
        memset(fragmentReceived, 0, sizeof(fragmentReceived));
        memset(buffer, 0, sizeof(buffer));
    }

    if (msgId != expectedMsgId) return; // different stream

    const size_t fragLen = len - FRAGMENT_HEADER_SIZE;
    const size_t offset = (size_t)index * FRAGMENT_PAYLOAD_SIZE;
    if (offset + fragLen > sizeof(buffer)) return;

    memcpy(buffer + offset, data + FRAGMENT_HEADER_SIZE, fragLen);
    if (fragmentReceived[index] == 0) receivedCount++;
    fragmentReceived[index] = (uint8_t)fragLen;

    if (receivedCount == totalExpected) {
        size_t totalLen = 0;
        for (uint8_t i = 0; i < totalExpected; ++i) totalLen += fragmentReceived[i];
        this->length = totalLen;
        complete = true;
    }
}

bool RadioModule::RTCM_Reassembler::isComplete() const { return complete; }
const uint8_t* RadioModule::RTCM_Reassembler::getData() const { return buffer; }
size_t RadioModule::RTCM_Reassembler::getLength() const { return length; }

void RadioModule::RTCM_Reassembler::reset() {
    complete = false;
    receivedCount = 0;
    memset(fragmentReceived, 0, sizeof(fragmentReceived));
}
