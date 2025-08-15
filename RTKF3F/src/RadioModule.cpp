//RadioModule.cpp

#include <Arduino.h>    
#include <SPI.h>
#include "RadioModule.h"
//#include <RFM69Debug.h>



RadioModule::RadioModule(int csPin, int irqPin, bool isHighPower)
    : _csPin(csPin), _irqPin(irqPin), _isHighPower(isHighPower),
    _radio(csPin, irqPin, isHighPower) {
}

void RadioModule::joeSetNetwork(uint8_t networkid) {
    // Set network ID (sync value)
    _radio.writeReg(REG_SYNCVALUE1, networkid);
    uint8_t syncVal = _radio.readReg(REG_SYNCVALUE1);
    Serial.printf("✅ Forced SyncValue1 to 0x%02X\n", syncVal);
}

bool RadioModule::init(int8_t pMISO, int8_t pMOSI, int8_t pSCK,
    uint16_t nodeid, uint8_t networkid, uint32_t frequencyHz)
{
    SPI.begin(pSCK, pMISO, pMOSI, _csPin); // _csPin only affects default, driver controls CS

    _nodeid = nodeid;
	_networkid = networkid;

    if (!_radio.initialize(RF69_868MHZ, nodeid)) {
        Serial.println("RFM69 init failed");
        return false;
    }

	Serial.printf("_isHighPower: %s\n", _isHighPower ? "true" : "false");
    _radio.setHighPower(_isHighPower);
    _radio.setFrequency(frequencyHz);
    _radio.encrypt(NULL);
    _radio.setAddress(nodeid);
    joeSetNetwork(networkid);

    _radio.setMode(RF69_MODE_STANDBY);

	debugRFM69(_radio);

    checkRadioSettings(nodeid, networkid, frequencyHz);

	sendMessageCode(0, frequencyHz, frequencyHz, MSG_INFORMATION, INFO_DEVICE_STARTING);

    while (true);

    uint8_t version = _radio.readReg(0x10);
    //Serial.printf("Radio version: 0x%02X (expected 0x24)\n", version);
    return version == 0x24;

}


void RadioModule::send(uint16_t toAddress, const void *buffer, uint8_t bufferSize, bool requestACK = false) {
    _radio.writeReg(REG_SYNCVALUE1, 100);
    _radio.send(toAddress, buffer, bufferSize, requestACK);
}



int RadioModule::getSenderId() {
	return _radio.SENDERID;
}

int RadioModule::getTargetId() {
	return _radio.TARGETID;
}

bool RadioModule::receive(uint8_t* data, size_t& len) {
    if (_radio.receiveDone()) {
        size_t available = _radio.DATALEN;
        if (available > len) { // Prevent buffer overflow
            return false;
        }
        memcpy(data, _radio.DATA, available);
        len = available;  // Update length by reference
        return true;
    }
    return false;
}

void dbgPrintFreq(RFM69& r) {
    uint32_t frf = ((uint32_t)r.readReg(0x07) << 16)
        | ((uint32_t)r.readReg(0x08) << 8)
        | (uint32_t)r.readReg(0x09);
    float mhz = frf * 61.03515625f / 1e6f; // Fstep = 32e6/2^19
    Serial.printf("FRF=0x%06lX -> %.3f MHz\r\n", frf, mhz);

}

static inline bool waitPacketSent(RFM69& r, uint16_t timeout_ms = 20) {
    uint32_t t0 = millis();
    while (!(r.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT)) {
        if ((millis() - t0) > timeout_ms) return false;
    }
    return true;
}

bool RadioModule::checkRadioSettings(uint8_t expectedNodeId,
    uint8_t expectedNetworkId,
    uint32_t expectedFreqHz)
{
    bool ok = true;

    // Node ID
    uint8_t actualNodeId = _radio.readReg(REG_NODEADRS);
    if (actualNodeId != expectedNodeId) {
        Serial.printf("⚠️ Node ID mismatch: actual=%u, expected=%u\n", actualNodeId, expectedNodeId);
        ok = false;
    }

    // Network ID
    uint8_t actualNetworkId = _radio.readReg(REG_SYNCVALUE1);
    if (actualNetworkId != expectedNetworkId) {
        Serial.printf("⚠️ Network ID mismatch: actual=%u, expected=%u\n", actualNetworkId, expectedNetworkId);
        ok = false;
    }

    // Frequency (FRF register is 3 bytes: 0x07, 0x08, 0x09)
    uint32_t frf = ((uint32_t)_radio.readReg(0x07) << 16) |
        ((uint32_t)_radio.readReg(0x08) << 8) |
        _radio.readReg(0x09);

    float actualFreqHz = (32.0f * 1000000.0f * frf) / 524288.0f;

    int32_t diffHz = abs((int32_t)(actualFreqHz - expectedFreqHz));
    if (diffHz > 1000) {  // 1 kHz tolerance
        Serial.printf("⚠️ Frequency mismatch: actual=%.1f Hz, expected=%lu Hz\n",
            actualFreqHz, expectedFreqHz);
        ok = false;
    }

    return ok;
}

static inline bool waitModeReady(RFM69& r, uint32_t timeoutMs = 50) {
    uint32_t t0 = millis();
    while (!(r.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY)) {
        if (millis() - t0 > timeoutMs) return false;
    }
    return true;
}

// Optional: wait for PLL lock after tuning (helps when hopping fast)
static inline bool waitPllLock(RFM69& r, uint32_t timeoutMs = 5) {
    // PLLLOCK bit is 0x10 in IRQFLAGS1 on RFM69
    uint32_t t0 = millis();
    while (!(r.readReg(REG_IRQFLAGS1) & 0x10)) { // RF_IRQFLAGS1_PLLLOCK
        if (millis() - t0 > timeoutMs) return false;
    }
    return true;
}

bool RadioModule::setFrequencyBlocking(uint32_t freqHz)
{
    // 1) Go to STANDBY and wait
    _radio.setMode(RF69_MODE_STANDBY);
    if (!waitModeReady(_radio)) {
        Serial.println("setFrequencyBlocking: standby timeout");
        return false;
    }

    // 2) Compute FRF with 64-bit precision
    // FRF = (freqHz * 2^19) / 32e6
    uint32_t frf = (uint32_t)((((uint64_t)freqHz) << 19) / 32000000ULL);

    // 3) Write FRF registers (MSB..LSB)
    _radio.writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    _radio.writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    _radio.writeReg(REG_FRFLSB, frf & 0xFF);

    // 4) Optional: verify
    uint32_t vr = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        (uint32_t)_radio.readReg(REG_FRFLSB);
    // Convert back to Hz: FXOSC / 2^19 = 32e6 / 524288 ≈ 61.03515625
    uint32_t actualHz = (uint32_t)((vr * 32000000ULL) >> 19); // exact inverse w/ 64-bit

    // 5) (Optional) nudge PLL by briefly entering SYNTH/FS or TX and wait lock
    _radio.setMode(RF69_MODE_SYNTH);
    if (!waitModeReady(_radio)) return false;
    waitPllLock(_radio); // don’t fail hard; just improve reliability

    // Caller sets final RX/TX mode; we leave radio in SYNTH (fast to switch)
    Serial.printf("--- setFrequencyBlocking: target=%lu Hz, actual=%lu Hz\n",
        (unsigned long)freqHz, (unsigned long)actualHz);
    return true;
}

bool RadioModule::sendWithReturnFreq(uint8_t destNode,
    uint32_t destFreqHz,
    uint32_t returnFreqHz,
    const uint8_t* msg,
    uint8_t len)
{
    if (!msg || len == 0 || len > 61) {
        Serial.println("sendWithReturnFreq: invalid msg buffer");
        return false;
    }

    _radio.setMode(RF69_MODE_SLEEP);

    // Hop to TX frequency
    Serial.printf(">>> switch-in destNode=%d, returnFreq=%.3f , destFreq=%.3f MHz MHz\n",
		destNode, returnFreqHz / 1e6f , destFreqHz / 1e6f);
    if (!setFrequencyBlocking(destFreqHz)) return false;

    // Enter TX and wait ready (optional but nice)
    _radio.setMode(RF69_MODE_TX);
    if (!waitModeReady(_radio)) {
        Serial.println("sendWithReturnFreq: TX mode timeout");
        return false;
    }

    Serial.println();
    for (int i = 0; i < len; ++i) {
        Serial.printf("%02X ", msg[i]);
	}
    Serial.println();

    // Fire
	debugRFM69(_radio);
    _radio.send(destNode, msg, len, false);

    // Hop back to RX frequency
    Serial.printf(">>> switch-out destNode=%d, destFreq=%.3f MHz, returnFreq=%.3f MHz\n",
        destNode, destFreqHz / 1e6f, returnFreqHz / 1e6f);
        if (!setFrequencyBlocking(returnFreqHz)) return false;

    // Enter RX and wait ready
    _radio.setMode(RF69_MODE_STANDBY);
    if (!waitModeReady(_radio)) {
        Serial.println("sendWithReturnFreq: RX mode timeout");
        return false;
    }

    return true;
}
void RadioModule::forceSetFrequency(uint32_t freqHz)
{
    // Formula from HopeRF/Semtech datasheet:
    // FRF = (DesiredFreq * 2^19) / FXOSC
    // FXOSC is 32 MHz for RFM69
    uint32_t frf = (uint32_t)((double)freqHz / 61.03515625); // 32e6 / 2^19 ≈ 61.035 Hz steps

    _radio.writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    _radio.writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    _radio.writeReg(REG_FRFLSB, frf & 0xFF);

    // Optional: verify
    uint32_t verifyFRF = ((uint32_t)_radio.readReg(REG_FRFMSB) << 16) |
        ((uint32_t)_radio.readReg(REG_FRFMID) << 8) |
        ((uint32_t)_radio.readReg(REG_FRFLSB));
    float actualHz = verifyFRF * 61.03515625f;
    Serial.printf("forceSetFrequency: Set %lu Hz, actual %lu Hz\n", freqHz, (uint32_t)actualHz);
}

void RadioModule::sendMessageCode(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq, uint8_t msgType, uint8_t msgCode) {
    uint8_t packet[2];
    packet[0] = msgType;  // tag
    packet[1] = msgCode;

    //Serial.printf("sendWithReturnFreq dest=%d network=%d\r\n", destNode, NETWORK_ID);
    if (!sendWithReturnFreq(destNode, destFreq, returnFreq, packet, sizeof(packet))) {
        Serial.println("sendWithReturnFreq failed");
    }
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
	Serial.printf("---> msgId " ANSI_GREEN "%d" ANSI_RESET "\r\n", currentMsgId);
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

void RadioModule::debugRFM69(RFM69& radio) {
    Serial.println(F("\n--- RFM69 Debug Dump ---"));

    // Operating mode
    byte opMode = radio.readReg(REG_OPMODE);
    Serial.printf("OpMode: 0x%02X\n", opMode);

    // Frequency (in Hz)
    uint32_t frf = ((uint32_t)radio.readReg(REG_FRFMSB) << 16) |
        ((uint16_t)radio.readReg(REG_FRFMID) << 8) |
        radio.readReg(REG_FRFLSB);
    float freqHz = (float)frf * 32000000.0 / 524288.0; // FXOSC / 2^19
    Serial.printf("Frequency: %.1f Hz\n", freqHz);

    // Bitrate
    uint16_t bitrate = ((uint16_t)radio.readReg(REG_BITRATEMSB) << 8) |
        radio.readReg(REG_BITRATELSB);
    float bitrateVal = 32000000.0 / bitrate;
    Serial.printf("Bitrate: %.2f bps\n", bitrateVal);

    // FSK Deviation
    uint16_t fdev = ((uint16_t)radio.readReg(REG_FDEVMSB) << 8) |
        radio.readReg(REG_FDEVLSB);
    float fdevHz = (float)fdev * 32000000.0 / 524288.0;
    Serial.printf("Freq Deviation: %.1f Hz\n", fdevHz);

    // Output power
    byte paLevel = radio.readReg(REG_PALEVEL);
    Serial.printf("PA Level: 0x%02X (Power: %d dBm)\n", paLevel, paLevel & 0x1F);

    // RSSI Threshold
    byte rssiThresh = radio.readReg(REG_RSSITHRESH);
    Serial.printf("RSSI Thresh: %d dB\n", -(rssiThresh / 2));

    // Sync Config
    Serial.printf("SyncConfig: 0x%02X\n", radio.readReg(REG_SYNCCONFIG));
    Serial.printf("SyncValue1 (NetworkID): 0x%02X\n", radio.readReg(REG_SYNCVALUE1));

    // Packet Config
    Serial.printf("PacketConfig1: 0x%02X\n", radio.readReg(REG_PACKETCONFIG1));
    Serial.printf("PacketConfig2: 0x%02X\n", radio.readReg(REG_PACKETCONFIG2));

    // Node & Broadcast IDs
    Serial.printf("NodeAddr: 0x%02X\n", radio.readReg(REG_NODEADRS));
    Serial.printf("BroadcastAddr: 0x%02X\n", radio.readReg(REG_BROADCASTADRS));

    // Version (check hardware)
    Serial.printf("Version: 0x%02X\n", radio.readReg(REG_VERSION));

    Serial.println(F("--- End of Dump ---\n"));
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


