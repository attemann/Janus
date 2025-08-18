// GNSSModule.cpp
#include <Arduino.h>    
#include <RFM69.h>
#include <HardwareSerial.h>
#include <RTKF3F.h>

#include "GNSSModule.h"

GNSSModule::GNSSModule(HardwareSerial& serial)
    : _serial(serial), _nmeaIdx(0), rtcmHandler(this) {
}

void GNSSModule::begin(uint32_t baud, int RX, int TX) {
    _serial.begin(baud, SERIAL_8N1, RX, TX);
}

bool GNSSModule::gpsDataAvailable() {
    return _serial.available() > 0;
}

int GNSSModule::detectUARTPort() {
    const char* testCommands[] = {
        "versiona com1",
        "versiona com2",
        "versiona com3"
    };

    for (int port = 1; port <= 3; port++) {
        _serial.flush(); // Clear any previous data
        delay(100);
        _serial.print(testCommands[port - 1]);
        _serial.print("\r\n");

		Serial.println("GNSS: Testing port " + String(port) + " with command: " + testCommands[port - 1]);

        unsigned long startTime = millis();
        String response = "";

        while (millis() - startTime < COMMANDDELAY) {
            while (_serial.available()) {
                char c = _serial.read();
                response += c;
            }
        }

        if (response.indexOf("VERSIONA") != -1 || response.indexOf("#VERSIONA") != -1) {
            return port;
        }
    }
    return 0; // None found
}

void GNSSModule::sendCommand(const String& command) {
    Serial.print("GPS: Sending command: ");
    Serial.println(command);
    unsigned long start = millis();
    _serial.print(command);
    while ((millis() - start) < COMMANDDELAY) {
        if (_serial.available()) Serial.write(_serial.read());
    }
}

void GNSSModule::sendReset() {
	String command = "freset\r\n";
    Serial.print("GPS: Sending command: ");
    Serial.println(command);
    unsigned long start = millis();
    _serial.print(command);
    while ((millis() - start) < 5000) {
        if (_serial.available()) Serial.write(_serial.read());
    }
}


String GNSSModule::fixTypeToString(int fixType) {
    switch (fixType) {
    case 0: return "Inv. (no fix)";
    case 1: return "GPS fix";
    case 2: return "DGPS fix";
    case 3: return "PPS fix";
    case 4: return "RTK Float";
    case 5: return "RTK Fixed";
    case 6: return "Dead Reckoning";
    case 7: return "Manual Input Mode";
    case 8: return "Simulation Mode";
    default: return "Other Fix Type";
    }
}

GNSSModule::GNSSMessage GNSSModule::readGPS() {
    GNSSMessage result;

    int first = _serial.peek();
    if (first < 0) return result;

    if (first == '$') {
        if (readNMEA()) {
            result.type = GNSSMessageType::NMEA;
            result.data = _gpsBuf;
            result.length = strlen((char*)_gpsBuf);
            return result;
        }
    }
    else if (first == '#') {
        if (readCommandResponse()) {
            printAscii();
            result.type = GNSSMessageType::COMMAND_RESPONSE;
            result.data = _gpsBuf;
            result.length = strlen((char*)_gpsBuf);
            return result;
        }
    }
    else if (first == 0xD3) {
        if (readRTCM(_gpsBuf, _gpsBufLen)) {
            result.type = GNSSMessageType::RTCM;
            result.data = _gpsBuf;
            result.length = _gpsBufLen;
            return result;
        }
    }
    else if (first == 0xB5) {
        if (_serial.available() >= 2) {
            int sync1 = _serial.read();
            int sync2 = _serial.read();
            if (sync1 == 0xB5 && sync2 == 0x62) {
                while (_serial.available() < 4);
                uint8_t cls = _serial.read();
                uint8_t id = _serial.read();
                uint16_t len = _serial.read() | (_serial.read() << 8);

                if (len > sizeof(_gpsBuf) - 8) return result;

                _gpsBuf[0] = cls;
                _gpsBuf[1] = id;
                for (int i = 0; i < len; ++i)
                    _gpsBuf[2 + i] = _serial.read();
                uint8_t ckA = _serial.read();
                uint8_t ckB = _serial.read();

                // TODO: optionally verify checksum

                result.type = GNSSMessageType::UBX;
                result.data = _gpsBuf;
                result.length = len + 2;

                if (cls == 0x01 && id == 0x3C)
                    result.type = GNSSMessageType::UBX_RELPOSNED;
                else if (cls == 0x01 && id == 0x07)
                    result.type = GNSSMessageType::UBX_NAV_PVT;
                else if (cls == 0x05 && id == 0x01)
                    result.type = GNSSMessageType::UBX_ACK_ACK;
                else if (cls == 0x05 && id == 0x00)
                    result.type = GNSSMessageType::UBX_ACK_NAK;

                return result;
            }
        }
    }
    else {
        _serial.read();  // consume unknown byte
    }

    return result;  // .type == NONE
}

void GNSSModule::printAscii() {
    for (size_t i = 0; i < _gpsBufLen; ++i) {
        char c = _gpsBuf[i];
        if (c >= 32 && c <= 126) // Printable ASCII range
            Serial.print(c);
        else if (c == '\r')
            Serial.print("\\r");
        else if (c == '\n')
            Serial.print("\\n");
        else
            Serial.print('.');
    }
    Serial.println();
}

bool GNSSModule::readNMEA() {
    static uint8_t nmeaBuf[128];
    static size_t nmeaPos = 0;
    static bool inSentence = false;

    while (_serial.available()) {
        char c = _serial.read();

        if (!inSentence) {
            if (c == '$') {
                inSentence = true;
                nmeaPos = 0;
                nmeaBuf[nmeaPos++] = c;
            }
        }
        else {
            if (nmeaPos < sizeof(nmeaBuf) - 1) {
                nmeaBuf[nmeaPos++] = c;
            }
            else {
                // Overflow, reset and wait for next sentence
                inSentence = false;
                nmeaPos = 0;
                continue;
            }
            if (c == '\n') {
                nmeaBuf[nmeaPos] = '\0'; // null-terminate
                // Save to class properties:
                memcpy(_gpsBuf, nmeaBuf, nmeaPos + 1);
                _gpsBufLen = nmeaPos;
                inSentence = false; // Ready for next one
                return true;
            }
        }
    }
    return false;
}

bool GNSSModule::readCommandResponse() {
    size_t idx = 0;
    while (_serial.available()) {
        char c = _serial.read();
        if (idx < 127) { // leave room for null terminator
            _gpsBuf[idx++] = c;
        }
        if (c == '\n' || idx >= 127) { // End of response or buffer full
            _gpsBufLen = idx;
            _gpsBuf[idx] = '\0'; // Null-terminate (safe for printing)
            // Optionally strip trailing '\r'
            if (idx >= 2 && _gpsBuf[idx - 2] == '\r') {
                _gpsBuf[idx - 2] = '\n';
                _gpsBuf[idx - 1] = '\0';
                idx--;
            }
            return true;
        }
    }
    _gpsBufLen = idx;
    return false;
}

bool GNSSModule::readRTCM(uint8_t* buffer, size_t& len) {
    const unsigned long timeout = 100;  // milliseconds
    unsigned long startTime;

    while (_serial.available()) {
        // Look for 0xD3 sync byte
        if (_serial.read() != 0xD3) continue;

        // Wait for length bytes
        startTime = millis();
        while (_serial.available() < 2) {
            if (millis() - startTime > timeout) return false;
        }
        uint8_t lenH = _serial.read();
        uint8_t lenL = _serial.read();
        uint16_t payloadLen = ((lenH & 0x03) << 8) | lenL;

        // Reject oversized messages
        if (payloadLen > 1023) return false;

        size_t totalLen = 3 + payloadLen + 3;  // header + payload + CRC

        // Wait for full message
        startTime = millis();
        while (_serial.available() < payloadLen + 3) {
            if (millis() - startTime > timeout) return false;
        }

        // Fill buffer
        buffer[0] = 0xD3;
        buffer[1] = lenH;
        buffer[2] = lenL;
        for (size_t i = 0; i < payloadLen + 3; i++) {
            buffer[3 + i] = _serial.read();
        }

        len = totalLen;
        return isValidRTCM(buffer, len);
    }

    return false;
}

uint16_t GNSSModule::getRTCMType(const uint8_t* buf, size_t len) {
    if (len < 6 || buf[0] != 0xD3) return 0; // sanity check
    uint16_t type = ((buf[3] << 4) | (buf[4] >> 4)) & 0x0FFF;
    return type;
}

bool GNSSModule::isValidRTCM(const uint8_t* data, size_t len) {
    if (data[0] != 0xD3 || len < 6) return false;
    uint16_t payloadLen = ((data[1] & 0x3F) << 8) | data[2];
    if (3 + payloadLen + 3 > len) return false; // not enough data

    uint32_t crc = calculateCRC24Q(data, 3 + payloadLen);
    uint32_t recvCrc = (data[3 + payloadLen] << 16) | (data[4 + payloadLen] << 8) | data[5 + payloadLen];

    return crc == recvCrc;
}

uint32_t GNSSModule::calculateCRC24Q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 16);
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

int GNSSModule::parseField(const String& line, int num) {
    int start = line.indexOf(",") + 1;
    for (int i = 0; i < num; i++) {
        start = line.indexOf(",", start) + 1;
    }
    int end = line.indexOf(",", start);
    return line.substring(start, end).toInt();
}

bool GNSSModule::parseGGA(const uint8_t* buf, size_t len, GNSSFix& fix) {
    const int MAX_FIELDS = 20;
    const int BUF_SIZE = 128;
    char local[BUF_SIZE];

    // 1. Make a safe local copy and ensure null-terminated
    if (len >= BUF_SIZE) len = BUF_SIZE - 1;
    memcpy(local, buf, len);
    local[len] = '\0';

    // 2. Skip initial '$' if present
    char* p = local;
    if (*p == '$') ++p;

    // 3. Tokenize by ','
    const char* fields[MAX_FIELDS] = { nullptr };
    int fieldCount = 0;
    fields[fieldCount++] = strtok(p, ",");
    while (fieldCount < MAX_FIELDS) {
        char* token = strtok(nullptr, ",");
        if (!token) break;
        fields[fieldCount++] = token;
    }

    if (fieldCount < 9) return false;

    // Parse time
    if (fields[1] && *fields[1]) {
        float rawTime = atof(fields[1]);
        fix.hour = int(rawTime / 10000);
        fix.minute = int(fmod(rawTime / 100, 100));
        fix.second = fmod(rawTime, 100);
    }
    else {
        fix.hour = fix.minute = fix.second = 0;
    }

    // Lat/lon
    float latRaw = fields[2] && *fields[2] ? atof(fields[2]) : 0;
    char latDir = fields[3] && *fields[3] ? fields[3][0] : 0;
    float lonRaw = fields[4] && *fields[4] ? atof(fields[4]) : 0;
    char lonDir = fields[5] && *fields[5] ? fields[5][0] : 0;

    if (latRaw > 0 && (latDir == 'N' || latDir == 'S')) {
        int deg = int(latRaw / 100);
        float min = latRaw - deg * 100;
        fix.lat = deg + min / 60.0f;
        if (latDir == 'S') fix.lat = -fix.lat;
    }
    else {
        fix.lat = 0;
    }

    if (lonRaw > 0 && (lonDir == 'E' || lonDir == 'W')) {
        int deg = int(lonRaw / 100);
        float min = lonRaw - deg * 100;
        fix.lon = deg + min / 60.0f;
        if (lonDir == 'W') fix.lon = -fix.lon;
    }
    else {
        fix.lon = 0;
    }

    fix.fixType = fields[6] && *fields[6] ? atoi(fields[6]) : 0;
    fix.SIV = fields[7] && *fields[7] ? atoi(fields[7]) : 0;
    float hdop = fields[8] && *fields[8] ? atof(fields[8]) : 0;
    fix.HDOP = int(hdop * 100);
    fix.alt = fields[9] && *fields[9] ? atof(fields[9]) : 0;

    fix.gpsFix = (fix.fixType > 0);
    fix.diffUsed = (fix.fixType >= 2);
    fix.rtkFix = (fix.fixType == 4);
    fix.rtkFloat = (fix.fixType == 5);

    return true;
}


const char* GNSSModule::getRTCMName(uint16_t type) {
    switch (type) {
    case 1001: return "RTCM 1001: GPS L1 Obs";
    case 1002: return "RTCM 1002: GPS L1 Obs (Ext)";
    case 1003: return "RTCM 1003: GPS L1/L2 Obs";
    case 1004: return "RTCM 1004: GPS L1/L2 Obs (Ext)";
    case 1005: return "RTCM 1005: Stationary RTK (No height)";
    case 1006: return "RTCM 1006: Stationary RTK (with height)";
    case 1007: return "RTCM 1007: Antenna Descriptor";
    case 1008: return "RTCM 1008: Antenna Descriptor + Serial Number";
    case 1033: return "RTCM 1033: Antenna Descriptor & Firmware Version";
    case 1071: return "RTCM 1071: GPS MSM1";
    case 1072: return "RTCM 1072: GPS MSM2";
    case 1073: return "RTCM 1073: GPS MSM3";
    case 1074: return "RTCM 1074: GPS MSM4";
    case 1075: return "RTCM 1075: GPS MSM5";
    case 1076: return "RTCM 1076: GPS MSM6";
    case 1077: return "RTCM 1077: GPS MSM7";
    case 1081: return "RTCM 1081: GLONASS MSM1";
    case 1082: return "RTCM 1082: GLONASS MSM2";
    case 1083: return "RTCM 1083: GLONASS MSM3";
    case 1084: return "RTCM 1084: GLONASS MSM4";
    case 1085: return "RTCM 1085: GLONASS MSM5";
    case 1086: return "RTCM 1086: GLONASS MSM6";
    case 1087: return "RTCM 1087: GLONASS MSM7";
    case 1091: return "RTCM 1091: Galileo MSM1";
    case 1092: return "RTCM 1092: Galileo MSM2";
    case 1093: return "RTCM 1093: Galileo MSM3";
    case 1094: return "RTCM 1094: Galileo MSM4";
    case 1095: return "RTCM 1095: Galileo MSM5";
    case 1096: return "RTCM 1096: Galileo MSM6";
    case 1097: return "RTCM 1097: Galileo MSM7";
    case 1121: return "RTCM 1121: BeiDou MSM1";
    case 1122: return "RTCM 1122: BeiDou MSM2";
    case 1123: return "RTCM 1123: BeiDou MSM3";
    case 1124: return "RTCM 1124: BeiDou MSM4";
    case 1125: return "RTCM 1125: BeiDou MSM5";
    case 1126: return "RTCM 1126: BeiDou MSM6";
    case 1127: return "RTCM 1127: BeiDou MSM7";
    case 1230: return "RTCM 1230: GLONASS Code-Phase Biases";
    case 4072: return "RTCM 4072: Proprietary (e.g. u-blox)";
    case 4073: return "RTCM 4073: u-blox Subtype A";
    case 4074: return "RTCM 4074: u-blox Subtype B";
    default: return "RTCM: Unknown or Unsupported Type";
    }
}

uint16_t GNSSModule::getRTCMBits(const uint8_t* buffer, int startBit, int bitLen) {
    uint32_t result = 0;
    for (int i = 0; i < bitLen; i++) {
        int byteIndex = (startBit + i) / 8;
        int bitIndex = 7 - ((startBit + i) % 8);
        result <<= 1;
        result |= (buffer[byteIndex] >> bitIndex) & 0x01;
    }
    return result;
}

void GNSSModule::showFix(const GNSSFix& fix) {
    Serial.printf(
        "Time %02d:%02d:%05.2f "
        " %.6f%c"
        " %.6f%c"
        " %.2fm\n"
        " %d %s"
        " SIV=%d"
        " HDOP=%.2f "
        " GPS Fix [%s]"
        " DGPS Used [%s]"
        " RTK Float [%s]"
        " RTK Fixed [%s]\n"
        //" relNorth %.3f m"
        //" relEast  %.3f m"
        //" relDown  %.3f m\n"
        //" adjNorth %.3f m"
        //" adjEast  %.3f m"
        //" adjDown  %.3f m\n"
        ,

        fix.hour, fix.minute, fix.second,
        fabs(fix.lat), (fix.lat < 0 ? 'S' : 'N'),
        fabs(fix.lon), (fix.lon < 0 ? 'W' : 'E'),
        fix.alt,
        fix.fixType, fixTypeToString(fix.fixType).c_str(),
        fix.SIV,
        fix.HDOP / 100.0f,  // convert HDOP from cm to float
        fix.gpsFix ? "1" : "0",
        fix.diffUsed ? "1" : "0",
        fix.rtkFloat ? "1" : "0",
        fix.rtkFix ? "1" : "0"

        //fix.relNorth, fix.relEast, fix.relDown,
        //fix.adjNorth, fix.adjEast, fix.adjDown
    );
}

GNSSModule::RTCMHandler::RTCMHandler(GNSSModule* gnss) : parent(gnss) {
    add("rtcm1033", 1033, 10.0f, true, "Antenna + Firmware");
    add("rtcm1006", 1006, 10.0f, true, "Stat. Ref (with height)");
    add("rtcm1074", 1074, 1.0f,  true, "GPS MSM4");
    add("rtcm1124", 1124, 1.0f,  true, "BeiDou MSM4");
    add("rtcm1084", 1084, 1.0f,  true, "GLONASS MSM4");
    add("rtcm1094", 1094, 1.0f,  true, "Galileo MSM4");

    // Example messages, add more as needed
    //add("rtcm1001", 1001, 1.0f, false, "GPS L1 Obs");
    //add("rtcm1002", 1002, 1.0f, false, "GPS L1 Obs (Ext)");
    //add("rtcm1003", 1003, 1.0f, false, "GPS L1/L2 Obs");
    //add("rtcm1004", 1004, 1.0f, false, "GPS L1/L2 Obs (Ext)");
    //add("rtcm1005", 1005, 1.0f, false, "Stat. Ref (no height)");

    //add("rtcm1007", 1007, 1.0f, false, "Antenna Descriptor");
    //add("rtcm1008", 1008, 1.0f, false, "Ant. Descriptor + SN");

    //add("rtcm1071", 1071, 1.0f, false, "GPS MSM1");
    //add("rtcm1072", 1072, 1.0f, false, "GPS MSM2");
    //add("rtcm1073", 1073, 1.0f, false, "GPS MSM3");

    //add("rtcm1075", 1075, 1.0f, false, "GPS MSM5");
    //add("rtcm1076", 1076, 1.0f, false, "GPS MSM6");
    //add("rtcm1077", 1077, 1.0f, false, "GPS MSM7");

    //add("rtcm1081", 1081, 1.0f, false, "GLONASS MSM1");
    //add("rtcm1082", 1082, 1.0f, false, "GLONASS MSM2");
    //add("rtcm1083", 1083, 1.0f, false, "GLONASS MSM3");

    //add("rtcm1085", 1085, 1.0f, false, "GLONASS MSM5");
    //add("rtcm1086", 1086, 1.0f, false, "GLONASS MSM6");
    //add("rtcm1087", 1087, 1.0f, false, "GLONASS MSM7");

    //add("rtcm1091", 1091, 1.0f, false, "Galileo MSM1");
    //add("rtcm1092", 1092, 1.0f, false, "Galileo MSM2");
    //add("rtcm1093", 1093, 1.0f, false, "Galileo MSM3");

    //add("rtcm1095", 1095, 1.0f, false, "Galileo MSM5");
    //add("rtcm1096", 1096, 1.0f, false, "Galileo MSM6");
    //add("rtcm1097", 1097, 1.0f, false, "Galileo MSM7");

    //add("rtcm1121", 1121, 1.0f, false, "BeiDou MSM1");
    //add("rtcm1122", 1122, 1.0f, false, "BeiDou MSM2");
    //add("rtcm1123", 1123, 1.0f, false, "BeiDou MSM3");

    //add("rtcm1125", 1125, 1.0f, false, "BeiDou MSM5");
    //add("rtcm1126", 1126, 1.0f, false, "BeiDou MSM6");
    //add("rtcm1127", 1127, 1.0f, false, "BeiDou MSM7");

    //add("rtcm1230", 1230, 10.0f, false, "GLONASS L1/L2 Bias");

    //add("rtcm4072", 4072, 1.0f, false, "u-blox Proprietary");
    //add("rtcm4073", 4073, 1.0f, false, "u-blox Subtype A");
    //add("rtcm4074", 4074, 1.0f, false, "u-blox Subtype B");

}

void GNSSModule::RTCMHandler::add(const char* name, uint16_t id, float freq, bool enabled, const char* desc) {
    if (count < MAX_MSGS) {
        messages[count++] = { name, id, freq, enabled, desc, 0 };
    }
}

void GNSSModule::RTCMHandler::enable(int index, bool value) {
    if (index < 0 || index >= count) return;
    if (messages[index].enabled == value) return;
    messages[index].enabled = value;
    sendConfig(index);
}

// Enable/disable RTCM by ID
void GNSSModule::RTCMHandler::enableById(uint16_t id, bool value) {
    int idx = findById(id);
    if (idx >= 0) enable(idx, value);
}

// Set frequency by ID
void GNSSModule::RTCMHandler::setFrequencyById(uint16_t id, float freq) {
    int idx = findById(id);
    if (idx >= 0) setFrequency(idx, freq);
}

void GNSSModule::RTCMHandler::setFrequency(int index, float freq) {
    if (index < 0 || index >= count) return;
    messages[index].frequency = freq;
    if (messages[index].enabled)
        sendConfig(index);
}

void GNSSModule::RTCMHandler::sendConfig(int index) {
    if (!parent) return;
    auto& msg = messages[index];
    String cmd;
    if (msg.enabled)
        cmd = String(msg.name) + " com2 " + String(msg.frequency, 1) + "\r\n";
    else
        cmd = "unlog " + String(msg.name) + " com2\r\n";
    parent->sendCommand(cmd);
    Serial.printf("Sending %s\r\n", cmd);
	delay(100); // Wait for command to be processed
}

void GNSSModule::RTCMHandler::sendAllConfig() {
    for (size_t i = 0; i < count; ++i)
        sendConfig(i);
}

void GNSSModule::RTCMHandler::printList(bool showOnlyEnabled) {
    Serial.println(" #  Name        |  Freq   | State    | Sent   | Description");
    Serial.println("-------------------------------------------------------------------");
    for (size_t i = 0; i < count; ++i) {
        if (showOnlyEnabled && !messages[i].enabled) continue;
        const char* color = messages[i].enabled ? ANSI_GREEN : ANSI_RED;
        Serial.printf("%s%2d: %-10s | %5.2f Hz | %-8s | %6lu | %s%s\n",
            color,
            (int)(i + 1),
            messages[i].name,
            messages[i].frequency,
            messages[i].enabled ? "ENABLED" : "DISABLED",
            messages[i].txCount,
            messages[i].description,
            ANSI_RESET
        );
    }
}

int GNSSModule::RTCMHandler::findById(uint16_t id) const {
    for (size_t i = 0; i < count; ++i)
        if (messages[i].id == id) return i;
    return -1;
}
int GNSSModule::RTCMHandler::findByName(const char* name) const {
    for (size_t i = 0; i < count; ++i)
        if (strcmp(messages[i].name, name) == 0) return i;
    return -1;
}

void GNSSModule::RTCMHandler::getNextRTCMCount(uint16_t* rtcmId, uint32_t* txCount) {
    size_t startIdx = _lastStatusIdx;
    size_t i = startIdx;
    if (count == 0) { // No messages at all
        if (rtcmId) *rtcmId = 0;
        if (txCount) *txCount = 0;
        return;
    }
    do {
        i = (i + 1) % count;  // Use member variable 'count' for array size
        if (messages[i].enabled) {
            if (rtcmId) *rtcmId = messages[i].id;
            if (txCount) *txCount = messages[i].txCount;
            _lastStatusIdx = i;
            return;
        }
    } while (i != startIdx);  // Stop after full loop
    // If no enabled found, return 0s
    if (rtcmId) *rtcmId = 0;
    if (txCount) *txCount = 0;
}

void GNSSModule::RTCMHandler::incrementSentCount(uint16_t type) {
    int idx = findById(type);
    if (idx >= 0) messages[idx].txCount++;
}