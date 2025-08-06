// GNSSModule.cpp
#include <Arduino.h>    
#include <RFM69.h>
#include <HardwareSerial.h>
#include <RTKF3F.h>

#include "GNSSModule.h"

GNSSModule::GNSSModule(HardwareSerial& serial)
    : _serial(serial), _nmeaIdx(0) {
}

void GNSSModule::begin(uint32_t baud, int RX, int TX) {
    _serial.begin(baud, SERIAL_8N1, RX, TX);
}

void GNSSModule::setDefaultRTCMs() {
    _rtcmCount = 6;
    _rtcmList[0] = { "rtcm1006", 1.0f, true };
    _rtcmList[1] = { "rtcm1033", 2.7f, true };
    _rtcmList[2] = { "rtcm1074", 1.1f, true };
    _rtcmList[3] = { "rtcm1084", 2.4f, true };
    _rtcmList[4] = { "rtcm1094", 3.6f, true };
    _rtcmList[5] = { "rtcm1124", 5.9f, true };
}

void GNSSModule::printRTCMConfig() {
    Serial.println("RTCM Configuration:");
    for (int i = 0; i < _rtcmCount; i++) {
        const RTCMMessage& msg = _rtcmList[i];
        Serial.printf("%d. %-10s | %5.2f Hz | %s\n",
            i+1,
            msg.name,
            msg.frequencyHz,
            msg.enabled ? "ENABLED" : "DISABLED");
    }
}

void GNSSModule::sendConfiguredRTCMs() {

    for (int i = 0; i < _rtcmCount; i++) {
        if (_rtcmList[i].enabled) {
            String cmd = String(_rtcmList[i].name) + " com2 " + String(_rtcmList[i].frequencyHz, 1) + "\r\n";
            sendCommand(cmd);
        }
    }

}

const GNSSModule::RTCMMessage& GNSSModule::getRTCM(int index) const {
    return _rtcmList[index];
}

int GNSSModule::getRTCMCount() const {
    return _rtcmCount;
}

void GNSSModule::toggleRTCM(int index) {
    if (index >= 0 && index < _rtcmCount)
        _rtcmList[index].enabled = !_rtcmList[index].enabled;
}

void GNSSModule::setRTCMFrequency(int index, float hz) {
    if (index >= 0 && index < _rtcmCount)
        _rtcmList[index].frequencyHz = hz;
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

bool GNSSModule::readFix(GNSSFix& fix) {
    while (_serial.available()) {
        char c = _serial.read();

        if (c == '$') {
            _nmeaIdx = 0;
        }

        if (_nmeaIdx < sizeof(_nmeaBuffer) - 1) {
            _nmeaBuffer[_nmeaIdx++] = c;
        }

        if (c == '\n' && _nmeaIdx > 6) {
            _nmeaBuffer[_nmeaIdx] = '\0';
            _nmeaIdx = 0;

            if (strncmp(_nmeaBuffer, "$GNGGA", 6) == 0) {
                return parseGGA(_nmeaBuffer, fix);
            }
        }
    }
    return false;
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

bool GNSSModule::readRTCM(uint8_t* buffer, size_t& len) {
    while (_serial.available()) {
        if (_serial.read() != MSG_RTCM) continue;

        while (_serial.available() < 2);
        uint8_t lenH = _serial.read();
        uint8_t lenL = _serial.read();
        uint16_t payloadLen = ((lenH & MSG_RTCM) << 8) | lenL;

        if (payloadLen > 1023) return false;

        size_t totalLen = 3 + payloadLen + 3;
        while (_serial.available() < payloadLen + 3);

        buffer[0] = MSG_RTCM;
        buffer[1] = lenH;
        buffer[2] = lenL;
        for (int i = 0; i < payloadLen + 3; i++) {
            buffer[3 + i] = _serial.read();
        }

        len = totalLen;
        return isValidRTCM(buffer, len);
    }
    return false;
}

bool GNSSModule::isValidRTCM(const uint8_t* data, size_t len) {
    if (data[0] != MSG_RTCM || len < 6) return false;
    uint16_t payloadLen = ((data[1] & MSG_RTCM) << 8) | data[2];
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

bool GNSSModule::readGNSSData(GNSSFix& fix, bool showRaw) {
    static char nmeaBuffer[100];
    static size_t idx = 0;

    while (_serial.available()) {
        char c = _serial.read();

        if (showRaw) {
            Serial.write(c);
        }

        if (c == '$') {
            idx = 0;
            nmeaBuffer[idx++] = c;
        }
        else if (c == '\n' && idx > 6) {
            nmeaBuffer[idx] = '\0';  // Null-terminate

            // Attempt to parse if it's a GGA sentence
            if (strncmp(nmeaBuffer, "$GNGGA", 6) == 0) {
                return parseGGA(nmeaBuffer, fix);  // returns true if parsed successfully
            }

            // Reset buffer for next sentence
            idx = 0;
        }
        else if (idx < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[idx++] = c;
        }
    }

    return false;  // No complete valid line yet
}


bool GNSSModule::parseGGA(const char* line, GNSSFix& fix) {
    char buf[100];
    strncpy(buf, line, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';

    // Tokenization preserving empty fields
    const int MAX_FIELDS = 20;
    const char* fields[MAX_FIELDS] = { nullptr };
    int fieldCount = 0;

    char* p = buf;
    while (fieldCount < MAX_FIELDS) {
        fields[fieldCount++] = p;
        char* comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }

    if (fieldCount < 9) return false;

    // Parse time
    if (fields[1] && *fields[1]) {
        float rawTime = atof(fields[1]);
        fix.hour = int(rawTime / 10000);
        fix.minute = int(fmod(rawTime / 100, 100));
        fix.second = fmod(rawTime, 100);
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
    else fix.lat = 0;

    if (lonRaw > 0 && (lonDir == 'E' || lonDir == 'W')) {
        int deg = int(lonRaw / 100);
        float min = lonRaw - deg * 100;
        fix.lon = deg + min / 60.0f;
        if (lonDir == 'W') fix.lon = -fix.lon;
    }
    else fix.lon = 0;

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
