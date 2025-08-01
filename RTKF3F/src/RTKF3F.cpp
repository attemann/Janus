#include "RTKF3F.h"

#define REG_VERSION 0x10
#define EXPECTED_RFM69_VERSION 0x24

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40

extern Slope slope;
extern HardwareSerial SerialGNSS;
extern bool showGngga;

bool verifyRadio(RFM69& radio) {
    uint8_t version = radio.readReg(REG_VERSION);
    if (version != EXPECTED_RFM69_VERSION) {
        Serial.printf("verifyRadio: Unexpected RFM69 version 0x%02X (expected 0x%02X)\n", version, EXPECTED_RFM69_VERSION);
        return false;
    }
    return true;
}

void monitorSerial(HardwareSerial& gnssSerial, String c, int wait) {
    unsigned long startTime = 0;
    startTime = millis();
    Serial.print("GPS: Sending ");
    Serial.print(c);

    while ((millis() - startTime) < wait) {  // Les data i 5 sekunder
        if (gnssSerial.available()) {
            Serial.write(gnssSerial.read());
        }
    }
}

int detectUM980Port(HardwareSerial& gnssSerial) {
    const char* testCommands[] = {
        "versiona com1",
        "versiona com2",
        "versiona com3"
    };

    for (int port = 1; port <= 3; port++) {
        gnssSerial.flush(); // Clear any previous data
        delay(100);
        gnssSerial.print(testCommands[port - 1]);
        gnssSerial.print("\r\n");

        unsigned long startTime = millis();
        String response = "";

        while (millis() - startTime < 500) {
            while (gnssSerial.available()) {
                char c = gnssSerial.read();
                response += c;
            }
        }

        if (response.indexOf("VERSIONA") != -1 || response.indexOf("#VERSIONA") != -1) {
            return port;
        }
    }
    return -1; // None found
}

uint32_t calculateCRC24Q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}


bool isValidRTCM(const uint8_t* rtcm, size_t totalLen) {
    if (rtcm[0] != 0xD3) return false;
    if (totalLen < 6) return false;  // Must be at least header + CRC

    uint16_t payloadLen = getBits(rtcm, 14, 10);
    if (payloadLen > 1023 || payloadLen + 6 > totalLen) return false;

    uint32_t crcCalc = calculateCRC24Q(rtcm, 3 + payloadLen);
    uint32_t crcRecv = (rtcm[3 + payloadLen] << 16) |
        (rtcm[3 + payloadLen + 1] << 8) |
        (rtcm[3 + payloadLen + 2]);

    return crcCalc == crcRecv;
}

bool readRTCMMessage(Stream& in, uint8_t* buffer, size_t& len) {
    while (in.available()) {
        uint8_t b = in.read();

        if (b != 0xD3) continue;  // Wait for sync

        // Read length bytes (2 bytes after 0xD3)
        while (in.available() < 2);
        uint8_t lenH = in.read();
        uint8_t lenL = in.read();
        uint16_t payloadLen = ((lenH & 0x03) << 8) | lenL;

        if (payloadLen > 1023) return false;

        size_t totalLen = 3 + payloadLen + 3;  // header + payload + CRC
        while (in.available() < payloadLen + 3);  // Wait for full message

        buffer[0] = 0xD3;
        buffer[1] = lenH;
        buffer[2] = lenL;

        for (uint16_t i = 0; i < payloadLen + 3; i++) {
            buffer[3 + i] = in.read();
        }

        len = totalLen;
        return true;
    }

    return false;
}


void updateRTCMForwarder() {
    uint8_t rtcmBuffer[1024];
    size_t rtcmLen = 0;

    if (readRTCMMessage(SerialGNSS, rtcmBuffer, rtcmLen)) {
        if (isValidRTCM(rtcmBuffer, rtcmLen)) {
            uint8_t packet[1025];
            packet[0] = MSG_RTCM;
            memcpy(&packet[1], rtcmBuffer, rtcmLen);

            //radio.send(0, packet, rtcmLen + 1);
            RTCM_Fragmenter::sendFragmented(radio, 0, rtcmBuffer, rtcmLen);

            uint16_t msgType = getBits(rtcmBuffer, 24, 12);
            Serial.printf("%4u: Sent %4zu bytes\n", msgType, rtcmLen);
        }
        else {
            Serial.println("RTCM failed CRC");
        }
    }
}

const char* getRTCMMessageName(uint16_t type) {
    switch (type) {
        // Legacy GPS observation messages
    case 1001: return "RTCM 1001: GPS L1 Obs";
    case 1002: return "RTCM 1002: GPS L1 Obs (Ext)";
    case 1003: return "RTCM 1003: GPS L1/L2 Obs";
    case 1004: return "RTCM 1004: GPS L1/L2 Obs (Ext)";

        // Base station reference data
    case 1005: return "RTCM 1005: Stationary RTK (No height)";
    case 1006: return "RTCM 1006: Stationary RTK (with height)";
    case 1007: return "RTCM 1007: Antenna Descriptor";
    case 1008: return "RTCM 1008: Antenna Descriptor + Serial Number";
    case 1033: return "RTCM 1033: Antenna Descriptor & Firmware Version";

        // MSM (Multiple Signal Message) for GPS
    case 1071: return "RTCM 1071: GPS MSM1";
    case 1072: return "RTCM 1072: GPS MSM2";
    case 1073: return "RTCM 1073: GPS MSM3";
    case 1074: return "RTCM 1074: GPS MSM4";
    case 1075: return "RTCM 1075: GPS MSM5";
    case 1076: return "RTCM 1076: GPS MSM6";
    case 1077: return "RTCM 1077: GPS MSM7";

        // MSM for GLONASS
    case 1081: return "RTCM 1081: GLONASS MSM1";
    case 1082: return "RTCM 1082: GLONASS MSM2";
    case 1083: return "RTCM 1083: GLONASS MSM3";
    case 1084: return "RTCM 1084: GLONASS MSM4";
    case 1085: return "RTCM 1085: GLONASS MSM5";
    case 1086: return "RTCM 1086: GLONASS MSM6";
    case 1087: return "RTCM 1087: GLONASS MSM7";

        // MSM for Galileo
    case 1091: return "RTCM 1091: Galileo MSM1";
    case 1092: return "RTCM 1092: Galileo MSM2";
    case 1093: return "RTCM 1093: Galileo MSM3";
    case 1094: return "RTCM 1094: Galileo MSM4";
    case 1095: return "RTCM 1095: Galileo MSM5";
    case 1096: return "RTCM 1096: Galileo MSM6";
    case 1097: return "RTCM 1097: Galileo MSM7";

        // MSM for BeiDou
    case 1121: return "RTCM 1121: BeiDou MSM1";
    case 1122: return "RTCM 1122: BeiDou MSM2";
    case 1123: return "RTCM 1123: BeiDou MSM3";
    case 1124: return "RTCM 1124: BeiDou MSM4";
    case 1125: return "RTCM 1125: BeiDou MSM5";
    case 1126: return "RTCM 1126: BeiDou MSM6";
    case 1127: return "RTCM 1127: BeiDou MSM7";

        // Bias and corrections
    case 1230: return "RTCM 1230: GLONASS Code-Phase Biases";

        // Proprietary or vendor-specific
    case 4072: return "RTCM 4072: Proprietary (e.g. u-blox)";
    case 4072 + 1: return "RTCM 4073: u-blox Subtype A";
    case 4072 + 2: return "RTCM 4074: u-blox Subtype B";

    default: return "RTCM: Unknown or Unsupported Type";
    }
}

uint16_t getBits(const uint8_t* buffer, int startBit, int bitLen) {
    uint32_t result = 0;
    for (int i = 0; i < bitLen; i++) {
        int byteIndex = (startBit + i) / 8;
        int bitIndex = 7 - ((startBit + i) % 8);
        result <<= 1;
        result |= (buffer[byteIndex] >> bitIndex) & 0x01;
    }
    return result;
}

bool readGNSSData(GNSSFix& fix) {
    static char nmeaBuffer[100];
    static size_t idx = 0;

    while (SerialGNSS.available()) {
        char c = SerialGNSS.read();

        if (c == '$') {
            idx = 0;
            nmeaBuffer[idx++] = c;
        }
        else if (c == '\n' && idx > 6) {
            nmeaBuffer[idx] = '\0';

            if (strncmp(nmeaBuffer, "$GNGGA", 6) == 0) {
                if (showGngga) {
					Serial.print("GNGGA: ");
                    Serial.println(nmeaBuffer);
                }
                // Reset fix
                fix.lat = fix.lon = fix.alt = 0.0f;
                fix.fixType = 0;
                fix.numSV = 0;
                fix.gpsFix = fix.diffUsed = fix.rtkFix = fix.rtkFloat = false;

                // Parse fields
                char* token;
                int field = 0;
                float latRaw = 0, lonRaw = 0, alt = 0;
                int fixQuality = 0, numSV = 0;
                float HDOP = 0.0f;
                char latDir = 0, lonDir = 0;

                token = strtok(nmeaBuffer, ",");
                field = 0;

                while (token) {
                    field++;

                    switch (field) {
                    case 2: {
                        float rawTime = atof(token);
                        fix.hour = int(rawTime / 10000);
                        fix.minute = int(fmod(rawTime / 100, 100));
                        fix.second = fmod(rawTime, 60.0f);
                        break;
                    }
                    case 3: latRaw = atof(token); break;
                    case 4: latDir = token[0]; break;
                    case 5: lonRaw = atof(token); break;
                    case 6: lonDir = token[0]; break;
                    case 7: fixQuality = atoi(token); break;
                    case 8: numSV = atoi(token); break;
                    case 9: HDOP = atof(token); break;
                    case 10: alt = atof(token); break;
                    default: break;
                    }

                    token = strtok(NULL, ",");
                }

                float lat = 0, lon = 0;
                if (latRaw > 0 && (latDir == 'N' || latDir == 'S')) {
                    int deg = int(latRaw / 100);
                    float min = latRaw - deg * 100;
                    lat = deg + min / 60.0f;
                    if (latDir == 'S') lat = -lat;
                }

                if (lonRaw > 0 && (lonDir == 'E' || lonDir == 'W')) {
                    int deg = int(lonRaw / 100);
                    float min = lonRaw - deg * 100;
                    lon = deg + min / 60.0f;
                    if (lonDir == 'W') lon = -lon;
                }

                fix.lat = lat;
                fix.lon = lon;
                fix.alt = alt;
                fix.fixType = fixQuality;
                fix.numSV = numSV;
                fix.HDOP = int(HDOP * 100);

                fix.gpsFix = (fixQuality > 0);
                fix.diffUsed = (fixQuality >= 2);
                fix.rtkFix = (fixQuality == 4);
                fix.rtkFloat = (fixQuality == 5);

                return true;
            }
            else {
                // Print other NMEA/NAV messages for debug
                Serial.println(nmeaBuffer);
            }

            idx = 0;
        }
        else if (idx < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[idx++] = c;
        }
    }

    return false;
}



