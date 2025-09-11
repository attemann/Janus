// GNSS.cpp - Minimal Ring Buffer Implementation
#include "GNSS.h"
#include <string.h>
#include "RadioModule.h"
#include "_macros.h"
#include "esp_task_wdt.h"

class RTCMParser {
private:
    enum ParseState {
        LOOKING_FOR_PREAMBLE,
        READING_HEADER,
        READING_PAYLOAD,
        READING_CRC
    };

    ParseState state = LOOKING_FOR_PREAMBLE;
    uint8_t* frameBuffer;  // Changed to pointer
    size_t framePos = 0;
    uint16_t expectedPayloadLength = 0;
    size_t totalFrameLength = 0;

    // Statistics
    uint32_t totalFramesProcessed = 0;
    uint32_t totalBytesDiscarded = 0;
    uint32_t crcFailures = 0;
    uint32_t invalidHeaders = 0;
    unsigned long lastStatsReport = 0;

public:

    RTCMParser() {
        frameBuffer = (uint8_t*)malloc(1500);
        if (!frameBuffer) {
            Serial.println("RTCM: Failed to allocate frame buffer!");
        }
    }

    ~RTCMParser() {
        if (frameBuffer) {
            free(frameBuffer);
        }
    }

    // Add safety check
    bool isValid() {
        return frameBuffer != nullptr;
    }

    void reset() {
        state = LOOKING_FOR_PREAMBLE;
        framePos = 0;
        expectedPayloadLength = 0;
        totalFrameLength = 0;
    }

    void processByte(uint8_t byte, RadioModule* radio) {
        bool reprocessByte = false;

        do {
            reprocessByte = false;

            switch (state) {
            case LOOKING_FOR_PREAMBLE:
                if (byte == 0xD3) {
                    frameBuffer[0] = byte;
                    framePos = 1;
                    state = READING_HEADER;
                }
                else {
                    totalBytesDiscarded++;
                }
                break;

            case READING_HEADER:
                frameBuffer[framePos++] = byte;

                if (framePos == 3) {
                    // We have the complete 3-byte header
                    uint8_t byte1 = frameBuffer[1];
                    uint8_t byte2 = frameBuffer[2];

                    // Validate reserved bits (6 MSB of byte1 should be 0)
                    if ((byte1 & 0xFC) != 0) {
                        // Invalid header - restart search
                        invalidHeaders++;
                        totalBytesDiscarded += 3; // Count the invalid frame bytes
                        reset();
                        // Check if current byte is a new preamble
                        if (byte == 0xD3) {
                            reprocessByte = true;
                        }
                        else {
                            totalBytesDiscarded++;
                        }
                        break;
                    }

                    // Extract payload length (10 bits)
                    expectedPayloadLength = ((byte1 & 0x03) << 8) | byte2;

                    // Validate length
                    if (expectedPayloadLength == 0 || expectedPayloadLength > 1023) {
                        invalidHeaders++;
                        totalBytesDiscarded += 3;
                        reset();
                        if (byte == 0xD3) {
                            reprocessByte = true;
                        }
                        else {
                            totalBytesDiscarded++;
                        }
                        break;
                    }

                    totalFrameLength = 3 + expectedPayloadLength + 3; // header + payload + CRC

                    if (expectedPayloadLength > 0) {
                        state = READING_PAYLOAD;
                    }
                    else {
                        state = READING_CRC; // No payload, go straight to CRC
                    }
                }
                break;

            case READING_PAYLOAD:
                frameBuffer[framePos++] = byte;

                if (framePos == 3 + expectedPayloadLength) {
                    // Payload complete, now read CRC
                    state = READING_CRC;
                }
                break;

            case READING_CRC:
                frameBuffer[framePos++] = byte;

                if (framePos == totalFrameLength) {
                    // Frame complete - process it
                    processCompleteFrame(radio);
                    reset();
                }
                break;
            }
        } while (reprocessByte);
    }

private:
    void processCompleteFrame(RadioModule* radio) {
        // Extract message type
        uint16_t messageType = (frameBuffer[3] << 4) | (frameBuffer[4] >> 4);

        // Validate message type
        if (messageType < 1000 || messageType > 4095) {
            totalBytesDiscarded += totalFrameLength;
            return;
        }

        // Verify CRC
        bool crcValid = verifyRTCMCRC24(frameBuffer, totalFrameLength);

        if (crcValid) {
            totalFramesProcessed++;
            logRTCMMessageType(messageType, totalFrameLength);

            if (radio) {
                radio->sendRTCM(frameBuffer, totalFrameLength);
                // Only log every 50th successful frame
                if (totalFramesProcessed % 50 == 0) {
                    Serial.print("✓ RTCM: ");
                    Serial.println(totalFramesProcessed);
                }
            }
        }
        else {
            crcFailures++;
            // Only log every 20th CRC failure
            if (crcFailures % 20 == 0) {
                Serial.print("CRC fails: ");
                Serial.println(crcFailures);
            }
        }
    }

    bool verifyRTCMCRC24(uint8_t* data, size_t length) {
        uint32_t crc = 0;

        for (size_t i = 0; i < length - 3; i++) {
            crc ^= (uint32_t)data[i] << 16;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x800000) {
                    crc = (crc << 1) ^ 0x1864CFB;
                }
                else {
                    crc <<= 1;
                }
            }
        }

        crc &= 0xFFFFFF;

        uint32_t frameCrc = ((uint32_t)data[length - 3] << 16) |
            ((uint32_t)data[length - 2] << 8) |
            data[length - 1];

        return (crc == frameCrc);
    }

    void logRTCMMessageType(uint16_t messageType, size_t frameSize) {
        static unsigned long lastTypeReport = 0;
        static uint32_t typeCounts[10] = { 0 };

        switch (messageType) {
        case 1006: typeCounts[0]++; break;
        case 1033: typeCounts[1]++; break;
        case 1074: typeCounts[2]++; break;
        case 1084: typeCounts[3]++; break;
        case 1094: typeCounts[4]++; break;
        case 1124: typeCounts[5]++; break;
        default: break;
        }

        if (millis() - lastTypeReport > 60000) {
            Serial.println("=== Message Type Stats ===");
            Serial.print("1006: "); Serial.print(typeCounts[0]); Serial.println(" (ref station)");
            Serial.print("1033: "); Serial.print(typeCounts[1]); Serial.println(" (descriptor)");
            Serial.print("1074: "); Serial.print(typeCounts[2]); Serial.println(" (GPS MSM4)");
            Serial.print("1084: "); Serial.print(typeCounts[3]); Serial.println(" (GLO MSM4)");
            Serial.print("1094: "); Serial.print(typeCounts[4]); Serial.println(" (GAL MSM4)");
            Serial.print("1124: "); Serial.print(typeCounts[5]); Serial.println(" (BDS MSM4)");
            Serial.println("==========================");
            lastTypeReport = millis();
        }
    }

public:
    void printStats() {
        if (millis() - lastStatsReport > 30000) {
            Serial.println("=== RTCM Parser Stats ===");
            Serial.print("✓ Frames: ");
            Serial.print(totalFramesProcessed);
            Serial.print(" | ✗ CRC: ");
            Serial.print(crcFailures);
            Serial.print(" | ✗ Headers: ");
            Serial.println(invalidHeaders);
            Serial.print("Discarded: ");
            Serial.print(totalBytesDiscarded);
            Serial.print(" | State: ");
            Serial.println((int)state);

            // Calculate efficiency
            float efficiency = 0;
            uint32_t totalValidBytes = totalFramesProcessed * 80; // Assume avg 80 bytes per frame
            if (totalBytesDiscarded + totalValidBytes > 0) {
                efficiency = (float)totalValidBytes / (totalBytesDiscarded + totalValidBytes) * 100;
            }
            Serial.print("Efficiency: ");
            Serial.print(efficiency, 1);
            Serial.println("%");
            Serial.println("=========================");
            lastStatsReport = millis();
        }
    }
};

bool GNSSModule::begin(long baud, int rxPin, int txPin, RadioModule* radio) {
    if (rxPin >= 0 && txPin >= 0) {
        _ser->begin(baud, SERIAL_8N1, rxPin, txPin);
    }
    else {
        _ser->begin(baud);
    }
    _ser->setTimeout(5);
    _buffer.clear();
    _radio = radio;
    return true;
}

void GNSSModule::sendCommand(const char* cmd) {
    if (!cmd) return;
    //Serial.printf("Sending command: [%s]\n", cmd);
    _ser->print(cmd);
    _ser->print("\r\n");
}

bool GNSSModule::sendWait(const char* cmd, const char* expected, uint32_t timeoutMs) {
    if (cmd && *cmd) sendCommand(cmd);
    const uint32_t t0 = millis();
    String response = "";

    while ((millis() - t0) < timeoutMs) {
        fillBufferFromUART();

        while (_buffer.available()) {
            char c = _buffer.read();

            // Only add printable characters and basic whitespace
            if (c >= 32 && c <= 126) {  // Printable ASCII
                response += c;
                //Serial.print(c);  // Debug: show what's being added
            }

            // Check after each character addition
            if (response.indexOf(expected) >= 0) {
                //Serial.println();
                GDBG_PRINTF("\r\nsendWait [%s] OK", cmd);
                return true;
            }
        }
        delay(1);
    }

    //Serial.print("\nFinal response length: ");
    //Serial.println(response.length());
    //Serial.print("Final response: '");
    //Serial.print(response);
    //Serial.println("'");
    return false;
}

void GNSSModule::fillBufferFromUART() {
    while (_ser->available()) {
        uint8_t byte = _ser->read();
        if (!_buffer.write(byte)) {
            // Buffer full - could log this
            static uint32_t lastWarn = 0;
            if (millis() - lastWarn > 1000) {
                Serial.println("UART buffer overflow");
                lastWarn = millis();
            }
        }
    }
}

bool GNSSModule::pumpGGA(GNSSFix& fix) {
    fillBufferFromUART();

    char line[128];
    if (findCompleteGGA(line, sizeof(line))) {
		//Serial.printf("GGA Line: %s\n", line);  
        return parseGGA(line, fix);
    }
    return false;
}

bool GNSSModule::findCompleteGGA(char* line, size_t maxLen) {
    static String accumulated = "";

    while (_buffer.available()) {
        char c = _buffer.read();

        if (c == '\n' || c == '\r') {
            if (accumulated.length() > 0) {
                if (accumulated.startsWith("$GNGGA") || accumulated.startsWith("$GPGGA")) {
                    if (accumulated.length() < maxLen) {
                        strcpy(line, accumulated.c_str());
                        accumulated = "";
                        return true;
                    }
                }
                accumulated = "";
            }
        }
        else {
            accumulated += c;
            if (accumulated.length() > 200) {  // Prevent memory issues
                accumulated = "";
            }
        }
    }
    return false;
}

bool GNSSModule::verifyRTCMCRC24(uint8_t* data, size_t length) {
    // RTCM CRC24Q polynomial: 0x1864CFB
    uint32_t crc = 0;

    // Process all bytes except the last 3 (which contain the CRC)
    for (size_t i = 0; i < length - 3; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ 0x1864CFB;
            }
            else {
                crc <<= 1;
            }
        }
    }

    crc &= 0xFFFFFF; // Keep only 24 bits

    // Extract CRC from frame (last 3 bytes)
    uint32_t frameCrc = ((uint32_t)data[length - 3] << 16) |
        ((uint32_t)data[length - 2] << 8) |
        data[length - 1];

    return (crc == frameCrc);
}

void GNSSModule::sendRTCMToRadio(uint8_t* data, size_t length) {
    if (_radio) {
        _radio->sendRTCM(data, length);
    }
}

void GNSSModule::pumpRTCM() {
    fillBufferFromUART();
    static uint32_t bytesProcessed = 0;
    static uint32_t rtcmFrames = 0;
    static uint8_t frameState = 0;
    static uint16_t frameLength = 0;
    static uint16_t framePos = 0;
    static uint8_t frameData[300];
    static uint8_t framesProcessedThisCall = 0; // Track frames per call

    framesProcessedThisCall = 0; // Reset counter

    while (_buffer.available()) {
        uint8_t byte = _buffer.read();
        bytesProcessed++;

        // Store bytes as we read them
        if (frameState > 0 && framePos < 300) {
            frameData[framePos - 1] = byte;
        }

        switch (frameState) {
        case 0: // Looking for preamble
            if (byte == 0xD3) {
                frameData[0] = byte;
                frameState = 1;
                framePos = 1;
            }
            break;
        case 1: // Reading 3-byte header
            framePos++;
            if (framePos == 3) {
                // Extract real length from RTCM header
                uint16_t payloadLength = ((frameData[1] & 0x03) << 8) | frameData[2];
                frameLength = 3 + payloadLength + 3; // header + payload + CRC
                frameState = 2;
            }
            break;
        case 2: // Reading frame body
            framePos++;
            if (framePos >= frameLength) {
                rtcmFrames++;
                framesProcessedThisCall++; // Increment frame counter

                if (_radio) {
                    static unsigned long lastTransmission = 0;
                    if (millis() - lastTransmission > 100) {
                        _radio->sendRTCM(frameData, frameLength);
                        lastTransmission = millis();
                        if (rtcmFrames % 20 == 0) {
                            Serial.print("Complete RTCM frames: ");
                            Serial.print(rtcmFrames);
                            Serial.println(" (transmitted to radio)");
                        }
                    }
                }
                frameState = 0;

                // Limit to 3 complete frames per call instead of 50 bytes
                if (framesProcessedThisCall >= 3) {
                    break;
                }
            }
            break;
        }
        yield();
    }
}

void GNSSModule::logRTCMMessageType(uint16_t messageType, size_t frameSize) {
    static unsigned long lastTypeReport = 0;
    static uint32_t typeCounts[20] = { 0 }; // Track common message types

    // Count common message types
    switch (messageType) {
    case 1006: typeCounts[0]++; break;
    case 1033: typeCounts[1]++; break;
    case 1074: typeCounts[2]++; break;
    case 1084: typeCounts[3]++; break;
    case 1094: typeCounts[4]++; break;
    case 1124: typeCounts[5]++; break;
    default: break;
    }

    // Report type statistics every 60 seconds
    if (millis() - lastTypeReport > 60000) {
        Serial.println("=== Message Type Stats ===");
        Serial.print("1006: "); Serial.print(typeCounts[0]); Serial.println(" (ref station)");
        Serial.print("1033: "); Serial.print(typeCounts[1]); Serial.println(" (descriptor)");
        Serial.print("1074: "); Serial.print(typeCounts[2]); Serial.println(" (GPS MSM4)");
        Serial.print("1084: "); Serial.print(typeCounts[3]); Serial.println(" (GLO MSM4)");
        Serial.print("1094: "); Serial.print(typeCounts[4]); Serial.println(" (GAL MSM4)");
        Serial.print("1124: "); Serial.print(typeCounts[5]); Serial.println(" (BDS MSM4)");
        Serial.println("==========================");
        lastTypeReport = millis();
    }
}

// Keep all your existing parsing methods unchanged
bool GNSSModule::parseLatLon(const char* dm, char hemi, float& outDeg) {
    if (!dm || !*dm) return false;
    const char* dot = strchr(dm, '.');
    if (!dot || dot - dm < 3) return false;
    int degDigits = (dot - dm > 4) ? 3 : 2;
    char degBuf[4] = { 0 };
    strncpy(degBuf, dm, degDigits);
    int deg = atoi(degBuf);
    float min = atof(dm + degDigits);
    outDeg = deg + (min / 60.0f);
    if (hemi == 'S' || hemi == 'W') outDeg = -outDeg;
    return true;
}

bool GNSSModule::parseGGA(const char* line, GNSSFix& fix) {
    if (!line || (strncmp(line, "$GNGGA", 6) && strncmp(line, "$GPGGA", 6))) return false;

    // Initialize fix with default values
    fix.hour = 0;
    fix.minute = 0;
    fix.second = 0.0f;
    fix.lat = 0.0f;
    fix.lon = 0.0f;
    fix.type = 0;
    fix.SIV = 0;
    fix.HDOP = 0.0f;
    fix.elevation = 0.0f;  // Initialize elevation

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    const char* f[16] = { 0 };
    int nf = 0;
    char* s = buf;
    while (nf < 16 && (f[nf++] = strsep(&s, ","))) {}

    if (nf < 10) return true; // Return with default values instead of false

    // Parse time (field 1)
    if (f[1] && strlen(f[1]) >= 6) {
        char hh[3] = { f[1][0], f[1][1], 0 };
        char mm[3] = { f[1][2], f[1][3], 0 };
        char ss[16];
        strncpy(ss, f[1] + 4, sizeof(ss) - 1); 
        ss[sizeof(ss) - 1] = 0;
        fix.hour = atoi(hh);
        fix.minute = atoi(mm);
        fix.second = atof(ss); 
    }

    // Parse latitude/longitude - don't fail if empty
    if (f[2] && strlen(f[2]) > 0 && f[3] && strlen(f[3]) > 0) {
        parseLatLon(f[2], f[3][0], fix.lat);
    }

    if (f[4] && strlen(f[4]) > 0 && f[5] && strlen(f[5]) > 0) {
        parseLatLon(f[4], f[5][0], fix.lon);
    }

    // Parse other fields with null/empty checks
    if (f[6] && strlen(f[6]) > 0) {
        fix.type = atoi(f[6]);
    }

    if (f[7] && strlen(f[7]) > 0) {
        fix.SIV = atoi(f[7]);
    }

    if (f[8] && strlen(f[8]) > 0) {
        fix.HDOP = atof(f[8]);
    }

    // Parse elevation (field 9) - altitude above mean sea level
    if (f[9] && strlen(f[9]) > 0) {
        fix.elevation = atof(f[9]);
    }

    return true; // Always return true, even with partial data
}

void GNSSModule::printFix(const GNSSFix& fix) {
    Serial.printf("Time: %02d:%02d:%06.3f\n", fix.hour, fix.minute, fix.second);
    Serial.printf("Lat/Lon: %.8f, %.8f\n", fix.lat, fix.lon);
    Serial.printf("SIV: %d  HDOP: %.2f  Type: %d\n", fix.SIV, fix.HDOP, fix.type);
}

void GNSSModule::clearUARTBuffer() {
    _buffer.clear();
    while (_ser->available()) {
        _ser->read();
    }
}

uint32_t GNSSModule::crc24q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;

    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i] << 16;

        for (int bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000) {  // Check if bit 24 is set
                crc ^= 0x1864CFB;   // RTCM CRC-24Q polynomial
            }
        }
        crc &= 0xFFFFFF;  // Keep only 24 bits
    }

    return crc & 0xFFFFFF;
}

bool GNSSModule::isValidRTCM(const uint8_t* frame, size_t len) {
    // Basic sanity checks
    if (!frame || len < 6) {
        return false;  // Too short for valid RTCM frame
    }

    // Check preamble
    if (frame[0] != 0xD3) {
        return false;  // Invalid preamble
    }

    // Extract payload length from header
    uint16_t payloadLen = ((uint16_t)(frame[1] & 0x03) << 8) | frame[2];

    // Validate frame structure
    size_t expectedLen = 3 + payloadLen + 3;  // header + payload + CRC
    if (len != expectedLen) {
        return false;  // Length mismatch
    }

    // Validate payload length is reasonable
    if (payloadLen == 0 || payloadLen > 1023) {
        return false;  // RTCM spec limits payload to 1023 bytes
    }

    // Calculate CRC-24Q over header + payload (exclude the 3-byte CRC itself)
    uint32_t calculatedCRC = crc24q(frame, len - 3);

    // Extract the 3-byte CRC from frame
    uint32_t frameCRC = ((uint32_t)frame[len - 3] << 16) |
        ((uint32_t)frame[len - 2] << 8) |
        (uint32_t)frame[len - 1];

    // Validate CRC
    return calculatedCRC == frameCRC;
}