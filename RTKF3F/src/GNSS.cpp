// GNSS.cpp - Cleaned Minimal Ring Buffer Implementation
#include "GNSS.h"
#include <string.h>
#include "RadioModule.h"
#include "_macros.h"
#include "esp_task_wdt.h"

// ---------------- GNSSModule ----------------

bool GNSSModule::begin(long baud, int rxPin, int txPin) {
    if (rxPin >= 0 && txPin >= 0) {
        _ser->begin(baud, SERIAL_8N1, rxPin, txPin);
    }
    else {
        _ser->begin(baud);
    }
    _ser->setTimeout(5);
    _buffer.clear();
    return true;
}

void GNSSModule::setRadio(RadioModule* radio) {
    _radio = radio;
    Serial.printf("GNSS: Radio %s\n", _radio ? "attached" : "detached");
}

void GNSSModule::sendCommand(const char* cmd) {
    if (!cmd) return;
    _ser->print(cmd);
    _ser->print("\r\n");
}

bool GNSSModule::sendWait(const char* cmd, const char* expected, uint32_t timeoutMs) {
    if (cmd && *cmd) sendCommand(cmd);
    const uint32_t t0 = millis();

    char response[256];
    size_t idx = 0;
    response[0] = '\0';

    while ((millis() - t0) < timeoutMs) {
        fillBufferFromUART();

        uint8_t b;
        while (_buffer.read(b)) {
            char c = (char)b;
            //Serial.printf("[%02X] %c ", b, c);
            if (c >= 32 && c <= 126 && idx < sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }

            if (strstr(response, expected)) {
                GDBG_PRINTF("\r\nsendWait [%s] OK", cmd);
                return true;
            }
        }
        delay(1);
    }
    return false;
}

void GNSSModule::fillBufferFromUART() {
    uint8_t temp[64];
    while (_ser->available()) {
        size_t n = _ser->readBytes(temp, sizeof(temp));
        for (size_t i = 0; i < n; i++) {
            if (!_buffer.write(temp[i])) {
                static uint32_t lastWarn = 0;
                if (millis() - lastWarn > 1000) {
                    Serial.println("UART buffer overflow");
                    lastWarn = millis();
                }
            }
        }
        yield();
    }
}

bool GNSSModule::pumpGGA(GNSSFix& fix) {
    fillBufferFromUART();
    char line[128];
    if (findCompleteGGA(line, sizeof(line))) {
        return parseGGA(line, fix);
    }
    return false;
}

bool GNSSModule::findCompleteGGA(char* line, size_t maxLen) {
    static char accum[256];
    static size_t pos = 0;

    uint8_t b;
    while (_buffer.read(b)) {
        char c = (char)b;

        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                accum[pos] = '\0';
                if ((strncmp(accum, "$GNGGA", 6) == 0 || strncmp(accum, "$GPGGA", 6) == 0) &&
                    pos < maxLen) {
                    strcpy(line, accum);
                    pos = 0;
                    return true;
                }
                pos = 0;
            }
        }
        else {
            if (pos < sizeof(accum) - 1) {
                accum[pos++] = c;
            }
            else {
                pos = 0; // overflow guard
            }
        }
    }
    return false;
}

uint16_t GNSSModule::getRTCMType(const uint8_t* data, size_t length) {
    if (!data || length < 6 || data[0] != 0xD3) return 0;
    return (data[3] << 4) | (data[4] >> 4);
}

uint32_t GNSSModule::crc24q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x800000) ? (crc << 1) ^ 0x1864CFB : (crc << 1);
        }
    }
    return crc & 0xFFFFFF;
}

bool GNSSModule::verifyRTCMCRC24(uint8_t* data, size_t length) {
    if (length < 6) return false;
    uint32_t calc = crc24q(data, length - 3);
    uint32_t frame = ((uint32_t)data[length - 3] << 16) |
        ((uint32_t)data[length - 2] << 8) |
        data[length - 1];
    return calc == frame;
}

bool GNSSModule::isValidRTCM(const uint8_t* frame, size_t len) {
    if (!frame || len < 6 || frame[0] != 0xD3) return false;
    uint16_t payloadLen = ((uint16_t)(frame[1] & 0x03) << 8) | frame[2];
    if (payloadLen == 0 || payloadLen > 1023) return false;
    if (len != 3 + payloadLen + 3) return false;
    return crc24q(frame, len - 3) ==
        (((uint32_t)frame[len - 3] << 16) |
            ((uint32_t)frame[len - 2] << 8) |
            frame[len - 1]);
}

void GNSSModule::sendRTCMToRadio(uint8_t* data, size_t length) {
    if (_radio) _radio->sendRTCM(data, length);
}

void GNSSModule::pumpRTCM() {
    fillBufferFromUART();

    static uint8_t frameData[1024];
    static size_t framePos = 0;
    static size_t frameLen = 0;
    static bool inFrame = false;
    static bool lengthKnown = false;
    static unsigned long frameStartTime = 0;
    static uint32_t totalFrames = 0;
    static uint32_t crcErrors = 0;
    static uint32_t syncLosses = 0;

    uint8_t byte;

    while (_buffer.read(byte)) {
        if (!inFrame) {
            // Look for RTCM frame start (0xD3)
            if (byte == 0xD3) {
                frameData[0] = byte;
                framePos = 1;
                frameLen = 0;
                inFrame = true;
                lengthKnown = false;
                frameStartTime = millis();
                continue;
            }
            // Skip non-frame bytes
            continue;
        }

        // We're in a frame - add the byte
        frameData[framePos++] = byte;

        // Check for frame timeout (partial frame stuck)
        if (millis() - frameStartTime > 1000) {  // 1 second timeout
            Serial.printf("⚠️ RTCM frame timeout after %lu ms, pos=%u\n",
                millis() - frameStartTime, framePos);
            inFrame = false;
            framePos = 0;
            lengthKnown = false;
            syncLosses++;
            continue;
        }

        // Once we have 3 bytes, we can determine frame length
        if (framePos == 3 && !lengthKnown) {
            uint16_t payloadLen = ((frameData[1] & 0x03) << 8) | frameData[2];
            frameLen = 3 + payloadLen + 3;  // header + payload + CRC
            lengthKnown = true;

            // Sanity check on frame length
            if (frameLen > sizeof(frameData) || frameLen < 6) {
                Serial.printf("⚠️ Invalid RTCM frame length: %u bytes (payload=%u)\n",
                    frameLen, payloadLen);
                inFrame = false;
                framePos = 0;
                lengthKnown = false;
                syncLosses++;
                continue;
            }
        }

        // Check if we have a complete frame
        if (lengthKnown && framePos >= frameLen) {
            totalFrames++;

            // Verify CRC before processing
            if (verifyRTCMCRC24(frameData, frameLen)) {
                // Extract message type for logging
                uint16_t messageType = 0;
                if (frameLen >= 6) {
                    messageType = (frameData[3] << 4) | (frameData[4] >> 4);
                }

                Serial.printf("✓ RTCM TX: type=%u, %u bytes\n", messageType, frameLen);

                // Send via radio
                if (_radio) {
                    _radio->sendRTCM(frameData, frameLen);
                }
                else {
                    Serial.println("⚠️ No radio module available");
                }
            }
            else {
                crcErrors++;
                Serial.printf("⚠️ RTCM CRC error: frame %u bytes", frameLen);

                // Add hex dump for first CRC error to help debug
                if (crcErrors <= 3) {
                    Serial.print(" [");
                    for (size_t i = 0; i < min(frameLen, (size_t)12); i++) {
                        Serial.printf("%02X", frameData[i]);
                        if (i < min(frameLen, (size_t)12) - 1) Serial.print(" ");
                    }
                    if (frameLen > 12) Serial.print("...");
                    Serial.print("]");
                }
                Serial.println();

                // Try to resync by looking for next 0xD3 in current frame
                bool foundSync = false;
                for (size_t i = 1; i < framePos; i++) {
                    if (frameData[i] == 0xD3) {
                        // Found potential new frame start
                        size_t remaining = framePos - i;
                        memmove(frameData, frameData + i, remaining);
                        framePos = remaining;
                        frameLen = 0;
                        lengthKnown = false;
                        frameStartTime = millis();
                        foundSync = true;
                        Serial.printf("↻ Resynced at offset %u, %u bytes remaining\n", i, remaining);
                        break;
                    }
                }

                if (!foundSync) {
                    // No sync found, restart completely
                    inFrame = false;
                    framePos = 0;
                    lengthKnown = false;
                    syncLosses++;
                }
            }

            if (!inFrame || (inFrame && lengthKnown && framePos >= frameLen)) {
                // Reset for next frame if we're done
                inFrame = false;
                framePos = 0;
                lengthKnown = false;
            }
        }

        // Safety check: prevent buffer overrun
        if (framePos >= sizeof(frameData)) {
            Serial.printf("⚠️ RTCM frame buffer overrun at %u bytes\n", framePos);
            inFrame = false;
            framePos = 0;
            lengthKnown = false;
            syncLosses++;
        }
    }

    // Periodic statistics (every 30 seconds)
    static unsigned long lastStatsTime = 0;
    if (millis() - lastStatsTime >= 30000) {
        if (totalFrames > 0 || crcErrors > 0) {
            float errorRate = (totalFrames + crcErrors) > 0 ?
                (crcErrors * 100.0f) / (totalFrames + crcErrors) : 0.0f;

            Serial.printf("📊 RTCM Stats: %u frames OK, %u CRC errors (%.1f%%), %u sync losses\n",
                totalFrames, crcErrors, errorRate, syncLosses);

            // Reset counters for next period
            totalFrames = 0;
            crcErrors = 0;
            syncLosses = 0;
        }
        lastStatsTime = millis();
    }
}

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

    fix = GNSSFix{}; // reset

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    const char* f[16] = { 0 };
    int nf = 0;
    char* s = buf;
    while (nf < 16 && (f[nf++] = strsep(&s, ","))) {}

    if (nf < 10) return true;

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
    if (f[2] && f[3]) parseLatLon(f[2], f[3][0], fix.lat);
    if (f[4] && f[5]) parseLatLon(f[4], f[5][0], fix.lon);
    if (f[6]) fix.type = atoi(f[6]);
    if (f[7]) fix.SIV = atoi(f[7]);
    if (f[8]) fix.HDOP = atof(f[8]);
    if (f[9]) fix.elevation = atof(f[9]);
    //Serial.printf("%s\r\n",line);
    return true;
}

void GNSSModule::printFix(const GNSSFix& fix) {
    Serial.printf("GGA OK: %02d:%02d:%.2f FIX=%u SIV=%d HDOP=%.2f lat=%.6f lon=%.6f elev=%.2f\n",
        fix.hour, fix.minute, fix.second,
        (unsigned)fix.type, fix.SIV, fix.HDOP,
        fix.lat, fix.lon, fix.elevation);
}

void GNSSModule::clearUARTBuffer() {
    _buffer.clear();
    while (_ser->available()) _ser->read();
}
