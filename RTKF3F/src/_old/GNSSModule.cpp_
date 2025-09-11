// GNSSModule.cpp
// GNSSModule.cpp
#include <Arduino.h>
#include <HardwareSerial.h>

#include "GNSSModule.h"
#include "_macros.h"

//--------------------------------------
// Internal helpers (file-local only)
//--------------------------------------
namespace {

    uint32_t calculateCRC24Q(const uint8_t* data, size_t len) {
        uint32_t crc = 0;
        for (size_t i = 0; i < len; ++i) {
            crc ^= ((uint32_t)data[i]) << 16;
            for (int j = 0; j < 8; ++j) {
                crc <<= 1;
                if (crc & 0x1000000) crc ^= 0x1864CFB;
            }
            crc &= 0xFFFFFF;
        }
        return crc;
    }

    // Wait until at least n bytes are available, with timeout.
    bool waitAvail(Stream& s, size_t n, uint32_t timeoutMs) {
        const uint32_t t0 = millis();
        while ((size_t)s.available() < n) {
            if (millis() - t0 > timeoutMs) return false;
            delay(1);
        }
        return true;
    }

    // Read a single RTCM frame (0xD3 ... CRC). Returns false on timeout/oversize.
    bool readOneRTCM(Stream& s,
        uint8_t* outBuf,
        size_t& outLen,
        size_t maxLen,
        uint32_t stepTimeoutMs = 50)
    {
        outLen = 0;
        if (maxLen < 6) return false;

        // 1) Seek sync (0xD3)
        uint32_t t0 = millis();
        while (true) {
            if (s.available() > 0) {
                int b = s.read();
                if (b == 0xD3) { outBuf[0] = 0xD3; break; }
            }
            else if (millis() - t0 > stepTimeoutMs) {
                return false;
            }
            else {
                delay(1);
            }
        }

        // 2) Read length (2 bytes)
        if (!waitAvail(s, 2, stepTimeoutMs)) return false;
        outBuf[1] = (uint8_t)s.read();
        outBuf[2] = (uint8_t)s.read();

        const uint16_t payloadLen = ((outBuf[1] & 0x03) << 8) | outBuf[2];
        const size_t   total = 3 + payloadLen + 3;

        // If total would exceed buffer, skip this frame (consume, report false)
        if (total > maxLen) {
            if (!waitAvail(s, payloadLen + 3, stepTimeoutMs)) return false;
            for (size_t i = 0; i < payloadLen + 3; ++i) (void)s.read();
            return false;
        }

        // 3) Read payload + CRC
        if (!waitAvail(s, payloadLen + 3, stepTimeoutMs)) return false;
        for (size_t i = 0; i < payloadLen + 3; ++i) outBuf[3 + i] = (uint8_t)s.read();

        outLen = total;
        return true;
    }

    // Read a line into a byte buffer, strip CR/LF, NUL terminate; timeout.
    bool readLineWithTimeout(HardwareSerial& serial,
        uint8_t* buffer,
        size_t& len,
        size_t maxLen,
        uint32_t timeoutMs)
    {
        len = 0;
        if (maxLen == 0) return false;
        const uint32_t start = millis();

        while (millis() - start < timeoutMs) {
            while (serial.available() > 0) {
                int v = serial.read();
                if (v < 0) break;
                const uint8_t c = (uint8_t)v;

                if (len < (maxLen - 1)) buffer[len++] = c;

                if (c == '\n') {
                    while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) --len;
                    buffer[len] = '\0';
                    return true;
                }
            }
            delay(1);
        }

        if (len > 0) {
            while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) --len;
            buffer[len] = '\0';
        }
        return false;
    }

    // Convenience overload for char* buffers
    inline bool readLineWithTimeout(HardwareSerial& serial,
        char* buffer,
        size_t& len,
        size_t maxLen,
        uint32_t timeoutMs)
    {
        return readLineWithTimeout(serial, reinterpret_cast<uint8_t*>(buffer), len, maxLen, timeoutMs);
    }

} // namespace

bool _asciiEnabled = true;   // set til false rett etter at du enable’r RTCMxxxx COM2 ...

static inline uint32_t nowMs() { return (uint32_t)millis(); }

//--------------------------------------
// GNSSModule: lifecycle & basic I/O
//--------------------------------------
GNSSModule::GNSSModule(HardwareSerial& serial)
    : _serial(serial) {
}

void GNSSModule::begin(uint32_t baud, int RX, int TX) {
    _serial.begin(baud, SERIAL_8N1, RX, TX);
    _serial.setRxBufferSize(4096);
}

void GNSSModule::sendCommand(const String& command, int eatResponseTime) {
    GDBG_PRINT("GPS: Command ["); GDBG_PRINT(command); GDBG_PRINT("]\r\n");
    const unsigned long start = millis();
    _serial.print(command + "\r\n");
    if (eatResponseTime > 0) {
        while ((millis() - start) < eatResponseTime) {
            if (_serial.available()) _serial.read();
            delay(1);
        }
    }
}

// Probe UM980 logical COMx by sending "unlog comN" and looking for OK
uint8_t GNSSModule::detectGPS() {
    static const char* cmds[] = { "versiona com1\r\n", "versiona com2\r\n", "versiona com3\r\n" };
    const unsigned long windowMs = 700; // ~0.7s is safe for UM980

    for (uint8_t port = 1; port <= 3; ++port) {
        GDBG_PRINTLN("GNSS: Testing port " + String(port) + " command: " + cmds[port - 1]);

        // Clear stale RX bytes
        while (_serial.available()) _serial.read();

        // Send probe (your sendCommand() adds \r\n)
        _serial.print(cmds[port - 1]);

        // Collect response for a bounded window
        String resp;
        const unsigned long t0 = millis();
        while (millis() - t0 < windowMs) {
            while (_serial.available()) {
                int c = _serial.read();
                if (c >= 0) resp += (char)c;   
            }
        }

        // Case-insensitive search for the VERSIONA log
        if (resp.indexOf("#VERSIONA") >= 0) {
            return port; // found!
        }
    }
    return 0; // none found
}


// Non-blocking, kort linje-lesing (3-5ms maks)
bool readLineQuick(Stream& s, uint8_t* buf, size_t& len, size_t cap, uint32_t max_ms = 5) {
    len = 0;
    uint32_t t0 = nowMs();
    while ((nowMs() - t0) < max_ms) {
        while (s.available()) {
            int c = s.read();
            if (c < 0) break;
            if (len + 1 < cap) buf[len++] = (uint8_t)c; // behold CR/LF om ønsket
            if (c == '\n') { return true; }             // ferdig på LF
        }
        // kort pust – ikke delay(), bare gi CPU en sjanse
    }
    return (len > 0 && buf[len - 1] == '\n');
}

// Robust RTCM-lesing med sliding resync
bool readOneRTCM_robust(Stream& stream, uint8_t* buf, size_t& len, size_t maxLen, uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    while ((millis() - t0) < timeoutMs) {
        // 1. Finn startbyte
        if (!stream.available()) continue;
        if (stream.peek() != 0xD3) {
            stream.read(); // dropp støy
            continue;
        }

        // 2. Startbyte OK – les header (3 byte)
        if (stream.available() < 3) continue;
        buf[0] = stream.read();  // 0xD3
        buf[1] = stream.read();  // lenH
        buf[2] = stream.read();  // lenL

        const uint16_t payloadLen = ((buf[1] & 0x03) << 8) | buf[2];
        const size_t totalLen = 3 + payloadLen + 3;

        // 3. Buffer check
        if (totalLen > maxLen) {
            Serial.printf("RTCM: Packet too big (%u bytes)\n", totalLen);
            // sliding resync: les én byte og prøv igjen
            for (int i = 0; i < 2; ++i) if (stream.available()) stream.read();
            continue;
        }

        // 4. Vent på resten av pakken
        uint32_t t1 = millis();
        while ((millis() - t1) < 20) { // 20ms max wait
            if (stream.available() >= (payloadLen + 3)) break;
        }
        if (stream.available() < (payloadLen + 3)) {
            Serial.println("RTCM: Timeout reading payload");
            continue;
        }

        // 5. Les resten (payload + CRC)
        stream.readBytes(buf + 3, payloadLen + 3);
        len = totalLen;

        // 6. Valider CRC
        if (GNSSModule::isValidRTCM(buf, len)) {
            return true;
        }
        else {
            Serial.println("RTCM: CRC error – resyncing");
            // Ikke les noe her – neste while-iterasjon vil fange neste 0xD3
        }
    }

    // Timeout
    return false;
}


GNSSModule::GNSSMessage GNSSModule::readGPS() {
    GNSSMessage result;
    int first = _serial.peek();

    if (first < 0) {
        result.type = GNSSMessageType::NODATA;
        return result;
    }

    // 1. BINÆR: RTCM – gi alltid høyeste prioritet
    if (first == 0xD3) {
        size_t len = 0;
        if (readOneRTCM_robust(_serial, _gpsBuf, len, sizeof(_gpsBuf), 15)) {
            result.type = GNSSMessageType::RTCM;
            result.data = _gpsBuf;
            result.length = len;

            for (int i = 0; i < len; i++) { Serial.printf("%02X ", _gpsBuf[i]); }
            return result;
        }
        else {
            result.type = GNSSMessageType::NODATA;
            return result; // sliding resync internt
        }
    }

#ifdef USE_UBX
    // 2. BINÆR: UBX – hvis aktivert
    if (first == 0xB5) {
        if (readUBX(_serial, _gpsBuf, result, 5)) {
            return result;
        }
        else {
            _serial.read(); // dropp 1 byte ved UBX-feil
            result.type = GNSSMessageType::NODATA;
            return result;
        }
    }
#endif

    // 3. ASCII: kun hvis aktivert
    if (_asciiEnabled && (first == '#' || first == '$')) {
        size_t len = 0;
        if (readLineQuick(_serial, _gpsBuf, len, sizeof(_gpsBuf), 5)) {
            if (first == '#') {
                result.type = GNSSMessageType::COMMAND_RESPONSE;
            }
            else {
                constexpr size_t tagLen = sizeof("$command,") - 1;
                if (len >= tagLen && memcmp(_gpsBuf, "$command,", tagLen) == 0) {
                    result.type = GNSSMessageType::COMMAND_RESPONSE;
                }
                else {
                    result.type = GNSSMessageType::NMEA;
                }
            }
            result.data = _gpsBuf;
            result.length = len;
            return result;
        }
        else {
            // linja ikke ferdig – vent
            result.type = GNSSMessageType::NODATA;
            return result;
        }
    }

    // 4. ASCII-støy etter RTCM aktivert – dropp én kjent ascii-byte
    if (!_asciiEnabled && (first == '#' || first == '$')) {
        _serial.read(); // slipp neste kall til
        result.type = GNSSMessageType::NODATA;
        return result;
    }

    // 5. Ukjent byte – dropp én byte og prøv igjen senere
    _serial.read();
    result.type = GNSSMessageType::NODATA;
    return result;
}


//--------------------------------------
// GNSSModule: GGA parser & presentation
//--------------------------------------
bool GNSSModule::parseGGA(const uint8_t* buf, size_t len, GNSSFix& fix) {
    const int MAX_FIELDS = 20;
    const int BUF_SIZE = 128;
    char local[BUF_SIZE];

    // Optional debug print
    for (size_t i = 0; i < len; i++) GDBG_PRINTF(buf[i] < 32 ? "." : "%c", buf[i]);
    GDBG_PRINTLN();

    if (len < 6 || buf[0] != '$') return false;

    if (len >= BUF_SIZE) len = BUF_SIZE - 1;
    memcpy(local, buf, len);
    local[len] = '\0';

    while (len && (local[len - 1] == '\r' || local[len - 1] == '\n')) local[--len] = '\0';

    char* p = local;
    if (*p == '$') ++p;

    if (char* star = strchr(p, '*')) *star = '\0';

    const char* fields[MAX_FIELDS] = { nullptr };
    int fieldCount = 0;
    char* fieldStart = p;

    for (char* s = p;; ++s) {
        if (*s == ',' || *s == '\0') {
            if (fieldCount < MAX_FIELDS) {
                char saved = *s;
                *s = '\0';
                fields[fieldCount++] = fieldStart;
                if (saved == '\0') break;
                fieldStart = s + 1;
            }
            else break;
        }
    }

    if (fieldCount == 0 || !fields[0]) return false;

    const char* type = fields[0];
    size_t tlen = strlen(type);
    if (tlen < 3 || strcmp(type + (tlen - 3), "GGA") != 0) return false;

    auto f = [&](int idx) -> const char* { return (idx < fieldCount && fields[idx]) ? fields[idx] : ""; };

    // Defaults
    fix.hour = fix.minute = 0; fix.second = 0.0f;
    fix.lat = fix.lon = 0.0f;
    fix.type = FIXTYPE::NOFIX; fix.SIV = 0; fix.HDOP = 0; fix.alt = 0.0f;

    // 1) Time hhmmss.ss
    if (*f(1)) {
        double rawTime = atof(f(1));
        int hh = int(rawTime / 10000.0);
        int mm = int(fmod(rawTime / 100.0, 100.0));
        double ss = fmod(rawTime, 100.0);
        if (hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0.0 && ss < 60.001) {
            fix.hour = hh; fix.minute = mm; fix.second = float(ss);
        }
    }

    // 2) Lat/Lon
    auto parseLatLon = [](const char* raw, const char* dir, bool isLat, double& out) {
        out = 0.0;
        if (!raw || !*raw || !dir || !*dir) return;
        double val = atof(raw);
        if (val <= 0.0) return;
        int deg = int(val / 100.0);
        double min = val - deg * 100.0;
        double degDec = double(deg) + (min / 60.0);
        char d = dir[0];
        if (isLat ? (d == 'N' || d == 'S') : (d == 'E' || d == 'W')) {
            if (d == 'S' || d == 'W') degDec = -degDec;
            out = degDec;
        }
        };
    double latDec = 0.0, lonDec = 0.0;
    parseLatLon(f(2), f(3), true, latDec);
    parseLatLon(f(4), f(5), false, lonDec);
    fix.lat = (float)latDec;
    fix.lon = (float)lonDec;

    // 3) Fix type / SIV
    if (*f(6)) fix.type = static_cast<FIXTYPE>(atoi(f(6)));
    if (*f(7)) fix.SIV = atoi(f(7));

    // 4) HDOP (*100 int)
    if (*f(8)) {
        double hd = atof(f(8));
        fix.HDOP = (hd > 0.0 && hd < 100.0) ? int(hd * 100.0 + 0.5) : 0;
    }
    else {
        fix.HDOP = 0;
    }

    // 5) Altitude (MSL)
    if (*f(9)) fix.alt = (float)atof(f(9)); else fix.alt = 0.0f;

    return true;
}

//--------------------------------------
// GNSSModule: RTCM utilities
//--------------------------------------
uint16_t GNSSModule::getRTCMType(const uint8_t* buf, size_t len) {
    if (len < 6 || buf[0] != 0xD3) return 0;
    return (uint16_t)(((uint16_t)buf[3] << 4) | (buf[4] >> 4)) & 0x0FFF;
}

bool GNSSModule::isValidRTCM(const uint8_t* data, size_t len) {
    if (len < 6 || data[0] != 0xD3) {
        Serial.println("RTCM: header mismatch or too short");
        return false;
    }
    const uint16_t payloadLen = ((data[1] & 0x03) << 8) | data[2];
    const uint32_t totalLen = 3 + payloadLen + 3;
    if (len != totalLen) {
        Serial.printf("RTCM: length mismatch (have %u, want %u)\n", (unsigned)len, (unsigned)totalLen);
        return false;
    }

    const uint32_t want = ((uint32_t)data[totalLen - 3] << 16) |
        ((uint32_t)data[totalLen - 2] << 8) |
        data[totalLen - 1];
    const uint32_t got = calculateCRC24Q(data, totalLen - 3);
    if (want != got) {
        Serial.printf("RTCM: CRC fail (type=%d len=%u)\n", getRTCMType(data, len), (unsigned)len);
        return false;
    }
    return true;
}

