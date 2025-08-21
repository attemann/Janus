// GNSSModule.cpp
#include <Arduino.h>    
#include <RFM69.h>
#include <HardwareSerial.h>
#include <RTKF3F.h>

#include "GNSSModule.h"
#include "_macros.h"

bool readOneRTCM(Stream& s,
    uint8_t* outBuf,
    size_t& outLen,
    size_t maxLen,
    uint32_t timeoutMs = 50); // default ok

GNSSModule::GNSSModule(HardwareSerial& serial)
    : _serial(serial), /* _nmeaIdx(0),*/ rtcmHandler(this) {
}

void GNSSModule::begin(uint32_t baud, int RX, int TX) {
    _serial.begin(baud, SERIAL_8N1, RX, TX);
    _serial.setRxBufferSize(4096);
}

bool GNSSModule::gpsDataAvailable() {
    return _serial.available() > 0;
}

int GNSSModule::detectUARTPort() {
    const char* testCommands[] = {
        "unlog com1",
        "unlog com2",
        "unlog com3"
    };

    for (int port = 1; port <= 3; port++) {
        _serial.flush(); // Clear any previous data
        delay(100);
        GDBG_PRINTLN("GNSS: Testing port " + String(port) + " with command: " + testCommands[port - 1]);

		sendCommand(testCommands[port - 1]);

        unsigned long startTime = millis();
        String response = "";

        while (millis() - startTime < COMMANDDELAY) {
            while (_serial.available()) {
                char c = _serial.read();
                response += c;
            }
        }

        if (response.indexOf("$command,unlog,response: OK")) {
            return port;
        }
    }
    return 0; // None found
}

void GNSSModule::sendCommand(const String& command) {
    GDBG_PRINT("GPS: Command [");
    GDBG_PRINT(command);
    GDBG_PRINT("] ");
    unsigned long start = millis();
    _serial.print(command);
	_serial.print("\r\n");
    while ((millis() - start) < COMMANDDELAY) {
        if (_serial.available()) GDBG_WRITE(_serial.read());
    }
}

void GNSSModule::sendReset() {
	String command = "freset";
    GDBG_PRINT("GPS: Command [");
    GDBG_PRINT(command);
    GDBG_PRINT("] ");
    unsigned long start = millis();
    _serial.print(command);
    _serial.print("\r\n");
    while ((millis() - start) < RESETDELAY) {
        if (_serial.available()) GDBG_WRITE(_serial.read());
    }
}

static bool waitAvail(Stream& s, size_t n, uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while ((size_t)s.available() < n) {
        if (millis() - t0 > timeoutMs) return false;
        delay(1);
    }
    return true;
}


bool readOneRTCM(Stream& s,uint8_t* outBuf,size_t& outLen,size_t maxLen,uint32_t stepTimeoutMs) {
    outLen = 0;
    if (maxLen < 6) return false;

    // 1) Resync to 0xD3
    uint32_t t0 = millis();
    while (true) {
        if (s.available() > 0) {
            int b = s.read();
            if (b == 0xD3) {
                outBuf[0] = 0xD3;
                break; // got sync
            }
            // else keep scanning
        }
        else if (millis() - t0 > stepTimeoutMs) {
            return false; // no data -> try again later
        }
        else {
            delay(1);
        }
    }

    // 2) Read 2 bytes of length
    if (!waitAvail(s, 2, stepTimeoutMs)) return false; // will re-sync next call
    outBuf[1] = (uint8_t)s.read();
    outBuf[2] = (uint8_t)s.read();

    const uint16_t payloadLen = ((outBuf[1] & 0x03) << 8) | outBuf[2];
    const size_t   total = 3 + payloadLen + 3;

    if (total > maxLen) {
        if (!waitAvail(s, payloadLen + 3, stepTimeoutMs)) {
            return false;
        }
        for (size_t i = 0; i < payloadLen + 3; ++i) (void)s.read();
        return false;
    }

    // 3) Read payload + CRC
    if (!waitAvail(s, payloadLen + 3, stepTimeoutMs)) return false;
    for (size_t i = 0; i < payloadLen + 3; ++i) {
        outBuf[3 + i] = (uint8_t)s.read();
    }

    outLen = total;
    return true;
}


inline bool readLineWithTimeout(HardwareSerial& serial,
    char* buffer,
    size_t& len,
    size_t maxLen,
    uint32_t timeoutMs);

bool readLineWithTimeout(HardwareSerial& serial,
    uint8_t* buffer,
    size_t& len,
    size_t maxLen,
    uint32_t timeoutMs)
{
    len = 0;
    const uint32_t start = millis();

    // Beskytt mot maxLen==0
    if (maxLen == 0) return false;

    while (millis() - start < timeoutMs) {
        while (serial.available() > 0) {
            int v = serial.read();
            if (v < 0) break;                 // ingenting å lese likevel
            uint8_t c = static_cast<uint8_t>(v);

            // Samle opp, men ikke gå over buffer (spar 1 byte til '\0')
            if (len < (maxLen - 1)) {
                buffer[len++] = c;
            }
            // Stopp på LF; strip CR/LF og nullterminér
            if (c == '\n') {
                // Fjern evt. CR/LF på slutten
                while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
                    --len;
                }
                buffer[len] = '\0';
                return true;
            }
        }
        // Gi andre tasks litt CPU
        delay(1);
    }

    // Timeout: nullterminer evt. delvis linje og strip CR/LF
    if (len > 0) {
        while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
            --len;
        }
        buffer[len] = '\0';
    }
    return false;
}

// Overload som lar deg sende inn char*-buffer uten cast i brukerstedet
inline bool readLineWithTimeout(HardwareSerial& serial,
    char* buffer,
    size_t& len,
    size_t maxLen,
    uint32_t timeoutMs)
{
    return readLineWithTimeout(serial,
        reinterpret_cast<uint8_t*>(buffer),
        len, maxLen, timeoutMs);
}

GNSSModule::GNSSMessage GNSSModule::readGPS() {
    GNSSMessage result;

    int first = _serial.peek();
    if (first < 0) return result;

    // 1) Kommandosvar (#...) eller $command,...
    if (first == '#') {
        size_t len = 0;
        if (readLineWithTimeout(_serial, _gpsBuf, len, sizeof(_gpsBuf), 50)) {  // <-- manglet ,50)
            result.type = GNSSMessageType::COMMAND_RESPONSE;
            result.data = _gpsBuf;
            result.length = len;
            return result;
        }
        // Ikke konsumér byte her; vent til hele linja kommer
        return result;
    }

    if (first == '$') {
        size_t len = 0;
        if (readLineWithTimeout(_serial, _gpsBuf, len, sizeof(_gpsBuf), 50)) {
            constexpr size_t tagLen = sizeof("$command,") - 1;
            if (len >= tagLen && memcmp(_gpsBuf, "$command,", tagLen) == 0) {
                result.type = GNSSMessageType::COMMAND_RESPONSE;
            }
            else {
                result.type = GNSSMessageType::NMEA;
            }
            result.data = _gpsBuf;
            result.length = len;
            return result;
        }
        // Ikke konsumér byte her; vent til hele linja kommer
        return result;
    }

    // 2) RTCM (binær)
    if (first == 0xD3) {
        size_t len = 0;
        if (readOneRTCM(_serial, _gpsBuf, len, sizeof(_gpsBuf), 200)) {
            result.type = GNSSMessageType::RTCM;
            result.data = _gpsBuf;
            result.length = len;
            return result;
        }
        // Feil/timeout i binær: dropp 1 byte for å søke ny sync
        if (_serial.available()) _serial.read();
        return result;
    }

    // 3) (valgfritt) UBX bak makro
#ifdef USE_UBX
    if (first == 0xB5) {
        if (readUBX(_serial, _gpsBuf, result, 50)) return result;
        // På UBX-timeout kan du også velge å *ikke* konsumere, men dette er greit:
        if (_serial.available()) _serial.read();
        return result;
    }
#endif

    // Ukjent byte → konsumér én for å komme videre
    if (_serial.available()) _serial.read();
    return result;
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

void GNSSModule::printAscii() {
    for (size_t i = 0; i < _gpsBufLen; ++i) {
        char c = _gpsBuf[i];
        if (c >= 32 && c <= 126) // Printable ASCII range
            GDBG_PRINT(c);
        else if (c == '\r')
            GDBG_PRINT("\\r");
        else if (c == '\n')
            GDBG_PRINT("\\n");
        else
            GDBG_PRINT('.');
    }
    GDBG_PRINTLN();
}

bool GNSSModule::parseGGA(const uint8_t* buf, size_t len, GNSSFix& fix) {
    const int MAX_FIELDS = 20;
    const int BUF_SIZE = 128;
    char local[BUF_SIZE];

    // Debug print (valgfritt)
    for (size_t i = 0; i < len; i++) {
        GDBG_PRINTF(buf[i] < 32 ? "." : "%c", buf[i]);
    }
    GDBG_PRINTLN();

    // Rask sanity
    if (len < 6 || buf[0] != '$') return false;

    // Lag sikker, null-terminert kopi
    if (len >= BUF_SIZE) len = BUF_SIZE - 1;
    memcpy(local, buf, len);
    local[len] = '\0';

    // Fjern CR/LF i enden om de finnes (ikke nødvendig, men ryddig)
    while (len && (local[len - 1] == '\r' || local[len - 1] == '\n')) {
        local[--len] = '\0';
    }

    // Finn start (hopp over '$')
    char* p = local;
    if (*p == '$') ++p;

    // Kapp av *checksum hvis til stede (vi validerer ikke her, men unngår at * havner i siste felt)
    char* star = strchr(p, '*');
    if (star) *star = '\0';

    // Del opp i felt og BEVAR tomme felter
    // fields[0] = talker+type (f.eks. "GNGGA" eller "GPGGA")
    const char* fields[MAX_FIELDS] = { nullptr };
    int fieldCount = 0;

    char* fieldStart = p;
    for (char* s = p; ; ++s) {
        if (*s == ',' || *s == '\0') {
            if (fieldCount < MAX_FIELDS) {
                // Sett terminator der komma var (om det var komma)
                char saved = *s;
                *s = '\0';
                fields[fieldCount++] = fieldStart; // Kan være tom streng ""
                if (saved == '\0') break;
                // Neste felt starter etter komma
                fieldStart = s + 1;
            }
            else {
                break;
            }
        }
    }

    // Må i det minste ha setningstype
    if (fieldCount == 0 || fields[0] == nullptr) return false;

    // Sjekk at dette er en GGA
    // Tillat både GN/G P GGA osv: slutt på "GGA"
    const char* type = fields[0];
    size_t tlen = strlen(type);
    if (tlen < 3 || strcmp(type + (tlen - 3), "GGA") != 0) {
        return false;
    }

    // Hjelper for trygg henting av felt (kan være nullptr eller "")
    auto f = [&](int idx) -> const char* {
        return (idx < fieldCount && fields[idx]) ? fields[idx] : "";
        };

    // Init default/0 (i tilfelle tomme felter)
    fix.hour = fix.minute = 0;
    fix.second = 0.0f;
    fix.lat = fix.lon = 0.0f;
    fix.fixType = 0;
    fix.SIV = 0;
    fix.HDOP = 0;  // lagres som *100
    fix.alt = 0.0f;

    // Feltindekser (NMEA GGA):
    // 0:  talker+type  (..GGA)
    // 1:  time (hhmmss.ss)
    // 2:  lat (ddmm.mmmm)
    // 3:  N/S
    // 4:  lon (dddmm.mmmm)
    // 5:  E/W
    // 6:  fix quality (0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float, ...)
    // 7:  num sats used
    // 8:  HDOP
    // 9:  alt (MSL)
    // 10: 'M'
    // 11: geoid sep
    // 12: 'M'
    // 13: age of diff
    // 14: diff ref station

    // 1) Tid
    if (*f(1)) {
        // Noen moduler kan sende "hhmmss", "hhmmss.sss" eller "hhmmss.ss"
        double rawTime = atof(f(1));
        int hh = int(rawTime / 10000.0);
        int mm = int(fmod(rawTime / 100.0, 100.0));
        double ss = fmod(rawTime, 100.0);
        if (hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0.0 && ss < 60.0 + 1e-3) {
            fix.hour = hh;
            fix.minute = mm;
            fix.second = float(ss);
        }
    }

    // 2) Lat/Lon
    auto parseLatLon = [](const char* raw, const char* dir, bool isLat, double& out) {
        out = 0.0;
        if (!raw || !*raw || !dir || !*dir) return;
        double val = atof(raw);
        if (val <= 0.0) return;

        int deg = isLat ? int(val / 100.0) : int(val / 100.0);
        double min = val - deg * 100.0;
        double degDec = double(deg) + (min / 60.0);
        char d = dir[0];
        if ((isLat && (d == 'N' || d == 'S')) ||
            (!isLat && (d == 'E' || d == 'W'))) {
            if (d == 'S' || d == 'W') degDec = -degDec;
            out = degDec;
        }
        };

    double latDec = 0.0, lonDec = 0.0;
    parseLatLon(f(2), f(3), true, latDec);
    parseLatLon(f(4), f(5), false, lonDec);
    fix.lat = (float)latDec;
    fix.lon = (float)lonDec;

    // 3) Fix-type og SIV
    if (*f(6)) fix.fixType = atoi(f(6)); // 0 = no fix
    if (*f(7)) fix.SIV = atoi(f(7)); // satelitter brukt

    // 4) HDOP
    if (*f(8)) {
        double hd = atof(f(8));
        // UM980 sender ofte 9999.0 når ikke gyldig -> sett til 0 for "ikke tilgjengelig"
        if (hd > 0.0 && hd < 100.0) {
            fix.HDOP = int(hd * 100.0 + 0.5);  // *100 for å bevare to desimaler som int
        }
        else {
            fix.HDOP = 0;
        }
    }
    else {
        fix.HDOP = 0;
    }

    // 5) Høyde (MSL)
    if (*f(9)) {
        fix.alt = (float)atof(f(9));
    }
    else {
        fix.alt = 0.0f;
    }

    // Avledede flagg
    fix.gpsFix = (fix.fixType > 0);
    fix.diffUsed = (fix.fixType >= 2);
    fix.rtkFix = (fix.fixType == 4);
    fix.rtkFloat = (fix.fixType == 5);

    // Vi returnerer TRUE selv om fixType==0, siden feltene (tid, SIV, osv.) er populert.
    return true;
}


 bool GNSSModule::readNMEA(uint8_t* buffer, size_t& len) {
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
                inSentence = false;
                nmeaPos = 0;
                continue;
            }
            if (c == '\n') {
                nmeaBuf[nmeaPos] = '\0';
                memcpy(buffer, nmeaBuf, nmeaPos + 1);
                len = nmeaPos;
                inSentence = false;
                return true;
            }
        }
    }
    return false;
}


bool GNSSModule::readCommandResponse(uint8_t* buffer, size_t& len) {
    size_t idx = 0;
    while (_serial.available()) {
        char c = _serial.read();
        if (idx < 127) {
            buffer[idx++] = c;
        }
        if (c == '\n' || idx >= 127) {
            buffer[idx] = '\0';
            len = idx;
            if (idx >= 2 && buffer[idx - 2] == '\r') {
                buffer[idx - 2] = '\n';
                buffer[idx - 1] = '\0';
                len--;
            }
            return true;
        }
    }
    len = idx;
    return false;
}

uint16_t GNSSModule::getRTCMType(const uint8_t* buf, size_t len) {
    if (len < 6 || buf[0] != 0xD3) return 0; // sanity check
    uint16_t type = ((buf[3] << 4) | (buf[4] >> 4)) & 0x0FFF;
    return type;
}

bool GNSSModule::isValidRTCM(const uint8_t* data, size_t len) {
    if (data[0] != 0xD3 || len < 6) {
        Serial.println("RTCM: header mismatch or too short");
        return false;
    }

    uint16_t payloadLen = ((data[1] & 0x03) << 8) | data[2]; // only 2 LSBs of byte 1    if (len != 3 + payloadLen + 3) return false;              // exact length check
    uint32_t totalLen = 3 + payloadLen + 3;

    if (len != totalLen) {
        Serial.printf("RTCM: length mismatch (have %u, want %u)\n", len, totalLen);
        return false;
    }

    uint32_t want = ((uint32_t)data[totalLen - 3] << 16) |
        ((uint32_t)data[totalLen - 2] << 8) |
        data[totalLen - 1];
    uint32_t got = calculateCRC24Q(data, totalLen - 3);
    if (want != got) {
        Serial.printf("RTCM: CRC fail (type=%d len=%u)\n", getRTCMType(data, len), len);
        return false;
    }
    return true;
}

uint32_t GNSSModule::calculateCRC24Q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint32_t)data[i]) << 16;
        for (int j = 0; j < 8; ++j) {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= 0x1864CFB;
        }
        crc &= 0xFFFFFF;
    }
    return crc;
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
    GDBG_PRINTF(
        "Time %02d:%02d:%02.0f "
        " %.6f%c"
        " %.6f%c"
        " %.2fm"
        ANSI_GREEN " FIX=%d" ANSI_RESET
        ANSI_GREEN " SIV=%d" ANSI_RESET
        " HDOP=%.2f\r\n"
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
        fix.fixType,
        fix.SIV,
        fix.HDOP / 100.0f
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

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

void GNSSModule::RTCMHandler::sendConfig(int index) {
    if (!parent) return;
    if (index < 0 || index >= (int)ARRAY_LEN(messages)) return; // bounds check

    auto& msg = messages[index];
    if (!msg.name) return; // sikkerhet

    String cmd;
    if (msg.enabled)
        cmd = String(msg.name) + " com2 " + String(msg.frequency, 1);
    else
        cmd = "unlog " + String(msg.name) + " com2";

    GDBG_PRINTF("sendConfig: Sending %s", cmd.c_str());  // <-- viktig

    parent->sendCommand(cmd); // antar denne tar String

    delay(100);
}

void GNSSModule::RTCMHandler::sendAllConfig() {
    for (size_t i = 0; i < count; ++i)
        sendConfig(i);
}

void GNSSModule::RTCMHandler::printList(bool showOnlyEnabled) {
    GDBG_PRINTLN(" #  Name        |  Freq   | State    | Sent   | Description");
    GDBG_PRINTLN("-------------------------------------------------------------------");
    for (size_t i = 0; i < count; ++i) {
        if (showOnlyEnabled && !messages[i].enabled) continue;
        const char* color = messages[i].enabled ? ANSI_GREEN : ANSI_RED;
        GDBG_PRINTF("%s%2d: %-10s | %5.2f Hz | %-8s | %6lu | %s%s\n",
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