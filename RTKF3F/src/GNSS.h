// GNSSModule.h
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

//--------------------------------------
// Timings & fix-type constants
//--------------------------------------
#define COMMANDDELAY 500u
#define RESETDELAY   5000u

//--------------------------------------
// FIXTYPE (kept from your header)
//--------------------------------------
enum class FIXTYPE : uint8_t {
    NOFIX = 0,
    GPS = 1,
    DGPS = 2,
    PPS = 3,
    RTK_FLOAT = 4,
    RTK_FIX = 5,
    DEAD_RECK = 6,
    MANUAL = 7,
    SIM = 8,
    OTHER = 9
};

//--------------------------------------
// GNSSModule
//--------------------------------------
class GNSSModule {
public:
    // -------- Message framing --------
    enum class GNSSMessageType {
        NONE,
        NMEA,
        RTCM,
        COMMAND_RESPONSE,
        UBX,
        UBX_RELPOSNED,
        UBX_NAV_PVT,
        UBX_ACK_ACK,
        UBX_ACK_NAK,
        NODATA
    };

    struct GNSSMessage {
        GNSSMessageType type = GNSSMessageType::NONE;
        const uint8_t* data = nullptr;
        size_t          length = 0;
    };

    // -------- Parsed fix snapshot --------
    struct GNSSFix {
        // UTC time
        int   hour = 0;
        int   minute = 0;
        float second = 0;

        // Position from GGA
        float lat = 0.0f;
        float lon = 0.0f;
        float alt = 0.0f;

        // Optional RELPOSNED (if you use it elsewhere)
        float relNorth = 0.0f;
        float relEast = 0.0f;
        float relDown = 0.0f;

        float adjNorth = 0.0f;
        float adjEast = 0.0f;
        float adjDown = 0.0f;

        int SIV = 0;  // Satellites used (from GGA field 7)
        int HDOP = 0;  // HDOP * 100 (dimensionless)
        FIXTYPE type = FIXTYPE::NOFIX;   // GGA fix quality code
    };

    // -------- Outbound parsed messages to the app --------
    enum class MsgType : uint8_t { NMEA_LINE, RTCM_FRAME, CMD_REPLY };

    struct NMEALine { char data[200]; size_t len; };
    struct RTCMFrame { uint16_t type; uint16_t bytes; uint8_t buf[1600]; };
    struct CmdReply { char line[200]; };

    struct ParsedMsg {
        MsgType type;
        union { NMEALine nmea; RTCMFrame rtcm; CmdReply reply; } u;
    };

    // Construction
    GNSSModule(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud = 460800)
        : _ser(serial), _rx(rxPin), _tx(txPin), _baud(baud) {
    }

    // Init UART, queues, tasks
    bool begin(size_t rxRingBytes = 8192) {
        _ser.setRxBufferSize(rxRingBytes);
        _ser.begin(_baud, SERIAL_8N1, _rx, _tx);

        if (!_parsedQ)  _parsedQ = xQueueCreate(16, sizeof(ParsedMsg));
        if (!_cmdQ)     _cmdQ = xQueueCreate(10, sizeof(CmdItem));
        if (!_respQ)    _respQ = xQueueCreate(20, sizeof(CmdReply));
        if (!_txMutex)  _txMutex = xSemaphoreCreateMutex();
        if (!_parsedQ || !_cmdQ || !_respQ || !_txMutex) return false;

        BaseType_t ok1 = xTaskCreatePinnedToCore(readerThunk, "gnssReader", 4096, this, 3, nullptr, ARDUINO_RUNNING_CORE);
        BaseType_t ok2 = xTaskCreatePinnedToCore(commandThunk, "gnssCommand", 2048, this, 2, nullptr, ARDUINO_RUNNING_CORE);
        return (ok1 == pdPASS && ok2 == pdPASS);
    }

    // Fire-and-forget command
    bool send(const char* cmd) {
        if (!cmd || !*cmd || !_cmdQ) return false;
        CmdItem it{}; strlcpy(it.text, cmd, sizeof(it.text));
        it.timeoutMs = 0; it.expect[0] = '\0';
        return xQueueSend(_cmdQ, &it, pdMS_TO_TICKS(50)) == pdTRUE;
    }

    // Blocking: send & wait for reply containing expectSubstr (optional)
    bool sendWait(const char* cmd, const char* expectSubstr, uint32_t timeoutMs, String* outLine = nullptr) {
        if (!cmd || !*cmd || !_cmdQ || !_respQ) return false;
        CmdItem it{};
        strlcpy(it.text, cmd, sizeof(it.text));
        it.timeoutMs = timeoutMs;
        if (expectSubstr) strlcpy(it.expect, expectSubstr, sizeof(it.expect));

		GDBG_PRINTF("GNSS: CMD -> %s", cmd);

        if (xQueueSend(_cmdQ, &it, pdMS_TO_TICKS(100)) != pdTRUE) return false;

        uint32_t t0 = millis();
        CmdReply r{};
        for (;;) {
            uint32_t elapsed = millis() - t0;
            if (elapsed >= timeoutMs) return false;
            uint32_t remain = timeoutMs - elapsed;
            if (xQueueReceive(_respQ, &r, pdMS_TO_TICKS(remain)) == pdTRUE) {
                if (!it.expect[0] || strstr(r.line, it.expect)) {
                    if (outLine) *outLine = r.line;
                    GDBG_PRINTF("Command [%s], response [%s]\r\n", cmd, r.line);
                    return true;
				}
                else {
                    GDBG_PRINTF("Command [%s], response [%s]", cmd, outLine->c_str());
                }
            }
        }
    }

    // --- GGA parser: fills your existing GNSSFix (time, lat, lon, alt, SIV, HDOP, fix type)
    static bool parseGGA(const char* s, GNSSModule::GNSSFix& fix) {
        if (!s || (strncmp(s, "$GNGGA", 6) && strncmp(s, "$GPGGA", 6))) return false;
        // very small tokenizer
        const char* f[15] = { 0 }; int n = 0;
        for (const char* p = s; *p && n < 15; ++p) { if ((p == s) || (*(p - 1) == ',')) f[n++] = p; while (*p && *p != ',') ++p; if (!*p) break; }
        if (n < 10) return false;

        auto toInt = [](const char* p) { return p ? atoi(p) : 0; };
        auto toFloat = [](const char* p) { return p ? (float)atof(p) : 0.f; };

        // time hhmmss.sss
        if (f[1]) {
            int hh = (f[1][0] - '0') * 10 + (f[1][1] - '0');
            int mm = (f[1][2] - '0') * 10 + (f[1][3] - '0');
            float ss = atof(f[1] + 4);
            fix.hour = hh; fix.minute = mm; fix.second = ss;
        }

        // lat ddmm.mmmm,N/S
        auto dmToDeg = [](float dm) { int dd = int(dm / 100); float m = dm - 100 * dd; return dd + m / 60.0f; };
        if (f[2] && f[3]) {
            float dm = atof(f[2]); float deg = dmToDeg(dm); if (f[3][0] == 'S') deg = -deg; fix.lat = deg;
        }
        // lon dddmm.mmmm,E/W
        if (f[4] && f[5]) {
            float dm = atof(f[4]); float deg = dmToDeg(dm); if (f[5][0] == 'W') deg = -deg; fix.lon = deg;
        }

        // fix quality (0..8) -> map to your enum
        int q = toInt(f[6]);
        switch (q) {
        case 0:  fix.type = FIXTYPE::NOFIX; break;
        case 1:  fix.type = FIXTYPE::GPS; break;
        case 2:  fix.type = FIXTYPE::DGPS; break;
        case 4:  fix.type = FIXTYPE::RTK_FIX; break;
        case 5:  fix.type = FIXTYPE::RTK_FLOAT; break;
        default: fix.type = FIXTYPE::OTHER; break;
        }

        fix.SIV = toInt(f[7]);
        fix.HDOP = int(100.0f * toFloat(f[8])); // *100 for your int HDOP storage
        fix.alt = toFloat(f[9]);
        return true;
    }


    static uint16_t getRTCMType(const uint8_t* frame, size_t len) {
        if (!isValidRTCM(frame, len)) return 0;
        return (static_cast<uint16_t>(frame[3]) << 4) | (frame[4] >> 4);
    }

    // Poll parsed outputs (non-blocking)
    bool readParsed(ParsedMsg& out) {
        if (!_parsedQ) return false;
        return xQueueReceive(_parsedQ, &out, 0) == pdTRUE;
    }

    static bool isValidRTCM(const uint8_t* frame, size_t len) {
        if (len < 6 || frame[0] != 0xD3) return false;
        uint16_t l = ((frame[1] & 0x03) << 8) | frame[2];
        size_t need = 3 + l + 3;
        if (need != len) return false;
        uint32_t exp = (uint32_t)frame[len - 3] << 16 | (uint32_t)frame[len - 2] << 8 | frame[len - 1];
        return crc24q(frame, len - 3) == exp;
    }

private:
    // UART & config
    HardwareSerial& _ser;
    int _rx, _tx; uint32_t _baud;

    // Queues & mutex
    struct CmdItem { char text[96]; uint32_t timeoutMs; char expect[48]; };
    QueueHandle_t     _parsedQ = nullptr;
    QueueHandle_t     _cmdQ = nullptr;
    QueueHandle_t     _respQ = nullptr;
    SemaphoreHandle_t _txMutex = nullptr;

    // Parser state
    static constexpr size_t NMEA_MAX = 200;
    static constexpr size_t RTCM_MAX = 1600;
    char    _nmeaBuf[NMEA_MAX]; size_t _nmeaFill = 0; bool _inNmea = false;
    uint8_t _rtcmBuf[RTCM_MAX]; size_t _rtcmFill = 0; bool _inRtcm = false; size_t _rtcmNeed = 0;

    // Tasks thunks
    static void readerThunk(void* arg) { static_cast<GNSSModule*>(arg)->readerTask(); }
    static void commandThunk(void* arg) { static_cast<GNSSModule*>(arg)->commandTask(); }

    static inline bool rtcmHeader(const uint8_t* f, size_t n,
        uint16_t& type, uint16_t& payLen) {
        if (n < 6 || f[0] != 0xD3) return false;
        payLen = (uint16_t)(((f[1] & 0x03) << 8) | f[2]);  // 10-bit payload length
        if (n < 3 + payLen + 3) return false;             // må ha full ramme inkl CRC

        const uint8_t* p = f + 3;                         // payload start
        type = (uint16_t)((p[0] << 4) | (p[1] >> 4));     // 12-bit meldingstype
        return true;
    }

    void emitRtcm(const uint8_t* frame, size_t len) {
        if (!_parsedQ || len < 6) return;

        uint16_t type = 0;
        uint16_t payLen = 0;

        // Bruk helper til å lese header
        if (!rtcmHeader(frame, len, type, payLen)) {
            Serial.println(F("RTCM: header parse error"));
            return;
        }

        // Debug-print
        //Serial.printf("RTCM: type=%u, payload=%u bytes, total=%u bytes\n",
        //    type, payLen, (unsigned)len);

        // Lag meldingsobjekt
        ParsedMsg m{};
        m.type = MsgType::RTCM_FRAME;
        m.u.rtcm.type = type;
        m.u.rtcm.bytes = len;

        size_t L = min(len, sizeof(m.u.rtcm.buf));
        memcpy(m.u.rtcm.buf, frame, L);

        // Send til queue
        xQueueSend(_parsedQ, &m, 0);
    }

    // Reader task
    void readerTask() {
        for (;;) {
            int avail = _ser.available();
            if (avail <= 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
            int ib = _ser.read(); if (ib < 0) continue;
            uint8_t b = (uint8_t)ib;

            // ASCII lines ($..., #...)
            if (!_inRtcm) {
                if (!_inNmea) {
                    if (b == '$' || b == '#') { _inNmea = true; _nmeaFill = 0; _nmeaBuf[_nmeaFill++] = (char)b; }
                }
                else {
                    if (_nmeaFill < NMEA_MAX - 1) _nmeaBuf[_nmeaFill++] = (char)b;
                    if (b == '\n') {
                        _nmeaBuf[_nmeaFill] = '\0';
                        handleAscii(_nmeaBuf, _nmeaFill);
                        _inNmea = false; _nmeaFill = 0;
                    }
                }
            }

            // RTCM3 frames
            if (!_inNmea) {
                if (!_inRtcm) {
                    if (b == 0xD3) { _inRtcm = true; _rtcmFill = 0; _rtcmNeed = 0; _rtcmBuf[_rtcmFill++] = 0xD3; }
                }
                else {
                    if (_rtcmFill < RTCM_MAX) _rtcmBuf[_rtcmFill++] = b;
                    if (_rtcmFill == 3) {
                        uint16_t len10 = (uint16_t)(((uint16_t)(_rtcmBuf[1] & 0x03) << 8) | _rtcmBuf[2]);
                        _rtcmNeed = 3 + (size_t)len10 + 3;
                        if (_rtcmNeed > RTCM_MAX) { _inRtcm = false; _rtcmFill = 0; _rtcmNeed = 0; }
                    }
                    else if (_rtcmNeed && _rtcmFill == _rtcmNeed) {
                        uint32_t expect = ((uint32_t)_rtcmBuf[_rtcmNeed - 3] << 16) |
                            ((uint32_t)_rtcmBuf[_rtcmNeed - 2] << 8) |
                            (uint32_t)_rtcmBuf[_rtcmNeed - 1];
                        uint32_t calc = crc24q(_rtcmBuf, _rtcmNeed - 3);
                        if (calc == expect) emitRtcm(_rtcmBuf, _rtcmNeed);
                        _inRtcm = false; _rtcmFill = 0; _rtcmNeed = 0;
                    }
                }
            }
        }
    }

    // Command task
    void commandTask() {
        CmdItem it{};
        for (;;) {
            if (xQueueReceive(_cmdQ, &it, portMAX_DELAY) == pdTRUE) {
                if (_txMutex && xSemaphoreTake(_txMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                    _ser.write((const uint8_t*)it.text, strlen(it.text));
                    _ser.write("\r\n");
                    _ser.flush();
                    xSemaphoreGive(_txMutex);
                }
            }
        }
    }

    // Helpers
    void handleAscii(const char* line, size_t len) {
        // Command replies start with '#' or $CONFIG,...
        if (len > 0 && (line[0] == '#' || (line[0] == '$' && strncmp(line, "$command", 8) == 0))) {
            if (_respQ) { CmdReply r{}; size_t L = min(len, sizeof(r.line) - 1); memcpy(r.line, line, L); r.line[L] = '\0'; xQueueSend(_respQ, &r, 0); }
            if (_parsedQ) {
                ParsedMsg m{}; m.type = MsgType::CMD_REPLY; size_t L = min(len, sizeof(m.u.reply.line) - 1);
                memcpy(m.u.reply.line, line, L); m.u.reply.line[L] = '\0'; xQueueSend(_parsedQ, &m, 0);
            }
        }
        // NMEA lines
        if (len > 0 && line[0] == '$' && _parsedQ) {
            ParsedMsg m{}; m.type = MsgType::NMEA_LINE; size_t L = min(len, sizeof(m.u.nmea.data) - 1);
            memcpy(m.u.nmea.data, line, L); m.u.nmea.data[L] = '\0'; m.u.nmea.len = L;
            xQueueSend(_parsedQ, &m, 0);
        }
    }

    static uint32_t crc24q(const uint8_t* data, size_t len) {
        uint32_t crc = 0; // init 0, poly 0x1864CFB
        for (size_t i = 0; i < len; ++i) {
            crc ^= (uint32_t)data[i] << 16;
            for (int b = 0; b < 8; ++b) {
                crc <<= 1; if (crc & 0x1000000) crc ^= 0x1864CFB; crc &= 0xFFFFFF;
            }
        }
        return crc & 0xFFFFFF;
    }



    // --- RTCM helpers (match your previous usage) ---



};