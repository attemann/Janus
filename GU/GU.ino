// GU.ino
#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include <RadioTask.h>
#include "GU.h"

#define APPNAME "GU 1.0"
#define THIS_NODE_ID NODEID_GU

// Radio
inline constexpr int8_t RFM69_MISO = 20;
inline constexpr int8_t RFM69_MOSI = 18;
inline constexpr int8_t RFM69_SCK  = 19;
inline constexpr int8_t RFM69_CSS  = 21;
inline constexpr int8_t RFM69_IRQ  = 16;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST  = -1;

RadioModule::RTCM_Reassembler reassembler;

bool showFix = true;
bool showGngga = false;

// UART (GNSS)
#define GNSS_BAUD 115200
#define UART_RX   5
#define UART_TX   4

HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS, UART_RX, UART_TX, GNSS_BAUD);
GNSSModule::GNSSFix fix;

Arena  arena;
Glider glider;

static unsigned long lastSpeakMs = 0;
#define SPEAK_INTERVAL_MS 30000

static inline void haltUnit(const String& msg1, const String& msg2) {
    Serial.print(msg1); Serial.print(": "); Serial.println(msg2);
    while (true);
}


// ------------------------------
// Centralized GNSS message handling
// ------------------------------
static inline void handleParsed(const GNSSModule::ParsedMsg& m) {
    switch (m.type) {
    case GNSSModule::MsgType::NMEA_LINE: {
        if (m.u.nmea.len > 6 && !strncmp(m.u.nmea.data, "$GNGGA", 6)) {
            // Ensure parseGGA is public in GNSSModule (as discussed)
            bool ok = GNSSModule::parseGGA(m.u.nmea.data, fix);
            if (ok) {
                if (showFix) {
                    Serial.printf("GGA OK: FIX=%u SIV=%d HDOP=%d lat=%.6f lon=%.6f alt=%.2f\n",
                        (unsigned)fix.type, fix.SIV, fix.HDOP, fix.lat, fix.lon, fix.alt);
                }
            }
            else if (showGngga) {
                Serial.println("GGA parse fail");
            }
        }
        else if (showGngga) {
            Serial.print("NMEA: "); Serial.println(m.u.nmea.data);
        }

        // Optional periodic SIV “beep” message upstream
        if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
            radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
            lastSpeakMs = millis();
        }
        break;
    }

    case GNSSModule::MsgType::RTCM_FRAME: {
        // Normally GU just *ingests* RTCM from radio and forwards to GNSS,
        // but if you ever hook GNSS->RTCM locally, you can inspect here:
        uint16_t t = m.u.rtcm.type;
        Serial.printf("GU (local) RTCM observed: type=%u len=%u\n", t, (unsigned)m.u.rtcm.bytes);
        break;
    }

    case GNSSModule::MsgType::CMD_REPLY: {
        // Optional debug echo of GNSS command replies
        // Serial.printf("GNSS REPLY: %s\n", m.u.reply.line);
        break;
    }

    default: break;
    }
}

// Poll once from GNSS parsed queue
static inline void pumpGnssOnce() {
    GNSSModule::ParsedMsg m;
    if (gnss.readParsed(m)) handleParsed(m);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(200);
    Serial.printf("%s starting\n", APPNAME);

    // Start radio task
    if (!radioStartTask(RFM69_MISO, RFM69_MOSI, RFM69_SCK, RFM69_CSS, RFM69_IRQ,
        THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        haltUnit(APPNAME, "Radio task start failed");
    }
    Serial.println("Radio started");

    // GNSS
    gnss.begin(8192); // large RX ring
    
    delay(500);
    String response;
	gnss.sendWait("unlog", "response: OK", COMMANDDELAY, &response);
	Serial.println("Unlog:"  + response);

    const char* ports[] = { "com1", "com2", "com3" };
    char cmd[32];
    bool ok = false;

    for (int i = 0; i < 3; ++i) {
        snprintf(cmd, sizeof(cmd), "versiona %s\r\n", ports[i]);
        Serial.printf("Trying %s...\n", ports[i]);

        if (gnss.sendWait(cmd, "#VERSIONA", COMMANDDELAY)) {
            Serial.printf("✔️  GNSS replied on %s\n", ports[i]);
            ok = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!ok) {
        Serial.println("❌ No GNSS COM port responded.");
        radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, (uint8_t)ERR_GPS);
        haltUnit("GNSS", "no reply");
    }
    // Configure rover + NMEA
    gnss.sendWait("unlog\r\n", "response: OK", COMMANDDELAY);
    gnss.sendWait("config signalgroup 2\r\n", "response: OK", COMMANDDELAY);
    gnss.sendWait("config pvtalg multi\r\n", "response: OK", COMMANDDELAY);
    gnss.sendWait("config rtk timeout 60\r\n", "response: OK", COMMANDDELAY); // if supported
    gnss.sendWait("mode rover\r\n", "response: OK", COMMANDDELAY);
    gnss.sendWait("gpgga com2 1\r\n", "response: OK", COMMANDDELAY);         // 1 Hz GGA on com2
    gnss.sendWait("saveconfig\r\n", "response: OK", COMMANDDELAY);

    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_STARTING);

    Serial.println("RTKF3F Glider unit ready!");
    showMenu();
}

void loop() {
    readConsole();      // your menu/console
    pumpGnssOnce();     // handle incoming NMEA/others from GNSS

    // Handle radio inbound (RTCM fragments, arena settings, etc.)
    RxPacket pkt;
    if (radioReceive(pkt, 0) && pkt.len > 0) {
        switch (pkt.data[0]) {

        case MSG_RTCM: {
            if (pkt.len <= 1) { Serial.println("GU:MSG_RTCM too short"); break; }

            const uint8_t* rtcm = pkt.data + 1;     // << strip MSG_RTCM
            size_t         rlen = pkt.len - 1;

            Serial.print("Lead/len: ");
            Serial.printf("%02X %u  | first bytes:", rtcm[0], (unsigned)rlen);
            for (int i = 0; i < (int)min<size_t>(8, rlen); ++i) Serial.printf(" %02X", rtcm[i]);
            Serial.println();

            if (rtcm[0] != 0xD3) {
                Serial.printf("GU:MSG_RTCM bad lead 0x%02X (expected 0xD3)\n", rtcm[0]);
                break;
            }

            if (GNSSModule::isValidRTCM(rtcm, rlen)) {
                uint16_t type = GNSSModule::getRTCMType(rtcm, rlen);
                Serial.printf("RTCM inserted: type [%4u] %u bytes\n", type, (unsigned)rlen);
                SerialGNSS.write(rtcm, rlen);       // forward pure RTCM to GNSS UART
            }
            else {
                Serial.println(ANSI_RED "GU: Invalid full RTCM message" ANSI_RESET);
            }
            break;
        }

        case MSG_RTCM_FRAGMENT: {
            if (pkt.len <= 1) { Serial.println("GU:MSG_RTCM_FRAGMENT too short"); break; }

            // If your fragment payload is *just* raw RTCM chunk:
            reassembler.acceptFragment(pkt.data + 4, pkt.len - 4);  // Skip [type, idx, count, msgId]

            // If you actually send a fragment meta header (seq/idx/cnt/len),
            // replace the line above with: reassembler.acceptFragment(bytes, lengthOnlyOfBytes);

            if (reassembler.isComplete()) {
                const uint8_t* rtcm = reassembler.getData();
                size_t         rlen = reassembler.getLength();

                Serial.print("Lead/len: ");
                Serial.printf("%02X %u  | first bytes:", rtcm[0], (unsigned)rlen);
                for (int i = 0; i < (int)min<size_t>(8, rlen); ++i) Serial.printf(" %02X", rtcm[i]);
                Serial.println();

                if (rlen >= 6 && rtcm[0] == 0xD3 && GNSSModule::isValidRTCM(rtcm, rlen)) {
                    uint16_t type = GNSSModule::getRTCMType(rtcm, rlen);
                    Serial.printf("GU: Reassembled RTCM " ANSI_GREEN "[%4u]" ANSI_RESET " %u bytes\n",
                        type, (unsigned)rlen);
                    SerialGNSS.write(rtcm, rlen);
                }
                else {
                    Serial.println("GU:MSG_RTCM_FRAGMENT " ANSI_RED "Invalid" ANSI_RESET);
                }
                reassembler.reset();
            }
            break;
        }

        case MSG_ARENA_SETTINGS: {
            Serial.println("GU:MSG_ARENA_SETTINGS: Received Arena settings");
            arena.decodeArenaSettings(pkt.data);
            break;
        }

        case MSG_REQ_POS: {
            Serial.println("GU:MSG_REQ_POS: Received position request");
            // txRelPos(fix, true); // future: relative position response
            break;
        }

        default: {
            Serial.printf("GU: Unknown packet: 0x%02X, len=%u\n", pkt.data[0], pkt.len);
            for (uint8_t i = 0; i < pkt.len; i++) Serial.printf(" %02X", pkt.data[i]);
            Serial.println();
            break;
        }
        }
    }

    yield(); // Let other tasks run
}
