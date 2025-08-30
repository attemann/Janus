//RTKBase.ino
// RTKBase.ino
#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>

#include <RTKF3F.h>
#include <DisplayTask.h>
#include <RadioTask.h>
#include "src/RTKBase.h"

#define APPNAME "RTKBase 1.0"
#define THIS_NODE_ID NODEID_RTKBASE

// RADIO
extern RadioModule* radioMod;
inline constexpr int8_t  RFM69_IRQ = 4;
inline constexpr int8_t  RFM69_CSS = 5;
inline constexpr int8_t  RFM69_SCK = 18;
inline constexpr int8_t  RFM69_MISO = 19;
inline constexpr int8_t  RFM69_MOSI = 23;
inline constexpr int8_t  RFM69_RST = -1;

ConfigManager config(APPNAME);  // NVS namespace

// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS, UART_RX, UART_TX, GNSS_BAUD);

GNSSModule::GNSSFix fix, oldFix;

int rtcmCount = 0; // Count of RTCM types sent
bool showFix = false;
bool showGngga = false;

unsigned int lastSpeakMs = 0;       // Last time we spoke a message
#define SPEAK_INTERVAL_MS 10000     // seconds delay between spoken messages

DeviceState deviceState = DeviceState::DEVICE_STARTING;

// ------------------------------
// Centralized GNSS message handling
// ------------------------------
static inline void handleParsed(const GNSSModule::ParsedMsg& m) {
    switch (m.type) {
    case GNSSModule::MsgType::NMEA_LINE: {
        if (m.u.nmea.len > 6 && !strncmp(m.u.nmea.data, "$GNGGA", 6)) {
            bool ok = GNSSModule::parseGGA(m.u.nmea.data, fix);
            if (ok) {
                // optional UI
            }
            else {
                // optional UI
            }
        }
        else {
            // optional UI
        }

        // periodic SIV beeps
        if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
            radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
            lastSpeakMs = millis();
        }
        break;
    }

    case GNSSModule::MsgType::RTCM_FRAME: {
        uint16_t rtcmtype = m.u.rtcm.type;   // <- use this
        bool isFragmented = m.u.rtcm.bytes > RadioModule::RTCM_Fragmenter::MAX_PAYLOAD;
        GDBG_PRINTF("Sending [RTCM%u] %s Len: %u\r\n",
            rtcmtype, isFragmented ? "Fragments" : "Single", (unsigned)m.u.rtcm.bytes);
        radioTxRtcmWrite(m.u.rtcm.buf, m.u.rtcm.bytes);
        break;
    }

    case GNSSModule::MsgType::CMD_REPLY: {
        // Optional logging
        GDBG_PRINTF("GNSS REPLY: %s\n", m.u.reply.line);
        break;
    }

    default: break;
    }
}

// Poll once; call this from each state branch
static inline void pumpGnssOnce() {
    GNSSModule::ParsedMsg m;
    if (gnss.readParsed(m)) handleParsed(m);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    Serial.printf("%s booting\r\n", APPNAME);

#ifdef WIFI
    if (!config.begin()) {
        Serial.println("❌ Kunne ikke starte NVS-lagring");
        return;
    }
    AdminOpts opts;
    opts.ssid = "";             // lar den generere f.eks. "JanusAB12"
    opts.password = "";         // ⚠️ ingen passord => åpent nett
    opts.windowMs = 120000;     // varighet: 2 minutter
    config.startAdminWindow(opts);
    Serial.println("🟢 Admin-modus aktivert, åpen WiFi");
#endif

    startDisplayTask();
    sendToDisplay(APPNAME, "Starting");

    if (!radioStartTask(RFM69_MISO, RFM69_MOSI, RFM69_SCK, RFM69_CSS, RFM69_IRQ,
        THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        sendToDisplay("Radio task", "start failed");
        while (true);  // halt
    }

    // GNSS
    gnss.begin(8192); // big RX ring
    gnss.send("unlog");
    bool ok = gnss.sendWait("versiona com2", "#VERSIONA", COMMANDDELAY);
    Serial.println(ok ? "UM980 OK" : "UM980 no reply");

    deviceState = DeviceState::DEVICE_STARTING;
    delay(200);
    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
        static_cast<uint8_t>(DEVICE_STARTING));
    delay(1000);
}

void loop() {
#ifdef WIFI
    config.loop();
#endif

    switch (deviceState) {
    case DeviceState::DEVICE_STARTING: {
        sendToDisplay("Starting", "Getting fix");

        gnss.sendWait("unlog", "response: OK", COMMANDDELAY);
        gnss.sendWait("mode rover", "response: OK", COMMANDDELAY);
        gnss.sendWait("gpgga com2 1", "response: OK", COMMANDDELAY);
        gnss.sendWait("saveconfig", "response: OK", COMMANDDELAY);

        deviceState = DeviceState::DEVICE_GETTINGFIX;
        radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
            static_cast<uint8_t>(DeviceState::DEVICE_GETTINGFIX));
        delay(1000);

        pumpGnssOnce(); // drain anything pending
        break;
    }

    case DeviceState::DEVICE_GETTINGFIX: {
        sendToDisplay("Getting fix", "Waiting for data");

        // Promote to SURVEYING when we have a basic GPS fix
        if (fix.type == FIXTYPE::GPS) {
            delay(1000);
            radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
                static_cast<uint8_t>(DeviceState::DEVICE_SURVEYING));
            //delay(1000);
            gnss.sendWait("unlog", "response: OK", COMMANDDELAY);
            gnss.sendWait("mode base time 15", "response: OK", COMMANDDELAY);
            gnss.sendWait("saveconfig", "response: OK", COMMANDDELAY);
            deviceState = DeviceState::DEVICE_SURVEYING;
        }

        pumpGnssOnce();
        break;
    }

    case DeviceState::DEVICE_SURVEYING: {
        static uint32_t surveyStartMs = millis();
        const uint32_t elapsed = millis() - surveyStartMs;

        if (elapsed < SURVEYINTIME) {
            uint32_t remaining = (SURVEYINTIME - elapsed) / 1000;
            sendToDisplay("Surveying", "Time left: " + String(remaining) + "s");

            if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
                Serial.printf("BASE_SURVEYING: Remaining: %lus\n", (unsigned long)remaining);
                uint8_t speakLeft = (remaining > 255) ? 255 : (uint8_t)remaining;
                radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SURVEY, speakLeft);
                lastSpeakMs = millis();
            }
        }
        else {
            sendToDisplay("Configuring GPS", "RTK parameters");
            Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");

            /*
                    snprintf(cmd, sizeof(cmd), "rtcm1006 %s 10", portName); send(cmd);
        snprintf(cmd, sizeof(cmd), "rtcm1033 %s 10", portName); send(cmd);
        snprintf(cmd, sizeof(cmd), "rtcm1074 %s 1", portName); send(cmd);
        snprintf(cmd, sizeof(cmd), "rtcm1084 %s 1", portName); send(cmd);
        snprintf(cmd, sizeof(cmd), "rtcm1094 %s 1", portName); send(cmd);
        snprintf(cmd, sizeof(cmd), "rtcm1124 %s 1", portName); send(cmd);*/

            gnss.sendWait("unlog", "response: OK", COMMANDDELAY);
            gnss.sendWait("saveconfig", "response: OK", COMMANDDELAY);
            gnss.sendWait("rtcm1006 com2 10", "response: OK", COMMANDDELAY);
            gnss.sendWait("rtcm1033 com2 10", "response: OK", COMMANDDELAY);
            gnss.sendWait("rtcm1074 com2 1", "response: OK", COMMANDDELAY);
            gnss.sendWait("rtcm1084 com2 1", "response: OK", COMMANDDELAY);
            gnss.sendWait("rtcm1094 com2 1", "response: OK", COMMANDDELAY);
            gnss.sendWait("rtcm1124 com2 1", "response: OK", COMMANDDELAY);

            radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
                static_cast<uint8_t>(DeviceState::DEVICE_OPERATING));

            sendToDisplay(APPNAME, "Operating");
            deviceState = DeviceState::DEVICE_OPERATING;
        }

        pumpGnssOnce();
        break;
    }

    case DeviceState::DEVICE_OPERATING: {
        pumpGnssOnce();
        break;
    }

    default: break;
    }

    if (fix.type != oldFix.type) {
        radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE,
            static_cast<uint8_t>(fix.type));
        oldFix = fix;
    }

    delay(100);
}
