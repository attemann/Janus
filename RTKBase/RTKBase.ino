//RTKBase.ino
#include <Arduino.h>
#include <RFM69.h>

#include <RTKF3F.h>
#include <DisplayTask.h>
#include <RadioTask.h>
#include "src/RTKBase.h"

#define APPNAME "RTKBase 1.0"
#define THIS_NODE_ID NODEID_RTKBASE

//RADIO
extern RadioModule* radioMod;
inline constexpr int8_t  RFM69_IRQ = 4;
inline constexpr int8_t  RFM69_CSS = 5;
inline constexpr int8_t  RFM69_SCK = 18;
inline constexpr int8_t  RFM69_MISO= 19;
inline constexpr int8_t  RFM69_MOSI= 23;
inline constexpr int8_t  RFM69_RST = -1;

ConfigManager config(APPNAME);  // NVS namespace


// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

GNSSModule::GNSSFix fix;
GNSSModule::GNSSFix oldFix;

int rtcmCount = 0; // Count of RTCM types sent

bool showFix = false;
bool showGngga = false;

unsigned int lastSpeakMs = 0; // Last time we spoke a message 
#define SPEAK_INTERVAL_MS    10000 // seconds delay between spoken messages


DeviceState deviceState = DeviceState::DEVICE_STARTING;

void updateRTCMTypeCountDisplay() {
    static unsigned long lastSwitchTime = 0;
    static uint16_t currentRTCMId = 0;
    static uint32_t currentRTCMCount = 0;
    static bool initialized = false;

    // Only update every 5 seconds (5000 ms)
    if (!initialized || millis() - lastSwitchTime >= 3000) {
        // Get next enabled RTCM type and count
        gnss.rtcmHandler.getNextRTCMCount(&currentRTCMId, &currentRTCMCount);

        sendToDisplay("RTCM" + String(currentRTCMId),
                      "Sent " + String(currentRTCMCount));

        lastSwitchTime = millis();
        initialized = true;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    Serial.printf("%s booting\r\n", APPNAME);

    #ifdef WIFI         // Start NVS

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

    sendToDisplay(APPNAME, "Starting");

    if (!radioStartTask(
        RFM69_MISO, RFM69_MOSI, RFM69_SCK,RFM69_CSS, RFM69_IRQ,
        THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        sendToDisplay("Radio task", "start failed");
        while (true);  // halt
    }

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);

	int port = gnss.detectUARTPort();
    if (port == 0) {
        delay(500);
        radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, ERR_UART);
        sendToDisplay(APPNAME, "Gnss port fail");
	} else Serial.printf("%s: Gnss found, COM%d\r\n", APPNAME, port);

    deviceState = DeviceState::DEVICE_STARTING;
    delay(200);
    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_STARTING);
    delay(1000);

}
void loop() {
    #ifdef WIFI
        config.loop();
    #endif

    switch (deviceState) {
    case DeviceState::DEVICE_STARTING: {

        gnss.sendCommand("unlog");
		gnss.sendCommand("mode rover");
        gnss.sendCommand("gpgga com2 1");
        gnss.sendCommand("saveconfig");

        deviceState = DeviceState::DEVICE_GETTINGFIX;
        delay(1000);
        radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_GETTINGFIX);
        delay(1000);
        break;
    }
    case DeviceState::DEVICE_GETTINGFIX: {

        GNSSModule::GNSSMessage gnssData = gnss.readGPS();
        if (gnssData.type == GNSSModule::GNSSMessageType::NMEA) {

            GNSSModule::GNSSFix fix;

            if (gnssData.length > 6 && memcmp(gnssData.data, "$GNGGA", 6) == 0) {
                if (gnss.parseGGA(gnssData.data, gnssData.length, fix)) {
                    sendToDisplay("GGA OK", "SIV=" + String(fix.SIV));
                }
                else {
                    sendToDisplay("GGA", "Parse fail");
                }
            }
            else {
                sendToDisplay("Awaiting $GNGGA", String((const char*)gnssData.data));
            }

            gnss.showFix(fix);

            if (fix.fixType > 0) {
                if (fix.fixType == FIXTYPE_GPS) {
                    delay(1000);
                    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_SURVEYING);
					delay(1000);
                    gnss.sendCommand("unlog");
                    gnss.sendCommand("mode base time 15");
                    gnss.sendCommand("saveconfig");
                    deviceState = DeviceState::DEVICE_SURVEYING;
                }
            }
            if (millis() > (lastSpeakMs + SPEAK_INTERVAL_MS)) {
                radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
                lastSpeakMs = millis();
            }
        }
        break;
    }

    case DeviceState::DEVICE_SURVEYING: {
        static uint32_t surveyStartMs = millis();
        const uint32_t elapsed = millis() - surveyStartMs;

        if (elapsed < SURVEYINTIME) {
            uint32_t remaining = (SURVEYINTIME - elapsed) / 1000;
			sendToDisplay("Surveying", "Time left: " + String(remaining) + "s");
            Serial.printf("BASE_SURVEYING: Remaining: %lus\n", (unsigned long)remaining);

            if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
                uint8_t speakLeft = (remaining > 255) ? 255 : (uint8_t)remaining;
                radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SURVEY, speakLeft);
                lastSpeakMs = millis();
            }
        }
        else {
			sendToDisplay("Configuring GPS", "RTK parameters");
            Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");

            gnss.rtcmHandler.printList(true);
            gnss.sendCommand("unlog");
            gnss.sendCommand("config signalgroup 2");
            gnss.sendCommand("config pvtalg multi");
            gnss.rtcmHandler.sendAllConfig();  // enable RTCM outputs
            gnss.sendCommand("saveconfig");

            radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_OPERATING);

            deviceState = DeviceState::DEVICE_OPERATING;
        }
        break;
    }


    case DeviceState::DEVICE_OPERATING: {

        GNSSModule::GNSSMessage gnssData = gnss.readGPS();

        if (gnssData.type == GNSSModule::GNSSMessageType::RTCM) {
            if (gnss.isValidRTCM(gnssData.data, gnssData.length)) {
                uint16_t rtcmtype = gnss.getRTCMType(gnssData.data, gnssData.length);
                bool isFragmented = gnssData.length > RadioModule::RTCM_Fragmenter::MAX_PAYLOAD;
                uint16_t count = gnss.rtcmHandler.messages[gnss.rtcmHandler.findById(rtcmtype)].txCount;

                Serial.printf("Sending [" ANSI_GREEN "RTCM%d" ANSI_RESET "] %s" ANSI_GREEN " Count: %d" ANSI_RESET " Len: %d\r\n",
                    rtcmtype,
                    isFragmented ? "Fragments" : "Single",
                    count, gnssData.length);

                gnss.rtcmHandler.incrementSentCount(rtcmtype);
                updateRTCMTypeCountDisplay();

                radioSendRTCM(gnssData.data, gnssData.length);
            }
            else Serial.println(ANSI_RED "RTCM: Invalid data received" ANSI_RESET);
   		}
        break;
    }
    default:
        break;
    }

    if (fix.fixType != oldFix.fixType) {
		switch (fix.fixType) {
            case FIXTYPE_NOFIX:     radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_NOFIX); break;
            case FIXTYPE_GPS:       radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_GPS); break;
			case FIXTYPE_DGPS:      radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_DGPS);break;
            case FIXTYPE_PPS:       radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_PPS);break;
            case FIXTYPE_RTK_FLOAT: radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_RTK_FLOAT); break;
            case FIXTYPE_RTK_FIX:   radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_RTK_FIX); break;
            case FIXTYPE_DEAD_RECK: radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_DEAD_RECK);break;
            case FIXTYPE_MANUAL:    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_MANUAL);break;
            case FIXTYPE_SIM:       radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_SIM);  break;
            default:                radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_FIXTYPE, FIXTYPE_OTHER); break;
        }
		oldFix = fix;

	}
    delay(100);
}
