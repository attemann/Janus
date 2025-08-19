//RTKBase.ino
#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include <RTKF3F.h>
#include <RadioModule.h>
#include "LCD.h"


#define APPNAME "RTKBase 1.0"

#define THIS_NODE_ID NODEID_RTKBASE

//RADIO
inline constexpr int8_t  RFM69_IRQ = 4;
inline constexpr int8_t  RFM69_CSS = 5;
inline constexpr int8_t  RFM69_SCK = 18;
inline constexpr int8_t  RFM69_MISO= 19;
inline constexpr int8_t  RFM69_MOSI= 23;
inline constexpr int8_t  RFM69_RST = -1;

LiquidCrystal_I2C lcd(0x27, 16, 2);
LCDManager screen(lcd);

ConfigManager config(APPNAME);  // NVS namespace

RadioModule radioMod(RFM69_CSS, RFM69_IRQ, true);

// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

GNSSModule::GNSSFix fix;
GNSSModule::GNSSFix oldFix;

int rtcmCount = 0; // Count of RTCM types sent

int timeSurveyStart = 0;
bool showFix = false;
bool showGngga = false;

unsigned int timeLastSpeak = 0; // Last time we spoke a message 
#define DELAYBETWEENSPEAK    10000 // seconds delay between spoken messages

enum DEVICESTATE {
    DEVICE_STARTING,
    DEVICE_GETTINGFIX,
    DEVICE_SURVEYING,
    DEVICE_OPERATING,
    DEVICE_MENU
};
DEVICESTATE deviceState = DEVICESTATE::DEVICE_STARTING;

void haltUnit(String msg1, String msg2) {
    Serial.print(msg1);
    Serial.print(":");
    Serial.println(msg2);
    screen.setLine(0, msg1);
    screen.setLine(1, msg2);
    delay(1000);
    //while (true);
}

// Called from loop() when state == Operating
void updateRTCMTypeCountDisplay() {
    static unsigned long lastSwitchTime = 0;
    static uint16_t currentRTCMId = 0;
    static uint32_t currentRTCMCount = 0;
    static bool initialized = false;

    // Only update every 5 seconds (5000 ms)
    if (!initialized || millis() - lastSwitchTime >= 3000) {
        // Get next enabled RTCM type and count
        gnss.rtcmHandler.getNextRTCMCount(&currentRTCMId, &currentRTCMCount);

        // Update LCD or display
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.printf("RTCM %u", currentRTCMId);
        lcd.setCursor(0, 1);
        lcd.printf("Sent: %lu", currentRTCMCount);

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

    // User interface
    screen.begin();
    screen.setLine(0, APPNAME);
    screen.setLine(1, "");

    // Radio
    if (!radioMod.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK, THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        Serial.println("Radio init failed");
        while (1);
    }
    Serial.printf("%s: Radio init ok\r\n", APPNAME);
    delay(1000);
    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_DEVICE_STARTING);
    delay(1000);

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);

	int port = gnss.detectUARTPort();
    if (port == 0) {
        delay(1000);
        radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, ERROR_UART);
        haltUnit("Gnss port", "Failure, freeze");
	} else Serial.printf("%s: Gnss found, COM%d\r\n", APPNAME, port);

    deviceState = DEVICESTATE::DEVICE_STARTING;

}
void loop() {
    #ifdef WIFI
        config.loop();
    #endif

    switch (deviceState) {
    case DEVICESTATE::DEVICE_STARTING: {

        gnss.sendCommand("unlog");
		gnss.sendCommand("mode rover");
        gnss.sendCommand("gpgga com2 1");
        gnss.sendCommand("saveconfig");

        deviceState = DEVICESTATE::DEVICE_GETTINGFIX;
        delay(1000);
		radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_TRANSITION_GETTINGFIX);
        delay(1000);
        break;
    }
    case DEVICESTATE::DEVICE_GETTINGFIX: {

        GNSSModule::GNSSMessage gnssData = gnss.readGPS();
        if (gnssData.type == GNSSModule::GNSSMessageType::NMEA) {

            GNSSModule::GNSSFix fix;

            if (gnssData.length > 6 && memcmp(gnssData.data, "$GNGGA", 6) == 0) {
                if (gnss.parseGGA(gnssData.data, gnssData.length, fix)) {
                    screen.setLine(0, "GGA OK");
                    screen.setLine(1, "SIV=" + String(fix.SIV));
                }
                else {
                    screen.setLine(0, "GGA Parse fail");
                }
            }
            else {
                screen.setLine(0, "Waiting GGA...");
                screen.setLine(1, String((const char*)gnssData.data));
            }

            gnss.showFix(fix);

            if (fix.fixType > 0) {
                if (fix.fixType == FIX_TYPE_GPS) {
                    delay(1000);
                    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_TRANSITION_SURVEYING);
					delay(1000);
                    timeSurveyStart = millis();
                    gnss.sendCommand("unlog");
                    gnss.sendCommand("mode base time 15");
                    gnss.sendCommand("saveconfig");
                    deviceState = DEVICESTATE::DEVICE_SURVEYING;
                }
            }
            if (millis() > (timeLastSpeak + DELAYBETWEENSPEAK)) {
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
				timeLastSpeak = millis();
            }
        }
        break;
    }

    case DEVICESTATE::DEVICE_SURVEYING: {
        unsigned long elapsed = millis() - timeSurveyStart;
        if (elapsed < SURVEYINTIME) {
            long remaining = (SURVEYINTIME - elapsed) / 1000;
            String timeLeftStr = "Time left: " + String(remaining, 0) + "s";
			Serial.println("BASE_SURVEYING: Remaining time: " + timeLeftStr);
            screen.setLine(0, "Survey-in");
            screen.setLine(1, timeLeftStr);

            if (millis() > (timeLastSpeak + DELAYBETWEENSPEAK)) {
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SURVEYING, static_cast<uint8_t>(remaining));
                timeLastSpeak = millis();
            }
        } else {
            screen.setLine(0, "Configuring GPS");
            screen.setLine(1, "RTK parameters");
            Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");
            gnss.rtcmHandler.printList(true);
            gnss.sendCommand("unlog");
            gnss.sendCommand("config signalgroup 2");
            gnss.sendCommand("config pvtalg multi");
            gnss.rtcmHandler.sendAllConfig(); // Enable RTCM messages
            gnss.sendCommand("saveconfig");

            //radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_CD, MSG_INFORMATION, INFO_TRANSITION_OPERATING);
            delay(500);

            deviceState = DEVICESTATE::DEVICE_OPERATING;
        }
        break;
    }

    case DEVICESTATE::DEVICE_OPERATING: {

        GNSSModule::GNSSMessage gnssData = gnss.readGPS();

        if (gnssData.type == GNSSModule::GNSSMessageType::RTCM) {
			Serial.println("RTCM: Received data");

            if (gnss.isValidRTCM(gnssData.data, gnssData.length)) {
				Serial.println("RTCM: Valid data received");

                radioMod.sendRTCM(gnssData.data, gnssData.length);

                uint16_t rtcmtype = gnss.getRTCMType(gnssData.data, gnssData.length);
				Serial.println("RTCM: Type = " + String(rtcmtype));
                gnss.rtcmHandler.incrementSentCount(rtcmtype);

                updateRTCMTypeCountDisplay();

                bool isFragmented = gnssData.length > RadioModule::RTCM_Fragmenter::MAX_PAYLOAD;
                uint16_t count = gnss.rtcmHandler.messages[gnss.rtcmHandler.findById(rtcmtype)].txCount;
                Serial.printf("Sending [" ANSI_GREEN "RTCM%d" ANSI_RESET "] %s" ANSI_GREEN " Count: %d" ANSI_RESET " Len: %d\r\n",
                    rtcmtype,
                    isFragmented ? "Fragments" : "Single",
                    count, gnssData.length);

            }
            else {
                Serial.println("RTCM: Invalid data received");
            }
		}
		else Serial.println("RTCM: Not RTCM data");
        break;
    }
    default:
        break;
    }

    if (fix.fixType != oldFix.fixType) {
		switch (fix.fixType) {
            case FIX_TYPE_NOFIX: // No fix
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_NOFIX);
			    break;
            case FIX_TYPE_GPS: // GPS fix
				radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_GPS);
                break;
			case FIX_TYPE_DGPS: // DGPS fix     
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_DGPS);
				break;
            case FIX_TYPE_PPS: // PPS fix
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_PPS);
				break;
            case FIX_TYPE_RTK_FLOAT: // RTK float
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_RTK_FLOAT);
                break;
            case FIX_TYPE_RTK_FIX: // RTK fixed
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_RTK_FIX);
                break;
            case FIX_TYPE_DEAD_RECKONING:   // Dead reckoning
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_DEAD_RECKONING);
				break;
            case FIX_TYPE_MANUAL: // Manual input mode
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_MANUAL);
				break;
            case FIX_TYPE_SIM: // Simulation mode
				radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_SIM);   
                break;
            default: // Other fix types
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_FIX_OTHER);
                break;
        }
		oldFix = fix;

	}
    delay(100);
}
