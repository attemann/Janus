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

//TwoButtonMenu menu(lcd);

RadioModule radioMod(RFM69_CSS, RFM69_IRQ, true);

// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

GNSSModule::GNSSFix fix;
int oldFixType = 0; // Store the last fix type to detect changes

int rtcmCount = 0; // Count of RTCM types sent

int timeSurveyStart = 0;
bool showFix = false;
bool showGngga = false;

unsigned int timeLastSpeak = 0; // Last time we spoke a message 
#define DELAYBETWEENSPEAK    8000 // 8 seconds delay between spoken messages

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
    while (true);
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

    // User interface
    screen.begin();
    screen.setLine(0, APPNAME);
    screen.setLine(1, "");

    // Radio
    // Radio
    if (!radioMod.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK, THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        Serial.println("Radio init failed");
        while (1);
    }
    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_DEVICE_STARTING);
    delay(1000);

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);

    if (gnss.detectUARTPort() == 0) {
        delay(1000);
        radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, ERROR_UART);
        haltUnit("Gnss port", "Failure, freeze");
	} else Serial.println("Gnss port ok");

    deviceState = DEVICESTATE::DEVICE_STARTING;

}

void loop() {

    switch (deviceState) {
    case DEVICESTATE::DEVICE_STARTING: {

        gnss.sendCommand("unlog\r\n");
		gnss.sendCommand("mode rover\r\n");
        gnss.sendCommand("gpgga com2 1\r\n");
        gnss.sendCommand("saveconfig\r\n");

        deviceState = DEVICESTATE::DEVICE_GETTINGFIX;
		radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_TRANSITION_GETTINGFIX);
        delay(1000);
        //while (1);
        break;
    }
    case DEVICESTATE::DEVICE_GETTINGFIX: {

        String line = SerialGNSS.readStringUntil('\n');
        line.trim();

        if (!line.startsWith("$GNGGA")) {
            //Serial.print("Awaiting $GNGGA");
			//Serial.println(line);   
            screen.setLine(0, "Awaiting $GNGGA");
            screen.setLine(1, line);
        } else {

			//gnss.parseGGA(line.c_str(), fix);

            screen.setLine(0, String(fix.fixType) + ":" + gnss.fixTypeToString(fix.fixType));
            screen.setLine(1, "SIV: " + String(fix.SIV));

            if (millis() - timeLastSpeak > DELAYBETWEENSPEAK) {
				radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
                timeLastSpeak = millis();
			}

            //Serial.println(line);
            gnss.showFix(fix);

            if (fix.fixType == FIX_TYPE_GPS) {
                delay(1000);
                radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_TRANSITION_SURVEYING);
                timeSurveyStart = millis();
				gnss.sendCommand("unlog\r\n");
                gnss.sendCommand("mode base time " + String(SURVEYINTIME/1000, 0) + " com2\r\n");
                gnss.sendCommand("saveconfig\r\n");

				timeSurveyStart = millis();
                deviceState = DEVICESTATE::DEVICE_SURVEYING;
            }
        }
        break;
    }
    case DEVICESTATE::DEVICE_SURVEYING: {
        unsigned long elapsed = millis() - timeSurveyStart;
        if (elapsed < SURVEYINTIME) {
            long remaining = (SURVEYINTIME - elapsed) / 1000;
            String timeLeftStr = "Time left: " + String(remaining, 0) + "s";
            screen.setLine(0, "Survey-in");
            screen.setLine(1, timeLeftStr);
        } else {
            screen.setLine(0, "Configuring GPS");
            screen.setLine(1, "RTK parameters");
            Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");
            gnss.sendCommand("unlog\r\n");
            gnss.rtcmHandler.sendAllConfig(); // Enable RTCM messages
            gnss.sendCommand("config signalgroup 2\r\n");
            gnss.sendCommand("config pvtalg multi\r\n");
            gnss.sendCommand("saveconfig\r\n");
            gnss.rtcmHandler.printList(true);
            //Serial.println("BASE_SURVEYING: RTCM config done, entering OPERATING mode.");

            radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_CD, MSG_INFORMATION, INFO_TRANSITION_OPERATING);
            delay(500);

            deviceState = DEVICESTATE::DEVICE_OPERATING;
        }
        break;
    }
    case DEVICESTATE::DEVICE_OPERATING: {
        uint8_t rtcmBuf[1024];
        size_t len = 0;

        //if (gnss.readGPS(rtcmBuf, len)) {
        if (gnss.readGPS()) {
            if (len > 0) {

				uint16_t type = gnss.getRTCMType(rtcmBuf, len);
				bool isValid = gnss.isValidRTCM(rtcmBuf, len);
				bool isFragmented = len > RadioModule::RTCM_Fragmenter::MAX_PAYLOAD;

				uint16_t count = gnss.rtcmHandler.messages[gnss.rtcmHandler.findById(type)].txCount;
				Serial.printf("Sending [" ANSI_GREEN "RTCM%d" ANSI_RESET "] %s %s" ANSI_GREEN " Count: %d" ANSI_RESET " Len: %d\r\n",
                    type,
                    isValid ? "Valid" : "Invalid",
                    isFragmented ? "Fragments" : "Single",
                    count,len);

                gnss.rtcmHandler.incrementSentCount(type);

                radioMod.sendRTCM(rtcmBuf, len);
            }
        }
        updateRTCMTypeCountDisplay();
        break;
    }
    default:
        break;
    }

    if (fix.fixType != oldFixType) {
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
		oldFixType = fix.fixType;

	}
    delay(10);
}
