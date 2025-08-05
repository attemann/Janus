//RTKBase.ino
#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include "RTKF3F.h"
#include "LCD.h"
#include "RTKBase.h"

#define APPNAME "RTKBase 1.0"

//RADIO
#define RFM69_IRQ       4
#define RFM69_CS        5
#define RFM69_SCK      18
#define RFM69_MISO     19
#define RFM69_MOSI     23
#define RFM69_RST      -1

RadioModule::HWPins radioPins = {
    .sck   = RFM69_SCK,
    .miso  = RFM69_MISO,
    .mosi  = RFM69_MOSI,
    .cs    = RFM69_CS,
    .irq   = RFM69_IRQ,
    .reset = RFM69_RST
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
LCDManager screen(lcd);

//TwoButtonMenu menu(lcd);

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);

// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

GNSSModule::GNSSFix fix;
int oldFixType = 0; // Store the last fix type to detect changes

int timeSurveyStart = 0;
bool showFix = false;
bool showGngga = false;

unsigned int timeLastSpeak = 0; // Last time we spoke a message 
#define DELAYBETWEENSPEAK    5000 // 5 seconds delay between spoken messages

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
    while (true);
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
    if (!radioMod.init(radioPins, NODEID_RTKBASE, NETWORK_ID, RTCM_TX_FREQ)) {
		radioMod.sendMessageCode(0,MSG_ERROR, ERROR_RADIO_INIT);   
        haltUnit("Radio init", "Failure, freeze");
    } else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        radioMod.sendMessageCode(0, MSG_ERROR, ERROR_RADIO_VERIFY);
        haltUnit("Radio verify", "Failure, freeze");
    } else Serial.println("Radio verified");

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);

    if (gnss.detectUARTPort() == 0) {
        radioMod.sendMessageCode(0, MSG_ERROR, ERROR_UART);
        haltUnit("Gnss port", "Failure, freeze");
	} else Serial.println("Gnss port ok");

    gnss.setDefaultRTCMs();
    gnss.printRTCMConfig();

    deviceState = DEVICESTATE::DEVICE_STARTING;
}

void loop() {

    readConsole();

    switch (deviceState) {
    case DEVICESTATE::DEVICE_STARTING: {

        gnss.sendCommand("unlog\r\n");
        gnss.sendCommand("gpgga com2 1\r\n");
        gnss.sendCommand("saveconfig\r\n");

        deviceState = DEVICESTATE::DEVICE_GETTINGFIX;
		radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_TRANSITION_GETTINGFIX);
        break;
    }
    case DEVICESTATE::DEVICE_GETTINGFIX: {

        String line = SerialGNSS.readStringUntil('\n');
        line.trim();

        if (!line.startsWith("$GNGGA")) {
            Serial.print("Awaiting $GNGGA");
			Serial.println(line);   
            screen.setLine(0, "Awaiting $GNGGA");
            screen.setLine(1, line);
        } else {

			gnss.parseGGA(line.c_str(), fix);

            screen.setLine(0, String(fix.fixType) + ":" + gnss.fixTypeToString(fix.fixType));
            screen.setLine(1, "SIV: " + String(fix.SIV));

            if (millis() - timeLastSpeak > DELAYBETWEENSPEAK) {
				radioMod.sendMessageCode(0, MSG_SIV, fix.SIV);
                timeLastSpeak = millis();
			}

            // For debug
            Serial.println(line);
            gnss.showFix(fix);

            if (fix.fixType == 1 || fix.fixType==7) {
                timeSurveyStart = millis();
				gnss.sendCommand("unlog\r\n");
                gnss.sendCommand("mode base time " + String(SURVEYINTIME) + " com2\r\n");
                gnss.sendCommand("saveconfig\r\n");

                deviceState = DEVICESTATE::DEVICE_SURVEYING;
				radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_TRANSITION_SURVEYING);
            }
        }
        break;
    }
    case DEVICESTATE::DEVICE_SURVEYING: {
        unsigned long elapsed = millis() - timeSurveyStart;
        if (elapsed < (SURVEYINTIME + 1) * 1000) {
            float err = 99.99;
            screen.setLine(0, "HDOP " + String(err, 2) + "m");
            screen.setLine(1, "Time left: " + String(((SURVEYINTIME * 1000) - elapsed) / 1000) + "s");
            break;
        }
        Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");
        gnss.setDefaultRTCMs();

        gnss.sendCommand("unlog\r\n"); 
        gnss.sendConfiguredRTCMs();
        gnss.sendCommand("saveconfig\r\n");
        Serial.println("BASE_SURVEYING: RTCM config done, entering OPERATING mode.");

        deviceState = DEVICESTATE::DEVICE_OPERATING;
		radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_TRANSITION_OPERATING);
        break;
    }
    case DEVICESTATE::DEVICE_OPERATING: {
        uint8_t rtcmBuf[1024];
        size_t len = 0;

        screen.setLine(0, "Operating");
        screen.setLine(1, String(radioMod.getRTCMNumMessages()));

        if (gnss.readRTCM(rtcmBuf, len)) {
            if (len > 0) {
                uint16_t type = gnss.getRTCMBits(rtcmBuf, 24, 12);
                Serial.printf("%4zu bytes RTCM%4u\n", len, type);
                radioMod.sendRTCM(rtcmBuf, len);
                delay(20);
				radioMod.sendRTCMNumMessages();  // Send traffic summary message
            }
        }
        break;
    }
    default:
        break;
    }

    if (fix.fixType != oldFixType) {
		switch (fix.fixType) {
            case 0: // No fix
                radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_FIX_NOFIX);
			    break;
            case 1: // GPS fix
				radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_FIX_GPS);
                break;
			case 2: // DGPS fix     
                radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_FIX_DGPS);
				break;
            case 3: // RTK float
                radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_FIX_RTK_FLOAT);
                break;
            case 4: // RTK fixed
                radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_FIX_RTK_FIX);
                break;
            default: // Other fix types
                radioMod.sendMessageCode(0, MSG_INFORMATION, INFO_FIX_OTHER);
                break;
        }
		oldFixType = fix.fixType;

	}
    delay(10);
}
