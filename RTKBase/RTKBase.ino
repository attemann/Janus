//RTKBase.ino
#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include "RTKF3F.h"
#include "LCD.h"
#include "TwoButtonMenu.h"

#define APPNAME "RTKBase 1.0"



// BUTTONS
#define BTN_MENU       12
#define BTN_SELECT     13
#define HOLD_TIME      800

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

TwoButtonMenu menu(lcd);

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);

// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);
GNSSModule::GNSSFix fix;

bool menuActive = true;
unsigned long menuPressStart = 0;
bool menuHeld = false;
int timeSurveyStart = 0;

enum BASESTATE {
    BASE_STARTING,
    BASE_GETTINGFIX,
    BASE_SURVEYING,
    BASE_OPERATING,
    BASE_MENU
};
BASESTATE baseState = BASESTATE::BASE_STARTING;

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
    Serial.println("Base: Booting");

    // User interface
    screen.begin();
    screen.setLine(0, APPNAME);
    screen.setLine(1, "");

    // Radio
    if (!radioMod.init(radioPins, NODEID_RTKBASE, NETWORK_ID, RTCM_TX_FREQ)) {
        haltUnit("Radio init", "Failure, freeze");
    } else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        haltUnit("Radio verify", "Failure, freeze");
    } else Serial.println("Radio verified");

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
    if (!gnss.init()) {
        haltUnit("Gnss init", "Failure, freeze");
    }  else Serial.println("Gnss init ok");

    if (gnss.detectUARTPort() == 0) {
        haltUnit("Gnss port", "Failure, freeze");
	} else Serial.println("Gnss port ok");

    pinMode(BTN_MENU, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);

    menu.addItem("Set Survey T", { "30s", "60s", "120s" });
    menu.begin();
    Serial.println("Base: Menu init ok");

    baseState = BASESTATE::BASE_STARTING;
}

void loop() {

    switch (baseState) {
    case BASESTATE::BASE_STARTING: {
        baseState = BASESTATE::BASE_GETTINGFIX;
        break;
    }
    case BASESTATE::BASE_GETTINGFIX: {

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

            // For debug
            Serial.println(line);
            gnss.showFix(fix);

            if (fix.fixType == 1 || fix.fixType==7) {
                timeSurveyStart = millis();
				gnss.sendCommand("unlog\r\n");
                gnss.sendCommand("mode base time " + String(SURVEYINTIME) + " com2\r\n");
                gnss.sendCommand("saveconfig\r\n");

                baseState = BASESTATE::BASE_SURVEYING;
            }
        }
        break;
    }
    case BASESTATE::BASE_SURVEYING: {
        unsigned long elapsed = millis() - timeSurveyStart;
        if (elapsed < (SURVEYINTIME + 1) * 1000) {
            float err = 99.99;
            screen.setLine(0, "HDOP " + String(err, 2) + "m");
            screen.setLine(1, "Time left: " + String(((SURVEYINTIME * 1000) - elapsed) / 1000) + "s");
            break;
        }
        Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");
        gnss.sendCommand("unlog\r\n"); 
        gnss.sendCommand("config signalgroup 1\r\n");
        gnss.sendCommand("rtcm1006 com2 1\r\n");
        gnss.sendCommand("rtcm1033 com2 5\r\n");
        gnss.sendCommand("rtcm1074 com2 5\r\n");
        gnss.sendCommand("rtcm1084 com2 2\r\n");
        gnss.sendCommand("rtcm1230 com2 2\r\n");
        gnss.sendCommand("saveconfig\r\n");
        Serial.println("BASE_SURVEYING: RTCM config done, entering OPERATING mode.");
        baseState = BASESTATE::BASE_OPERATING;
        break;
    }
    case BASESTATE::BASE_OPERATING: {
        uint8_t rtcmBuf[1024];
        size_t len = 0;

        screen.setLine(0, "Operating");
        screen.setLine(1, "***");

        if (gnss.readRTCM(rtcmBuf, len)) {
            if (len > 0) {
                uint16_t type = gnss.getRTCMBits(rtcmBuf, 24, 12);
                Serial.printf("%4zu bytes RTCM%4u\n", len, type);
                radioMod.sendFragmentedRTCM(rtcmBuf, len);
            }
        }
        break;
    }
    default:
        break;
    }
    delay(10);
}
