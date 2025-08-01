//RTKBase.ino
#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include "RTKF3F.h"
#include "LCD.h"
#include "TwoButtonMenu.h"

#define APPNAME "RTKBase 1.0"

//LCD i2C
#define LCD_SDA       21
#define LCD_SCL       22

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

// UART
#define GNSS_BAUD 115200
#define UART_RX 16
#define UART_TX 17



LiquidCrystal_I2C lcd(0x27, 16, 2);
TwoButtonMenu menu(lcd);

HardwareSerial SerialGNSS(2);
RFM69 radio(RFM69_CS, RFM69_IRQ, true);

GNSSModule gnss(SerialGNSS);
RadioModule radioMod(radio);

bool menuActive = true;
unsigned long menuPressStart = 0;
bool menuHeld = false;
int timeSurveyStart = 0;
String prevlcdLine1 = "";
String prevlcdLine2 = "";

enum BASESTATE {
    BASE_STARTING,
    BASE_GETTINGFIX,
    BASE_SURVEYING,
    BASE_OPERATING,
    BASE_MENU
};
BASESTATE baseState = BASESTATE::BASE_STARTING;

GNSSModule::GNSSFix fix;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    Serial.println("Base: Boot");

    Wire.begin(LCD_SDA, LCD_SCL);
    lcd.init();
    lcd.backlight();
    Serial.println("Base: LCD Init ok");

    if (!radioMod.init(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS, RFM69_IRQ, RFM69_RST)) {
        Serial.println("Base: Stuck in radio init failure loop, freeze");
        while (true);
    }
    Serial.println("Base: Radio init ok");

    if (!radioMod.verify()) {
        Serial.println("Base: Radio verification failed, freeze");
        while (true);
    }
    Serial.println("Base: Radio verified");

    pinMode(BTN_MENU, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);

    menu.addItem("Set Survey T", { "30s", "60s", "120s" });
    menu.begin();
    Serial.println("Base: Menu init ok");

    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
    if (!gnss.init()) {
        Serial.println("Base: GPS comm init fail, freeze");
        while (true);
    }
    if (gnss.detectUARTPort() == 0) {
        Serial.println("Base: GPS port detection fail, freeze");
        while (true);
	}
    Serial.println("Base: GPS init ok");

    baseState = BASESTATE::BASE_STARTING;
}

void loop() {
    uint8_t rtcmBuf[1024];
    size_t len = 0;

    String line1 = "";
    String line2 = "";
    String cmd = "";

    switch (baseState) {
    case BASESTATE::BASE_STARTING: {
        baseState = BASESTATE::BASE_GETTINGFIX;
        break;
    }
    case BASESTATE::BASE_GETTINGFIX: {
        Serial.print("BASE_GETTINGFIX > ");

        String line = SerialGNSS.readStringUntil('\n');
        line.trim();

        if (line.startsWith("$GNGGA")) {
            Serial.println(line);
			gnss.parseGGA(line.c_str(), fix);
            gnss.showFix(fix);

            line1 = String(fix.fixType) + " " + gnss.fixTypeToString(fix.fixType);
            line2 = "SIV: " + String(fix.SIV);
            //Serial.print(" " + line1);
            //Serial.println(", " + line2);
            updateLCD(false, line1, line2);

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
            line1 = "HDOP " + String(err, 2) + " m";
            line2 = "Time left: " + String(((SURVEYINTIME * 1000) - elapsed) / 1000) + "s";
            updateLCD(false, line1, line2);
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
        line1 = "Operating";
        line2 = "***";
        updateLCD(false, line1, line2);


        if (gnss.readRTCM(rtcmBuf, len)) {
            if (len > 0) {
                radioMod.sendFragmentedRTCM(rtcmBuf, len);
                uint16_t type = gnss.getRTCMBits(rtcmBuf, 24, 12);
                Serial.printf("%4u: Sent %4zu bytes\n", type, len);
            }
        }
        break;
    }
    default:
        break;
    }
    delay(100);
}
