//RTKBase.ino
/*
 Name:		RTKBase.ino
 Created:	6/19/2025 8:45:24 PM
 Author:	JanOlavEndrerud
*/
#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "TwoButtonMenu.h"

#include <RTKF3F.h>
#include "src/RTCMFwd.h"
#include "src/SurveyIn.h"
#include "LCD.h"

#define APPNAME "RTKBase 1.0"

//LCD I2C PINS
#define LCD_SDA       21
#define LCD_SCL       22

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Buttons
#define BTN_MENU     12
#define BTN_SELECT   13
#define HOLD_TIME    800  // ms

TwoButtonMenu menu(lcd);

// Menu state
bool menuActive = true;
unsigned long menuPressStart = 0;
bool menuHeld = false;

// Radio RFM69HCW SDI PINS
#define RFM69_CS       5
#define RFM69_IRQ      4
#define RFM9_SCK      18
#define RFM9_MISO     19
#define RFM9_MOSI     23  

RFM69 txRadio;  // RTCM to GU
bool txRadioReady = false;

// GPS UART PINS
#define GNSS_BAUD 115200
#define UART_TX       17
#define UART_RX       16

HardwareSerial& serialGPS = Serial2;

#define SURVEYINTIME 15000

int surveyStart = 0;
int timeUpdateLCD = 0;

String prevlcdLine1 = "";
String prevlcdLine2 = "";

enum BASESTATE {
    BASE_IDLE,
    BASE_SURVEYING,
    BASE_OPERATING,
    BASE_MENU
};

BASESTATE baseState = BASESTATE::BASE_IDLE;

void applySettings() {
    menuActive = false;

    Serial.println("=== Settings Applied ===");
    Serial.printf("RTCM Hz   : %s\n", menu.getValueByLabel("RTCM Hz").c_str());
    Serial.printf("Survey T  : %s\n", menu.getValueByLabel("Survey T").c_str());

    updateLCD(true, APPNAME, "Settings applied");

    delay(1500);
}

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	while (!Serial);

	Wire.begin(LCD_SDA, LCD_SCL); // Set I2C pins for ESP32
	lcd.init();
	lcd.backlight();

	// Buttons preparation
    pinMode(BTN_MENU, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);

    // Menu setup
    menu.addItem("Set RTCM Hz",  { "1", "5", "10", "20" });
    menu.addItem("Set Survey T", { "30s", "60s", "120s" });

    menu.begin();

    // --- TX radio ---
    updateLCD(true, APPNAME, "Init TX Radio");
    if (!txRadio.initialize(RF69_868MHZ, BASE_NODE_ID, NETWORK_ID) || !verifyRadio(txRadio, "TX")) {
        Serial.println("TX radio init failed");
        updateLCD(true, APPNAME, "Radio fail");
    }
    else {
        txRadio.setHighPower();
        txRadio.setPowerLevel(31);
        txRadio.setFrequency(BS_TX_FREQ);
        txRadioReady = true;
    }

    // --- GNSS UART ---
    serialGPS.begin(GNSS_BAUD, SERIAL_8N1, UART_RX, UART_TX);

    initRTCMForwarder(&serialGPS, &txRadio, 255); //255 = Broadcast
}

// the loop function runs over and over again until power down or reset
void loop() {

int fix = 0;
int sats = 0;
float HDOP = 0.0f;
String fixTitle = "";
String line1;
String line2;

    fix = checkFIX(fixTitle, sats, HDOP);

    switch (baseState) {
    case BASESTATE::BASE_IDLE: {
        enableStableGPS();
        while (fix < 1) {
            line1 = "S" + String(sats) + " " + String(fix) + ":" + fixTitle;
            line2 = "HDOP " + String(HDOP);
            updateLCD(true, line1, line2);
            delay(1000);
            fix = checkFIX(fixTitle, sats, HDOP);
        }
        baseState = BASESTATE::BASE_SURVEYING;
        surveyStart = millis();
        break;
    }
    case BASESTATE::BASE_SURVEYING: {
        enableSurveyIn(int(SURVEYINTIME / 1000));
        while ((millis() - surveyStart) < SURVEYINTIME) {
            line1 = "S" + String(sats) + " " + String(fix) + ":" + fixTitle;
            line2 = "SVY" + String((int(millis() - surveyStart) / 1000)) + "s HDOP " + String(HDOP);
            updateLCD(true, line1, line2);
            delay(500);
            fix = checkFIX(fixTitle, sats, HDOP);
        }
        baseState = BASESTATE::BASE_OPERATING;
        timeUpdateLCD = millis();
        break;
    }
    case BASESTATE::BASE_OPERATING: {
        //if ((millis() - timeUpdateLCD) > 3000) {
        line1 = "S" + String(sats) + " " + String(fix) + ":" + fixTitle;
        line2 = "OPER HDOP " + String(HDOP);
        updateLCD(true, line1, line2);
        timeUpdateLCD = millis();
        //}
        updateRTCMForwarder();
        break;
    }
        default:
			break;
	}

    delay(1000);

    if (!menuActive) return;

    // MENU button: long vs short press
    if (!digitalRead(BTN_MENU)) {
        if (menuPressStart == 0) {
            menuPressStart = millis();
        }
        else if (!menuHeld && millis() - menuPressStart > HOLD_TIME) {
            menuHeld = true;
            applySettings();  // Long press
        }
    } else {
        if (menuPressStart > 0 && !menuHeld) {
            menu.nextItem();  // Short press
        }
        menuPressStart = 0;
        menuHeld = false;
    }

    // SELECT button (cycle value)
    if (!digitalRead(BTN_SELECT)) {
        menu.selectValue();
        delay(200);
    }


}
