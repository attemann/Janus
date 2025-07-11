//RTKBase.ino
/*
 Name:		RTKBase.ino
 Created:	6/19/2025 8:45:24 PM
 Author:	Jan Olav Endrerud
*/
#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS


//#include <RTKF3F.h>
#include "LCD.h"

#include "TwoButtonMenu.h"

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
#define RFM69_SCK      18
#define RFM69_MISO     19
#define RFM69_MOSI     23  
#define RFM69_RST      -1
RFM69 radio(RFM69_CS, RFM69_IRQ, true); // true = SPI driver override

// GPS UART PINS
#define GNSS_BAUD 115200
#define UART_TX       17
#define UART_RX       16

UM980 myGNSS;
HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

#define BUFFER_SIZE 3000
uint8_t rtcmBuffer[BUFFER_SIZE];
uint16_t rtcmIndex = 0;
bool readingRtcm = false;
uint16_t expectedLength = 0;

#define SURVEYINTIME 10

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

void applySettings() {
    menuActive = false;

    Serial.println("=== Settings Applied ===");
    Serial.printf("RTCM Hz   : %s\n", menu.getValueByLabel("RTCM Hz").c_str());
    Serial.printf("Survey T  : %s\n", menu.getValueByLabel("Survey T").c_str());

    updateLCD(true, APPNAME, "Settings applied");

    delay(1500);
}

float getHDOP() {
    float latDev = myGNSS.getLatitudeDeviation();
    float lonDev = myGNSS.getLongitudeDeviation();

    return sqrt((latDev * latDev) + (lonDev * lonDev));
}


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	while (!Serial);
    delay(500);
    Serial.println("Boot");

	Wire.begin(LCD_SDA, LCD_SCL); // Set I2C pins for ESP32
	lcd.init();
	lcd.backlight();
    Serial.println("> LCD Init ok");

	// --- RADIO ---
    SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS); // SCK, MISO, MOSI, SS

    if (RFM69_RST != -1) {
        pinMode(RFM69_RST, OUTPUT);
        digitalWrite(RFM69_RST, LOW);
        delay(10);
        digitalWrite(RFM69_RST, HIGH);
        delay(10);
    }

    pinMode(RFM69_IRQ, INPUT);

    if (!radio.initialize(RF69_868MHZ, 1, 100)) {
        Serial.println("RFM69 init failed!");
        while (1) {
            Serial.println("Stuck in init failure loop");
            delay(1000);
        }
    }
    else {
        Serial.println("> Radio init ok");
    }

    radio.setHighPower();
    radio.encrypt(NULL); // No encryption, matching sender

    // Buttons preparation
    pinMode(BTN_MENU, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);

    // Menu setup
    //menu.addItem("Set RTCM Hz", { "1", "5", "10", "20" });
    menu.addItem("Set Survey T", { "30s", "60s", "120s" });
    menu.begin();
    Serial.println("> Menu init ok");

    // --- GNSS UART ---
    //We must start the serial port before using it in the library
    SerialGNSS.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

    //myGNSS.enableDebugging(); // Print all debug to Serial
    if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
    {
        Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
        while (true);
    }

    Serial.println("> GPS init ok");

    baseState = BASESTATE::BASE_STARTING;
}

// the loop function runs over and over again until power down or reset
void loop() {
bool allOk = true;
String line1 = "";
String line2 = "";
String cmd = "";
int SIV = 0;
int fixType = 0;

    myGNSS.update(); //Regularly call to parse any new data

    SIV = myGNSS.getSatellitesUsed();
    fixType = myGNSS.getPositionType();
	//Serial.println("Base state: " + String(int(baseState)));

    switch (baseState) {
    case BASESTATE::BASE_STARTING: {
        //Serial.println("baseState: BASE_STARTING");
        allOk = myGNSS.sendCommand("unlog");
        allOk &= myGNSS.sendCommand("config pvtalg multi");

        myGNSS.setModeRover("rover");
        myGNSS.initBestnav(1);
        allOk &= myGNSS.sendCommand("saveconfig");

        if (allOk) {
            baseState = BASESTATE::BASE_GETTINGFIX;
            Serial.println("BASE_STARTING: GNSS commands sent successfully");
        } else {
            Serial.println("BASE_STARTING: Failed to send GNSS commands, freeze");
            while (1);
		}

        break;
    }
    case BASESTATE::BASE_GETTINGFIX: {
        //Serial.println("baseState: BASE_GETTINGFIX");
		line1 = "GetFix " + String(SIV) + " sats";
        line2 = "Fix: " + String(fixType);
		updateLCD(false, line1, line2);
        if (fixType == 1 || fixType == 16) {
            timeSurveyStart = millis();
            //myGNSS.setModeBaseAverage(SURVEYINTIME);
            allOk = true;
            allOk &= myGNSS.sendCommand("unlog");
            cmd = "mode base time " + String(SURVEYINTIME) + " com2";
            allOk &= myGNSS.sendCommand(cmd.c_str());
            allOk &= myGNSS.sendCommand("saveconfig");

            if (allOk) {
                Serial.println("BASE_GETTINGFIX: Enter mode base time success");
            }
            else {
                Serial.println("BASE_GETTINGFIX: Enter mode base time fail, freeze");
                while (1);
            }
			baseState = BASESTATE::BASE_SURVEYING;
        }
        break;
    }
    case BASESTATE::BASE_SURVEYING: {
        Serial.println("baseState: BASE_SURVEYING");

        float err = getHDOP();

        // Still surveying
        unsigned long elapsed = millis() - timeSurveyStart;
        if (elapsed < SURVEYINTIME * 1000) {
            line1 = "HDOP " + String(err, 2) + " m";
            line2 = "Time left: " + String((SURVEYINTIME * 1000 - elapsed) / 1000) + "s";
            updateLCD(false, line1, line2);
            break;
        }

        // Survey complete — configure base output
        Serial.println("BASE_SURVEYING: Survey complete. Enabling RTCM...");

        allOk = true;

        allOk &= myGNSS.sendCommand("rtcm1006 com2 10");
        allOk &= myGNSS.sendCommand("rtcm1033 com2 10");
        allOk &= myGNSS.sendCommand("rtcm1074 com2 1");
        allOk &= myGNSS.sendCommand("rtcm1124 com2 1");
        allOk &= myGNSS.sendCommand("rtcm1084 com2 1");
        allOk &= myGNSS.sendCommand("rtcm1094 com2 1");
        allOk &= myGNSS.sendCommand("saveconfig");
        if (allOk) {
            Serial.println("BASE_SURVEYING: RTCM config successful. Entering OPERATING mode.");
            baseState = BASESTATE::BASE_OPERATING;
        }
        else {
            Serial.println("BASE_SURVEYING: RTCM config fail. Freezing.");
            while (1); // Lock system if config fails
        }

        break;
  
    }
    case BASESTATE::BASE_OPERATING: {
        //Serial.println("baseState: BASE_OPERATING");
        line1 = "Oper sat: " + String(SIV);
        line2 = "Fix " + String(fixType) + " err " + String(getHDOP());
        updateLCD(false, line1, line2);

        while (SerialGNSS.available()) {
            uint8_t b = SerialGNSS.read();

            //Serial.printf("Raw: 0x%02X\n", b);

            // Detect RTCM3 start
            if (!readingRtcm) {
                if (b == 0xD3) {
                    rtcmBuffer[0] = b;
                    rtcmIndex = 1;
                    readingRtcm = true;
                }
            }
            else {
                if (rtcmIndex < BUFFER_SIZE - 1) {
                    rtcmBuffer[rtcmIndex++] = b;

                    // Once we have enough for length (after 3 bytes)
                    if (rtcmIndex == 3) {
                        expectedLength = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];
                        expectedLength += 6; // Add header (3) + CRC (3)
                    }

                    if (rtcmIndex == expectedLength) {
                        // Full RTCM message received
                        radio.send(255, rtcmBuffer, rtcmIndex);
                        Serial.printf("Sent RTCM (%d bytes)\n", rtcmIndex);
                        readingRtcm = false;
                        rtcmIndex = 0;
                    }
                }
                else {
                    // Overflow
                    readingRtcm = false;
                    rtcmIndex = 0;
                }
            }
        }

        break;
    }
    default:
		break;
	}

    delay(500);

    /* if (!menuActive) return;

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
    */


}
