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

#define BUFFER_SIZE 256
uint8_t buffer[BUFFER_SIZE];
uint16_t bufIndex = 0;
volatile bool commandDone = false;

#define SURVEYINTIME 15000

int surveyStart = 0;
int timeUpdateLCD = 0;

String prevlcdLine1 = "";
String prevlcdLine2 = "";

// Kommando kø
#define MAX_COMMANDS 20
#define MAX_RESPONSE_TIME 1000  // ms to wait for a response

String commandQueue[MAX_COMMANDS];
int queueHead = 0;
int queueTail = 0;
bool commandPending = false;
unsigned long commandSentTime = 0;
String lastCommandSent = "";

enum BASESTATE {
    BASE_IDLE,
    BASE_SURVEYING,
    BASE_OPERATING,
    BASE_MENU
};

BASESTATE baseState = BASESTATE::BASE_IDLE;


void enqueueCommand(const String& cmd) {

    Serial.println("Queing up " + cmd);
    int nextTail = (queueTail + 1) % MAX_COMMANDS;
    if (nextTail != queueHead) { // Avoid overflow
        commandQueue[queueTail] = cmd;
        queueTail = nextTail;
    }
    else {
        Serial.println("[CMD QUEUE] Full!");
    }
}

void processCommandQueue() {
    if (commandPending) {
        // Check if response received
        if (commandDone) {
            Serial.println("[CMD OK]");
            commandPending = false;
            commandDone = false;  // Reset for next command
        }
        // Timeout check
        else if (millis() - commandSentTime > MAX_RESPONSE_TIME) {
            Serial.println("[CMD TIMEOUT]");
            commandPending = false;
            commandDone = false;  // Reset to be safe
        }
        return;
    }

    // Command queue not empty
    if (queueHead != queueTail) {
        String cmd = commandQueue[queueHead];
        queueHead = (queueHead + 1) % MAX_COMMANDS;

        Serial.println("Queue: processing " + cmd);

        myGNSS.println(cmd);  // Send to GNSS
        lastCommandSent = cmd;

        commandPending = true;
        commandSentTime = millis();
        commandDone = false;

        Serial.print("[CMD SEND] ");
        Serial.println(cmd);
    }
}


void applySettings() {
    menuActive = false;

    Serial.println("=== Settings Applied ===");
    Serial.printf("RTCM Hz   : %s\n", menu.getValueByLabel("RTCM Hz").c_str());
    Serial.printf("Survey T  : %s\n", menu.getValueByLabel("Survey T").c_str());

    updateLCD(true, APPNAME, "Settings applied");

    delay(1500);
}

void IRAM_ATTR onGNSSData() {
    static char lineBuffer[BUFFER_SIZE];
    static uint16_t lineIndex = 0;

    while (myGNSS.available()) {
        uint8_t b = myGNSS.read();

        // --- Prevent overflow ---
        if (bufIndex >= BUFFER_SIZE - 1) bufIndex = 0;
        if (lineIndex >= BUFFER_SIZE - 1) lineIndex = 0;

        buffer[bufIndex++] = b;
        lineBuffer[lineIndex++] = b;
        lineBuffer[lineIndex] = '\0';

        // --- RTCM3 detection ---
        if (buffer[0] == 0xD3 && bufIndex >= 3) {
            uint16_t rtcm_len = ((buffer[1] & 0x03) << 8) | buffer[2];
            if (bufIndex == rtcm_len + 6) {
                radio.send(255, buffer, rtcm_len + 6);
                Serial.printf("[RTCM3] Sent %d bytes\n", rtcm_len + 6);
                bufIndex = 0;
                continue;
            }
        }

        // --- New line detected ---
        if (b == '\n') {
            // Handle NMEA (starts with $GN, $GP, $GA, etc.)
            if (strncmp(lineBuffer, "$GN", 3) == 0 || strncmp(lineBuffer, "$GP", 3) == 0 ||
                strncmp(lineBuffer, "$GA", 3) == 0 || strncmp(lineBuffer, "$BD", 3) == 0 ||
                strncmp(lineBuffer, "$GL", 3) == 0) {
                Serial.print("[NMEA] ");
                Serial.println(lineBuffer);

                if (strstr(lineBuffer, "$GPGGA") || strstr(lineBuffer, "$GNGGA")) {
                    Serial.print("[GPGGA] ");
                    Serial.println(lineBuffer);
                }
            }
            // Handle command responses
            else if (strncmp(lineBuffer, "$command", 8) == 0) {
                Serial.print("[GPS RESP] ");
                Serial.println(lineBuffer);

                if (strstr(lineBuffer, "OK")) {
                    Serial.println("[GPS] Received OK response");
                    commandDone = true;
                }
                else if (strstr(lineBuffer, "ERROR")) {
                    Serial.println("[GPS] Received ERROR response");
                    commandDone = true;
                }
            }
            // Other text-based info
            else if (lineBuffer[0] == '#') {
                Serial.print("[GPS RESP] ");
                Serial.println(lineBuffer);
                Serial.println("[GPS] Version or info:");
            }

            // Reset buffer for next line
            lineIndex = 0;
        }
    }
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
    menu.addItem("Set RTCM Hz", { "1", "5", "10", "20" });
    menu.addItem("Set Survey T", { "30s", "60s", "120s" });
    menu.begin();
    Serial.println("> Menu init ok");

    // --- GNSS UART ---
    if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
    {
        Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
        while (true);
    }
    Serial.println("UM980 detected!");
    Serial.println("> GPS init ok");
}

// the loop function runs over and over again until power down or reset
void loop() {

int fix = 0;
int sats = 0;
float HDOP = 0.0f;
String fixTitle = "";
String line1;
String line2;

    processCommandQueue();

    switch (baseState) {
    case BASESTATE::BASE_SURVEYING: {
        enqueueCommand("unlog");
        enqueueCommand("config pvtalg multi");
        enqueueCommand("mode base time " + String(int(SURVEYINTIME / 1000)));
        enqueueCommand("gpgga 1");

        baseState = BASESTATE::BASE_OPERATING;
        
        // Beskrivelse : Sender basestasjonens Antenne Referanse Punkt(ARP) koordinater, inkludert antennehøyden.Dette er kritisk slik at roveren vet basestasjonens nøyaktige posisjon.
        enqueueCommand("rtcm1006 com2 10");

        // Beskrivelse: Sender mottaker- og antennebeskrivelser. Dette gir roveren viktig metadata om basestasjonens utstyr.
        // Utgangsfrekvens : 10 Hz er anbefalt.
        enqueueCommand("rtcm1033 com2 10");	

        // Beskrivelse : Sender GPS - korreksjonsdata(MSM4).Dette er fullstendige GPS pseudorange - og faserange - observasjoner pluss CNR(Carrier - to - Noise Ratio).Essensielt for GPS RTK.
        // Utgangsfrekvens : 1 Hz er en vanlig frekvens for observasjonsdata.
        enqueueCommand("rtcm1074 com2 1");

        // Beskrivelse: Sender BDS - korreksjonsdata(MSM4).Tilbyr fullstendige BeiDou pseudorange - og faserange - observasjoner pluss CNR.
        // Essensielt for BDS RT
        enqueueCommand("rtcm1124 com2 1");

        // Beskrivelse: Sender GLONASS - korreksjonsdata(MSM4).Inkluderer fullstendige GLONASS pseudorange - og faserange - observasjoner pluss CNR.Essensielt for GLONASS RTK.
        enqueueCommand("rtcm1084 com2 1");

        // Beskrivelse: Sender Galileo - korreksjonsdata.Essensielt for Galileo RTK
        enqueueCommand("rtcm1094 com2 1");

        enqueueCommand("gpgga 1");

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
        //updateRTCMForwarder();
        break;
    }
        default:
			break;
	}

    yield();

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
