//bs.ino
//#define DEBUG_TASKSTATE
//#define DEBUG_BUTTONS

#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include <RTKF3F.h>

#include "src/StateMachine.h"
#include "src/Receiver.h"
#include "src/BSx.h"



// ESP32-WROOM-32 Pinout

// Radio RFM69HCW SDI
#define RFM69_CS       5
#define RFM69_IRQ      4
#define RFM9_SCK      18
#define RFM9_MISO     19
#define RFM9_MOSI     23  



//LCD I2C
#define LCD_SDA       21
#define LCD_SCL       22

// Buttons, LED, Buzzer
#define BUZZER_PIN    14
#define BTN_MNU       32
#define BTN_INC       25
#define BTN_DEC       33
#define BTN_ESC       26

// Ledige pinner
// GPIO0  (can be used, but keep HIGH at boot if not flashing)
// GPIO2  (can be used, but has boot strapping role — keep LOW at boot)
// GPIO1  (avoid using unless you know what you're doing — affects flash voltage selection
// GPIO13
// GPIO15 (has boot function — must be LOW at boot)
// GPIO27
// GPIO34 (input only)
// GPIO35 (input only)
// GPIO36(VP) (input only)
// GPIO39(VN) (input only)


RFM69 rxRadio;  // Events from GU


bool rxRadioReady = false;


volatile bool guMessageReceived = false;

Slope slope;
StateMachine taskStateMachine;

BSState oldBSState;

// Konfigurasjon av knapp-pinner
const int buttonPins[] = { BTN_MNU, BTN_INC, BTN_DEC, BTN_ESC };
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
bool lastState[numButtons];

LiquidCrystal_I2C lcd(0x27, 16, 2);

void handleAllButtons() {
    for (int i = 0; i < numButtons; i++) {
        bool currentState = digitalRead(buttonPins[i]);

        if (currentState != lastState[i]) {
            lastState[i] = currentState;
            if (currentState == LOW) {  // kun p  trykk ned

                taskStateMachine.handleButton(buttonPins[i]);

                for (int j = 0; j < 100; j++) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delayMicroseconds(500); // 250us high
                    digitalWrite(BUZZER_PIN, LOW);
                    delayMicroseconds(1000); // 250us low (1000us = 1ms = 1kHz)
                }
            }
        }
    }
}

void IRAM_ATTR onGUMessageInterrupt() {
    guMessageReceived = true;
}

void updateStartLCD(bool clear, String line1, String line2) {
    if (clear) lcd.clear();
    lcd.setCursor(0, 0); lcd.print(line1);
    lcd.setCursor(0, 1); lcd.print(line2);

    Serial.println("---");
    Serial.println(line1);
    Serial.println(line2);
}



void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(BUZZER_PIN, OUTPUT);

    for (int i = 0; i < numButtons; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);  // Bruk intern pull-up
        lastState[i] = digitalRead(buttonPins[i]);  // Initial status
    }

    Wire.begin(LCD_SDA, LCD_SCL); // Set I2C pins for ESP32
    lcd.init();
    lcd.backlight();



    // --- RX radio ---
    updateStartLCD(true, "RTKF3F BS", "Init RX Radio");
    if (!rxRadio.initialize(RF69_868MHZ, BASE_NODE_ID, NETWORK_ID) || !verifyRadio(rxRadio, "RX")) {
        Serial.println("RX radio init failed");
        updateStartLCD(true, "RTKF3F BS", "RX init fail");
        //while (true) delay(1000);
    }
    else {
        rxRadio.setHighPower();
        rxRadio.setPowerLevel(31);
        rxRadio.setFrequency(BS_RX_FREQ);
        txRadioReady = true;
    }

    attachInterrupt(digitalPinToInterrupt(RFM69_IRQ), onGUMessageInterrupt, RISING);



    taskStateMachine.baseState(BS_WAITING);
    taskStateMachine.taskState(TASK_UNKNOWN);
}

void loop() {

    handleAllButtons();

    taskStateMachine.setPilotOffset(5000+random(20), 5000+random(20), 0);



    if (guMessageReceived) {
        guMessageReceived = false;
        if (rxRadio.receiveDone()) {
            handleIncomingMessage(rxRadio.DATA, rxRadio.DATALEN);
        }
    }

    //if ((taskStateMachine.baseState() != BS_AIRBORNE)) {
        taskStateMachine.updateLCD();
        oldBSState = taskStateMachine.baseState();
    //}

    yield();
}

