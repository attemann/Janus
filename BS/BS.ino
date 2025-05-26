//bs.ino
//#define DEBUG_TASKSTATE
//#define DEBUG_BUTTONS

#include <Arduino.h>
#include <RFM69.h>
#include <LiquidCrystal_I2C.h>

#include <RTKF3F.h>
#include "src/RTCMFwd.h"
#include "src/StateMachine.h"
#include "src/Receiver.h"
#include "src/SurveyIn.h"
#include "src/BSx.h"

#define GNSS_BAUD 115200
#define RX_RADIO_DIO0_PIN 6

#define BUZZER_PIN 14

RFM69 txRadio;  // RTCM to GU
RFM69 rxRadio;  // Events from GU

bool txRadioReady = false;
bool rxRadioReady = false;

HardwareSerial& serialGPS = Serial2;
volatile bool guMessageReceived = false;

Slope slope;
StateMachine taskStateMachine;

BSState oldBSState;

// Konfigurasjon av knapp-pinner
const int buttonPins[] = { BTN_MNU, BTN_INC, BTN_DEC, BTN_ESC };
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
bool lastState[numButtons];

LiquidCrystal_I2C lcd(0x27, 16, 2);

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

void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(BUZZER_PIN, OUTPUT);

    for (int i = 0; i < numButtons; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);  // Bruk intern pull-up
        lastState[i] = digitalRead(buttonPins[i]);  // Initial status
    }

    lcd.init();
    lcd.backlight();

    // --- TX radio ---
    updateStartLCD(true, "RTKF3F BS", "Init TX Radio");
    if (!txRadio.initialize(RF69_868MHZ, BASE_NODE_ID, NETWORK_ID) || !verifyRadio(txRadio, "TX")) {
        Serial.println("TX radio init failed");
        updateStartLCD(true, "RTKF3F BS", "TX init fail");
        //while (true) delay(1000);
    }
    else {
        txRadio.setHighPower();
        txRadio.setPowerLevel(31);
        txRadio.setFrequency(BS_TX_FREQ);
        txRadioReady = true;
    }

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

    attachInterrupt(digitalPinToInterrupt(RX_RADIO_DIO0_PIN), onGUMessageInterrupt, RISING);

    // --- GNSS UART ---
    serialGPS.begin(GNSS_BAUD, SERIAL_8N1, 4, 5); // RX = GPIO4, TX = GPIO5
    updateStartLCD(true, "RTKF3F BS", "Starting....");
    delay(1000);

    configureSurveyIn();
    taskStateMachine.begin();

    // --- Survey-in loop ---
    unsigned long durationMs = 0;
    float accuracyM = 0.0;
    int numStar = 4;

    while (!checkSurveyStatus(durationMs, accuracyM)) {
        String line1 = "Survey-in " + generateStars(numStar);
        String line2 = "T:" + String(durationMs) + "s A:" + String(accuracyM, 2) + "m";
        updateStartLCD(false, line1, line2);
        delay(300);
        if (++numStar > 4) numStar = 0;
    }

    initRTCMForwarder(&serialGPS, &txRadio, NODE_ID_INIT);
    updateStartLCD(true, "RTKF3F BS", "Ready to fly");
    //taskStateMachine.baseState(BS_WAITING);
    taskStateMachine.taskState(TASK_UNKNOWN);
}

void loop() {

    

    handleAllButtons();

    taskStateMachine.setPilotOffset(5000+random(20), 5000+random(20), 0);

    updateRTCMForwarder();

    if (guMessageReceived) {
        guMessageReceived = false;
        if (rxRadio.receiveDone()) {
            handleIncomingMessage(rxRadio.DATA, rxRadio.DATALEN);
        }
    }

    if ((taskStateMachine.baseState() != BS_AIRBORNE)) {
        taskStateMachine.updateLCD();
        oldBSState = taskStateMachine.baseState();
    }

    yield();
}
