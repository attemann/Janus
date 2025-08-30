//DisplayTask.cpp
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "DisplayTask.h"
#include "_macros.h"

#ifndef ARDUINO_ARCH_ESP32
#error ESP32 only
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if (configNUM_CORES > 1)
#define CORE_APP   1
#define CORE_COMM  0
#define CORE_ANY   tskNO_AFFINITY
#else
#define CORE_APP   0
#define CORE_COMM  0
#define CORE_ANY   tskNO_AFFINITY
#endif

// LCD I2C

#define LCD_SDA 21
#define LCD_SCL 22

LiquidCrystal_I2C lcd(0x27, 16, 2);

TaskHandle_t displayTaskHandle = nullptr;
QueueHandle_t displayQueue = nullptr;

static String prevLine[2] = { String(16, ' '), String(16, ' ') };

// Initialize LCD
static void lcdBegin() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

// Set padded line with flicker prevention
static void setLine(uint8_t line, const String& text) {
    if (line > 1) return;

    String padded = text;
    if (padded.length() > 16)
        padded = padded.substring(0, 16);
    while (padded.length() < 16)
        padded += ' ';

    if (prevLine[line] != padded) {
        lcd.setCursor(0, line);
        lcd.print(padded);
        prevLine[line] = padded;

		DDBG_PRINTF(" LCD%d [%s]", line, padded.c_str());
        if (line == 1) DDBG_PRINTLN();
    }
}

// Display task loop
void displayTask(void* pvParameters) {
    lcdBegin();

    DisplayMessage msg;
    while (true) {
        if (xQueueReceive(displayQueue, &msg, pdMS_TO_TICKS(3000))) {
            setLine(0, msg.line1);
            setLine(1, msg.line2);
        }
        // else: do nothing (keep last displayed)
    }
}

// Call this from setup()
void startDisplayTask() {
    displayQueue = xQueueCreate(4, sizeof(DisplayMessage));
    xTaskCreatePinnedToCore(
        displayTask,
        "LCDTask",
        2048,
        nullptr,
        1,
        &displayTaskHandle,
        CORE_COMM  // Core 1 (UI core)
    );
}

// Helper to send message
void sendToDisplay(const String& l1, const String& l2) {
    if (!displayQueue) return;

    DisplayMessage msg;
    strncpy(msg.line1, l1.c_str(), 16);
    strncpy(msg.line2, l2.c_str(), 16);
    msg.line1[16] = '\0';
    msg.line2[16] = '\0';
            
    xQueueSend(displayQueue, &msg, 0);
}

void sendToDisplayIfChanged(const String& l1, const String& l2, uint32_t minIntervalMs = 1000) {
    static uint32_t lastSendMs = 0;
    static String prevL1 = "", prevL2 = "";

    uint32_t now = millis();
    if ((l1 != prevL1 || l2 != prevL2) && (now - lastSendMs >= minIntervalMs)) {
        sendToDisplay(l1, l2);
        prevL1 = l1;
        prevL2 = l2;
        lastSendMs = now;
    }
}