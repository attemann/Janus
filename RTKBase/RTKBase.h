#pragma once

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LCD I2C
#define LCD_SDA 4
#define LCD_SCL 5
#define I2C_TIMEOUT_MS 100
#define I2C_RETRY_COUNT 3

LiquidCrystal_I2C lcd(0x27, 16, 2);

static char prevLine[2][17] = { "                ", "                " };
static bool lcdInitialized = false;

// Initialize LCD with error handling
static bool lcdBegin() {
    for (int retry = 0; retry < I2C_RETRY_COUNT; retry++) {
        Wire.begin(LCD_SDA, LCD_SCL);
        Wire.setTimeout(I2C_TIMEOUT_MS);

        // Test I2C communication
        Wire.beginTransmission(0x27);
        if (Wire.endTransmission() == 0) {
            lcd.init();
            lcd.backlight();
            lcd.clear();
            lcdInitialized = true;
            DDBG_PRINTLN("LCD initialized successfully");
            return true;
        }

        DDBG_PRINTF("LCD init retry %d\n", retry + 1);
        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    DDBG_PRINTLN("LCD initialization failed!");
    lcdInitialized = false;
    return false;
}

// Safe I2C recovery
static void recoverI2C() {
    DDBG_PRINTLN("Attempting I2C recovery...");
    Wire.end();
    delay(100);
    lcdInitialized = false;
    lcdBegin();
}

// Set padded line with flicker prevention and error handling
static void setLine(uint8_t line, const char* text) {

    // Prepare padded string (fixed size, no dynamic allocation)
    char padded[17] = { 0 };
    strncpy(padded, text, 16);
    padded[16] = '\0';

    // Pad with spaces
    for (int i = strlen(padded); i < 16; i++) {
        padded[i] = ' ';
    }

    // Only update if changed
    if (strcmp(prevLine[line], padded) != 0) {


        lcd.setCursor(0, line);
        lcd.print(padded);
        strcpy(prevLine[line], padded);

        //DDBG_PRINTF(" LCD%d [%s]", line, padded);
        if (line == 1) DDBG_PRINTLN();
    }
}

void sendToDisplay(const String& l1, const String& l2) {
    if (!lcdInitialized) {
        if (!lcdBegin()) {
            DDBG_PRINTLN("LCD not initialized, cannot display");
            return;
        }
    }

    // Try I2C operation with error handling
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission() != 0) {
        DDBG_PRINTLN("I2C error detected, attempting recovery");
        recoverI2C();
        return;
    }
    setLine(0, l1.c_str());
    setLine(1, l2.c_str());
}
